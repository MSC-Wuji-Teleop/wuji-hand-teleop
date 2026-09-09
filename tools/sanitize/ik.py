"""Staged IK: shoulder to elbow first, then the wrist, then the two together
with clearance in the same solve.

The solve order is the one docs/spec/RoboSTAR_dropin_fix.md asks for --
"start from shoulder to elbow, then take the target elbow positions as
starting position and solve for wrist pose" -- and it is also what the
kinematics want, because the split is exact rather than a convenience:

    stage 1, shoulder -> elbow
        The origin of {side}_elbow_link is the elbow joint centre, and the
        upper arm is rigid, so that point depends on the three shoulder
        joints and on nothing else -- not even on the elbow angle. Three
        unknowns for three equations: a square solve, no weighting, no
        trade-off with anything downstream.

    stage 2, elbow -> wrist
        With the shoulder fixed, the four joints left (elbow flexion and the
        three wrist joints) carry the 6-DoF placement of
        {side}_wrist_yaw_link. Four unknowns for six equations, so this one
        is least squares, and the residual it cannot absorb is reported per
        frame as the clip's pose error.

    stage 3, refine, with clearance in the same solve
        All 14 arm joints at once, both pose tasks, and one separation row
        per pair the frame is known to be close to, weighted three times the
        pose tasks and re-linearised on every iteration. This is the one stage
        that is allowed to break the hierarchy -- the shoulder has to be
        free, or a cross-side contact has nowhere to go -- and a frame that
        cannot be both collision-free and pose-faithful gives up pose.

        Clearance is a term this solve trades against, not a push applied
        to a finished frame: the pose rows and the separation rows are
        minimised together from the first iteration, so what comes back is
        their compromise. The stage runs twice on a frame with active pairs,
        once on the pose alone and once with the rows, but the second solve
        starts where the first one did rather than from its answer -- the
        pose-only solve is the centre of the trust region (see
        DEFAULT_AVOID_BUDGET_DEG), not a step on the way.

        Which pairs are active is the caller's business: cli.py sweeps the
        pair set and carries the active set from frame to frame, so a contact
        found on one frame is already a constraint on the next and the arm is
        held out of it before it arrives rather than pushed out after.

Why this fixes the branch flipping. A wrist pose on its own does not
determine the arm: solving only the 6-DoF wrist task from random seeds finds
32 distinct exact solutions for a single pose on this model, and a per-frame
solver is free to return a different one each frame -- the flipping the spec
reports, and the velocity spikes that come with it. Pinning the elbow first
removes that freedom entirely; the same experiment with the elbow position
imposed returns one solution, the source angles, bit for bit.

The exception is a frame where the SOURCE elbow is itself on the far branch,
which is what unwrap.detect_flips finds. Following it there would reproduce
the flip, so stage 1 is skipped for the length of a flipped span and stage 2
runs over all seven joints of that arm, with the continuity term to the
previous frame deciding the branch.

Every iterate of every stage is projected onto the intersection of the URDF
joint limits and a per-frame step box, so what comes back is always
commandable; what could not be reached inside them is the residual.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from clip_audit import SIDES

from .model import ARM_JOINTS_PER_SIDE, NUM_ARM_JOINTS, ArmTargets, SanitizeModel

# Constants. Each one says where its value comes from.

# Stage membership: yaw is stage 2, it swings the elbow only 0.0158 m/rad.
SHOULDER_ELBOW_LOCAL = (0, 1)
WRIST_CHAIN_LOCAL = (2, 3, 4, 5, 6)
ALL_LOCAL = SHOULDER_ELBOW_LOCAL + WRIST_CHAIN_LOCAL

# Null-space term only: two orders below the weakest task column.
DEFAULT_CONTINUITY_WEIGHT = 1e-4

# Elbow weight in stage 3, against the wrist pose at 1.
DEFAULT_REFINE_ELBOW_WEIGHT = 0.5

# Separation vs pose at 1. Measured: 3 leaves 757 pair-frames, 10 leaves 1249.
DEFAULT_SEPARATION_WEIGHT = 3.0

# LM damping: only bites where the source is singular (a straight arm).
DEFAULT_DAMPING = 1e-8

# Per stage. An exact round-trip needed 13 iterations; tolerances are its floor.
DEFAULT_MAX_ITERS = 60
DEFAULT_TOL_POS_M = 1e-6
DEFAULT_TOL_ORI_RAD = 1e-6

# A stage also stops once its step moves nothing (an unreachable target).
STEP_EPS_RAD = 1e-10

# Stage 3 with rows active; each iteration re-measures every pair, ~1.8 ms each.
DEFAULT_AVOID_ITERS = 12

# Trust region on the clearance solve: unbounded it walks 48 deg off the pose.
DEFAULT_AVOID_BUDGET_DEG = 5.0

# Extra step cap on top of the URDF velocity limit. Off: smoothing is upstream.
DEFAULT_MAX_STEP_DEG = math.inf


def side_columns(side: str, local: Sequence[int] = ALL_LOCAL) -> np.ndarray:
    """Global indices, in the 14-DoF arm vector, of one side's joints."""
    offset = SIDES.index(side) * ARM_JOINTS_PER_SIDE
    return np.array([offset + i for i in local], dtype=int)


@dataclass(frozen=True)
class IKWeights:
    """Weights that are not fixed by the staging itself."""
    continuity: float = DEFAULT_CONTINUITY_WEIGHT
    refine_elbow: float = DEFAULT_REFINE_ELBOW_WEIGHT
    separation: float = DEFAULT_SEPARATION_WEIGHT
    damping: float = DEFAULT_DAMPING

    def as_dict(self) -> dict:
        return {"continuity": self.continuity, "refine_elbow": self.refine_elbow,
                "separation": self.separation, "damping": self.damping}


@dataclass
class IKResult:
    """One frame's solve, after every stage that ran."""
    arm: np.ndarray                                  # (14,) solution
    pos_err_m: Dict[str, float] = field(default_factory=dict)
    ori_err_rad: Dict[str, float] = field(default_factory=dict)
    elbow_err_m: Dict[str, float] = field(default_factory=dict)
    iterations: Dict[str, int] = field(default_factory=dict)
    converged: bool = False
    constrained: bool = False       # stage 3 carried separation rows
    at_limit: np.ndarray = field(default_factory=lambda: np.zeros(NUM_ARM_JOINTS, dtype=bool))
    at_step_bound: np.ndarray = field(default_factory=lambda: np.zeros(NUM_ARM_JOINTS, dtype=bool))

    @property
    def max_pos_err_m(self) -> float:
        return max(self.pos_err_m.values()) if self.pos_err_m else 0.0

    @property
    def max_ori_err_rad(self) -> float:
        return max(self.ori_err_rad.values()) if self.ori_err_rad else 0.0

    @property
    def max_elbow_err_m(self) -> float:
        return max(self.elbow_err_m.values()) if self.elbow_err_m else 0.0


class ArmIK:
    """The staged solver, holding the model, the weights and the limits."""

    def __init__(self, sm: SanitizeModel,
                 weights: IKWeights = IKWeights(),
                 max_iters: int = DEFAULT_MAX_ITERS,
                 tol_pos_m: float = DEFAULT_TOL_POS_M,
                 tol_ori_rad: float = DEFAULT_TOL_ORI_RAD,
                 max_step_rad: Optional[np.ndarray] = None) -> None:
        self.sm = sm
        self.pin = sm.pin
        self.weights = weights
        self.max_iters = int(max_iters)
        self.tol_pos_m = float(tol_pos_m)
        self.tol_ori_rad = float(tol_ori_rad)
        # No bound unless given; step_limit() is how the CLI builds it.
        self.max_step_rad = (np.full(NUM_ARM_JOINTS, np.inf) if max_step_rad is None
                             else np.asarray(max_step_rad, dtype=float))
        if self.max_step_rad.shape != (NUM_ARM_JOINTS,):
            raise ValueError(f"max_step_rad must be ({NUM_ARM_JOINTS},), "
                             f"got {self.max_step_rad.shape}")

    # -- kinematics --------------------------------------------------------

    def _prepare(self, arm: np.ndarray, hand: Dict[str, np.ndarray]) -> np.ndarray:
        """Place frames and joint Jacobians for this arm configuration."""
        q = self.sm.configuration(arm_all=arm, hand=hand)
        self.pin.forwardKinematics(self.sm.model, self.sm.data, q)
        self.pin.computeJointJacobians(self.sm.model, self.sm.data, q)
        self.pin.updateFramePlacements(self.sm.model, self.sm.data)
        return q

    def point_jacobian(self, joint_id: int, point_world: np.ndarray) -> np.ndarray:
        """(3, 14): world velocity of a point rigidly attached to a joint.

        A point offset r from the joint origin moves at v_o + w x r, and
        w x r = -skew(r) w. Only the 14 arm columns are kept, since every
        other joint is locked or passes through.
        """
        J = self.pin.getJointJacobian(self.sm.model, self.sm.data, joint_id,
                                      self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        origin = self.sm.data.oMi[joint_id].translation
        r = np.asarray(point_world, dtype=float) - origin
        skew = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        return (J[:3, :] - skew @ J[3:, :])[:, self.sm.arm_q_idx_all]

    def _wrist_task(self, side: str, target, free: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """6 rows: the placement error of this side's wrist frame, LOCAL."""
        fid = self.sm.wrist_frame[side]
        error = self.pin.log6(self.sm.data.oMf[fid].actInv(target.wrist)).vector
        J = np.zeros((6, NUM_ARM_JOINTS))
        J[:, side_columns(side)] = self.pin.getFrameJacobian(
            self.sm.model, self.sm.data, fid,
            self.pin.ReferenceFrame.LOCAL)[:, self.sm.arm_q_idx[side]]
        return J[:, free], error

    def _elbow_task(self, side: str, target, free: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """3 rows: the position error of this side's elbow joint centre."""
        eid = self.sm.elbow_frame[side]
        error = target.elbow - self.sm.data.oMf[eid].translation
        J = np.zeros((3, NUM_ARM_JOINTS))
        J[:, side_columns(side)] = self.pin.getFrameJacobian(
            self.sm.model, self.sm.data, eid,
            self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, self.sm.arm_q_idx[side]]
        return J[:, free], error

    # -- one stage ---------------------------------------------------------

    def _stage(self, arm: np.ndarray, *,
               hand: Dict[str, np.ndarray],
               targets: Dict[str, ArmTargets],
               free: np.ndarray,
               seed: np.ndarray,
               lower: np.ndarray,
               upper: np.ndarray,
               use_wrist: Dict[str, bool],
               use_elbow: Dict[str, bool],
               separation: Optional[Callable[[np.ndarray], Sequence]] = None,
               separation_weight: float = 0.0,
               separation_refresh: bool = False,
               elbow_weight: float = 1.0,
               max_iters: Optional[int] = None) -> Tuple[np.ndarray, int]:
        """Damped least squares on the free columns only.

        free is what this stage may move; every other joint is held where it
        came in. That is what makes the staging a hierarchy: stage 2 cannot
        undo stage 1, because those columns are not in its free set.
        """
        arm = arm.copy()
        n = free.size
        eye = np.eye(n)
        budget = self.max_iters if max_iters is None else int(max_iters)
        iterations = 0
        rows: Optional[List[Tuple[np.ndarray, float]]] = None
        anchor = arm.copy()

        for iterations in range(1, budget + 1):
            q = self._prepare(arm, hand)
            blocks: List[np.ndarray] = []
            residuals: List[np.ndarray] = []
            worst_pos = 0.0
            worst_ori = 0.0

            for side in SIDES:
                if use_wrist.get(side, False):
                    J, error = self._wrist_task(side, targets[side], free)
                    blocks.append(J)
                    residuals.append(error)
                    worst_pos = max(worst_pos, float(np.linalg.norm(error[:3])))
                    worst_ori = max(worst_ori, float(np.linalg.norm(error[3:])))
                if use_elbow.get(side, False):
                    J, error = self._elbow_task(side, targets[side], free)
                    blocks.append(elbow_weight * J)
                    residuals.append(elbow_weight * error)
                    worst_pos = max(worst_pos, float(np.linalg.norm(error)))

            if self.weights.continuity > 0.0:
                blocks.append(self.weights.continuity * eye)
                residuals.append(self.weights.continuity * (seed[free] - arm[free]))

            if separation is not None and separation_weight > 0.0:
                # Refreshed rows follow the arm; a reused row nets off motion since.
                if rows is None or separation_refresh:
                    rows = [(r.jacobian_row(self), r.required_gain_m)
                            for r in separation(q)]
                    anchor = arm.copy()
                for row, gain in rows:
                    remaining = gain - float(row @ (arm - anchor))
                    blocks.append(separation_weight * row[free].reshape(1, -1))
                    residuals.append(separation_weight * np.array([remaining]))

            A = np.vstack(blocks)
            b = np.concatenate(residuals)
            step = np.linalg.solve(A.T @ A + self.weights.damping * eye, A.T @ b)
            before = arm[free].copy()
            arm[free] = np.clip(before + step, lower[free], upper[free])

            if worst_pos <= self.tol_pos_m and worst_ori <= self.tol_ori_rad:
                break
            if float(np.abs(arm[free] - before).max()) < STEP_EPS_RAD:
                break

        return arm, iterations

    # -- the frame ---------------------------------------------------------

    def _box(self, previous: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The feasible box: URDF limits intersected with the step bound."""
        lower = np.maximum(self.sm.arm_lower, previous - self.max_step_rad)
        upper = np.minimum(self.sm.arm_upper, previous + self.max_step_rad)
        return lower, np.maximum(upper, lower)

    def _measure(self, arm: np.ndarray, hand: Dict[str, np.ndarray],
                 targets: Dict[str, ArmTargets], result: IKResult) -> None:
        """Fill in the residuals actually achieved at these angles."""
        self._prepare(arm, hand)
        for side in SIDES:
            error = self.pin.log6(
                self.sm.data.oMf[self.sm.wrist_frame[side]].actInv(targets[side].wrist)).vector
            result.pos_err_m[side] = float(np.linalg.norm(error[:3]))
            result.ori_err_rad[side] = float(np.linalg.norm(error[3:]))
            result.elbow_err_m[side] = float(np.linalg.norm(
                targets[side].elbow - self.sm.data.oMf[self.sm.elbow_frame[side]].translation))
        result.converged = (result.max_pos_err_m <= self.tol_pos_m
                            and result.max_ori_err_rad <= self.tol_ori_rad)

    def solve(self,
              targets: Dict[str, ArmTargets],
              seed: np.ndarray,
              hand: Dict[str, np.ndarray],
              elbow_enabled: Optional[Dict[str, bool]] = None,
              previous: Optional[np.ndarray] = None,
              separation: Optional[Callable[[np.ndarray], Sequence]] = None,
              separation_iters: Optional[int] = None,
              separation_budget_rad: Optional[np.ndarray] = None) -> IKResult:
        """All three stages for one frame.

        seed starts the iteration and bounds the step. elbow_enabled False
        means a flipped span. separation returns this frame's rows, re-called
        every stage-3 iteration; with rows, stage 3 runs twice.
        """
        elbow_enabled = {s: True for s in SIDES} if elbow_enabled is None else elbow_enabled
        seed = np.asarray(seed, dtype=float).copy()
        previous = seed if previous is None else np.asarray(previous, dtype=float)
        lower, upper = self._box(previous)
        arm = np.clip(seed, lower, upper)
        result = IKResult(arm=arm.copy())

        trusted = [s for s in SIDES if elbow_enabled.get(s, True)]

        # Stage 1: shoulder pitch and roll place the elbow, ungated sides only.
        if trusted:
            free = np.concatenate([side_columns(s, SHOULDER_ELBOW_LOCAL) for s in trusted])
            arm, iterations = self._stage(
                arm, hand=hand, targets=targets, free=free, seed=seed,
                lower=lower, upper=upper,
                use_wrist={s: False for s in SIDES},
                use_elbow={s: elbow_enabled.get(s, True) for s in SIDES})
            result.iterations["shoulder_to_elbow"] = iterations

        # Stage 2: the wrist pose, on what stage 1 left (all seven if skipped).
        free = np.concatenate([
            side_columns(s, WRIST_CHAIN_LOCAL if elbow_enabled.get(s, True) else ALL_LOCAL)
            for s in SIDES])
        arm, iterations = self._stage(
            arm, hand=hand, targets=targets, free=free, seed=seed,
            lower=lower, upper=upper,
            use_wrist={s: True for s in SIDES},
            use_elbow={s: False for s in SIDES})
        result.iterations["elbow_to_wrist"] = iterations

        # Stage 3: both tasks, all seven; five joints cannot close a 6-DoF pose.
        free = np.concatenate([side_columns(s, ALL_LOCAL) for s in SIDES])
        staged = arm.copy()
        use_wrist = {s: True for s in SIDES}
        use_elbow = {s: elbow_enabled.get(s, True) for s in SIDES}
        arm, iterations = self._stage(
            arm, hand=hand, targets=targets, free=free, seed=seed,
            lower=lower, upper=upper, use_wrist=use_wrist, use_elbow=use_elbow,
            elbow_weight=self.weights.refine_elbow)
        result.iterations["refine"] = iterations

        # Clearance: same stage and start, plus rows, inside the trust region.
        if separation is not None:
            budget = (np.full(NUM_ARM_JOINTS, math.radians(DEFAULT_AVOID_BUDGET_DEG))
                      if separation_budget_rad is None
                      else np.asarray(separation_budget_rad, dtype=float))
            near_lower = np.maximum(lower, arm - budget)
            near_upper = np.maximum(np.minimum(upper, arm + budget), near_lower)
            arm, iterations = self._stage(
                staged, hand=hand, targets=targets, free=free, seed=seed,
                lower=near_lower, upper=near_upper,
                use_wrist=use_wrist, use_elbow=use_elbow,
                elbow_weight=self.weights.refine_elbow,
                separation=separation,
                separation_weight=self.weights.separation,
                separation_refresh=True,
                max_iters=separation_iters)
            result.iterations["clearance"] = iterations
            result.constrained = True

        result.arm = arm
        self._measure(arm, hand, targets, result)
        self._bounds(result, previous)
        return result

    def _bounds(self, result: IKResult, previous: np.ndarray) -> None:
        """Record which joints came back sitting on one of their bounds."""
        tol = 1e-9
        result.at_limit = ((result.arm <= self.sm.arm_lower + tol)
                           | (result.arm >= self.sm.arm_upper - tol))
        result.at_step_bound = np.abs(result.arm - previous) >= self.max_step_rad - tol


def step_limit(sm: SanitizeModel, rate_hz: float,
               max_step_deg: float = DEFAULT_MAX_STEP_DEG) -> np.ndarray:
    """(14,) per-frame step bound: the URDF's velocity limit, and any cap.

    The limits are the G1's own joint speeds (37 rad/s shoulder and elbow, 22
    wrist), which at 50 Hz allow 42 and 25 deg per frame. --max-step-deg is
    intersected with that, and is off by default.
    """
    from_velocity = sm.arm_velocity / float(rate_hz)
    if not math.isfinite(max_step_deg):
        return from_velocity
    return np.minimum(np.full(NUM_ARM_JOINTS, math.radians(max_step_deg)), from_velocity)
