"""Angle continuity: 2pi wraps, accumulated drift, and branch-flip detection.

Three source defects live here, all of them measured on the joint trajectory
before any IK runs (docs/spec/RoboSTAR_dropin_fix.md):

    wraps   a joint that crosses +-pi and comes back as its wrapped value.
            Removed exactly, because a 2pi shift is the same rotation.
    drift   a slow accumulation in the wrist joints that ends the clip with
            the hands rotated away from where they started -- "inverted by
            the end". Measured between an opening and a closing window and
            capped, never silently removed in full.
    flips   the branch flipping: a single-frame jump in the joint angles that
            leaves the wrist POSE where it was. That signature is what
            separates a solver jumping between arm configurations from a fast
            human motion, and it is why the elbow position task has to be
            dropped for the length of a flipped span -- the flipped source
            elbow is exactly what the solver must not follow.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Constants. Each one says where its value comes from.

TWO_PI = 2.0 * math.pi

# Not human motion: 45 deg per 50 Hz frame is 39 rad/s, past the wrist's 22.
DEFAULT_FLIP_STEP_DEG = 45.0

# The most a genuine flip can move the pose: an exact one moves it not at all.
DEFAULT_FLIP_POSE_POS_M = 0.020
DEFAULT_FLIP_POSE_ORI_DEG = 15.0

# Averaged over this many frames at each end, so one noisy frame invents none.
DEFAULT_DRIFT_WINDOW_FRAMES = 10

# Infinite: capping a real 77.5 deg end pose at 30 rewrote 47 deg of signing.
DEFAULT_MAX_DRIFT_DEG = math.inf

# Worth a line on stderr: half of the spec's "inverted", above a measured 77.5.
DEFAULT_DRIFT_WARN_DEG = 90.0

# Where drift is measured; the spec's defect is wrist rotation.
DRIFT_JOINT_SUFFIXES = ("wrist_roll", "wrist_pitch", "wrist_yaw")


@dataclass
class WrapResult:
    """What happened to each joint's 2pi bookkeeping.

    removed     jumps taken out of the column, per joint
    recentred   whole turns applied to reach the joint's own revolution
    kept        jumps left alone, no unwrapped representation fitting
    """
    q: np.ndarray
    removed: Dict[str, int] = field(default_factory=dict)
    recentred: Dict[str, int] = field(default_factory=dict)
    kept: Dict[str, int] = field(default_factory=dict)

    @property
    def total(self) -> int:
        return int(sum(self.removed.values()))

    def as_dict(self) -> dict:
        return {"removed": self.removed, "recentred": self.recentred,
                "kept_unrepresentable": self.kept}


# Whole turns tried when fitting a column; every range here is under 2pi.
FIT_TURNS = (0, 1, -1, 2, -2)


def _fit_inside(column: np.ndarray, low_limit: float,
                high_limit: float) -> Tuple[Optional[np.ndarray], int]:
    """The column shifted by whole turns so it fits the limits.

    Returns the shifted column and how many turns it took, or (None, 0) when
    no offset fits -- which happens when the motion itself straddles the
    joint's limit, and no way of writing it down can help.
    """
    if float(column.max() - column.min()) > (high_limit - low_limit) + 1e-12:
        return None, 0                   # too wide for the joint at any offset
    for turns in FIT_TURNS:
        shifted = column + turns * TWO_PI
        if shifted.min() >= low_limit - 1e-12 and shifted.max() <= high_limit + 1e-12:
            return shifted, turns
    return None, 0


def unwrap_trajectory(q: np.ndarray, names: Sequence[str],
                      lower: np.ndarray, upper: np.ndarray) -> WrapResult:
    """Remove 2pi jumps, but only where the result is still commandable.

    Every limit here is under 2pi, so np.unwrap can leave a joint out of range
    (3.0 to -3.0 unwraps to 3.28, past a 3.1 limit at every offset). Such a
    column is kept as it came and left to detect_flips.
    """
    q = np.asarray(q, dtype=float)
    out = q.copy()
    removed: Dict[str, int] = {}
    recentred: Dict[str, int] = {}
    kept: Dict[str, int] = {}
    for j, name in enumerate(names):
        unwrapped = np.unwrap(q[:, j])
        jumps = int(np.count_nonzero(
            np.abs(np.diff(q[:, j]) - np.diff(unwrapped)) > 1e-9))
        fitted, turns = _fit_inside(unwrapped, float(lower[j]), float(upper[j]))
        if fitted is None:
            if jumps:
                kept[str(name)] = jumps
            continue
        if not jumps and not turns:
            continue                     # already continuous and in range
        out[:, j] = fitted
        if jumps:
            removed[str(name)] = jumps
        if turns:
            recentred[str(name)] = turns
    return WrapResult(q=out, removed=removed, recentred=recentred, kept=kept)


@dataclass
class DriftResult:
    """Accumulated end-to-start rotation of one joint, and what was removed."""
    joint: str
    measured_deg: float
    removed_deg: float
    residual_deg: float


def measure_drift(column: np.ndarray, window: int = DEFAULT_DRIFT_WINDOW_FRAMES) -> float:
    """End-minus-start angle, in radians, from window-averaged endpoints."""
    column = np.asarray(column, dtype=float)
    n = column.shape[0]
    w = max(1, min(int(window), n // 2 if n >= 2 else 1))
    return float(np.mean(column[-w:]) - np.mean(column[:w]))


def cap_drift(column: np.ndarray, max_drift_rad: float,
              window: int = DEFAULT_DRIFT_WINDOW_FRAMES) -> Tuple[np.ndarray, float]:
    """Remove the part of the accumulated drift above the cap.

    Taken out as a ramp linear in frame index, so it spreads evenly instead
    of stepping. Returns the corrected column and how much was removed.
    """
    column = np.asarray(column, dtype=float)
    drift = measure_drift(column, window)
    if not math.isfinite(max_drift_rad) or abs(drift) <= max_drift_rad:
        return column.copy(), 0.0
    excess = drift - math.copysign(max_drift_rad, drift)
    n = column.shape[0]
    if n < 2:
        return column.copy(), 0.0
    ramp = np.linspace(0.0, 1.0, n)
    return column - excess * ramp, float(excess)


def cap_wrist_drift(q: np.ndarray, names: Sequence[str], side: str,
                    max_drift_rad: float,
                    window: int = DEFAULT_DRIFT_WINDOW_FRAMES) -> Tuple[np.ndarray, List[DriftResult]]:
    """Measure, and cap, drift on this side's wrist joints."""
    out = np.asarray(q, dtype=float).copy()
    results: List[DriftResult] = []
    for j, name in enumerate(names):
        if not any(str(name).endswith(suffix) for suffix in DRIFT_JOINT_SUFFIXES):
            continue
        measured = measure_drift(out[:, j], window)
        corrected, removed = cap_drift(out[:, j], max_drift_rad, window)
        out[:, j] = corrected
        results.append(DriftResult(joint=str(name),
                                   measured_deg=math.degrees(measured),
                                   removed_deg=math.degrees(removed),
                                   residual_deg=math.degrees(measured - removed)))
    return out, results


@dataclass
class Flip:
    """One detected branch flip in the source joint trajectory."""
    side: str
    frame: int
    t_s: float
    joint: str
    step_deg: float
    pose_pos_mm: float
    pose_ori_deg: float


def pose_deltas(poses: Sequence[object]) -> Tuple[np.ndarray, np.ndarray]:
    """Per-frame change of a placement sequence: metres and radians.

    Both arrays are length len(poses) - 1; entry k is the change from frame k
    to frame k + 1.
    """
    import pinocchio as pin

    n = len(poses)
    pos = np.zeros(max(0, n - 1))
    ori = np.zeros(max(0, n - 1))
    for k in range(n - 1):
        pos[k] = float(np.linalg.norm(poses[k + 1].translation - poses[k].translation))
        ori[k] = float(np.linalg.norm(pin.log3(poses[k].rotation.T @ poses[k + 1].rotation)))
    return pos, ori


def detect_flips(q: np.ndarray, names: Sequence[str], side: str, rate_hz: float,
                 pose_pos_delta: np.ndarray, pose_ori_delta: np.ndarray,
                 step_threshold_deg: float = DEFAULT_FLIP_STEP_DEG,
                 pose_pos_tol_m: float = DEFAULT_FLIP_POSE_POS_M,
                 pose_ori_tol_deg: float = DEFAULT_FLIP_POSE_ORI_DEG) -> List[Flip]:
    """Frames where the joints jumped but the wrist pose did not.

    pose_pos_delta and pose_ori_delta are the per-frame wrist pose changes
    from pose_deltas(); entry k describes the step into frame k + 1, which is
    the frame the flip is reported at.
    """
    q = np.asarray(q, dtype=float)
    step = np.abs(np.diff(q, axis=0))
    threshold = math.radians(step_threshold_deg)
    ori_tol = math.radians(pose_ori_tol_deg)
    flips: List[Flip] = []
    for k in range(step.shape[0]):
        j = int(np.argmax(step[k]))
        if step[k, j] < threshold:
            continue
        if pose_pos_delta[k] > pose_pos_tol_m or pose_ori_delta[k] > ori_tol:
            # Joints and pose both moved: fast motion, not a flip.
            continue
        flips.append(Flip(side=side, frame=k + 1, t_s=(k + 1) / float(rate_hz),
                          joint=str(names[j]), step_deg=math.degrees(step[k, j]),
                          pose_pos_mm=1e3 * float(pose_pos_delta[k]),
                          pose_ori_deg=math.degrees(float(pose_ori_delta[k]))))
    return flips


class ElbowGate:
    """Whether the source elbow target may be trusted at this frame.

    A flip throws the source elbow across the arm's null space, so the elbow
    task is dropped from the flip until the source comes back near the elbow
    the solver holds -- the source rejoining the output's branch.
    """

    def __init__(self, rejoin_tol_m: float) -> None:
        self.rejoin_tol_m = float(rejoin_tol_m)
        self.open = True
        self.opened_at: Optional[int] = None
        self.closed_frames = 0

    def close(self, frame: int) -> None:
        if self.open:
            self.open = False
            self.opened_at = frame

    def update(self, frame: int, source_elbow: np.ndarray,
               solved_elbow: Optional[np.ndarray]) -> bool:
        """Advance the gate one frame and return whether the task is on."""
        if not self.open and solved_elbow is not None:
            if float(np.linalg.norm(np.asarray(source_elbow) - np.asarray(solved_elbow))) \
                    <= self.rejoin_tol_m:
                self.open = True
        if not self.open:
            self.closed_frames += 1
        return self.open
