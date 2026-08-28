"""Arm replay safety chain (spec_1 component 3, Layer 1).

Pure numpy, no ROS imports: unit-testable outside the container. The node
composes these pieces in the joint_replay control loop; the per-joint DDS
write-thread clip reuses rate_limit_step with the ceiling velocities.

Every failure response here is hold, never zero (TUITION.md section 8):
stale input means hold the last command, divergence raises a fault and the
caller keeps commanding the last safe value. This module decides and
reports; the device state machine owns the responses.

Limits come from config/g1_deploy_limits.yaml, which carries two kinds of
row: sourced hardware ceilings (asserted always) and provisional deploy
caps (replaced in Stage A). Position clamps use the ceilings with a margin;
the control-loop rate limit uses the deploy velocity; the DDS clip uses the
ceiling velocity.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml


class LimitsError(ValueError):
    """The limits file is missing, malformed, or lacks a required joint."""


@dataclass
class ArmLimits:
    """Per-joint limits, ordered to match a caller-supplied joint-name list.

    All arrays are float64 with one entry per joint in ``names`` order, so a
    G1_23 caller gets 10-element arrays and a G1_29 caller gets 14. Waist
    rows exist in the file but are only selected if named.
    """

    names: list
    pos_lower: np.ndarray
    pos_upper: np.ndarray
    vel_ceiling: np.ndarray
    effort_ceiling: np.ndarray
    deploy_velocity: np.ndarray
    deploy_acceleration: np.ndarray
    source_path: str

    @classmethod
    def from_yaml(cls, path, joint_names: Sequence[str]) -> 'ArmLimits':
        path = Path(path)
        if not path.exists():
            raise LimitsError(f"limits file not found: {path}")
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        ceilings = raw.get('hardware_ceilings')
        deploy = raw.get('deploy')
        if not ceilings or not deploy:
            raise LimitsError(
                f"{path} must carry both 'hardware_ceilings' and 'deploy' blocks"
            )
        missing = [n for n in joint_names if n not in ceilings]
        if missing:
            raise LimitsError(
                f"{path} has no hardware_ceilings row for {missing}; refusing "
                "to guess -- every commanded joint needs a sourced ceiling"
            )

        n = len(joint_names)
        pos_lower = np.empty(n)
        pos_upper = np.empty(n)
        vel_ceiling = np.empty(n)
        effort_ceiling = np.empty(n)
        for i, name in enumerate(joint_names):
            row = ceilings[name]
            try:
                lo, hi = float(row['position'][0]), float(row['position'][1])
                vel = float(row['velocity'])
                eff = float(row['effort'])
            except (KeyError, TypeError, IndexError) as exc:
                raise LimitsError(f"{path}: malformed ceiling row for {name}") from exc
            if not (lo < hi):
                raise LimitsError(f"{path}: {name} position bounds inverted [{lo}, {hi}]")
            if vel <= 0 or eff <= 0:
                raise LimitsError(f"{path}: {name} velocity/effort must be > 0")
            pos_lower[i], pos_upper[i] = lo, hi
            vel_ceiling[i], effort_ceiling[i] = vel, eff

        dep_vel = float(deploy['velocity'])
        dep_acc = float(deploy['acceleration'])
        if dep_vel <= 0 or dep_acc <= 0:
            raise LimitsError(f"{path}: deploy velocity/acceleration must be > 0")
        deploy_velocity = np.full(n, dep_vel)
        deploy_acceleration = np.full(n, dep_acc)
        for name, row in (deploy.get('per_joint') or {}).items():
            if name not in joint_names:
                continue
            i = list(joint_names).index(name)
            if 'velocity' in row:
                deploy_velocity[i] = float(row['velocity'])
            if 'acceleration' in row:
                deploy_acceleration[i] = float(row['acceleration'])
        # The deploy cap is our choice, but it may never exceed the sourced
        # ceiling: a per_joint override above the ceiling is a config error.
        over = deploy_velocity > vel_ceiling
        if np.any(over):
            bad = [joint_names[i] for i in np.flatnonzero(over)]
            raise LimitsError(
                f"{path}: deploy velocity exceeds the hardware ceiling for {bad}"
            )

        return cls(
            names=list(joint_names),
            pos_lower=pos_lower,
            pos_upper=pos_upper,
            vel_ceiling=vel_ceiling,
            effort_ceiling=effort_ceiling,
            deploy_velocity=deploy_velocity,
            deploy_acceleration=deploy_acceleration,
            source_path=str(path),
        )


class PositionClamp:
    """Clamp targets to [lower + margin, upper - margin], per joint."""

    def __init__(self, lower: np.ndarray, upper: np.ndarray, margin: float = 0.0):
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if margin < 0:
            raise ValueError(f"margin must be >= 0, got {margin}")
        eff_lower = lower + margin
        eff_upper = upper - margin
        if np.any(eff_lower >= eff_upper):
            bad = np.flatnonzero(eff_lower >= eff_upper).tolist()
            raise ValueError(
                f"margin {margin} inverts the bounds of joint indices {bad}"
            )
        self.lower = eff_lower
        self.upper = eff_upper

    def apply(self, q: np.ndarray):
        """Return (clamped q, per-joint bool mask of violations)."""
        q = np.asarray(q, dtype=float)
        clamped = np.clip(q, self.lower, self.upper)
        return clamped, clamped != q


def rate_limit_step(
    q_from: np.ndarray,
    q_to: np.ndarray,
    vel_limits: np.ndarray,
    dt: float,
) -> np.ndarray:
    """One tick of per-joint rate limiting with uniform scaling.

    The whole step shrinks by the single factor that brings the fastest
    joint inside its limit, so the commanded path keeps its direction
    (spec_1: "uniform scaling, preserves path direction"). This is the same
    mechanism as the existing DDS clip, made per-joint.
    """
    q_from = np.asarray(q_from, dtype=float)
    q_to = np.asarray(q_to, dtype=float)
    delta = q_to - q_from
    max_step = np.asarray(vel_limits, dtype=float) * float(dt)
    with np.errstate(divide='ignore', invalid='ignore'):
        ratios = np.abs(delta) / max_step
    scale = float(np.max(ratios)) if ratios.size else 0.0
    if not np.isfinite(scale):
        raise ValueError("rate_limit_step: non-finite target or zero limit")
    if scale <= 1.0:
        return q_to.copy()
    return q_from + delta / scale


class StalenessTracker:
    """Track input freshness; stale means the caller holds the last command.

    mark(t) on every accepted input; is_stale(now) in every control tick.
    Never seen an input counts as stale (nothing to track yet -- the caller
    holds whatever it is currently commanding).
    """

    def __init__(self, timeout_s: float):
        if timeout_s <= 0:
            raise ValueError(f"timeout_s must be > 0, got {timeout_s}")
        self.timeout_s = float(timeout_s)
        self._last_input: Optional[float] = None
        self.stale_episodes = 0
        self._was_stale = True

    def mark(self, t: float) -> None:
        self._last_input = float(t)

    def age(self, now: float) -> Optional[float]:
        if self._last_input is None:
            return None
        return float(now) - self._last_input

    def is_stale(self, now: float) -> bool:
        age = self.age(now)
        stale = age is None or age > self.timeout_s
        if stale and not self._was_stale:
            self.stale_episodes += 1
        self._was_stale = stale
        return stale


class DivergenceMonitor:
    """Fault when |measured - commanded| exceeds a threshold for M ticks.

    The count requires *consecutive* over-threshold ticks so a single noisy
    lowstate frame cannot fault a run. The fault latches; only reset()
    (operator clear-fault) releases it.
    """

    def __init__(self, threshold_rad: float, m_consecutive: int):
        if threshold_rad <= 0 or m_consecutive < 1:
            raise ValueError(
                f"need threshold_rad > 0 and m_consecutive >= 1, got "
                f"{threshold_rad}, {m_consecutive}"
            )
        self.threshold_rad = float(threshold_rad)
        self.m_consecutive = int(m_consecutive)
        self._count = 0
        self.faulted = False
        self.worst_joint: Optional[int] = None
        self.worst_error = 0.0

    def update(self, measured: np.ndarray, commanded: np.ndarray) -> bool:
        if self.faulted:
            return True
        err = np.abs(np.asarray(measured, dtype=float) - np.asarray(commanded, dtype=float))
        worst = int(np.argmax(err))
        if err[worst] > self.worst_error:
            self.worst_error = float(err[worst])
            self.worst_joint = worst
        if err[worst] > self.threshold_rad:
            self._count += 1
            if self._count >= self.m_consecutive:
                self.faulted = True
        else:
            self._count = 0
        return self.faulted

    def reset(self) -> None:
        self._count = 0
        self.faulted = False
        self.worst_joint = None
        self.worst_error = 0.0


@dataclass
class SafetyResult:
    cmd: np.ndarray            # what to command this tick
    stale: bool                # input stale -> cmd is a hold
    clamped: np.ndarray        # per-joint bool, position clamp acted
    rate_limited: bool         # the step was shrunk
    divergence_fault: bool     # latched divergence fault


class ReplaySafetyChain:
    """Composition used by the joint_replay control loop, one per side.

    Order per tick: staleness (stale -> hold last command), position clamp
    (ceilings with margin), per-joint rate limit from the last command
    (deploy velocities), divergence check against measured. The chain never
    zeroes and never invents a target: with no input yet, cmd is the
    caller's last command unchanged.
    """

    def __init__(
        self,
        limits: ArmLimits,
        control_dt: float,
        position_margin: float = 0.0,
        staleness_timeout_s: float = 0.5,
        divergence_threshold_rad: float = 0.35,
        divergence_ticks: int = 10,
    ):
        if control_dt <= 0:
            raise ValueError(f"control_dt must be > 0, got {control_dt}")
        self.limits = limits
        self.control_dt = float(control_dt)
        self.position_clamp = PositionClamp(
            limits.pos_lower, limits.pos_upper, margin=position_margin
        )
        self.staleness = StalenessTracker(staleness_timeout_s)
        self.divergence = DivergenceMonitor(divergence_threshold_rad, divergence_ticks)

    def mark_input(self, t: float) -> None:
        self.staleness.mark(t)

    def process(
        self,
        now: float,
        target_q: Optional[np.ndarray],
        last_cmd_q: np.ndarray,
        measured_q: Optional[np.ndarray] = None,
    ) -> SafetyResult:
        last_cmd_q = np.asarray(last_cmd_q, dtype=float)
        n = last_cmd_q.shape[0]
        stale = self.staleness.is_stale(now) or target_q is None

        if stale:
            cmd = last_cmd_q.copy()
            clamped = np.zeros(n, dtype=bool)
            rate_limited = False
        else:
            clamped_target, clamped = self.position_clamp.apply(target_q)
            cmd = rate_limit_step(
                last_cmd_q, clamped_target, self.limits.deploy_velocity, self.control_dt
            )
            rate_limited = bool(np.any(np.abs(cmd - clamped_target) > 1e-12))

        fault = self.divergence.faulted
        if measured_q is not None:
            fault = self.divergence.update(measured_q, last_cmd_q)

        return SafetyResult(
            cmd=cmd,
            stale=stale,
            clamped=clamped,
            rate_limited=rate_limited,
            divergence_fault=fault,
        )
