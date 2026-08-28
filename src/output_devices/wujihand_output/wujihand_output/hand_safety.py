"""Hand q20 safety pieces (spec_1 component 4, Layer 1).

Pure numpy, no ROS imports. Used by the wujihand_controller q20_topic
branch: position clamp and rate limit on every command, staleness-to-hold
for both the target stream and the driver feedback, and an effort guard
fed by the per-joint limits the driver publishes at runtime.

PositionClamp / rate_limit_step / StalenessTracker mirror
g1_world_output/replay_safety.py. The duplication is deliberate: the two
packages run in different containers (different numpy majors) with no
shared package between them (spec_1 keeps safety at the last hop inside
each device node). Keep behavioral changes in sync manually.

Units. Hand positions are radians. The effort guard is CURRENT-space:
joint_states.effort is the driver's filtered actuator output in drive
current (amps), and hand_diagnostics.effort_limits are per-joint current
limits (amps, default 1.5, max 3.5 per wujihandros2 docs). The URDF's N*m
efforts are a different quantity and are never compared against these.

Every failure response is hold, never zero (TUITION.md section 8).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

NUM_JOINTS = 20


class LimitsError(ValueError):
    """The hand limits file is missing, malformed, or incomplete."""


@dataclass
class HandLimits:
    """The 20-joint limit table from config/hand_limits.yaml, flat order.

    Order is URDF declaration order, thumb to pinky, [flex, abd, pip/mcp,
    dip/ip] per finger -- the same flat convention the driver indexes.
    """

    names: list                    # suffix names, no side prefix
    pos_lower: np.ndarray
    pos_upper: np.ndarray
    effort_urdf: np.ndarray        # N*m, reference only
    sim_model_velocity: np.ndarray  # rad/s, URDF values, reference only
    deploy_velocity: np.ndarray    # rad/s, the governing runtime cap
    deploy_acceleration: np.ndarray
    driver_names: list             # finger{1..5}_joint{1..4}
    side_prefix: dict              # {'left': 'l_', 'right': 'r_'}
    source_path: str

    def side_names(self, side: str) -> list:
        prefix = self.side_prefix[side]
        return [prefix + n for n in self.names]

    @classmethod
    def from_yaml(cls, path) -> 'HandLimits':
        path = Path(path)
        if not path.exists():
            raise LimitsError(f"hand limits file not found: {path}")
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        rows = raw.get('joints')
        deploy = raw.get('deploy')
        tables = raw.get('name_tables')
        if not rows or not deploy or not tables:
            raise LimitsError(
                f"{path} must carry 'joints', 'deploy', and 'name_tables'"
            )
        if len(rows) != NUM_JOINTS:
            raise LimitsError(f"{path}: expected {NUM_JOINTS} joint rows, got {len(rows)}")

        names = []
        pos_lower = np.empty(NUM_JOINTS)
        pos_upper = np.empty(NUM_JOINTS)
        effort_urdf = np.empty(NUM_JOINTS)
        sim_vel = np.empty(NUM_JOINTS)
        for i, row in enumerate(rows):
            try:
                if int(row['index']) != i:
                    raise LimitsError(
                        f"{path}: joint rows out of order at position {i} "
                        f"(index says {row['index']})"
                    )
                lo, hi = float(row['position'][0]), float(row['position'][1])
                eff = float(row['effort_urdf'])
                vel = float(row['sim_model_velocity'])
                names.append(str(row['name']))
            except (KeyError, TypeError, IndexError) as exc:
                raise LimitsError(f"{path}: malformed joint row {i}") from exc
            if not (lo < hi) or eff <= 0 or vel <= 0:
                raise LimitsError(f"{path}: bad values in joint row {i} ({row.get('name')})")
            pos_lower[i], pos_upper[i] = lo, hi
            effort_urdf[i], sim_vel[i] = eff, vel

        dep_vel = float(deploy['velocity'])
        dep_acc = float(deploy['acceleration'])
        if dep_vel <= 0 or dep_acc <= 0:
            raise LimitsError(f"{path}: deploy values must be > 0")

        driver_names = list(tables.get('driver') or [])
        side_prefix = dict(tables.get('side_prefix') or {})
        if len(driver_names) != NUM_JOINTS or set(side_prefix) != {'left', 'right'}:
            raise LimitsError(f"{path}: name_tables incomplete")

        return cls(
            names=names,
            pos_lower=pos_lower,
            pos_upper=pos_upper,
            effort_urdf=effort_urdf,
            sim_model_velocity=sim_vel,
            deploy_velocity=np.full(NUM_JOINTS, dep_vel),
            deploy_acceleration=np.full(NUM_JOINTS, dep_acc),
            driver_names=driver_names,
            side_prefix=side_prefix,
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
            raise ValueError(f"margin {margin} inverts the bounds of joint indices {bad}")
        self.lower = eff_lower
        self.upper = eff_upper

    def apply(self, q: np.ndarray):
        q = np.asarray(q, dtype=float)
        clamped = np.clip(q, self.lower, self.upper)
        return clamped, clamped != q


def rate_limit_step(
    q_from: np.ndarray,
    q_to: np.ndarray,
    vel_limits: np.ndarray,
    dt: float,
) -> np.ndarray:
    """One tick of per-joint rate limiting, uniform scaling (direction kept)."""
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
    """Track input freshness; stale means the caller holds the last command."""

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


class EffortGuard:
    """Fault when measured drive current stays over its limit for M ticks.

    Commands to the driver are position-only (the spec's named section-5
    deviation), so the effort clamp cannot act on the command; it acts as a
    Layer-1 monitor that holds and faults. Thresholds come from
    hand_diagnostics.effort_limits (amps) at runtime via set_limits();
    until they arrive the guard is inactive, and the node separately treats
    missing diagnostics as a fault condition.

    A scale factor below 1.0 trips before the driver's own limit.
    """

    def __init__(self, m_consecutive: int, scale: float = 1.0):
        if m_consecutive < 1 or not (0.0 < scale <= 1.0):
            raise ValueError(
                f"need m_consecutive >= 1 and 0 < scale <= 1, got "
                f"{m_consecutive}, {scale}"
            )
        self.m_consecutive = int(m_consecutive)
        self.scale = float(scale)
        self._limits_amps: Optional[np.ndarray] = None
        self._count = 0
        self.faulted = False
        self.worst_joint: Optional[int] = None

    @property
    def active(self) -> bool:
        return self._limits_amps is not None

    def set_limits(self, limits_amps) -> None:
        arr = np.asarray(limits_amps, dtype=float)
        if arr.shape != (NUM_JOINTS,) or np.any(arr <= 0):
            raise ValueError(
                f"effort limits must be {NUM_JOINTS} positive values, got {arr.shape}"
            )
        self._limits_amps = arr * self.scale

    def update(self, measured_effort_amps) -> bool:
        if self.faulted:
            return True
        if self._limits_amps is None:
            return False
        eff = np.abs(np.asarray(measured_effort_amps, dtype=float))
        over = eff > self._limits_amps
        if over.any():
            self._count += 1
            self.worst_joint = int(np.argmax(eff / self._limits_amps))
            if self._count >= self.m_consecutive:
                self.faulted = True
        else:
            self._count = 0
        return self.faulted

    def reset(self) -> None:
        self._count = 0
        self.faulted = False
        self.worst_joint = None
