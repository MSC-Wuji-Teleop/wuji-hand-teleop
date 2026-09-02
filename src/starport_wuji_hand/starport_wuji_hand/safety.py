"""The guard chain: everything that stands between a published command and the hardware.

Wuji's own deployment stack clamps position in its env layer, above the driver. That works for a
single-consumer Python script and fails in ROS, where anything can publish to a command topic and
skip it. So the clamp lives HERE, in the one place every publisher must traverse.

The firmware clamps out-of-range targets on its own regardless. This guard is not what makes the
hand safe from an over-range command -- it is what makes such a command VISIBLE, by reporting the
truncation instead of letting it happen silently.

Pure module: no ROS, no wujihandpy.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from .joint_map import NUM_JOINTS

# Fallback tick used whenever dt is not a usable duration -- non-positive, NaN or infinite (all of
# them clock glitches). Small enough that a stalled clock cannot buy meaningful travel, large
# enough that motion does not deadlock.
_NOMINAL_DT = 0.01

# How far apart two numbers must be before the difference counts as guard ACTIVITY rather than
# float dust, as a multiple of the machine epsilon scaled by the magnitudes being compared. A
# publisher whose per-frame step is arithmetically equal to its budget still lands a few units in
# the last place above it -- 0.4 * 6 / 20 - 0.4 * 5 / 20 is 0.020000000000000018 against a budget
# of exactly 0.02 -- and calling that a truncation turned a signal check into a guard check on 115
# of 200 frames. Taken from the quantities themselves rather than fixed in radians, so it tracks
# each joint's own scale: the multiplier is 1.8e-15, which at this hand's widest bound (2.094 rad)
# makes the tolerance 3.7e-15 rad, and at the 0.02 rad scale of one nominal tick's budget makes it
# 3.6e-17 rad. Both are orders of magnitude below anything an operator or an encoder could see.
# Eight units in the last place is deliberately a handful of ulps of the operands and nothing more:
# it absorbs the rounding of an arithmetically-equal step, not a small real motion, and it is not
# sized against any particular publisher's dust.
_ACTIVITY_EPS = 8.0 * np.finfo(np.float64).eps


class ConfigurationFault(ValueError):
    """A disagreement between the hardware and its configuration that no retry can change.

    Its own type so a caller can tell it apart from every other bad value, and NARROW on purpose --
    the part worth stating here, because the callers only point back at it. A malformed read and an
    unexpected array shape are ``ValueError`` too, and a driver that latches on those answers a link
    glitch with "refusing to run", sending a bench engineer after a limits mismatch that does not
    exist. A ``ValueError`` subclass because it is still a bad value, so anything catching the
    general case keeps seeing it.
    """


def _truncated(before: np.ndarray, after: np.ndarray, *magnitudes: np.ndarray) -> np.ndarray:
    """Per-joint mask of ``before`` -> ``after`` differences too large to be rounding dust.

    ``magnitudes`` are the quantities whose own rounding produced the difference, and default to
    the two being compared. They matter because the dust is in the OPERANDS rather than in the
    difference: a delta of 0.02 computed from two values near 0.3 can sit several units in the last
    place of 0.02 from the exact answer, so a tolerance scaled by the difference alone would be too
    tight to absorb it.

    This only decides what gets REPORTED: the clamp and the rate limit themselves are unchanged, so
    a joint truncated by dust is still truncated, just not called out for it.
    """
    scale = np.zeros_like(np.asarray(before, dtype=np.float64))
    for magnitude in magnitudes or (before, after):
        scale = np.maximum(scale, np.abs(magnitude))
    return np.abs(after - before) > _ACTIVITY_EPS * scale


@dataclass(frozen=True)
class Limits:
    """Per-joint soft position limits, margin already applied.

    ``raw_lower``/``raw_upper`` keep the pre-margin bounds so ``cross_check`` compares hardware
    against the *declared* envelope rather than against our shrunken one -- otherwise the margin
    itself would read as a disagreement.
    """

    lower: np.ndarray
    upper: np.ndarray
    raw_lower: np.ndarray
    raw_upper: np.ndarray
    # The joint names these vectors are indexed by, in hardware order. Held so a Limits built for
    # one hand cannot report the other hand's joint in a failure message.
    names: tuple[str, ...]

    @classmethod
    def from_mapping(cls, raw: Mapping[str, tuple[float, float]], margin: float, names: Sequence[str]) -> "Limits":
        names = tuple(names)
        if len(names) != NUM_JOINTS:
            raise ValueError(f"names must list {NUM_JOINTS} joints, got {len(names)}")
        unknown = sorted(set(raw) - set(names))
        if unknown:
            raise ValueError(f"unknown joint(s) in limits: {unknown}")
        missing = [name for name in names if name not in raw]
        if missing:
            raise ValueError(f"limits missing joint(s): {missing}")
        margin = float(margin)
        if not np.isfinite(margin):
            raise ValueError(f"margin must be finite, got {margin}")
        # A negative margin would WIDEN the soft limits past the declared envelope, so an
        # out-of-envelope command would pass with clamped=False and the truncation the firmware
        # then performs would be exactly the invisible one this module exists to expose.
        if margin < 0.0:
            raise ValueError(f"margin must be non-negative, got {margin}")

        raw_lower = np.array([float(raw[name][0]) for name in names], dtype=np.float64)
        raw_upper = np.array([float(raw[name][1]) for name in names], dtype=np.float64)

        # A non-finite bound passes every ordering check below -- NaN compares false against
        # everything -- and then turns every clamped command into NaN. Reject it here instead.
        unusable = np.flatnonzero(~np.isfinite(raw_lower) | ~np.isfinite(raw_upper))
        if unusable.size:
            raise ValueError(f"non-finite bound(s) for joint(s): {[names[i] for i in unusable]}")

        inverted = [names[i] for i in np.flatnonzero(raw_lower >= raw_upper)]
        if inverted:
            raise ValueError(f"lower >= upper for joint(s): {inverted}")

        lower = raw_lower + margin
        upper = raw_upper - margin
        collapsed = [names[i] for i in np.flatnonzero(lower >= upper)]
        if collapsed:
            raise ValueError(f"margin {margin} collapses the range of joint(s): {collapsed}")
        return cls(lower=lower, upper=upper, raw_lower=raw_lower, raw_upper=raw_upper, names=names)

    def cross_check(self, hw_lower: np.ndarray, hw_upper: np.ndarray, tol: float = 1e-4) -> None:
        """Compare hardware-reported limits against the declared envelope this was built from.

        A disagreement means the hand is not held to the envelope the caller believes in. That is a
        fact to surface, never to quietly reconcile -- so this raises with the offending joints
        named, and the node refuses to start.

        The envelope is the pre-margin one, and the CALLER owns what frame it is in: the driver
        builds it in the hand's own frame, so a declared sign/zero correction is already in these
        numbers and they will not all be found in the source table. The message says "declared" for
        that reason -- naming a file the numbers may not appear in would send a bench engineer
        looking for them there.

        A non-finite hardware bound counts as a disagreement rather than sliding through: NaN
        compares false against any tolerance, so it would otherwise read as perfect agreement.

        A disagreement is a ``ConfigurationFault``; a hardware array of the wrong SHAPE is a plain
        ``ValueError``, because a read that did not come back in the hand's own geometry says
        nothing about whether the two envelopes agree.
        """
        hw_lower = np.asarray(hw_lower, dtype=np.float64).reshape(-1)
        hw_upper = np.asarray(hw_upper, dtype=np.float64).reshape(-1)
        if hw_lower.shape != (NUM_JOINTS,) or hw_upper.shape != (NUM_JOINTS,):
            raise ValueError(f"hardware limits must be ({NUM_JOINTS},), got {hw_lower.shape} / {hw_upper.shape}")
        bad = np.flatnonzero(
            (np.abs(hw_lower - self.raw_lower) > tol)
            | (np.abs(hw_upper - self.raw_upper) > tol)
            | ~np.isfinite(hw_lower)
            | ~np.isfinite(hw_upper)
        )
        if bad.size:
            detail = ", ".join(
                f"{self.names[i]}: hardware [{hw_lower[i]:.4f}, {hw_upper[i]:.4f}] "
                f"vs declared [{self.raw_lower[i]:.4f}, {self.raw_upper[i]:.4f}]"
                for i in bad
            )
            raise ConfigurationFault(
                f"hardware joint limits disagree with the declared envelope for {bad.size} joint(s): {detail}"
            )


@dataclass
class GuardReport:
    accepted: bool
    clamped: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=bool))
    rate_limited: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=bool))
    reason: str = ""
    # The setpoint's own velocity, rad/s, POST-slew: the chain is the only thing that knows how
    # far the target was actually allowed to travel this tick. Zero on every hold, because a held
    # target is not moving. Bounded by max_velocity for the same reason the travel is.
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))


class GuardChain:
    """Guards applied in order to every command, whatever published it.

    01 finite check -- NaN/Inf drops the WHOLE message
    02 position clamp -- to soft limits
    03 slew-rate limit -- bounds per-tick travel
    04 command watchdog -- holds the last safe target when the stream stops
    """

    def __init__(
        self,
        limits: Limits,
        max_velocity: np.ndarray,
        timeout_s: float,
        initial: np.ndarray,
    ) -> None:
        initial = np.asarray(initial, dtype=np.float64)
        if initial.shape != (NUM_JOINTS,):
            raise ValueError(f"initial must be ({NUM_JOINTS},), got {initial.shape}")
        # A non-finite seed would make the held target non-finite for the lifetime of the chain,
        # so every later output -- including a rejection that only holds it -- would carry NaN.
        if not np.isfinite(initial).all():
            raise ValueError("initial must be finite")
        self._limits = limits
        self._max_velocity = np.asarray(max_velocity, dtype=np.float64)
        if self._max_velocity.shape != (NUM_JOINTS,):
            raise ValueError(f"max_velocity must be ({NUM_JOINTS},)")
        # A NaN budget makes the rate limit emit NaN, an Inf one makes it a no-op, and a negative
        # one inverts the clip and drives away from the target. None of those may reach an actuator.
        if not np.isfinite(self._max_velocity).all() or np.any(self._max_velocity < 0.0):
            raise ValueError("max_velocity must be finite and non-negative")
        self._timeout_s = float(timeout_s)
        # Every rejected value costs the node its only staleness signal, in one of two directions
        # and with nothing raised to explain either. A NaN timeout compares false forever and an
        # infinite one is never exceeded, so `stale` would never fire; a non-positive one is
        # exceeded by every gap, so it fires from the first command onward and a real dead
        # publisher is indistinguishable from a healthy one.
        if not math.isfinite(self._timeout_s) or self._timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be finite and positive, got {self._timeout_s}")
        # Seed the held target INSIDE the limits, so a bad `initial` cannot leak past the clamp.
        self._last_safe = np.clip(initial, limits.lower, limits.upper)
        self._last_command_at: float | None = None

    @property
    def last_safe(self) -> np.ndarray:
        return self._last_safe.copy()

    def stale(self, now: float) -> bool:
        """True once no command has arrived for longer than the timeout.

        False before the first command ever arrives: startup silence is not a fault, it is
        startup. The node holds the home pose either way.
        """
        if self._last_command_at is None:
            return False
        return (now - self._last_command_at) > self._timeout_s

    def apply(self, target: np.ndarray | None, dt: float, now: float) -> tuple[np.ndarray, GuardReport]:
        """Run every guard over one command and return the target the hardware may be given.

        ``dt`` is the caller's TICK PERIOD -- the measured gap between ticks, capped at a small
        multiple of the nominal period -- and never the wall-clock gap since the last message. The
        travel budget is ``max_velocity * dt``, so an uncapped gap would authorise precisely the
        large jump the rate limit exists to prevent: after a stall, the tick period is still the
        tick period. Measuring it is what keeps the limit honest when the caller's timer runs slow,
        and it is measured rather than assumed in both directions -- the budget tracks elapsed
        time, so a short gap grants proportionally less. The cap is what stops a stall from buying
        travel. ``now`` feeds the staleness signal only, never the budget.

        The returned array is always finite and always inside the soft limits, accepted or not. A
        rejected command yields the held target -- never the offending one, and never a partially
        honored version of it -- and ``last_safe`` advances only on acceptance.

        The report's two per-joint flags mean "this joint was actually truncated", which is a
        judgement about magnitude and not an exact comparison -- see ``_truncated``.
        """
        # ---- guard 04: watchdog. No new command -> hold the last safe target. Holding, not
        # zeroing and not homing: releasing a loaded finger is the unsafe option.
        if target is None:
            reason = "stale command stream: holding last safe target" if self.stale(now) else "no command"
            return self._last_safe.copy(), GuardReport(accepted=False, reason=reason)

        candidate = np.asarray(target, dtype=np.float64)
        if candidate.shape != (NUM_JOINTS,):
            return self._last_safe.copy(), GuardReport(
                accepted=False, reason=f"bad shape {candidate.shape}, expected ({NUM_JOINTS},)"
            )
        # ---- guard 01: finite. Reject the WHOLE message; a partially-valid command is a bug.
        if not np.isfinite(candidate).all():
            return self._last_safe.copy(), GuardReport(accepted=False, reason="command rejected: not finite (NaN/Inf)")

        # ---- guard 02: clamp. Runs BEFORE the rate limit so the travel we permit heads toward a
        # legal pose rather than spending the tick's budget aiming at an illegal one.
        clamped_target = np.clip(candidate, self._limits.lower, self._limits.upper)
        clamped = _truncated(candidate, clamped_target)

        # ---- guard 03: slew rate. An unusable dt means a clock jump; grant one nominal tick rather
        # than permitting a free jump. Infinity has to be excluded explicitly, not just handled by
        # `dt > 0.0`: an infinite budget is a free jump, and `inf * 0.0` -- a joint deliberately
        # locked with a zero max_velocity -- is NaN. Clipping against array bounds is elementwise,
        # so that NaN stays on the locked joint's index alone: a single non-finite setpoint hiding
        # in an otherwise plausible 20-vector, which is harder to notice than a whole bad command.
        effective_dt = dt if math.isfinite(dt) and dt > 0.0 else _NOMINAL_DT
        budget = self._max_velocity * effective_dt
        delta = clamped_target - self._last_safe
        limited_delta = np.clip(delta, -budget, budget)
        rate_limited = _truncated(delta, limited_delta, clamped_target, self._last_safe, budget)
        safe = self._last_safe + limited_delta

        self._last_safe = safe
        self._last_command_at = float(now)
        return safe.copy(), GuardReport(
            accepted=True,
            clamped=clamped,
            rate_limited=rate_limited,
            velocity=limited_delta / effective_dt,
        )
