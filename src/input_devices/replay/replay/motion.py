"""Clip-time interpolation and the approach-ramp curve. Pure numpy, no ROS.

The publisher used to emit one clip frame per timer tick. At ``--speed 0.25``
that is 12.5 Hz, a staircase the hand driver slews toward. These helpers turn
that into a continuous command:

- ``lerp_clip`` blends adjacent clip frames at a fractional index.
- ``quintic_blend`` is a rest-capable ramp (zero acceleration at both ends).
  Rest-to-rest is the standard min-jerk 10u^3-15u^4+6u^5. Matching the clip's
  first-frame velocity at u=1 keeps the join C1 so the ramp is not a second
  discontinuity.
- ``named_positions`` reads a JointState-shaped name/position pair into a
  clip column order, filling missing names from a fallback (frame 0).
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

Array = np.ndarray


def clip_unit(u: float) -> float:
    """Clamp ``u`` into [0, 1]."""
    return min(1.0, max(0.0, float(u)))


def lerp_clip(q: Array, frame_f: float) -> Array:
    """Linear blend of clip frames at fractional index ``frame_f``.

    ``q`` is ``(T, n)``. Indices past the last frame hold the last row.
    A single-frame clip returns that row.
    """
    q = np.asarray(q, dtype=np.float64)
    last = q.shape[0] - 1
    if last <= 0:
        return np.array(q[0], dtype=np.float64, copy=True)
    f = min(max(float(frame_f), 0.0), float(last))
    i0 = int(f)
    if i0 >= last:
        return np.array(q[last], dtype=np.float64, copy=True)
    alpha = f - i0
    if alpha == 0.0:
        return np.array(q[i0], dtype=np.float64, copy=True)
    return (1.0 - alpha) * q[i0] + alpha * q[i0 + 1]


def quintic_blend(q0: Array, qd0: Array, q1: Array, qd1: Array, u: float) -> Array:
    """Quintic Hermite from ``(q0, qd0)`` to ``(q1, qd1)`` at unit time ``u``.

    ``qd0`` / ``qd1`` are derivatives with respect to ``u`` (scale a rad/s
    velocity by the ramp duration before calling). End accelerations are
    zero. Rest-to-rest (``qd0 = qd1 = 0``) is min-jerk.
    """
    u = clip_unit(u)
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    qd0 = np.asarray(qd0, dtype=np.float64)
    qd1 = np.asarray(qd1, dtype=np.float64)
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    # Standard Hermite quintic, qdd(0) = qdd(1) = 0.
    h00 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5
    h10 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5
    h01 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    h11 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5
    return h00 * q0 + h10 * qd0 + h01 * q1 + h11 * qd1


def min_jerk_u(u: float) -> float:
    """Scalar rest-to-rest min-jerk weight on [0, 1]."""
    u = clip_unit(u)
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def named_positions(
    wanted: Sequence[str],
    names: Sequence[str],
    positions: Sequence[float],
    fallback: Array,
) -> Array:
    """Map a named JointState onto ``wanted`` order; missing names use ``fallback``."""
    fallback = np.asarray(fallback, dtype=np.float64)
    lookup = {str(n): float(p) for n, p in zip(names, positions)}
    out = np.array(fallback, dtype=np.float64, copy=True)
    for i, name in enumerate(wanted):
        if name in lookup:
            out[i] = lookup[name]
    return out


def clip_start_velocity(q: Array, rate_hz: float, speed: float) -> Array:
    """rad/s at frame 0, from the first clip step. Zero for a single-frame clip."""
    q = np.asarray(q, dtype=np.float64)
    if q.shape[0] < 2:
        return np.zeros(q.shape[1], dtype=np.float64)
    return (q[1] - q[0]) * float(rate_hz) * float(speed)
