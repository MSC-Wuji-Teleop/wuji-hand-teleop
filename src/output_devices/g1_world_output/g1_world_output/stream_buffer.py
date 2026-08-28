"""Stamped-target stream buffer: the ZOH fix (spec_1 component 3). ROS-free.

The old _SideBuffer interpolated between the two most recent samples keyed
on ARRIVAL time; under a single-threaded executor the control timer only
runs after the subscription callback, so alpha always clamped to 1 and the
output was a zero-order hold at the publish rate -- exactly the 50 fps
stepping TUITION section 5 forbids.

The fix (spec_1: "interpolate from the currently commanded value toward the
newest sample over one inter-arrival period"): on each new stamped sample,
snapshot the currently commanded value as the ramp base, and ramp linearly
from it toward the new sample over one inter-arrival period. The period is
measured from HEADER stamps (the publisher's deterministic t0 + j*dt_play
timeline), the ramp starts at arrival. Cost: about one period of latency,
irrelevant open-loop. Output is piecewise-linear and converges to each
sample before the next arrives.

Edge semantics (plan amendments D10/A4):
  - no previous stamp, or stamp delta <= 0 (publish_first repeats frame 0
    with... actually advancing stamps; a duplicate or reordered stamp can
    still happen): hold the newest target (alpha = 1). Never divide by an
    unchecked stamp delta.
  - the period is clamped to [1 ms, max_period_s]; a dropped frame doubles
    the demanded ramp velocity, which the downstream safety-chain rate
    limiter clips.
  - seed(q) (mode switch / startup) holds q until a real sample arrives.

Same scheme is mirrored by the hand q20 branch (controller package).
"""

from __future__ import annotations

from typing import Optional

import numpy as np


class StreamBuffer:
    def __init__(self, min_period_s: float = 1e-3, max_period_s: float = 0.5):
        if not (0 < min_period_s < max_period_s):
            raise ValueError(f"bad period bounds [{min_period_s}, {max_period_s}]")
        self.min_period_s = float(min_period_s)
        self.max_period_s = float(max_period_s)
        self._base: Optional[np.ndarray] = None
        self._target: Optional[np.ndarray] = None
        self._arrival: Optional[float] = None
        self._period: Optional[float] = None
        self._prev_stamp: Optional[float] = None

    def seed(self, q) -> None:
        """Hold q (typically the measured pose) until a sample arrives."""
        q = np.asarray(q, dtype=float).copy()
        self._base = q
        self._target = q.copy()
        self._arrival = None
        self._period = None
        self._prev_stamp = None

    def push(self, arrival_t: float, stamp_s: float, q,
             current_cmd=None) -> None:
        """New stamped sample. current_cmd is the value the control loop is
        commanding right now -- the ramp base. None falls back to the
        previous target (startup before any command exists)."""
        q = np.asarray(q, dtype=float).copy()
        if current_cmd is not None:
            base = np.asarray(current_cmd, dtype=float).copy()
        elif self._target is not None:
            base = self._target.copy()
        else:
            base = q.copy()

        if self._prev_stamp is None or stamp_s <= self._prev_stamp:
            period = None  # hold newest (alpha = 1)
        else:
            period = min(max(stamp_s - self._prev_stamp, self.min_period_s),
                         self.max_period_s)

        self._base = base
        self._target = q
        self._arrival = float(arrival_t)
        self._period = period
        self._prev_stamp = float(stamp_s)

    def interpolate(self, now: float) -> Optional[np.ndarray]:
        """Desired stream point at `now`; None until seeded or pushed."""
        if self._target is None:
            return None
        if self._period is None or self._arrival is None:
            return self._target.copy()
        alpha = (now - self._arrival) / self._period
        alpha = min(max(alpha, 0.0), 1.0)
        return self._base + alpha * (self._target - self._base)

    @property
    def has_data(self) -> bool:
        return self._target is not None
