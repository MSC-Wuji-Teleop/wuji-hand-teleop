"""Joint-target interpolation buffer for one arm in joint_replay mode.

The G1 node polls this buffer at its control rate (250 Hz on the rig) while
a publisher pushes clip frames at a lower rate (50 Hz times the replay
speed). Without interpolation the arm sees a staircase at the publish rate,
and a stiff PD turns every step into a jolt. This module holds the rule
that turns the sample stream into a continuous command.

The rule (spec1.md "Arms", contract decision 7):

- Before any data: nothing to command. ``interpolate`` returns None.
- After ``seed`` (the measured pose at a mode switch) and before the first
  real sample: hold the seed value.
- With one real sample: hold it. The first frame is a step from wherever
  the arm was. The seed does not count as a sample, so the time the node
  sat idle before the first frame never turns into a ramp.
- With two or more real samples: interpolate one publish period behind,
  between the two newest samples, with
  ``alpha = (now - t_next) / (t_next - t_prev)`` clamped to [0, 1].
  The command reaches ``q_next`` exactly when the following sample is due,
  so the output is continuous at any replay speed, with one publish
  period of latency.

Why one period behind: at poll time the newest sample is the only future
we know. Interpolating from the previous sample toward it, over the last
observed period, is the latest continuous command that uses no prediction.

Pure Python, no ROS imports, so the rule runs and is tested on any machine.
"""

from __future__ import annotations

from typing import Optional, Sequence

# Interpolation parameter bounds. alpha is time since the newest sample as a
# fraction of the last publish period. Clamped so a late poll holds q_next and
# an early poll (clock skew) holds q_prev instead of extrapolating past the data.
ALPHA_MIN = 0.0
ALPHA_MAX = 1.0


class SideBuffer:
    """Two newest (arrival time, q) samples for one arm, plus a held seed."""

    def __init__(self) -> None:
        self._seed_q: Optional[list[float]] = None
        self._prev_t: Optional[float] = None
        self._prev_q: Optional[list[float]] = None
        self._next_t: Optional[float] = None
        self._next_q: Optional[list[float]] = None

    def seed(self, t: float, q: Sequence[float]) -> None:
        """Hold ``q`` (a measured pose) and forget every real sample.

        Called at a mode switch so the arm is commanded where it is until
        the first frame arrives. ``t`` keeps the signature parallel to
        ``push`` and is not used: a seed is never an interpolation endpoint.
        """
        self._seed_q = [float(x) for x in q]
        self._prev_t = None
        self._prev_q = None
        self._next_t = None
        self._next_q = None

    def push(self, t: float, q: Sequence[float]) -> None:
        """Record a real sample that arrived at time ``t`` (seconds)."""
        if self._next_q is not None:
            self._prev_t, self._prev_q = self._next_t, self._next_q
        self._next_t = float(t)
        self._next_q = [float(x) for x in q]

    def interpolate(self, now: float) -> Optional[list[float]]:
        """Command for time ``now`` under the rule in the module docstring.

        Returns a new list of floats, or None when nothing was seeded or
        pushed. Fewer than two real samples: the newest value (seed or
        sample). Two samples with the same arrival time: the newest.
        """
        if self._next_q is None:
            return None if self._seed_q is None else list(self._seed_q)
        if self._prev_q is None or self._next_t <= self._prev_t:
            return list(self._next_q)
        alpha = (now - self._next_t) / (self._next_t - self._prev_t)
        alpha = min(max(alpha, ALPHA_MIN), ALPHA_MAX)
        return [p + alpha * (n - p) for p, n in zip(self._prev_q, self._next_q)]
