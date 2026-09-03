#!/usr/bin/env python3
"""Tests for the joint_replay interpolation rule (side_buffer.SideBuffer).

No ROS. The package root is put on sys.path so the buffer imports on a
machine without rclpy or an ament install.

The schedule used here is the rig's: a publisher at 50 Hz (a clip at speed
1.0) and the G1 control loop at 250 Hz. Both clocks are built from the same
poll period so a publish instant and a poll instant compare equal in float.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Package root (the directory holding g1_world_output/), so the import below
# works whether pytest is run as `python -m pytest` or as a bare `pytest`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_world_output.side_buffer import SideBuffer  # noqa: E402

# Publisher rate: clip.json rate_hz for the bundle is 50 Hz, played at speed 1.0.
PUBLISH_HZ = 50.0
# Poll rate: the G1 node's control_rate on the rig (replay.sh passes 250.0).
POLL_HZ = 250.0
# 250 / 50. Integer so publishes land exactly on poll instants.
POLLS_PER_PUBLISH = 5
POLL_DT_S = 1.0 / POLL_HZ
PUBLISH_PERIOD_S = 1.0 / PUBLISH_HZ
# 20 publishes = 0.4 s: several interpolation segments, still instant to run.
N_PUBLISHES = 20
# Arbitrary joint velocity for the constant-velocity stream (about 40 deg/s).
VELOCITY_RAD_S = 0.7
# Per-joint offset so a joint mix-up in the buffer would show as a wrong value.
JOINT_OFFSET_RAD = 0.1
# G1_29 arm DoF per side.
N_JOINTS = 7
# Float slack for values built from a few dozen multiplies and adds.
FLOAT_TOL = 1e-9
# Idle gaps between seed() and the first sample. 0 s, one poll, one second,
# one hour: the first frame must step in every case.
IDLE_GAPS_S = (0.0, POLL_DT_S, 1.0, 3600.0)


def _q_at(t: float) -> list[float]:
    """Constant-velocity stream, per-joint offset: q_j(t) = v t + j * offset."""
    return [VELOCITY_RAD_S * t + JOINT_OFFSET_RAD * j for j in range(N_JOINTS)]


def _schedule():
    """Yield (now, sample) per poll. sample is (t, q) when a publish lands on
    this poll instant, else None. A publish is processed before the poll at
    the same instant, as the node's callbacks run before the timer."""
    for n in range(N_PUBLISHES * POLLS_PER_PUBLISH):
        now = n * POLL_DT_S
        sample = (now, _q_at(now)) if n % POLLS_PER_PUBLISH == 0 else None
        yield now, sample


def _old_rule(prev_t, prev_q, next_t, next_q, now):
    """The formula g1_world_output_node._SideBuffer.interpolate used before
    2026-09-02, copied verbatim: alpha measured from the previous sample's
    arrival time toward the newest sample's arrival time."""
    if next_q is None:
        return None
    if prev_q is None or next_t <= prev_t:
        return next_q
    alpha = (now - prev_t) / (next_t - prev_t)
    alpha = min(max(alpha, 0.0), 1.0)
    return [(1.0 - alpha) * p + alpha * n for p, n in zip(prev_q, next_q)]


def test_old_rule_was_a_zero_order_hold():
    """Regression: with the old formula every poll returns the newest sample.

    A poll always happens at or after the newest sample's arrival, so the
    old alpha is at or past 1 at every poll and the clamp pins it to 1.
    """
    prev_t = prev_q = next_t = next_q = None
    distinct_outputs = 0
    last = None
    for now, sample in _schedule():
        if sample is not None:
            t, q = sample
            if next_q is not None:
                prev_t, prev_q = next_t, next_q
            else:
                prev_t, prev_q = t, q
            next_t, next_q = t, q
        out = _old_rule(prev_t, prev_q, next_t, next_q, now)
        assert out == next_q
        if out != last:
            distinct_outputs += 1
            last = out
    assert distinct_outputs == N_PUBLISHES


def test_new_rule_is_linear_one_period_behind():
    """Constant-velocity stream: after the second sample the output equals
    q(now - one publish period), so it rises by the same amount every poll
    and never repeats a value. Before that it holds the first sample."""
    buf = SideBuffer()
    outputs = []
    for now, sample in _schedule():
        if sample is not None:
            buf.push(*sample)
        out = buf.interpolate(now)
        assert isinstance(out, list) and len(out) == N_JOINTS
        outputs.append(out)
        if now >= PUBLISH_PERIOD_S:
            assert out == pytest.approx(_q_at(now - PUBLISH_PERIOD_S), abs=FLOAT_TOL)

    first_q = _q_at(0.0)
    for out in outputs[:POLLS_PER_PUBLISH]:
        assert out == first_q

    tail = np.asarray(outputs[POLLS_PER_PUBLISH:])
    increments = np.diff(tail, axis=0)
    assert np.all(increments > 0.0)
    assert np.allclose(increments, increments[0], atol=FLOAT_TOL)
    assert np.allclose(increments[0], VELOCITY_RAD_S * POLL_DT_S, atol=FLOAT_TOL)


def test_formula_midpoint():
    """alpha = (now - t_next) / (t_next - t_prev): half a period after the
    newest sample the command is the midpoint of the two newest samples."""
    buf = SideBuffer()
    buf.push(0.0, [0.0] * N_JOINTS)
    buf.push(PUBLISH_PERIOD_S, [1.0] * N_JOINTS)
    out = buf.interpolate(PUBLISH_PERIOD_S + 0.5 * PUBLISH_PERIOD_S)
    assert out == pytest.approx([0.5] * N_JOINTS, abs=FLOAT_TOL)


@pytest.mark.parametrize("idle_s", IDLE_GAPS_S)
def test_first_sample_after_seed_is_a_step(idle_s):
    """The seed is held until the first sample, then the first sample is
    held: no ramp from the seed value, however long the node sat idle."""
    seed_q = [0.5] * N_JOINTS
    first_q = [1.5] * N_JOINTS
    buf = SideBuffer()
    buf.seed(0.0, seed_q)
    assert buf.interpolate(0.5 * idle_s) == seed_q

    buf.push(idle_s, first_q)
    for now in (idle_s, idle_s + POLL_DT_S, idle_s + 0.5 * PUBLISH_PERIOD_S,
                idle_s + PUBLISH_PERIOD_S, idle_s + 10.0):
        assert buf.interpolate(now) == first_q

    # The second sample interpolates from the first sample, not from the seed.
    second_q = [2.5] * N_JOINTS
    buf.push(idle_s + PUBLISH_PERIOD_S, second_q)
    mid = buf.interpolate(idle_s + PUBLISH_PERIOD_S + 0.5 * PUBLISH_PERIOD_S)
    assert mid == pytest.approx([2.0] * N_JOINTS, abs=FLOAT_TOL)


def test_seed_clears_sample_history():
    """A mode switch re-seeds. Samples from before the seed must not become
    an interpolation endpoint for the first sample after it."""
    buf = SideBuffer()
    buf.push(0.0, [0.0] * N_JOINTS)
    buf.push(PUBLISH_PERIOD_S, [1.0] * N_JOINTS)
    measured = [0.3] * N_JOINTS
    buf.seed(2.0, measured)
    assert buf.interpolate(2.0) == measured
    first_q = [5.0] * N_JOINTS
    buf.push(3.0, first_q)
    assert buf.interpolate(3.0 + POLL_DT_S) == first_q


def test_duplicate_arrival_times_hold_newest():
    """Two samples with the same arrival time give a zero period. The buffer
    holds the newest instead of dividing by zero."""
    q_a = [0.0] * N_JOINTS
    q_b = [1.0] * N_JOINTS
    q_c = [2.0] * N_JOINTS
    buf = SideBuffer()
    buf.push(0.0, q_a)
    buf.push(0.0, q_b)
    assert buf.interpolate(0.0) == q_b
    assert buf.interpolate(1.0) == q_b

    buf = SideBuffer()
    buf.push(0.0, q_a)
    buf.push(PUBLISH_PERIOD_S, q_b)
    buf.push(PUBLISH_PERIOD_S, q_c)
    for now in (PUBLISH_PERIOD_S, PUBLISH_PERIOD_S + POLL_DT_S, 1.0):
        assert buf.interpolate(now) == q_c


def test_interpolate_before_any_data_returns_none():
    buf = SideBuffer()
    assert buf.interpolate(0.0) is None
    assert buf.interpolate(1.0) is None
    assert buf.interpolate(-1.0) is None


def test_late_poll_is_clamped_to_newest():
    """A poll more than one period after the newest sample (the publisher
    stalled or stopped) holds q_next. An early poll (clock skew, now before
    t_next) holds q_prev. Neither extrapolates."""
    q_prev = [0.0] * N_JOINTS
    q_next = [1.0] * N_JOINTS
    buf = SideBuffer()
    buf.push(0.0, q_prev)
    buf.push(PUBLISH_PERIOD_S, q_next)
    assert buf.interpolate(2.0 * PUBLISH_PERIOD_S) == q_next
    assert buf.interpolate(2.0 * PUBLISH_PERIOD_S + POLL_DT_S) == q_next
    assert buf.interpolate(10.0) == q_next
    assert buf.interpolate(0.5 * PUBLISH_PERIOD_S) == q_prev


def test_accepts_numpy_and_returns_plain_floats():
    """The node hands lists to the controller. Inputs may be numpy rows."""
    buf = SideBuffer()
    buf.push(0.0, np.zeros(N_JOINTS))
    buf.push(PUBLISH_PERIOD_S, np.ones(N_JOINTS))
    out = buf.interpolate(PUBLISH_PERIOD_S + 0.25 * PUBLISH_PERIOD_S)
    assert type(out) is list
    assert all(type(x) is float for x in out)
    assert out == pytest.approx([0.25] * N_JOINTS, abs=FLOAT_TOL)
