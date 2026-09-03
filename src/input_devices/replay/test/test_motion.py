"""Pins replay.motion: clip lerp, the quintic approach, named JointState mapping."""

import numpy as np

from replay.motion import (
    clip_start_velocity,
    lerp_clip,
    min_jerk_u,
    named_positions,
    quintic_blend,
)


def test_lerp_clip_holds_the_last_row_and_blends_between():
    q = np.array([[0.0, 10.0], [1.0, 20.0], [2.0, 30.0]])
    np.testing.assert_allclose(lerp_clip(q, 0.0), [0.0, 10.0])
    np.testing.assert_allclose(lerp_clip(q, 1.5), [1.5, 25.0])
    np.testing.assert_allclose(lerp_clip(q, 2.0), [2.0, 30.0])
    np.testing.assert_allclose(lerp_clip(q, 99.0), [2.0, 30.0])
    np.testing.assert_allclose(lerp_clip(q[:1], 0.7), [0.0, 10.0])


def test_quintic_rest_to_rest_is_min_jerk():
    q0 = np.array([1.0, 2.0])
    q1 = np.array([5.0, -2.0])
    zero = np.zeros(2)
    for u in (0.0, 0.25, 0.5, 0.75, 1.0):
        np.testing.assert_allclose(quintic_blend(q0, zero, q1, zero, u), q0 + min_jerk_u(u) * (q1 - q0))


def test_quintic_matches_end_position_and_velocity():
    q0 = np.array([0.0])
    q1 = np.array([2.0])
    qd0 = np.array([0.0])
    qd1 = np.array([4.0])  # dq/du at u=1
    np.testing.assert_allclose(quintic_blend(q0, qd0, q1, qd1, 0.0), q0)
    np.testing.assert_allclose(quintic_blend(q0, qd0, q1, qd1, 1.0), q1)
    du = 1e-6
    numeric = (quintic_blend(q0, qd0, q1, qd1, 1.0) - quintic_blend(q0, qd0, q1, qd1, 1.0 - du)) / du
    np.testing.assert_allclose(numeric, qd1, rtol=0.0, atol=1e-4)
    numeric0 = (quintic_blend(q0, qd0, q1, qd1, du) - quintic_blend(q0, qd0, q1, qd1, 0.0)) / du
    np.testing.assert_allclose(numeric0, qd0, rtol=0.0, atol=1e-4)


def test_quintic_is_not_a_linear_staircase():
    q0 = np.array([0.0])
    q1 = np.array([1.0])
    zero = np.zeros(1)
    mid = quintic_blend(q0, zero, q1, zero, 0.2)
    assert mid[0] < 0.2  # min-jerk starts slower than a linear ramp


def test_named_positions_fills_from_fallback():
    wanted = ("l_a", "l_b", "l_c")
    fallback = np.array([9.0, 8.0, 7.0])
    out = named_positions(wanted, ["l_c", "l_a"], [1.0, 2.0], fallback)
    np.testing.assert_allclose(out, [2.0, 8.0, 1.0])


def test_clip_start_velocity_is_first_step_times_rate():
    q = np.array([[0.0, 1.0], [0.1, 1.2], [0.2, 1.4]])
    np.testing.assert_allclose(clip_start_velocity(q, 50.0, 0.25), [1.25, 2.5])
    np.testing.assert_allclose(clip_start_velocity(q[:1], 50.0, 1.0), [0.0, 0.0])
