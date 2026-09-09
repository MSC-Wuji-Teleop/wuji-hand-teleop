"""Pins for tools/sanitize/unwrap.py: wraps, drift, and the flip signature."""

from __future__ import annotations

import math

import numpy as np
import pytest

from clip_audit import ARM_JOINT_NAMES
from sanitize import unwrap


NAMES = ARM_JOINT_NAMES["left"]
WIDE_LOWER = np.full(7, -math.pi)
WIDE_UPPER = np.full(7, math.pi)


def smooth(frames=40, amplitude=0.3):
    """A smooth per-joint trajectory, nothing to unwrap or cap."""
    t = np.linspace(0.0, 1.0, frames)
    return np.stack([amplitude * np.sin(2 * math.pi * t + j) for j in range(7)], axis=1)


# -- 2pi wraps --------------------------------------------------------------

def test_unwrap_leaves_a_smooth_trajectory_alone():
    q = smooth()
    result = unwrap.unwrap_trajectory(q, NAMES, WIDE_LOWER, WIDE_UPPER)
    assert np.allclose(result.q, q)
    assert result.removed == {} and result.total == 0


def test_unwrap_removes_a_wrap_and_names_the_joint():
    q = smooth()
    q[20:, 3] -= 2 * math.pi                       # the same angle, wrapped
    result = unwrap.unwrap_trajectory(q, NAMES, WIDE_LOWER, WIDE_UPPER)
    assert np.allclose(result.q[:, 3], smooth()[:, 3])
    assert NAMES[3] in result.removed
    assert result.total >= 1


def test_unwrap_recentres_a_column_written_a_whole_turn_away():
    # The same rotation a revolution out: 6.4 rad on a joint that stops at 3.1.
    q = np.zeros((10, 7))
    q[:, 0] = 6.4
    lower = np.full(7, -3.1)
    upper = np.full(7, 3.1)
    result = unwrap.unwrap_trajectory(q, NAMES, lower, upper)
    assert result.recentred == {NAMES[0]: -1}
    assert np.allclose(result.q[:, 0], 6.4 - 2 * math.pi)
    assert result.q[:, 0].max() <= upper[0] + 1e-9


def test_unwrap_leaves_an_in_range_column_alone_entirely():
    q = smooth()
    result = unwrap.unwrap_trajectory(q, NAMES, WIDE_LOWER, WIDE_UPPER)
    assert result.recentred == {} and result.removed == {} and result.kept == {}
    assert np.allclose(result.q, q)


def test_unwrap_keeps_a_wrap_it_cannot_represent_and_says_so():
    # 3.0 -> -3.0 unwraps to 3.28, which no offset fits inside a +-3.1 joint.
    q = np.zeros((10, 7))
    q[:, 0] = 3.0
    q[5:, 0] = -3.0
    lower = np.full(7, -3.1)
    upper = np.full(7, 3.1)
    result = unwrap.unwrap_trajectory(q, NAMES, lower, upper)
    assert result.removed == {}
    assert result.kept == {NAMES[0]: 1}
    assert np.allclose(result.q, q)
    assert result.q[:, 0].min() >= lower[0] - 1e-9
    assert result.q[:, 0].max() <= upper[0] + 1e-9


# -- drift ------------------------------------------------------------------

def test_measure_drift_uses_windowed_endpoints_not_single_frames():
    q = np.linspace(0.0, 1.0, 100)
    q[0] = 50.0                                    # one absurd first frame
    assert unwrap.measure_drift(q, window=10) == pytest.approx(0.0, abs=6.0)
    # ... which a single-frame measurement would not survive.
    assert abs(q[-1] - q[0]) > 40.0


def test_cap_drift_is_a_no_op_under_the_cap():
    q = np.linspace(0.0, 0.2, 50)
    out, removed = unwrap.cap_drift(q, math.radians(30.0))
    assert removed == 0.0 and np.allclose(out, q)


def test_cap_drift_leaves_exactly_the_cap_and_reports_the_rest():
    q = np.linspace(0.0, math.radians(90.0), 200)
    out, removed = unwrap.cap_drift(q, math.radians(30.0), window=1)
    assert math.degrees(removed) == pytest.approx(60.0, abs=0.5)
    assert math.degrees(unwrap.measure_drift(out, window=1)) == pytest.approx(30.0, abs=0.5)
    # Removed as a ramp, so the ends move and the middle keeps its shape.
    assert out[0] == pytest.approx(q[0])


def test_cap_drift_off_by_default_leaves_everything():
    q = np.linspace(0.0, math.radians(180.0), 100)
    out, removed = unwrap.cap_drift(q, unwrap.DEFAULT_MAX_DRIFT_DEG)
    assert removed == 0.0 and np.allclose(out, q)


def test_cap_wrist_drift_only_touches_wrist_joints():
    q = np.zeros((100, 7))
    for j in range(7):
        q[:, j] = np.linspace(0.0, math.radians(120.0), 100)
    out, results = unwrap.cap_wrist_drift(q, NAMES, "left", math.radians(30.0), window=1)
    capped = {r.joint for r in results}
    assert capped == {"left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw"}
    for j, name in enumerate(NAMES):
        if name in capped:
            assert not np.allclose(out[:, j], q[:, j])
        else:
            assert np.allclose(out[:, j], q[:, j])       # shoulder and elbow untouched
    assert all(r.measured_deg == pytest.approx(120.0, abs=0.5) for r in results)
    assert all(r.residual_deg == pytest.approx(30.0, abs=0.5) for r in results)


# -- branch flips -----------------------------------------------------------

def test_detect_flips_needs_a_jump_the_pose_did_not_follow():
    frames = 20
    q = np.zeros((frames, 7))
    q[10:, 4] += math.radians(120.0)               # a large single-frame jump
    still = np.zeros(frames - 1)                   # ... and the pose did not move
    flips = unwrap.detect_flips(q, NAMES, "left", 50.0, still, still)
    assert [f.frame for f in flips] == [10]
    assert flips[0].joint == "left_wrist_roll"
    assert flips[0].step_deg == pytest.approx(120.0)
    assert flips[0].t_s == pytest.approx(0.2)


def test_detect_flips_ignores_a_jump_the_pose_did_follow():
    # Same jump, but the wrist travelled: fast motion for the step clamp, not a flip.
    frames = 20
    q = np.zeros((frames, 7))
    q[10:, 4] += math.radians(120.0)
    moved = np.zeros(frames - 1)
    moved[9] = 0.10                                # 100 mm in one frame
    assert unwrap.detect_flips(q, NAMES, "left", 50.0, moved, np.zeros(frames - 1)) == []

    rotated = np.zeros(frames - 1)
    rotated[9] = math.radians(40.0)
    assert unwrap.detect_flips(q, NAMES, "left", 50.0, np.zeros(frames - 1), rotated) == []


def test_detect_flips_ignores_small_steps():
    frames = 20
    q = np.zeros((frames, 7))
    q[10:, 4] += math.radians(20.0)                # under the 45 deg threshold
    still = np.zeros(frames - 1)
    assert unwrap.detect_flips(q, NAMES, "left", 50.0, still, still) == []


# -- the elbow gate ---------------------------------------------------------

def test_elbow_gate_opens_only_when_the_source_rejoins():
    gate = unwrap.ElbowGate(rejoin_tol_m=0.05)
    assert gate.open
    gate.close(frame=10)
    assert not gate.open

    solved = np.array([0.0, 0.0, 0.0])
    far = np.array([0.30, 0.0, 0.0])
    assert not gate.update(11, far, solved)        # source elbow still off-branch
    assert not gate.update(12, far, solved)
    near = np.array([0.02, 0.0, 0.0])
    assert gate.update(13, near, solved)           # back within tolerance
    assert gate.open
    assert gate.closed_frames == 2


def test_elbow_gate_stays_shut_without_a_solved_elbow_to_compare():
    gate = unwrap.ElbowGate(rejoin_tol_m=0.05)
    gate.close(frame=0)
    assert not gate.update(1, np.zeros(3), None)
