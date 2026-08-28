"""Unit tests for replay_safety.py -- pure numpy, no ROS.

Run with plain pytest from the package root, or colcon test inside the
container. The limits fixture is the real shipped g1_deploy_limits.yaml so
the file's schema is under test too.
"""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_world_output.replay_safety import (  # noqa: E402
    ArmLimits,
    DivergenceMonitor,
    LimitsError,
    PositionClamp,
    ReplaySafetyChain,
    StalenessTracker,
    rate_limit_step,
)

LIMITS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'g1_deploy_limits.yaml'

G1_29_NAMES = [
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
    'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
    'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
]
G1_23_NAMES = [n for n in G1_29_NAMES if 'wrist_pitch' not in n and 'wrist_yaw' not in n]


# ---------------------------------------------------------------- ArmLimits

class TestArmLimits:
    def test_loads_g1_29_rows_in_name_order(self):
        lim = ArmLimits.from_yaml(LIMITS_PATH, G1_29_NAMES)
        assert lim.names == G1_29_NAMES
        assert lim.pos_lower.shape == (14,)
        # Spot values from the Unitree URDF.
        i = G1_29_NAMES.index('left_wrist_pitch')
        assert lim.vel_ceiling[i] == 22.0
        assert lim.effort_ceiling[i] == 5.0
        assert lim.pos_upper[i] == pytest.approx(1.614429558)
        j = G1_29_NAMES.index('left_elbow')
        assert lim.vel_ceiling[j] == 37.0
        assert (lim.pos_lower[j], lim.pos_upper[j]) == (pytest.approx(-1.0472),
                                                        pytest.approx(2.0944))

    def test_shoulder_roll_is_asymmetric_and_mirrored(self):
        lim = ArmLimits.from_yaml(LIMITS_PATH, G1_29_NAMES)
        li = G1_29_NAMES.index('left_shoulder_roll')
        ri = G1_29_NAMES.index('right_shoulder_roll')
        assert lim.pos_lower[li] == pytest.approx(-1.5882)
        assert lim.pos_upper[li] == pytest.approx(2.2515)
        assert lim.pos_lower[ri] == pytest.approx(-2.2515)
        assert lim.pos_upper[ri] == pytest.approx(1.5882)

    def test_g1_23_subset_works(self):
        lim = ArmLimits.from_yaml(LIMITS_PATH, G1_23_NAMES)
        assert len(lim.names) == 10
        assert np.all(lim.vel_ceiling == 37.0)

    def test_waist_rows_available_by_name(self):
        lim = ArmLimits.from_yaml(LIMITS_PATH, ['waist_yaw', 'waist_roll', 'waist_pitch'])
        assert lim.effort_ceiling.tolist() == [88.0, 35.0, 35.0]
        assert lim.vel_ceiling.tolist() == [32.0, 30.0, 30.0]

    def test_missing_joint_raises(self):
        with pytest.raises(LimitsError, match='no_such_joint'):
            ArmLimits.from_yaml(LIMITS_PATH, ['no_such_joint'])

    def test_deploy_velocity_is_screening_value(self):
        lim = ArmLimits.from_yaml(LIMITS_PATH, G1_29_NAMES)
        assert np.all(lim.deploy_velocity == 0.5)
        assert np.all(lim.deploy_acceleration == 3.0)

    def test_deploy_above_ceiling_rejected(self, tmp_path):
        bad = tmp_path / 'bad.yaml'
        bad.write_text(
            "hardware_ceilings:\n"
            "  j0: {position: [-1.0, 1.0], velocity: 5.0, effort: 1.0}\n"
            "deploy:\n  velocity: 6.0\n  acceleration: 3.0\n"
        )
        with pytest.raises(LimitsError, match='exceeds the hardware ceiling'):
            ArmLimits.from_yaml(bad, ['j0'])

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(LimitsError, match='not found'):
            ArmLimits.from_yaml(tmp_path / 'absent.yaml', G1_29_NAMES)


# ------------------------------------------------------------ PositionClamp

class TestPositionClamp:
    def test_inside_passes_through(self):
        c = PositionClamp(np.array([-1.0, -2.0]), np.array([1.0, 2.0]))
        q, hit = c.apply(np.array([0.5, -1.5]))
        assert np.allclose(q, [0.5, -1.5])
        assert not hit.any()

    def test_clamps_and_flags(self):
        c = PositionClamp(np.array([-1.0, -2.0]), np.array([1.0, 2.0]))
        q, hit = c.apply(np.array([1.5, -3.0]))
        assert np.allclose(q, [1.0, -2.0])
        assert hit.tolist() == [True, True]

    def test_margin_shrinks_bounds(self):
        c = PositionClamp(np.array([-1.0]), np.array([1.0]), margin=0.1)
        q, hit = c.apply(np.array([0.95]))
        assert q[0] == pytest.approx(0.9)
        assert hit[0]

    def test_margin_inverting_bounds_rejected(self):
        with pytest.raises(ValueError, match='inverts'):
            PositionClamp(np.array([-0.1]), np.array([0.1]), margin=0.2)


# ---------------------------------------------------------- rate_limit_step

class TestRateLimitStep:
    def test_slow_step_untouched(self):
        out = rate_limit_step(np.zeros(2), np.array([0.001, 0.001]),
                              np.array([1.0, 1.0]), dt=0.01)
        assert np.allclose(out, [0.001, 0.001])

    def test_fast_step_scaled_uniformly(self):
        # Joint 0 wants 10x its per-tick budget; joint 1 is inside its own.
        # Uniform scaling shrinks BOTH by 10x: direction preserved.
        vel = np.array([1.0, 1.0])
        out = rate_limit_step(np.zeros(2), np.array([0.1, 0.005]), vel, dt=0.01)
        assert out[0] == pytest.approx(0.01)     # exactly its budget
        assert out[1] == pytest.approx(0.0005)   # shrunk by the same 10x
        # Direction (ratio between components) preserved:
        assert out[1] / out[0] == pytest.approx(0.005 / 0.1)

    def test_per_joint_limits_bind_on_the_tightest(self):
        # Same request both joints, joint 1 has a 10x lower limit -> it binds.
        vel = np.array([10.0, 1.0])
        out = rate_limit_step(np.zeros(2), np.array([0.1, 0.1]), vel, dt=0.01)
        assert out[1] == pytest.approx(0.01)  # joint 1 at its budget
        assert out[0] == pytest.approx(0.01)  # scaled with it (uniform)

    def test_negative_direction(self):
        out = rate_limit_step(np.zeros(1), np.array([-5.0]), np.array([1.0]), dt=0.01)
        assert out[0] == pytest.approx(-0.01)

    def test_non_finite_target_raises(self):
        with pytest.raises(ValueError):
            rate_limit_step(np.zeros(1), np.array([np.nan]), np.array([1.0]), dt=0.01)

    def test_wrist_ceiling_dds_clip_shape(self):
        # The DDS-thread use: ceilings at 250 Hz. A 3.23 rad single-frame
        # jump (measured fact 3) must be cut to the wrist budget.
        lim = ArmLimits.from_yaml(LIMITS_PATH, G1_29_NAMES)
        dt = 1.0 / 250.0
        q0 = np.zeros(14)
        q1 = np.zeros(14)
        wrist = G1_29_NAMES.index('left_wrist_pitch')
        q1[wrist] = 3.23
        out = rate_limit_step(q0, q1, lim.vel_ceiling, dt)
        assert out[wrist] == pytest.approx(22.0 * dt)


# --------------------------------------------------------- StalenessTracker

class TestStalenessTracker:
    def test_no_input_is_stale(self):
        s = StalenessTracker(0.1)
        assert s.is_stale(now=0.0)

    def test_fresh_then_stale(self):
        s = StalenessTracker(0.1)
        s.mark(1.0)
        assert not s.is_stale(1.05)
        assert s.is_stale(1.2)
        assert s.age(1.2) == pytest.approx(0.2)

    def test_episode_counting(self):
        s = StalenessTracker(0.1)
        s.mark(0.0)
        s.is_stale(0.05)   # fresh
        s.is_stale(0.2)    # episode 1 starts
        s.is_stale(0.3)    # still episode 1
        s.mark(0.35)
        s.is_stale(0.36)   # fresh again
        s.is_stale(0.6)    # episode 2
        assert s.stale_episodes == 2


# ------------------------------------------------------- DivergenceMonitor

class TestDivergenceMonitor:
    def test_single_spike_does_not_fault(self):
        d = DivergenceMonitor(threshold_rad=0.1, m_consecutive=3)
        assert not d.update(np.array([0.2]), np.array([0.0]))
        assert not d.update(np.array([0.0]), np.array([0.0]))  # resets count
        assert not d.update(np.array([0.2]), np.array([0.0]))
        assert not d.faulted

    def test_m_consecutive_faults_and_latches(self):
        d = DivergenceMonitor(threshold_rad=0.1, m_consecutive=3)
        for _ in range(3):
            fault = d.update(np.array([0.2]), np.array([0.0]))
        assert fault and d.faulted
        # Latched: healthy readings do not clear it.
        assert d.update(np.array([0.0]), np.array([0.0]))
        d.reset()
        assert not d.faulted

    def test_worst_joint_reported(self):
        d = DivergenceMonitor(threshold_rad=0.1, m_consecutive=1)
        d.update(np.array([0.0, 0.5, 0.1]), np.zeros(3))
        assert d.worst_joint == 1
        assert d.worst_error == pytest.approx(0.5)


# -------------------------------------------------------- ReplaySafetyChain

@pytest.fixture
def chain():
    lim = ArmLimits.from_yaml(LIMITS_PATH, G1_29_NAMES)
    return ReplaySafetyChain(
        lim, control_dt=1.0 / 250.0, position_margin=0.01,
        staleness_timeout_s=0.1, divergence_threshold_rad=0.35,
        divergence_ticks=5,
    )


class TestReplaySafetyChain:
    def test_no_input_holds_last_command(self, chain):
        last = np.full(14, 0.3)
        r = chain.process(now=0.0, target_q=None, last_cmd_q=last)
        assert r.stale
        assert np.allclose(r.cmd, last)

    def test_fresh_target_rate_limited_toward(self, chain):
        chain.mark_input(0.0)
        last = np.zeros(14)
        target = np.full(14, 1.0)
        r = chain.process(now=0.01, target_q=target, last_cmd_q=last)
        assert not r.stale
        assert r.rate_limited
        # Deploy velocity 0.5 rad/s at 250 Hz -> 2 mrad per tick.
        assert np.allclose(r.cmd, 0.5 / 250.0)

    def test_stale_input_holds(self, chain):
        chain.mark_input(0.0)
        last = np.full(14, 0.2)
        r = chain.process(now=1.0, target_q=np.zeros(14), last_cmd_q=last)
        assert r.stale
        assert np.allclose(r.cmd, last)

    def test_out_of_bounds_target_clamped(self, chain):
        chain.mark_input(0.0)
        target = np.zeros(14)
        i = G1_29_NAMES.index('left_wrist_pitch')
        target[i] = 5.0  # far past +1.6144
        # Start close to the bound so the rate limit is not what binds.
        last = np.full(14, 0.0)
        last[i] = 1.60
        r = chain.process(now=0.001, target_q=target, last_cmd_q=last)
        assert r.clamped[i]
        assert r.cmd[i] <= 1.614429558 - 0.01 + 1e-12

    def test_divergence_faults_after_m_ticks(self, chain):
        chain.mark_input(0.0)
        last = np.zeros(14)
        measured = np.zeros(14)
        measured[0] = 1.0  # 1 rad off the command
        fault = False
        for k in range(5):
            r = chain.process(now=0.001 * (k + 1), target_q=np.zeros(14),
                              last_cmd_q=last, measured_q=measured)
            fault = r.divergence_fault
        assert fault

    def test_no_measured_no_divergence(self, chain):
        chain.mark_input(0.0)
        r = chain.process(now=0.001, target_q=np.zeros(14),
                          last_cmd_q=np.zeros(14), measured_q=None)
        assert not r.divergence_fault
