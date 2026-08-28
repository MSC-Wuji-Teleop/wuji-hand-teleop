"""Unit tests for hand_safety.py -- pure numpy, no ROS.

Loads the real shipped hand_limits.yaml so the file schema is under test.
"""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wujihand_output.hand_safety import (  # noqa: E402
    NUM_JOINTS,
    EffortGuard,
    HandLimits,
    LimitsError,
    PositionClamp,
    StalenessTracker,
    rate_limit_step,
)

LIMITS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'hand_limits.yaml'


class TestHandLimits:
    def test_loads_20_rows_in_declaration_order(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        assert len(lim.names) == NUM_JOINTS
        assert lim.names[0] == 'thumb_cmc_flex'
        assert lim.names[19] == 'pinky_dip'
        # Spot values from the Beta 2 URDF.
        assert (lim.pos_lower[0], lim.pos_upper[0]) == (pytest.approx(-1.187),
                                                        pytest.approx(1.291))
        assert lim.sim_model_velocity[0] == pytest.approx(8.587)
        # thumb_cmc_abd is the asymmetric one.
        assert (lim.pos_lower[1], lim.pos_upper[1]) == (pytest.approx(-1.484),
                                                        pytest.approx(0.698))

    def test_deploy_velocity_is_screening_not_urdf(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        assert np.all(lim.deploy_velocity == 4.0)
        assert np.all(lim.deploy_acceleration == 20.0)
        # The URDF values are strictly larger -- they must never be the cap.
        assert np.all(lim.sim_model_velocity > lim.deploy_velocity)

    def test_side_names(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        assert lim.side_names('right')[0] == 'r_thumb_cmc_flex'
        assert lim.side_names('left')[19] == 'l_pinky_dip'

    def test_driver_name_table(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        assert lim.driver_names[0] == 'finger1_joint1'
        assert lim.driver_names[19] == 'finger5_joint4'
        assert len(lim.driver_names) == NUM_JOINTS

    def test_flexion_first_convention(self):
        # Position 0 of each finger group is flexion, position 1 abduction.
        lim = HandLimits.from_yaml(LIMITS_PATH)
        for f in range(5):
            n0, n1 = lim.names[4 * f], lim.names[4 * f + 1]
            assert 'flex' in n0, (f, n0)
            assert 'abd' in n1, (f, n1)

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(LimitsError, match='not found'):
            HandLimits.from_yaml(tmp_path / 'absent.yaml')

    def test_out_of_order_rows_rejected(self, tmp_path):
        import yaml as _yaml
        raw = _yaml.safe_load(LIMITS_PATH.read_text())
        raw['joints'][0], raw['joints'][1] = raw['joints'][1], raw['joints'][0]
        bad = tmp_path / 'bad.yaml'
        bad.write_text(_yaml.safe_dump(raw))
        with pytest.raises(LimitsError, match='out of order'):
            HandLimits.from_yaml(bad)


class TestClampAndRate:
    def test_clamp_flags_violations(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        c = PositionClamp(lim.pos_lower, lim.pos_upper)
        q = np.zeros(NUM_JOINTS)
        q[1] = 2.0  # thumb_cmc_abd upper is 0.698
        out, hit = c.apply(q)
        assert out[1] == pytest.approx(0.698)
        assert hit[1] and hit.sum() == 1

    def test_rate_limit_at_deploy_cap(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        dt = 1.0 / 200.0  # replay control rate
        out = rate_limit_step(np.zeros(NUM_JOINTS), np.full(NUM_JOINTS, 1.0),
                              lim.deploy_velocity, dt)
        assert np.allclose(out, 4.0 * dt)

    def test_small_step_untouched(self):
        lim = HandLimits.from_yaml(LIMITS_PATH)
        target = np.full(NUM_JOINTS, 0.001)
        out = rate_limit_step(np.zeros(NUM_JOINTS), target,
                              lim.deploy_velocity, 1.0 / 200.0)
        assert np.allclose(out, target)


class TestStaleness:
    def test_hold_semantics(self):
        s = StalenessTracker(0.05)
        assert s.is_stale(0.0)
        s.mark(0.0)
        assert not s.is_stale(0.04)
        assert s.is_stale(0.06)


class TestEffortGuard:
    def test_inactive_until_limits_arrive(self):
        g = EffortGuard(m_consecutive=3)
        assert not g.active
        assert not g.update(np.full(NUM_JOINTS, 99.0))  # no limits yet

    def test_sustained_over_current_faults(self):
        g = EffortGuard(m_consecutive=3)
        g.set_limits(np.full(NUM_JOINTS, 1.5))
        eff = np.zeros(NUM_JOINTS)
        eff[7] = 2.0
        assert not g.update(eff)
        assert not g.update(eff)
        assert g.update(eff)
        assert g.faulted and g.worst_joint == 7
        # Latched.
        assert g.update(np.zeros(NUM_JOINTS))
        g.reset()
        assert not g.faulted

    def test_transient_spike_does_not_fault(self):
        g = EffortGuard(m_consecutive=3)
        g.set_limits(np.full(NUM_JOINTS, 1.5))
        spike = np.full(NUM_JOINTS, 2.0)
        calm = np.full(NUM_JOINTS, 0.5)
        assert not g.update(spike)
        assert not g.update(calm)   # resets the count
        assert not g.update(spike)
        assert not g.update(spike)
        assert not g.faulted

    def test_scale_tightens_threshold(self):
        g = EffortGuard(m_consecutive=1, scale=0.5)
        g.set_limits(np.full(NUM_JOINTS, 2.0))  # effective 1.0
        assert g.update(np.full(NUM_JOINTS, 1.2))

    def test_bad_limits_rejected(self):
        g = EffortGuard(m_consecutive=1)
        with pytest.raises(ValueError):
            g.set_limits(np.zeros(NUM_JOINTS))       # non-positive
        with pytest.raises(ValueError):
            g.set_limits(np.ones(NUM_JOINTS - 1))    # wrong shape
