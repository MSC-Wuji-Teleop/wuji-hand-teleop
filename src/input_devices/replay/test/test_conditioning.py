"""Conditioning core: extraction, audits, k selection, verdict, generator."""

import math

import numpy as np
import pytest

from conftest import ARM_LIMITS_PATH, BODY_ACTUATORS

from replay.clip_artifact import CANONICAL_ARM_JOINTS
from replay.conditioning import (
    allowed_speed_scale,
    audit_tracks,
    choose_k_extra,
    collect_verdict_reasons,
    extract_arm_q,
    single_joint_ramp,
    waist_motion,
)

from g1_world_output.replay_safety import ArmLimits


@pytest.fixture(scope='module')
def arm_limits():
    return ArmLimits.from_yaml(ARM_LIMITS_PATH, CANONICAL_ARM_JOINTS)


def _audit(q, lim, dt=0.02, k_extra=1, ceiling=True):
    return audit_tracks(
        q, CANONICAL_ARM_JOINTS, lim.pos_lower, lim.pos_upper,
        lim.vel_ceiling if ceiling else None,
        lim.deploy_velocity, lim.deploy_acceleration, dt, k_extra,
    )


class TestExtraction:
    def test_by_name_not_index(self):
        # Shuffle the actuator order; extraction must still land by name.
        rng = np.random.default_rng(0)
        order = list(BODY_ACTUATORS)
        rng.shuffle(order)
        body_q = np.zeros((5, 29))
        col = order.index('left_wrist_yaw')
        body_q[:, col] = 0.7
        q14 = extract_arm_q(body_q, order)
        assert np.all(q14[:, CANONICAL_ARM_JOINTS.index('left_wrist_yaw')] == 0.7)
        assert q14.shape == (5, 14)

    def test_missing_arm_joint_raises(self):
        names = [n for n in BODY_ACTUATORS if n != 'right_elbow']
        with pytest.raises(ValueError, match='right_elbow'):
            extract_arm_q(np.zeros((5, 28)), names)

    def test_waist_motion(self):
        body_q = np.zeros((5, 29))
        body_q[2, BODY_ACTUATORS.index('waist_roll')] = 0.01
        w = waist_motion(body_q, BODY_ACTUATORS)
        assert w['waist_roll'] == pytest.approx(0.01)
        assert w['waist_yaw'] == 0.0


class TestAudit:
    def test_slow_clip_clean(self, arm_limits):
        t = np.arange(200) / 50.0
        q = 0.3 * np.sin(2 * np.pi * 0.2 * t)[:, None] * np.ones((1, 14))
        audit = _audit(q, arm_limits)
        assert audit['finite']
        assert audit['position_violations'] == 0
        assert audit['spike_count'] == 0
        # peak velocity ~0.3 * 2pi * 0.2 = 0.377 rad/s
        assert max(audit['native']['peak_velocity']) == pytest.approx(0.377, abs=0.01)

    def test_spike_detected_and_measured(self, arm_limits):
        # Measured fact 3: 3.23 rad in one 20 ms frame = 161 rad/s, 7.3x the
        # 22 rad/s wrist ceiling.
        q = np.zeros((50, 14))
        j = CANONICAL_ARM_JOINTS.index('left_wrist_pitch')
        q[25:, j] = 1.4  # single-frame jump of 1.4 rad = 70 rad/s at 50 fps
        audit = _audit(q, arm_limits)
        assert audit['spike_count'] == 1
        s = audit['spikes'][0]
        assert s['joint'] == 'left_wrist_pitch'
        assert s['v_native_rad_s'] == pytest.approx(70.0)
        assert s['ceiling_rad_s'] == 22.0

    def test_below_ceiling_fast_motion_is_not_a_spike(self, arm_limits):
        q = np.zeros((50, 14))
        j = CANONICAL_ARM_JOINTS.index('left_elbow')
        q[25:, j] = 0.5  # 25 rad/s at 50 fps, under the 37 ceiling
        audit = _audit(q, arm_limits)
        assert audit['spike_count'] == 0

    def test_position_violation_counted(self, arm_limits):
        q = np.zeros((10, 14))
        q[4:6, CANONICAL_ARM_JOINTS.index('left_elbow')] = 2.5  # > 2.0944
        audit = _audit(q, arm_limits)
        assert audit['position_violations'] == 2

    def test_play_grid_scales(self, arm_limits):
        t = np.arange(200) / 50.0
        q = 1.0 * np.sin(2 * np.pi * 0.5 * t)[:, None] * np.ones((1, 14))
        a1 = _audit(q, arm_limits, k_extra=1)
        a4 = _audit(q, arm_limits, k_extra=4)
        assert a4['play']['peak_velocity'][0] == pytest.approx(
            a1['native']['peak_velocity'][0] / 4)
        assert a4['play']['peak_acceleration'][0] == pytest.approx(
            a1['native']['peak_acceleration'][0] / 16)
        assert a4['play_dt_s'] == pytest.approx(4 / 50.0)


class TestChooseK:
    def test_slow_clip_k1(self, arm_limits):
        t = np.arange(200) / 50.0
        q = 0.3 * np.sin(2 * np.pi * 0.2 * t)[:, None] * np.ones((1, 14))
        k, capped = choose_k_extra([_audit(q, arm_limits)], k_max=8)
        assert (k, capped) == (1, False)

    def test_sustained_overspeed_sets_k(self, arm_limits):
        # 1.0 rad at 0.5 Hz: peak v = pi ~ 3.14 rad/s vs deploy 0.5 -> k=7;
        # accel term: peak a = (2 pi 0.5)^2 ~ 9.87 vs 3.0 -> ceil(sqrt(3.29))=2.
        t = np.arange(500) / 50.0
        q = 1.0 * np.sin(2 * np.pi * 0.5 * t)[:, None] * np.ones((1, 14))
        audit = _audit(q, arm_limits)
        k, capped = choose_k_extra([audit], k_max=8)
        sustained = max(audit['native']['sustained_velocity_p99_5'])
        assert k == math.ceil(sustained / 0.5)
        assert not capped

    def test_k_capped(self, arm_limits):
        q = np.zeros((50, 14))
        q[:, 0] = np.linspace(0, 25.0, 50)  # ~25 rad/s sustained everywhere
        k, capped = choose_k_extra([_audit(q, arm_limits)], k_max=8)
        assert k == 8 and capped


class TestAllowedScale:
    def test_slow_clip_full_scale(self, arm_limits):
        t = np.arange(200) / 50.0
        q = 0.05 * np.sin(2 * np.pi * 0.2 * t)[:, None] * np.ones((1, 14))
        assert allowed_speed_scale([_audit(q, arm_limits)]) == 1.0

    def test_scale_is_min_of_velocity_and_accel_bounds(self, arm_limits):
        # Triangle wave: constant slope ~1.02 rad/s (velocity bound 0.5/1.02
        # ~ 0.49) but the apex carries an FD acceleration corner (~51
        # rad/s^2), so the sqrt accel bound (~0.24) binds. Velocity scales
        # linearly with speed_scale, acceleration quadratically.
        q = np.zeros((100, 14))
        q[:, 0] = np.concatenate([np.linspace(0, 1.0, 50),
                                  np.linspace(1.0, 0, 50)])
        audit = _audit(q, arm_limits)
        scale = allowed_speed_scale([audit])
        vel_bound = 0.5 / audit['play']['peak_velocity'][0]
        acc_bound = math.sqrt(3.0 / audit['play']['peak_acceleration'][0])
        assert scale == pytest.approx(min(vel_bound, acc_bound))
        assert scale == pytest.approx(acc_bound)  # the corner binds here

    def test_never_above_one(self, arm_limits):
        q = np.zeros((50, 14))  # static clip
        assert allowed_speed_scale([_audit(q, arm_limits)]) == 1.0


class TestVerdict:
    def test_clean_pass(self, arm_limits):
        t = np.arange(200) / 50.0
        q = 0.3 * np.sin(2 * np.pi * 0.2 * t)[:, None] * np.ones((1, 14))
        reasons = collect_verdict_reasons(
            _audit(q, arm_limits), None,
            {'waist_yaw': 0.0, 'waist_roll': 0.0, 'waist_pitch': 0.0},
            k_capped=False, k_max=8,
        )
        assert reasons == []

    def test_spike_fails_with_7e_language(self, arm_limits):
        q = np.zeros((50, 14))
        q[25:, CANONICAL_ARM_JOINTS.index('left_wrist_pitch')] = 1.4
        reasons = collect_verdict_reasons(
            _audit(q, arm_limits), None,
            {'waist_yaw': 0.0, 'waist_roll': 0.0, 'waist_pitch': 0.0},
            k_capped=False, k_max=8,
        )
        assert any('retiming does not fix' in r for r in reasons)

    def test_waist_motion_fails(self, arm_limits):
        q = np.zeros((50, 14))
        reasons = collect_verdict_reasons(
            _audit(q, arm_limits), None,
            {'waist_yaw': 0.2, 'waist_roll': 0.0, 'waist_pitch': 0.0},
            k_capped=False, k_max=8,
        )
        assert any('waist_yaw moves' in r for r in reasons)

    def test_manifest_mismatch_fails(self, arm_limits):
        q = np.zeros((50, 14))
        reasons = collect_verdict_reasons(
            _audit(q, arm_limits), None,
            {'waist_yaw': 0.0, 'waist_roll': 0.0, 'waist_pitch': 0.0},
            k_capped=False, k_max=8,
            manifest_mismatches=['samples/x/GT/g1_reference/foo.npz'],
        )
        assert any('hash mismatch' in r for r in reasons)


class TestSingleJointRamp:
    def test_profile_within_deploy_caps(self):
        fps, dep_v, dep_a = 50.0, 0.5, 3.0
        q = single_joint_ramp(14, 3, 0.2, dep_v, dep_a, fps, headroom=0.5)
        v = np.diff(q[:, 3]) * fps
        a = np.diff(v) * fps
        assert np.max(np.abs(v)) <= 0.5 * dep_v * 1.05  # headroom + discretization
        assert np.max(np.abs(a)) <= 0.5 * dep_a * 1.1
        # Other joints never move; ramp returns to start.
        assert np.all(q[:, [j for j in range(14) if j != 3]] == 0)
        assert q[0, 3] == pytest.approx(0.0)
        assert abs(q[-1, 3]) < 1e-6
        assert np.max(np.abs(q[:, 3])) == pytest.approx(0.2, rel=1e-2)

    def test_zero_amplitude_rejected(self):
        with pytest.raises(ValueError):
            single_joint_ramp(14, 0, 0.0, 0.5, 3.0, 50.0)
