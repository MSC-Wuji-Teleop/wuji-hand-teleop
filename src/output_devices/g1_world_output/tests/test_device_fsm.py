"""Device FSM: traversal, gates, resets, fault freezes, A1 holds. ROS-free.

These cover exactly the behaviors dry-run sim CANNOT exercise (plan A6):
engage-gate rejection, lowstate-loss reset, divergence faults, and the
constant-snapshot hold property.
"""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_world_output.device_fsm import (  # noqa: E402
    ArmDeviceFSM,
    DeviceState,
    FsmConfig,
    TickInputs,
)
from g1_world_output.replay_safety import ArmLimits, ReplaySafetyChain  # noqa: E402

LIMITS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'g1_deploy_limits.yaml'

NAMES = [
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
    'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
    'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
]
DT = 1.0 / 250.0


def make_fsm(sim=False, **cfg_kw):
    cfg = FsmConfig(control_dt=DT, engage_fresh_ticks=5, sim=sim, **cfg_kw)
    chains = {}
    limits14 = ArmLimits.from_yaml(LIMITS_PATH, NAMES)
    for side, sl in (('left', slice(0, 7)), ('right', slice(7, 14))):
        side_names = NAMES[sl]
        lim = ArmLimits.from_yaml(LIMITS_PATH, side_names)
        chains[side] = ReplaySafetyChain(
            lim, control_dt=DT, staleness_timeout_s=0.1,
            divergence_threshold_rad=0.35, divergence_ticks=5,
        )
    return ArmDeviceFSM(NAMES, chains, limits14.deploy_velocity, cfg)


def hw_inputs(now, q=None, dq=None, age=0.001, stream=None):
    return TickInputs(
        now=now,
        measured_q=np.zeros(14) if q is None else q,
        measured_dq=np.zeros(14) if dq is None else dq,
        lowstate_age=age,
        stream=stream or {},
    )


def settle_ready(fsm, ticks=6, t0=0.0):
    """Feed fresh, still lowstate until the engage gate opens."""
    for i in range(ticks):
        fsm.tick(hw_inputs(t0 + i * DT))
    return t0 + ticks * DT


class TestEngageGate:
    def test_no_tick_refused(self):
        fsm = make_fsm()
        ok, msg = fsm.request_engage()
        assert not ok and 'no tick' in msg

    def test_streak_too_short_refused(self):
        fsm = make_fsm()
        fsm.tick(hw_inputs(0.0))
        ok, msg = fsm.request_engage()
        assert not ok and 'engage gate' in msg

    def test_moving_arm_resets_streak(self):
        fsm = make_fsm()
        for i in range(4):
            fsm.tick(hw_inputs(i * DT))
        # One moving frame resets the streak.
        fsm.tick(hw_inputs(5 * DT, dq=np.full(14, 0.2)))
        for i in range(3):
            fsm.tick(hw_inputs((6 + i) * DT))
        ok, msg = fsm.request_engage()
        assert not ok

    def test_stale_lowstate_resets_streak(self):
        fsm = make_fsm()
        for i in range(4):
            fsm.tick(hw_inputs(i * DT))
        fsm.tick(hw_inputs(5 * DT, age=1.0))
        ok, _ = fsm.request_engage()
        assert not ok

    def test_gate_opens_and_snapshot_taken(self):
        fsm = make_fsm()
        q = np.linspace(0, 1, 14)
        for i in range(6):
            fsm.tick(hw_inputs(i * DT, q=q))
        ok, _ = fsm.request_engage()
        assert ok
        assert fsm.state is DeviceState.ENGAGE
        np.testing.assert_array_equal(fsm.snapshot, q)


class TestEngageRamp:
    def test_weight_ramps_over_at_least_2s(self):
        fsm = make_fsm()
        t = settle_ready(fsm)
        fsm.request_engage()
        out = fsm.tick(hw_inputs(t + 1.0))
        assert 0.4 < out.weight < 0.6           # halfway through a 2 s ramp
        assert not fsm._engage_done
        out = fsm.tick(hw_inputs(t + 2.5))
        assert out.weight == 1.0
        assert fsm._engage_done
        # Command is the constant snapshot, never live measured (A1).
        drooped = np.full(14, -0.1)
        out = fsm.tick(hw_inputs(t + 2.6, q=drooped))
        np.testing.assert_array_equal(out.cmd, fsm.snapshot)


class TestApproachTrack:
    def _to_engaged(self, fsm):
        t = settle_ready(fsm)
        fsm.request_engage()
        fsm.tick(hw_inputs(t + 2.5))
        return t + 2.5

    def test_approach_needs_fresh_stream(self):
        fsm = make_fsm()
        self._to_engaged(fsm)
        ok, msg = fsm.request_approach()
        assert not ok and 'publish_first' in msg

    def test_full_traversal_to_release(self):
        fsm = make_fsm(sim=False)
        t = self._to_engaged(fsm)
        # Frame-0 stream on both sides, at the snapshot pose (zeros).
        frame0 = np.zeros(7)
        for side in ('left', 'right'):
            fsm.chains[side].mark_input(t)
        ok, _ = fsm.request_approach()
        assert ok and fsm.active_sides == ['left', 'right']

        stream = {'left': frame0, 'right': frame0}
        out = fsm.tick(hw_inputs(t + DT, stream=stream))
        assert fsm.state is DeviceState.APPROACH
        assert fsm._approach_done          # measured == target == 0
        ok, _ = fsm.request_track()
        assert ok and fsm.state is DeviceState.TRACK

        # Track a slow stream for a few ticks.
        for i in range(5):
            now = t + (2 + i) * DT
            for side in ('left', 'right'):
                fsm.chains[side].mark_input(now)
            tgt = np.full(7, 0.0005 * i)
            out = fsm.tick(hw_inputs(now, q=out.cmd, stream={'left': tgt, 'right': tgt}))
        assert fsm.state is DeviceState.TRACK

        ok, _ = fsm.request_end_hold()
        assert ok
        frozen = out.cmd.copy()
        out = fsm.tick(hw_inputs(t + 1.0))
        np.testing.assert_array_equal(out.cmd, frozen)  # cmd frozen
        out = fsm.tick(hw_inputs(t + 2.5))
        assert fsm._settled                 # dq ~ 0 for >= 1 s

        ok, _ = fsm.request_park()
        assert ok and fsm.state is DeviceState.APPROACH
        assert fsm.approach_target_kind == 'snapshot'
        # Drive approach until done (measured == snapshot == 0 immediately
        # since cmd converges and measured is fed as cmd here).
        out = fsm.tick(hw_inputs(t + 3.0, q=fsm.snapshot.copy()))
        assert fsm._approach_done
        ok, _ = fsm.request_release()
        assert ok and fsm.state is DeviceState.RELEASE
        out = fsm.tick(hw_inputs(t + 4.0))
        assert 0 < out.weight < 1
        out = fsm.tick(hw_inputs(t + 6.0))
        assert fsm.state is DeviceState.READY
        assert out.weight == 0.0
        assert fsm.snapshot is None

    def test_scoped_side_holds_snapshot_constant(self):
        # Only the left stream is fresh; the right side must stay at its
        # constant snapshot slice through track (A1).
        fsm = make_fsm()
        t = self._to_engaged(fsm)
        fsm.chains['left'].mark_input(t)
        ok, _ = fsm.request_approach()
        assert ok and fsm.active_sides == ['left']
        snapshot_right = fsm.snapshot[7:].copy()

        out = fsm.tick(hw_inputs(t + DT, stream={'left': np.zeros(7)}))
        fsm.request_track()
        cmds_right = []
        for i in range(5):
            now = t + (2 + i) * DT
            fsm.chains['left'].mark_input(now)
            out = fsm.tick(hw_inputs(
                now, q=out.cmd, stream={'left': np.full(7, 0.0004 * i)}))
            cmds_right.append(out.cmd[7:].copy())
        for c in cmds_right:
            np.testing.assert_array_equal(c, snapshot_right)


class TestFault:
    def _tracking(self, fsm):
        t = settle_ready(fsm)
        fsm.request_engage()
        fsm.tick(hw_inputs(t + 2.5))
        for side in ('left', 'right'):
            fsm.chains[side].mark_input(t + 2.5)
        fsm.request_approach()
        out = fsm.tick(hw_inputs(t + 2.5 + DT,
                                 stream={'left': np.zeros(7), 'right': np.zeros(7)}))
        fsm.request_track()
        return t + 2.5 + DT, out

    def test_fault_freezes_cmd_and_weight(self):
        fsm = make_fsm()
        t, out = self._tracking(fsm)
        fsm.fault('operator stop')
        frozen_cmd = fsm.cmd.copy()
        out = fsm.tick(hw_inputs(t + 1.0, q=np.full(14, 0.5)))
        np.testing.assert_array_equal(out.cmd, frozen_cmd)
        assert out.weight == 1.0
        assert fsm.state is DeviceState.FAULT

    def test_fault_mid_engage_freezes_partial_weight(self):
        fsm = make_fsm()
        t = settle_ready(fsm)
        fsm.request_engage()
        fsm.tick(hw_inputs(t + 1.0))            # ~half ramp
        w = fsm.weight
        fsm.fault('e-stop drill')
        out = fsm.tick(hw_inputs(t + 5.0))
        assert out.weight == pytest.approx(w)   # frozen, not ramped on

    def test_divergence_faults_from_track(self):
        fsm = make_fsm()
        t, out = self._tracking(fsm)
        # Measured pinned 1 rad away from command on the left arm.
        bad = np.zeros(14)
        bad[0] = 1.0
        for i in range(6):
            now = t + (1 + i) * DT
            for side in ('left', 'right'):
                fsm.chains[side].mark_input(now)
            out = fsm.tick(hw_inputs(now, q=bad,
                                     stream={'left': np.zeros(7), 'right': np.zeros(7)}))
        assert fsm.state is DeviceState.FAULT
        assert 'divergence' in fsm.fault_info['reason']

    def test_deescalation_park_release_clear(self):
        fsm = make_fsm()
        t, out = self._tracking(fsm)
        fsm.fault('drill')
        ok, msg = fsm.request_clear_fault()
        assert not ok and 'weight' in msg       # cannot clear while powered
        ok, _ = fsm.request_park()
        assert ok
        out = fsm.tick(hw_inputs(t + 1.0, q=fsm.snapshot.copy()))
        assert fsm._approach_done
        fsm.request_release()
        fsm.tick(hw_inputs(t + 4.0))
        out = fsm.tick(hw_inputs(t + 6.0))
        assert out.weight == 0.0
        # fault survives ready until explicitly cleared; motion refused.
        settle_ready(fsm, t0=t + 7.0)
        ok, msg = fsm.request_engage()
        assert not ok and 'fault latched' in msg
        ok, _ = fsm.request_clear_fault()
        assert ok and fsm.fault_info is None


class TestLowstateLoss:
    def test_loss_resets_to_ready(self):
        fsm = make_fsm()
        t = settle_ready(fsm)
        fsm.request_engage()
        fsm.tick(hw_inputs(t + 2.5))
        assert fsm.weight == 1.0
        out = fsm.tick(hw_inputs(t + 3.0, age=5.0))     # gap
        assert fsm.state is DeviceState.READY
        assert out.weight == 0.0
        assert fsm.snapshot is None
        # Fresh engage required, and the streak restarted.
        ok, _ = fsm.request_engage()
        assert not ok

    def test_sim_never_resets(self):
        fsm = make_fsm(sim=True)
        t = settle_ready(fsm)
        fsm.request_engage()
        fsm.tick(TickInputs(now=t + 2.5, measured_q=None, measured_dq=None,
                            lowstate_age=None, stream={}))
        assert fsm.state is DeviceState.ENGAGE


class TestStatusSchema:
    def test_keys(self):
        fsm = make_fsm()
        fsm.tick(hw_inputs(0.0))
        s = fsm.status()
        for key in ('fsm_state', 'weight', 'engage_done', 'approach_done',
                    'settled', 'max_target_error_rad', 'fault',
                    'active_sides', 'snapshot_present', 'fresh_streak'):
            assert key in s
