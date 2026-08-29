"""Hand FSM: traversal, watchdog faults, hold semantics. ROS-free."""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wujihand_output.hand_fsm import (  # noqa: E402
    HandDeviceFSM,
    HandFsmConfig,
    HandState,
    HandTickInputs,
)
from wujihand_output.hand_safety import NUM_JOINTS, HandLimits  # noqa: E402

LIMITS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'hand_limits.yaml'
DT = 1.0 / 200.0


def make_fsm(require_feedback=True, **kw):
    limits = HandLimits.from_yaml(LIMITS_PATH)
    cfg = HandFsmConfig(control_dt=DT, require_feedback=require_feedback, **kw)
    return HandDeviceFSM(limits, cfg)


def good_diag():
    return {
        'error_codes': [0] * NUM_JOINTS,
        'enabled': [True] * NUM_JOINTS,
        'joint_temperatures': [30.0] * NUM_JOINTS,
        'effort_limits': [1.5] * NUM_JOINTS,
    }


def inputs(now, q=None, effort=None, state_age=0.001, diag=None, diag_age=0.01,
           stream=None):
    return HandTickInputs(
        now=now,
        measured_q=np.full(NUM_JOINTS, 0.1) if q is None else q,
        measured_effort=np.zeros(NUM_JOINTS) if effort is None else effort,
        state_age=state_age,
        diagnostics=good_diag() if diag is None else diag,
        diagnostics_age=diag_age,
        stream=stream,
    )


class TestTraversal:
    def test_hold_publishes_nothing_until_commanded(self):
        fsm = make_fsm()
        out = fsm.tick(inputs(0.0))
        assert out.cmd is None
        assert fsm.state is HandState.HOLD

    def test_approach_gate_needs_fresh_stream(self):
        fsm = make_fsm()
        fsm.tick(inputs(0.0))
        ok, msg = fsm.request_approach()
        assert not ok and 'publish_first' in msg

    def test_full_traversal(self):
        fsm = make_fsm()
        target = np.full(NUM_JOINTS, 0.1)
        fsm.tick(inputs(0.0, q=target))
        fsm.mark_target_input(0.0)
        ok, _ = fsm.request_approach()
        assert ok
        out = fsm.tick(inputs(DT, q=target, stream=target))
        assert fsm._approach_done      # measured == target
        np.testing.assert_allclose(out.cmd, target)
        ok, _ = fsm.request_track()
        assert ok and fsm.state is HandState.TRACK

        # Track a slowly moving stream.
        for i in range(5):
            now = (2 + i) * DT
            fsm.mark_target_input(now)
            tgt = target + 0.001 * i
            out = fsm.tick(inputs(now, q=out.cmd, stream=tgt))
        ok, _ = fsm.request_end_hold()
        assert ok
        frozen = out.cmd.copy()
        out = fsm.tick(inputs(1.0, q=np.zeros(NUM_JOINTS)))
        np.testing.assert_array_equal(out.cmd, frozen)   # frozen, not measured

        ok, _ = fsm.request_park()
        assert ok and fsm.approach_target_kind == 'neutral'
        # Slew toward neutral under deploy limits.
        out = fsm.tick(inputs(1.0 + DT, q=frozen))
        step = np.abs(out.cmd - frozen)
        assert np.max(step) <= 4.0 * DT + 1e-9   # 4.0 rad/s cap

    def test_park_completion_returns_to_hold(self):
        # A parked hand must be re-approachable for the next clip: staying
        # in approach would refuse the next run and stall its barrier.
        fsm = make_fsm()
        target = np.full(NUM_JOINTS, 0.1)
        fsm.tick(inputs(0.0, q=target))
        fsm.mark_target_input(0.0)
        fsm.request_approach()
        out = fsm.tick(inputs(DT, q=target, stream=target))
        fsm.request_track()
        fsm.request_end_hold()
        ok, _ = fsm.request_park()
        assert ok
        for i in range(3000):
            now = 1.0 + i * DT
            out = fsm.tick(inputs(now, q=out.cmd))
            if fsm.state is HandState.HOLD:
                break
        assert fsm.state is HandState.HOLD
        np.testing.assert_allclose(out.cmd, 0.0, atol=0.05)   # neutral
        # And the next run can approach again.
        fsm.mark_target_input(now)
        ok, msg = fsm.request_approach()
        assert ok, msg

    def test_track_holds_on_stale_stream(self):
        fsm = make_fsm()
        target = np.full(NUM_JOINTS, 0.1)
        fsm.tick(inputs(0.0, q=target))
        fsm.mark_target_input(0.0)
        fsm.request_approach()
        out = fsm.tick(inputs(DT, q=target, stream=target))
        fsm.request_track()
        held = out.cmd.copy()
        # No mark_target_input for a long gap -> stale -> hold.
        out = fsm.tick(inputs(5.0, q=target, stream=np.zeros(NUM_JOINTS)))
        np.testing.assert_array_equal(out.cmd, held)

    def test_rate_limited_toward_target(self):
        fsm = make_fsm()
        limits = HandLimits.from_yaml(LIMITS_PATH)
        start = np.zeros(NUM_JOINTS)
        fsm.tick(inputs(0.0, q=start))
        fsm.mark_target_input(0.0)
        fsm.request_approach()
        far = np.full(NUM_JOINTS, 1.0)
        out = fsm.tick(inputs(DT, q=start, stream=far))
        # Uniform scaling: the fastest joint sits exactly at the 4.0 rad/s
        # deploy cap; joints whose targets were position-clamped lower
        # (abductions at 0.698) step proportionally less, preserving the
        # path direction.
        clamped_target = np.clip(far, limits.pos_lower, limits.pos_upper)
        assert np.max(out.cmd) == pytest.approx(4.0 * DT)
        np.testing.assert_allclose(out.cmd / np.max(out.cmd),
                                   clamped_target / np.max(clamped_target))
        assert not fsm._approach_done

    def test_position_clamp_on_targets(self):
        fsm = make_fsm()
        fsm.tick(inputs(0.0, q=np.zeros(NUM_JOINTS)))
        fsm.mark_target_input(0.0)
        fsm.request_approach()
        crazy = np.full(NUM_JOINTS, 3.0)   # beyond every upper bound
        limits = HandLimits.from_yaml(LIMITS_PATH)
        for i in range(3000):
            now = (1 + i) * DT
            fsm.mark_target_input(now)
            out = fsm.tick(inputs(now, q=out.cmd if i else np.zeros(NUM_JOINTS),
                                  stream=crazy))
        assert np.all(out.cmd <= limits.pos_upper + 1e-9)


class TestWatchdogs:
    def _tracking(self, fsm):
        target = np.full(NUM_JOINTS, 0.1)
        fsm.tick(inputs(0.0, q=target))
        fsm.mark_target_input(0.0)
        fsm.request_approach()
        out = fsm.tick(inputs(DT, q=target, stream=target))
        fsm.request_track()
        return out

    @pytest.mark.parametrize('kw,phrase', [
        (dict(state_age=5.0), 'joint_states stale'),
        (dict(diag_age=10.0), 'hand_diagnostics stale'),
        (dict(diag={**good_diag(), 'error_codes': [0] * 7 + [3] + [0] * 12}),
         'error_codes'),
        (dict(diag={**good_diag(), 'enabled': [True] * 19 + [False]}),
         'offline'),
        (dict(diag={**good_diag(), 'joint_temperatures': [30.0] * 19 + [80.0]}),
         'over-temperature'),
    ])
    def test_watchdog_faults(self, kw, phrase):
        fsm = make_fsm()
        out = self._tracking(fsm)
        held = out.cmd.copy()
        out = fsm.tick(inputs(1.0, **kw))
        assert fsm.state is HandState.FAULT
        assert phrase in fsm.fault_info['reason']
        np.testing.assert_array_equal(out.cmd, held)   # hold, never zero

    def test_sustained_overcurrent_faults(self):
        fsm = make_fsm(effort_guard_ticks=3)
        self._tracking(fsm)
        hot = np.full(NUM_JOINTS, 2.5)                 # > 1.5 A limit
        for i in range(4):
            fsm.tick(inputs(1.0 + i * DT, effort=hot))
        assert fsm.state is HandState.FAULT
        assert 'over-current' in fsm.fault_info['reason']

    def test_transient_current_spike_ok(self):
        fsm = make_fsm(effort_guard_ticks=3)
        self._tracking(fsm)
        hot = np.full(NUM_JOINTS, 2.5)
        fsm.tick(inputs(1.0, effort=hot))
        fsm.tick(inputs(1.005, effort=np.zeros(NUM_JOINTS)))
        fsm.tick(inputs(1.01, effort=hot))
        assert fsm.state is not HandState.FAULT

    def test_sim_mode_disables_watchdogs(self):
        fsm = make_fsm(require_feedback=False)
        fsm.tick(HandTickInputs(now=0.0, measured_q=None, measured_effort=None,
                                state_age=None, diagnostics=None,
                                diagnostics_age=None, stream=None))
        assert fsm.state is HandState.HOLD

    def test_clear_fault_returns_to_hold(self):
        fsm = make_fsm()
        self._tracking(fsm)
        fsm.fault('drill')
        ok, _ = fsm.request_clear_fault()
        assert ok and fsm.state is HandState.HOLD
        assert fsm.fault_info is None


class TestSimTraversal:
    def test_traversal_without_feedback(self):
        # Stage 0: hand FSM drives MuJoCo with no driver in the loop.
        fsm = make_fsm(require_feedback=False)
        fsm.tick(HandTickInputs(0.0, None, None, None, None, None, None))
        fsm.mark_target_input(0.0)
        ok, msg = fsm.request_approach()
        assert ok, msg
        target = np.full(NUM_JOINTS, 0.2)
        out = None
        for i in range(2000):
            now = (1 + i) * DT
            fsm.mark_target_input(now)
            out = fsm.tick(HandTickInputs(now, None, None, None, None, None,
                                          stream=target))
            if fsm._approach_done:
                break
        assert fsm._approach_done
        np.testing.assert_allclose(out.cmd, target, atol=0.05)
        ok, _ = fsm.request_track()
        assert ok


class TestStatusSchema:
    def test_keys(self):
        fsm = make_fsm()
        fsm.tick(inputs(0.0))
        s = fsm.status()
        for key in ('fsm_state', 'approach_done', 'max_target_error_rad',
                    'fault', 'target_age_s', 'state_age_s',
                    'diagnostics_age_s', 'effort_guard_active'):
            assert key in s
