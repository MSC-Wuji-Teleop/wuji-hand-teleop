"""Supervisor core: load gates, Layer 3 detectors, arm sequence. ROS-free."""

import json

import pytest

from replay.run_gates import (
    ArmSeqState,
    ArmSequence,
    GateError,
    Layer3Monitor,
    MonitorConfig,
    RunState,
    SupervisorLoadRequest,
    check_load_gates,
    fault_actions,
    load_run_history,
    start_actions,
)


def meta(**kw):
    base = {
        'verdict': 'pass',
        'verdict_reasons': [],
        'max_allowed_speed_scale': 1.0,
        'hands_conditioned': True,
        'sample': '03_test_sample',
        'method': 'GT',
    }
    base.update(kw)
    return base


def req(**kw):
    base = {'clip': '/x.npz'}
    base.update(kw)
    return SupervisorLoadRequest.from_json(json.dumps(base))


class TestLoadGates:
    def test_clean_gt_passes(self):
        check_load_gates(req(), meta(), 'G1_29', run_history=[])

    def test_fail_verdict_refused(self):
        with pytest.raises(GateError, match='verdict'):
            check_load_gates(req(), meta(verdict='fail'), 'G1_29', [])

    def test_speed_gate_reads_artifact_field(self):
        with pytest.raises(GateError, match='max_allowed_speed_scale'):
            check_load_gates(req(speed_scale=1.0),
                             meta(max_allowed_speed_scale=0.5), 'G1_29', [])
        check_load_gates(req(speed_scale=0.5),
                         meta(max_allowed_speed_scale=0.5), 'G1_29', [])

    def test_rig_variant_gate(self):
        with pytest.raises(GateError, match='G1_29'):
            check_load_gates(req(), meta(), 'G1_23', [])

    def test_sample_01_banned_as_first_clip(self):
        m = meta(sample='01_test_x-fZc293MpJk_2-1-rgb_front')
        with pytest.raises(GateError, match='banned as the first clip'):
            check_load_gates(req(), m, 'G1_29', run_history=[])
        # A prior passing run of another sample unblocks it.
        history = [{'sample': '03_x', 'method': 'GT', 'pass': True,
                    'scope': {'arms': ['left', 'right'], 'hands': ['left', 'right']},
                    'speed_scale': 0.5}]
        check_load_gates(req(speed_scale=0.5), m, 'G1_29', history)
        # Or the explicit override.
        check_load_gates(req(override_first_clip=True), m, 'G1_29', [])

    def test_gt_before_ours(self):
        m = meta(method='Ours')
        with pytest.raises(GateError, match='GT-before-Ours'):
            check_load_gates(req(), m, 'G1_29', run_history=[])
        history = [{'sample': '03_test_sample', 'method': 'GT', 'pass': True,
                    'scope': {'arms': ['left', 'right'], 'hands': ['left', 'right']},
                    'speed_scale': 1.0}]
        check_load_gates(req(), m, 'G1_29', history)

    def test_gt_gate_matches_scope_and_scale(self):
        m = meta(method='Ours')
        history = [{'sample': '03_test_sample', 'method': 'GT', 'pass': True,
                    'scope': {'arms': ['left'], 'hands': []},
                    'speed_scale': 0.25}]
        # Wrong scope and lower scale: still gated.
        with pytest.raises(GateError, match='GT-before-Ours'):
            check_load_gates(req(speed_scale=0.5), m, 'G1_29', history)
        # Explicit override passes.
        check_load_gates(req(override_gt_gate=True), m, 'G1_29', history)

    def test_hand_scope_on_arm_only_artifact(self):
        with pytest.raises(GateError, match='arm-only'):
            check_load_gates(req(), meta(hands_conditioned=False), 'G1_29', [])
        check_load_gates(req(hands=[]), meta(hands_conditioned=False), 'G1_29', [])

    def test_run_history_loader(self, tmp_path):
        run = tmp_path / '20260828T000000_03_GT_0.5_full'
        run.mkdir()
        (run / 'tracking_summary.json').write_text(json.dumps({
            'sample': '03_test_sample', 'method': 'GT', 'pass': True,
            'scope': {'arms': ['left', 'right'], 'hands': []},
            'speed_scale': 0.5,
        }))
        history = load_run_history(tmp_path)
        assert len(history) == 1
        assert history[0]['pass'] is True


FULL_SCOPE = {'arms': ['left', 'right'], 'hands': ['left', 'right']}


def snap(age=0.1, **data):
    return {'age': age, 'data': data}


def healthy_snapshots(mode_machine=3):
    diag = {
        'error_codes': [0] * 20, 'enabled': [True] * 20,
        'joint_temperatures': [35.0] * 20, 'effort_limits': [1.5] * 20,
    }
    return {
        'replay': snap(state='running'),
        'g1': snap(mode_machine=mode_machine, max_motor_temp_c=40.0),
        'left_hand': snap(fsm_state='track', approach_done=True, fault=None),
        'right_hand': snap(fsm_state='track', approach_done=True, fault=None),
        'left_hand_diag': {'age': 0.1, 'data': dict(diag)},
        'right_hand_diag': {'age': 0.1, 'data': dict(diag)},
        'left_hand_effort': {'age': 0.05, 'data': [0.2] * 20},
        'right_hand_effort': {'age': 0.05, 'data': [0.2] * 20},
    }


class TestLayer3:
    def make(self):
        mon = Layer3Monitor(MonitorConfig(), FULL_SCOPE)
        mon.record_mode_machine(healthy_snapshots())
        return mon

    def test_healthy_no_faults(self):
        faults, warns = self.make().update(0.0, healthy_snapshots())
        assert faults == [] and warns == []

    def test_liveness(self):
        s = healthy_snapshots()
        s['g1']['age'] = 5.0
        faults, _ = self.make().update(0.0, s)
        assert any('liveness' in f and 'arm node' in f for f in faults)

    def test_mode_machine_change(self):
        faults, _ = self.make().update(0.0, healthy_snapshots(mode_machine=7))
        assert any('mode_machine changed' in f for f in faults)

    def test_hand_error_codes_and_offline(self):
        s = healthy_snapshots()
        s['left_hand_diag']['data']['error_codes'] = [0] * 5 + [9] + [0] * 14
        s['right_hand_diag']['data']['enabled'] = [True] * 19 + [False]
        faults, _ = self.make().update(0.0, s)
        assert any('error_codes' in f for f in faults)
        assert any('offline' in f for f in faults)

    def test_temperature_warn_then_trip(self):
        mon = self.make()
        s = healthy_snapshots()
        s['left_hand_diag']['data']['joint_temperatures'] = [55.0] * 20
        faults, warns = mon.update(0.0, s)
        assert faults == [] and any('warning' in w for w in warns)
        s['left_hand_diag']['data']['joint_temperatures'] = [70.0] * 20
        faults, _ = mon.update(1.0, s)
        assert any('over-temperature' in f for f in faults)

    def test_effort_saturation_needs_sustained(self):
        mon = self.make()
        s = healthy_snapshots()
        s['left_hand_effort']['data'] = [1.6] * 20  # over the 1.5 limit
        faults, _ = mon.update(0.0, s)
        assert faults == []                         # just started
        faults, _ = mon.update(0.5, s)
        assert faults == []                         # < 1 s
        faults, _ = mon.update(1.2, s)
        assert any('saturated' in f for f in faults)
        # Recovery resets the clock.
        s['left_hand_effort']['data'] = [0.1] * 20
        mon.update(1.3, s)
        s['left_hand_effort']['data'] = [1.6] * 20
        faults, _ = mon.update(1.4, s)
        assert faults == []


class TestArmSequence:
    def test_pinned_order_and_barrier(self):
        seq = ArmSequence(FULL_SCOPE, now=0.0)
        s = healthy_snapshots()
        s['replay']['data'] = {'state': 'loaded'}
        for dev in ('g1', 'left_hand', 'right_hand'):
            s[dev]['data'] = {'approach_done': False, 'engage_done': False,
                              'fault': None}

        acts = seq.step(0.1, s)
        assert acts == [('replay', 'publish_first')]
        assert seq.step(0.2, s) == []               # sent once, waiting

        s['replay']['data'] = {'state': 'first_frame'}
        acts = seq.step(0.3, s)
        assert ('g1', 'engage') in acts

        s['g1']['data']['engage_done'] = True
        acts = seq.step(0.4, s)
        assert set(acts) == {('g1', 'approach'), ('left_hand', 'approach'),
                             ('right_hand', 'approach')}

        for dev in ('g1', 'left_hand', 'right_hand'):
            s[dev]['data']['approach_done'] = True
        seq.step(0.5, s)
        assert seq.state is ArmSeqState.DONE

    def test_timeout_fails(self):
        seq = ArmSequence(FULL_SCOPE, now=0.0, timeout_s=1.0)
        s = healthy_snapshots()
        s['replay']['data'] = {'state': 'loaded'}
        seq.step(0.1, s)
        seq.step(2.0, s)
        assert seq.state is ArmSeqState.FAILED
        assert 'barrier timeout' in seq.reason

    def test_hands_only_scope_skips_engage(self):
        seq = ArmSequence({'arms': [], 'hands': ['left']}, now=0.0)
        s = healthy_snapshots()
        s['replay']['data'] = {'state': 'first_frame'}
        s['left_hand']['data'] = {'approach_done': False, 'fault': None}
        acts = seq.step(0.1, s)
        assert acts == [('left_hand', 'approach')]

    def test_faulted_device_blocks_barrier(self):
        seq = ArmSequence({'arms': [], 'hands': ['left']}, now=0.0)
        s = healthy_snapshots()
        s['replay']['data'] = {'state': 'first_frame'}
        s['left_hand']['data'] = {'approach_done': True,
                                  'fault': {'reason': 'x'}}
        seq.step(0.1, s)
        assert seq.state is ArmSeqState.BARRIER    # stuck, not DONE


class TestActions:
    def test_start_actions_order(self):
        acts = start_actions(FULL_SCOPE)
        # Devices first, pacer last: nothing advances until everyone tracks.
        assert acts[-1] == ('replay', 'start')
        assert ('g1', 'track') in acts

    def test_fault_fanout(self):
        acts = fault_actions({'arms': ['left'], 'hands': ['right']})
        assert ('replay', 'fault') in acts
        assert ('g1', 'fault') in acts
        assert ('right_hand', 'fault') in acts
