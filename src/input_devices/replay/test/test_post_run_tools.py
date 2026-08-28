"""make_artifacts pure computation + choose_first_clip classification."""

import json

import numpy as np
import pytest

from replay.choose_first_clip import classify_pair, evaluate_clip, rank_key
from replay.make_artifacts import (
    THRESHOLDS,
    summarize,
    tracking_stats,
    write_fault_log,
)


class TestTrackingStats:
    def test_perfect_tracking(self):
        t = np.arange(0, 5, 0.02)
        q = np.column_stack([np.sin(t), np.cos(t)])
        stats = tracking_stats(t, q, t, q)
        assert max(stats['rmse']) < 1e-12
        assert max(stats['max_error']) < 1e-12
        assert abs(stats['lag_s']) <= 0.01

    def test_constant_offset(self):
        t = np.arange(0, 5, 0.02)
        q = np.column_stack([np.sin(t)])
        stats = tracking_stats(t, q, t, q + 0.1)
        assert stats['rmse'][0] == pytest.approx(0.1, rel=1e-6)
        assert stats['max_error'][0] == pytest.approx(0.1, rel=1e-6)

    def test_lag_detected(self):
        t = np.arange(0, 10, 0.005)
        q = np.column_stack([np.sin(2 * np.pi * 0.5 * t)])
        lag = 0.1
        meas = np.column_stack([np.sin(2 * np.pi * 0.5 * (t - lag))])
        stats = tracking_stats(t, q, t, meas)
        assert stats['lag_s'] == pytest.approx(lag, abs=0.02)

    def test_disjoint_timelines(self):
        stats = tracking_stats(np.array([0.0, 1.0]), np.zeros((2, 1)),
                               np.array([5.0, 6.0]), np.zeros((2, 1)))
        assert stats['rmse'] is None


class TestSummarize:
    def _series(self, err=0.0):
        t = np.arange(0, 3, 0.02)
        q = np.column_stack([np.sin(t)] * 3)
        return {
            'left_arm': {'cmd_t': t, 'cmd_q': q, 'meas_t': t,
                         'meas_q': q + err, 'names': ['a', 'b', 'c']},
            'left_hand': {'cmd_t': t, 'cmd_q': q, 'meas_t': t,
                          'meas_q': q, 'names': None},
        }

    def test_clean_run_passes(self):
        s = summarize(self._series(), events=[], manifest={'sample': 'x'})
        assert s['pass'] is True
        assert s['checks']['zero_faults']

    def test_fault_event_fails(self):
        s = summarize(self._series(), events=[{'severity': 'fault'}],
                      manifest={})
        assert s['pass'] is False
        assert not s['checks']['zero_faults']

    def test_rmse_over_threshold_fails(self):
        s = summarize(self._series(err=THRESHOLDS['arm_rmse_rad'] * 2),
                      events=[], manifest={})
        assert not s['checks']['arm_rmse_ok']
        assert s['pass'] is False


class TestFaultLog:
    def test_filters_severities(self, tmp_path):
        events = [
            {'severity': 'info', 'event': 'load'},
            {'severity': 'warn', 'event': 'temp'},
            {'severity': 'fault', 'event': 'FAULT_HOLD'},
        ]
        out = tmp_path / 'fault_log.jsonl'
        n = write_fault_log(events, out)
        assert n == 2
        lines = [json.loads(x) for x in out.read_text().splitlines()]
        assert {e['severity'] for e in lines} == {'warn', 'fault'}


class TestClassifyPair:
    @pytest.mark.parametrize('pair,kind', [
        ('right_wrist_roll_link:right_wrist_yaw_link', 'same_arm_artifact'),
        ('right_wuji_r_palm:torso_link', 'hand_body'),
        ('left_wuji_l_thumb_distal:right_wuji_r_index_distal', 'two_hand'),
        ('right_shoulder_yaw_link:torso_link', 'arm_body'),
        ('left_elbow_link:left_shoulder_roll_link', 'other'),
    ])
    def test_kinds(self, pair, kind):
        assert classify_pair(pair) == kind


class TestEvaluateAndRank:
    def test_sample_01_never_eligible(self, tmp_path):
        art = tmp_path / 'clips' / '01_x_GT'
        art.mkdir(parents=True)
        j = art / 'conditioned_clip_v1.json'
        j.write_text(json.dumps({
            'sample': '01_x', 'method': 'GT', 'verdict': 'pass',
            'max_allowed_speed_scale': 1.0,
            'audit': {'arm': {'position_min': [0], 'position_max': [0.1],
                              'joint_names': [], 'spike_count': 0},
                      'k_extra': 1},
        }))
        (tmp_path / 'bundle' / 'samples').mkdir(parents=True)
        entry = evaluate_clip(j, tmp_path / 'bundle')
        assert entry['banned_first']
        assert not entry['eligible']

    def test_rank_prefers_low_contact_low_amplitude(self):
        a = {'eligible': True, 'contact_by_kind': {'hand_body': 1.0},
             'amplitude_rad': 0.5}
        b = {'eligible': True, 'contact_by_kind': {'hand_body': 30.0},
             'amplitude_rad': 0.1}
        c = {'eligible': False, 'contact_by_kind': {}, 'amplitude_rad': 0.0}
        ranked = sorted([c, b, a], key=rank_key)
        assert ranked[0] is a and ranked[-1] is c
