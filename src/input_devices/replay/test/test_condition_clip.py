"""End-to-end conditioning on a synthetic bundle (mock retargeter)."""

import hashlib
import json

import numpy as np
import pytest

from conftest import (
    ARM_LIMITS_PATH,
    HAND_LIMITS_PATH,
    make_bundle_sample,
)

from replay.clip_artifact import load_artifact
from replay.condition_clip import (
    condition_bundle_sample,
    condition_single_joint,
)
from replay.hand_pipeline import retime_to_grid


def _condition(tmp_path, method_dir, factory, **kw):
    return condition_bundle_sample(
        method_dir, tmp_path / 'out', ARM_LIMITS_PATH, HAND_LIMITS_PATH,
        k_max=kw.pop('k_max', 8), hands=kw.pop('hands', True),
        retarget_configs={'left': ARM_LIMITS_PATH, 'right': ARM_LIMITS_PATH},
        retargeter_factory=factory, **kw,
    )
    # retarget_configs point at any existing yaml: the mock factory ignores
    # the path, and provenance hashing just needs a real file.


class TestBundleMode:
    def test_clean_sample_passes(self, tmp_path, mock_retargeter_factory):
        method_dir = make_bundle_sample(tmp_path / 'bundle')
        base, verdict = _condition(tmp_path, method_dir, mock_retargeter_factory)
        assert verdict == 'pass'

        clip = load_artifact(base)
        assert clip.num_frames == 100
        assert clip.k == 1
        assert clip.target_fps == 50.0
        # Hands were retimed from 40 source frames onto the 100-frame grid.
        assert clip.left_hand_q20.shape == (100, 20)
        meta = clip.meta
        assert meta['verdict_reasons'] == []
        assert meta['audit']['k_extra'] == 1
        assert 0 < meta['max_allowed_speed_scale'] <= 1.0
        # reset() called once per side (TUITION 3.1).
        sides = dict(mock_retargeter_factory.made)
        assert sides['left'].reset_calls == 1
        assert sides['right'].reset_calls == 1
        # All bundle inputs verified against the manifest.
        matches = [v['manifest_match'] for k, v in meta['input_sha256'].items()
                   if not k.endswith('.yaml')]
        assert matches and all(m is True for m in matches)

    def test_spiky_sample_fails(self, tmp_path, mock_retargeter_factory):
        method_dir = make_bundle_sample(
            tmp_path / 'bundle',
            spike={'joint': 'left_wrist_pitch', 'frame': 50, 'dq': 3.23},
        )
        base, verdict = _condition(tmp_path, method_dir, mock_retargeter_factory)
        assert verdict == 'fail'
        meta = json.loads(base.with_suffix('.json').read_text())
        assert any('retiming does not fix' in r for r in meta['verdict_reasons'])
        assert meta['audit']['arm']['spike_count'] >= 1

    def test_fast_sample_gets_k(self, tmp_path, mock_retargeter_factory):
        # 1.0 rad at 0.5 Hz: sustained ~3 rad/s vs 0.5 deploy -> k_extra > 1,
        # no ceiling spikes -> still a pass.
        method_dir = make_bundle_sample(
            tmp_path / 'bundle', frames=500, source_frames=200,
            arm_amplitude=1.0, arm_freq_hz=0.5,
        )
        base, verdict = _condition(tmp_path, method_dir, mock_retargeter_factory)
        assert verdict == 'pass'
        clip = load_artifact(base)
        assert clip.k >= 5
        assert clip.meta['audit']['k_extra'] == clip.k  # bundle k = 1
        # dt_play stretched accordingly.
        assert clip.dt_play(1.0) == pytest.approx(clip.k / 50.0)

    def test_manifest_corruption_fails(self, tmp_path, mock_retargeter_factory):
        method_dir = make_bundle_sample(tmp_path / 'bundle', corrupt_manifest=True)
        base, verdict = _condition(tmp_path, method_dir, mock_retargeter_factory)
        assert verdict == 'fail'
        meta = json.loads(base.with_suffix('.json').read_text())
        assert any('hash mismatch' in r for r in meta['verdict_reasons'])

    def test_no_hands_mode(self, tmp_path, mock_retargeter_factory):
        method_dir = make_bundle_sample(tmp_path / 'bundle')
        base, verdict = _condition(tmp_path, method_dir, None, hands=False)
        assert verdict == 'pass'
        clip = load_artifact(base)
        assert not clip.hands_conditioned
        assert np.all(clip.left_hand_q20 == 0)

    def test_determinism_across_runs(self, tmp_path, mock_retargeter_factory):
        bundle = tmp_path / 'bundle'
        method_dir = make_bundle_sample(bundle)
        base1, _ = condition_bundle_sample(
            method_dir, tmp_path / 'out1', ARM_LIMITS_PATH, HAND_LIMITS_PATH,
            k_max=8, hands=True,
            retarget_configs={'left': ARM_LIMITS_PATH, 'right': ARM_LIMITS_PATH},
            retargeter_factory=lambda c, s: __import__('conftest').MockRetargeter(),
        )
        base2, _ = condition_bundle_sample(
            method_dir, tmp_path / 'out2', ARM_LIMITS_PATH, HAND_LIMITS_PATH,
            k_max=8, hands=True,
            retarget_configs={'left': ARM_LIMITS_PATH, 'right': ARM_LIMITS_PATH},
            retargeter_factory=lambda c, s: __import__('conftest').MockRetargeter(),
        )
        h1 = hashlib.sha256(base1.with_suffix('.npz').read_bytes()).hexdigest()
        h2 = hashlib.sha256(base2.with_suffix('.npz').read_bytes()).hexdigest()
        assert h1 == h2
        # JSON differs only in the out-dir-independent fields; compare with
        # the path-bearing fields normalized.
        m1 = json.loads(base1.with_suffix('.json').read_text())
        m2 = json.loads(base2.with_suffix('.json').read_text())
        assert m1 == m2


class TestRetime:
    def test_endpoint_preservation(self):
        q = np.linspace([0.0, 1.0], [1.0, 0.0], 40)
        out = retime_to_grid(q, 100)
        assert out.shape == (100, 2)
        np.testing.assert_allclose(out[0], q[0], atol=1e-12)
        np.testing.assert_allclose(out[-1], q[-1], atol=1e-12)

    def test_pchip_no_overshoot(self):
        # Shape preservation: a monotone source stays inside its range.
        q = np.concatenate([np.zeros(5), np.ones(5)])[:, None]
        out = retime_to_grid(q, 50)
        assert out.min() >= -1e-12
        assert out.max() <= 1.0 + 1e-12


class TestSingleJoint:
    def test_arm_single_joint_passes(self, tmp_path):
        base, verdict = condition_single_joint(
            'arm:left_elbow', 0.2, tmp_path / 'out',
            ARM_LIMITS_PATH, HAND_LIMITS_PATH,
        )
        assert verdict == 'pass'
        clip = load_artifact(base)
        j = clip.arm_joint_names.index('left_elbow')
        moving = np.abs(clip.arm_q).max(axis=0)
        assert moving[j] == pytest.approx(0.2, rel=1e-2)
        assert np.all(np.delete(moving, j) == 0)
        assert np.all(clip.left_hand_q20 == 0)
        assert clip.meta['scope_hint'] == {'arms': ['left'], 'hands': []}

    def test_hand_single_joint_passes(self, tmp_path):
        base, verdict = condition_single_joint(
            'right_hand:index_finger_mcp_flex', 0.3, tmp_path / 'out',
            ARM_LIMITS_PATH, HAND_LIMITS_PATH,
        )
        assert verdict == 'pass'
        clip = load_artifact(base)
        assert np.abs(clip.right_hand_q20).max() == pytest.approx(0.3, rel=1e-2)
        assert np.all(clip.left_hand_q20 == 0)
        assert np.all(clip.arm_q == 0)
        assert clip.meta['scope_hint'] == {'arms': [], 'hands': ['right']}

    def test_unknown_joint_rejected(self, tmp_path):
        with pytest.raises(ValueError, match='unknown arm joint'):
            condition_single_joint('arm:left_pinky', 0.2, tmp_path / 'out',
                                   ARM_LIMITS_PATH, HAND_LIMITS_PATH)
