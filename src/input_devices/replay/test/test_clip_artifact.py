"""Artifact schema: roundtrip, validation, determinism."""

import hashlib

import numpy as np
import pytest

from replay.clip_artifact import (
    ArtifactError,
    CANONICAL_ARM_JOINTS,
    load_artifact,
    save_artifact,
    synthetic_artifact,
)


class TestRoundtrip:
    def test_synthetic_roundtrip(self, tmp_path):
        npz, js = synthetic_artifact(tmp_path / 'clip')
        clip = load_artifact(npz)
        assert clip.num_frames == 50
        assert clip.k == 1
        assert clip.target_fps == 50.0
        assert clip.verdict == 'pass'
        assert clip.arm_joint_names == CANONICAL_ARM_JOINTS
        assert clip.left_hand_q20.shape == (50, 20)
        assert clip.max_allowed_speed_scale == 1.0
        assert clip.hands_conditioned

    def test_load_by_basename_and_json(self, tmp_path):
        synthetic_artifact(tmp_path / 'clip')
        assert load_artifact(tmp_path / 'clip').num_frames == 50
        assert load_artifact(tmp_path / 'clip.json').num_frames == 50

    def test_dt_play(self, tmp_path):
        synthetic_artifact(tmp_path / 'clip', k=3)
        clip = load_artifact(tmp_path / 'clip')
        assert clip.dt_play(1.0) == pytest.approx(3 / 50.0)
        assert clip.dt_play(0.5) == pytest.approx(6 / 50.0)
        with pytest.raises(ValueError):
            clip.dt_play(0.0)


class TestValidation:
    def test_missing_sidecar(self, tmp_path):
        npz, js = synthetic_artifact(tmp_path / 'clip')
        js.unlink()
        with pytest.raises(ArtifactError, match='missing'):
            load_artifact(npz)

    def test_bad_verdict_rejected_on_save(self, tmp_path):
        with pytest.raises(ArtifactError, match='verdict'):
            synthetic_artifact(tmp_path / 'clip', verdict='maybe')

    def test_shape_mismatch_rejected(self, tmp_path):
        t = 10
        arm = np.zeros((t, 14))
        hand = np.zeros((t + 1, 20))  # wrong T
        meta = {'schema_version': 1, 'verdict': 'pass',
                'max_allowed_speed_scale': 1.0, 'audit': {}}
        with pytest.raises(ArtifactError, match='hand_q20'):
            save_artifact(tmp_path / 'c', arm, hand, np.zeros((t, 20)), 50.0, 1, meta)

    def test_nonfinite_rejected_on_load(self, tmp_path):
        npz, js = synthetic_artifact(tmp_path / 'clip')
        data = dict(np.load(npz))
        data['arm_q'][3, 2] = np.nan
        np.savez(npz, **data)
        with pytest.raises(ArtifactError, match='non-finite'):
            load_artifact(npz)

    def test_wrong_names_rejected(self, tmp_path):
        npz, js = synthetic_artifact(tmp_path / 'clip')
        data = dict(np.load(npz))
        names = [str(n) for n in data['arm_joint_names']]
        names[0] = 'left_shoulder_pinch'
        data['arm_joint_names'] = np.array(names)
        np.savez(npz, **data)
        with pytest.raises(ArtifactError, match='canonical'):
            load_artifact(npz)


class TestDeterminism:
    def test_same_inputs_same_bytes(self, tmp_path):
        a = synthetic_artifact(tmp_path / 'a' / 'clip')
        b = synthetic_artifact(tmp_path / 'b' / 'clip')
        for pa, pb in zip(a, b):
            ha = hashlib.sha256(pa.read_bytes()).hexdigest()
            hb = hashlib.sha256(pb.read_bytes()).hexdigest()
            assert ha == hb, f"{pa.name} not deterministic"
