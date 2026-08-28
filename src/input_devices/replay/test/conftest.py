"""Shared paths + synthetic bundle fixture for the replay package tests.

Cross-package imports (g1_world_output.replay_safety, wujihand_output)
resolve via the source tree here; inside the container the built workspace
provides them instead.
"""

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

_SRC = Path(__file__).resolve().parents[3]  # .../src
for pkg in ('input_devices/replay',
            'output_devices/g1_world_output',
            'output_devices/wujihand_output'):
    p = str(_SRC / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

from replay.clip_artifact import CANONICAL_ARM_JOINTS  # noqa: E402

LEG_NAMES = [
    'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee',
    'left_ankle_pitch', 'left_ankle_roll',
    'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee',
    'right_ankle_pitch', 'right_ankle_roll',
]
WAIST_NAMES = ['waist_yaw', 'waist_roll', 'waist_pitch']
BODY_ACTUATORS = LEG_NAMES + WAIST_NAMES + CANONICAL_ARM_JOINTS  # 29, bundle order

ARM_LIMITS_PATH = (_SRC / 'output_devices/g1_world_output/config'
                   / 'g1_deploy_limits.yaml')
HAND_LIMITS_PATH = (_SRC / 'output_devices/wujihand_output/config'
                    / 'hand_limits.yaml')


def make_bundle_sample(
    root: Path,
    sample: str = '03_test_sample',
    method: str = 'GT',
    frames: int = 100,
    source_frames: int = 40,
    target_fps: float = 50.0,
    source_fps: float = 20.0,
    arm_amplitude: float = 0.3,
    arm_freq_hz: float = 0.2,
    spike: dict = None,
    corrupt_manifest: bool = False,
) -> Path:
    """Write a minimal-but-faithful bundle sample tree; returns method_dir.

    Default motion: slow sinusoid on all 14 arm joints, peak velocity
    ~0.38 rad/s (inside the 0.5 rad/s deploy screening row -> k stays 1).
    spike={'joint': name, 'frame': i, 'dq': rad} injects a single-frame jump.
    """
    method_dir = root / 'samples' / sample / method
    (method_dir / 'g1_reference').mkdir(parents=True, exist_ok=True)
    (method_dir / 'hand2_input').mkdir(parents=True, exist_ok=True)

    t = np.arange(frames) / target_fps
    body_q = np.zeros((frames, len(BODY_ACTUATORS)))
    for i, name in enumerate(CANONICAL_ARM_JOINTS):
        col = BODY_ACTUATORS.index(name)
        body_q[:, col] = arm_amplitude * np.sin(
            2 * np.pi * arm_freq_hz * t + 0.3 * i
        )
    if spike:
        col = BODY_ACTUATORS.index(spike['joint'])
        body_q[spike['frame']:, col] += spike['dq']

    npz_path = method_dir / 'g1_reference' / 'controller_reference_v7.npz'
    np.savez(
        npz_path,
        body_q=body_q.astype(np.float32),
        target_fps=np.float32(target_fps),
        time_scale=np.int32(1),
    )

    meta = {
        'frames': frames,
        'source_frames': source_frames,
        'source_fps': source_fps,
        'target_fps': target_fps,
        'time_scale': 1,
        'end_behavior': 'hold_last_target',
        'joint_actuator_order': {'body_actuators': BODY_ACTUATORS},
    }
    meta_path = method_dir / 'g1_reference' / 'target_meta.json'
    meta_path.write_text(json.dumps(meta, indent=1))

    rng = np.random.default_rng(7)
    kp = 0.1 * rng.standard_normal((source_frames, 21, 3)).astype(np.float32)
    kp_path = method_dir / 'hand2_input' / f'{method.lower()}_human_targets_v5.npz'
    np.savez(kp_path,
             left_hand_keypoints21=kp,
             right_hand_keypoints21=kp + 0.01)

    lines = []
    for p in (meta_path, npz_path, kp_path):
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        if corrupt_manifest and p is npz_path:
            digest = '0' * 64
        lines.append(f"{digest}  {p.relative_to(root).as_posix()}")
    (root / 'MANIFEST.sha256').write_text('\n'.join(lines) + '\n')
    return method_dir


class MockRetargeter:
    """Stands in for wuji_retargeting.Retargeter in venv tests.

    Deterministic keypoints -> q20 map with values inside the hand limits;
    config carries no optimizer.urdf_path, so build_qpos_perm returns None
    (identity), which matches the Hand 1 fallback path.
    """

    def __init__(self):
        self.config = {}
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1

    def retarget(self, kp):
        kp = np.asarray(kp, dtype=np.float64)
        base = 0.2 + 0.2 * np.tanh(kp.mean() * np.arange(1, 21) / 20.0)
        return base


@pytest.fixture
def mock_retargeter_factory():
    made = []

    def factory(config_path, side):
        r = MockRetargeter()
        made.append((side, r))
        return r

    factory.made = made
    return factory
