"""Offline hand retargeting for conditioning (spec_1 component 1, hands).

Per side: Retargeter.from_yaml(config, side), then reset() (TUITION 3.1:
clear warm-start and filter state before each clip), then step every
source-rate keypoint frame to q20, remap optimizer qpos order to device
order with the SAME permutation the live controller uses
(wujihand_output.wujihand_controller.build_qpos_perm), then PCHIP-retime
onto the arm frame grid so both devices share one timeline.

wuji_retargeting (NLopt, Pinocchio) is imported lazily inside
retarget_clip(), and the retargeter is injectable, so this module imports
-- and retime_to_grid() runs -- in a bare numpy/scipy venv. The real
retarget path needs the teleop container.
"""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from typing import Callable, Optional

import numpy as np

NUM_HAND_JOINTS = 20


def default_retargeter_factory(config_path: str, side: str):
    """Build the production Retargeter (lazy import; container-only)."""
    from wuji_retargeting import Retargeter  # NLopt/Pinocchio: teleop container
    return Retargeter.from_yaml(config_path, side)


def retarget_clip(
    keypoints: np.ndarray,
    side: str,
    config_path: str,
    retargeter_factory: Optional[Callable] = None,
) -> np.ndarray:
    """[Tsrc, 21, 3] human keypoints (meters, MediaPipe order) -> [Tsrc, 20]
    Hand 2 angles in device (URDF declaration / driver flat) order."""
    from wujihand_output.wujihand_controller import build_qpos_perm

    keypoints = np.asarray(keypoints, dtype=np.float32)
    if keypoints.ndim != 3 or keypoints.shape[1:] != (21, 3):
        raise ValueError(f"keypoints shape {keypoints.shape}, expected (T, 21, 3)")

    factory = retargeter_factory or default_retargeter_factory
    retargeter = factory(config_path, side)
    retargeter.reset()  # TUITION 3.1: required before each new clip
    perm = build_qpos_perm(retargeter, side)

    q = np.empty((keypoints.shape[0], NUM_HAND_JOINTS), dtype=np.float64)
    for t in range(keypoints.shape[0]):
        qt = np.asarray(retargeter.retarget(keypoints[t]), dtype=np.float64)
        if qt.shape != (NUM_HAND_JOINTS,):
            raise ValueError(
                f"{side} retarget returned shape {qt.shape} at frame {t}"
            )
        q[t] = qt if perm is None else qt[perm]
    return q


def retime_to_grid(
    q_src: np.ndarray,
    num_frames_target: int,
) -> np.ndarray:
    """PCHIP-retime [Tsrc, J] onto num_frames_target frames, same span.

    Both grids cover the same normalized time [0, 1] (the bundle guarantees
    a uniform resample between source and target rates; the publisher's old
    per-tick index mapping used the same normalization). PCHIP is
    shape-preserving: no overshoot beyond the source values, monotone
    segments stay monotone -- the property that makes it safe for joint
    trajectories (and it is the bundle's own retiming interpolant).
    """
    from scipy.interpolate import PchipInterpolator

    q_src = np.asarray(q_src, dtype=np.float64)
    t_src, _ = q_src.shape
    if t_src < 2:
        return np.repeat(q_src, num_frames_target, axis=0)
    u_src = np.linspace(0.0, 1.0, t_src)
    u_dst = np.linspace(0.0, 1.0, num_frames_target)
    return PchipInterpolator(u_src, q_src, axis=0)(u_dst)


def sha256_file(path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def retargeter_provenance(config_path: str, retargeting_dir: Optional[str] = None) -> dict:
    """Submodule commit + config hash + hand model id for the artifact JSON.

    Deterministic for fixed inputs: the commit is a repo fact, the hash is
    of the config bytes, the model id is the URDF filename the optimizer
    fits. Missing pieces are recorded as None, never guessed.
    """
    config_path = Path(config_path)
    commit = None
    search = Path(retargeting_dir) if retargeting_dir else _find_retargeting_dir(config_path)
    if search is not None:
        try:
            commit = subprocess.run(
                ['git', '-C', str(search), 'rev-parse', 'HEAD'],
                capture_output=True, text=True, timeout=10, check=True,
            ).stdout.strip()
        except (subprocess.SubprocessError, FileNotFoundError):
            commit = None

    hand_model = None
    try:
        import yaml
        cfg = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
        rel = (cfg.get('optimizer') or {}).get('urdf_path')
        if rel:
            hand_model = Path(rel).name
    except Exception:
        hand_model = None

    return {
        'config_path': str(config_path),
        'config_sha256': sha256_file(config_path),
        'submodule_commit': commit,
        'hand_model': hand_model,
    }


def _find_retargeting_dir(start: Path) -> Optional[Path]:
    for parent in [start, *start.parents]:
        candidate = parent / 'wuji-retargeting'
        if candidate.is_dir():
            return candidate
        if (parent / 'src' / 'wuji-retargeting').is_dir():
            return parent / 'src' / 'wuji-retargeting'
    return None
