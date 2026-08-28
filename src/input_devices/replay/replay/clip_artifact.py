"""Conditioned clip artifact: schema, writer, loader, validator.

The artifact is the only thing that crosses from offline conditioning into
the runtime pipeline (spec_1 component 1 -> component 2). One clip is two
files with a shared basename:

    <name>.npz    arm_q [T,14], arm_joint_names [14], left_hand_q20 [T,20],
                  right_hand_q20 [T,20], target_fps (scalar), k (scalar int)
    <name>.json   provenance, audit, verdict, max_allowed_speed_scale, poses

Field-level contract: docs/spec/spec_1_interfaces.md. Determinism: no
wall-clock fields; same inputs produce byte-identical outputs (JSON is
sorted-key, npz arrays are written in a fixed order).

This module is pure numpy + stdlib so the publisher, the supervisor, and
tests all load artifacts through one code path.
"""

from __future__ import annotations

import io
import json
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

SCHEMA_VERSION = 1
NUM_ARM_JOINTS = 14
NUM_HAND_JOINTS = 20

# The rig's arm joints, robot_arm.py G1_29 order (left 7 then right 7),
# unsuffixed names. Kept here verbatim rather than imported so the replay
# package stays runnable in containers where g1_world_output is not built.
CANONICAL_ARM_JOINTS = [
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
    'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
    'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
]

# Hand 2 joint-name suffixes in URDF declaration order (thumb to pinky,
# [flex, abd, pip/mcp, dip/ip] per finger) -- the flat 20-element device
# convention. Must match wujihand_output/config/hand_limits.yaml's name
# rows (cross-checked by a test); the side prefix is l_ / r_.
CANONICAL_HAND_JOINTS = [
    'thumb_cmc_flex', 'thumb_cmc_abd', 'thumb_mcp', 'thumb_ip',
    'index_finger_mcp_flex', 'index_finger_mcp_abd', 'index_finger_pip',
    'index_finger_dip',
    'middle_finger_mcp_flex', 'middle_finger_mcp_abd', 'middle_finger_pip',
    'middle_finger_dip',
    'ring_finger_mcp_flex', 'ring_finger_mcp_abd', 'ring_finger_pip',
    'ring_finger_dip',
    'pinky_mcp_flex', 'pinky_mcp_abd', 'pinky_pip', 'pinky_dip',
]

HAND_SIDE_PREFIX = {'left': 'l_', 'right': 'r_'}


def hand_joint_names(side: str) -> list:
    """Side-prefixed Hand 2 joint names, device order."""
    prefix = HAND_SIDE_PREFIX[side]
    return [prefix + n for n in CANONICAL_HAND_JOINTS]

VERDICT_PASS = 'pass'
VERDICT_FAIL = 'fail'

# Samples banned as the FIRST hardware clip (TUITION 7F): sample 01's
# physical audit records a shoulder-torso contact and fails its own
# deployment gate. Grows if Stage A audits ban more samples; the load
# gate (run_gates) and the offline chooser (choose_first_clip) both
# read THIS table so they can never disagree.
BANNED_FIRST_SAMPLE_PREFIXES = ('01_',)


class ArtifactError(ValueError):
    """Artifact is missing, malformed, or fails validation."""


@dataclass
class ConditionedClip:
    """One loaded artifact, validated."""

    arm_q: np.ndarray                 # [T, 14]
    arm_joint_names: list             # 14 canonical names
    left_hand_q20: np.ndarray         # [T, 20]
    right_hand_q20: np.ndarray        # [T, 20]
    target_fps: float
    k: int
    meta: dict                        # the parsed JSON sidecar
    npz_path: Path
    json_path: Path

    @property
    def num_frames(self) -> int:
        return int(self.arm_q.shape[0])

    @property
    def verdict(self) -> str:
        return self.meta['verdict']

    @property
    def max_allowed_speed_scale(self) -> float:
        return float(self.meta['max_allowed_speed_scale'])

    @property
    def hands_conditioned(self) -> bool:
        return bool(self.meta.get('hands_conditioned', True))

    def dt_play(self, speed_scale: float) -> float:
        """Tick interval: dt_play = k / (target_fps * speed_scale)."""
        if speed_scale <= 0:
            raise ValueError(f"speed_scale must be > 0, got {speed_scale}")
        return self.k / (self.target_fps * speed_scale)


def save_artifact(
    out_base: Path,
    arm_q: np.ndarray,
    left_hand_q20: np.ndarray,
    right_hand_q20: np.ndarray,
    target_fps: float,
    k: int,
    meta: dict,
) -> tuple:
    """Write <out_base>.npz + <out_base>.json. Returns the two paths.

    meta must already carry schema_version, verdict, audit,
    max_allowed_speed_scale, etc. (built by conditioning); this function
    only enforces shape/type invariants and deterministic serialization.
    """
    out_base = Path(out_base)
    arm_q = np.ascontiguousarray(arm_q, dtype=np.float64)
    left_hand_q20 = np.ascontiguousarray(left_hand_q20, dtype=np.float64)
    right_hand_q20 = np.ascontiguousarray(right_hand_q20, dtype=np.float64)

    t = arm_q.shape[0]
    if arm_q.shape != (t, NUM_ARM_JOINTS):
        raise ArtifactError(f"arm_q shape {arm_q.shape}, expected (T, {NUM_ARM_JOINTS})")
    for side, q in (('left', left_hand_q20), ('right', right_hand_q20)):
        if q.shape != (t, NUM_HAND_JOINTS):
            raise ArtifactError(
                f"{side}_hand_q20 shape {q.shape}, expected ({t}, {NUM_HAND_JOINTS})"
            )
    if int(k) < 1:
        raise ArtifactError(f"k must be >= 1, got {k}")
    if meta.get('schema_version') != SCHEMA_VERSION:
        raise ArtifactError(
            f"meta.schema_version must be {SCHEMA_VERSION}, got {meta.get('schema_version')}"
        )
    if meta.get('verdict') not in (VERDICT_PASS, VERDICT_FAIL):
        raise ArtifactError(f"meta.verdict must be pass/fail, got {meta.get('verdict')}")
    if 'max_allowed_speed_scale' not in meta:
        raise ArtifactError("meta.max_allowed_speed_scale is required")

    out_base.parent.mkdir(parents=True, exist_ok=True)
    npz_path = out_base.with_suffix('.npz')
    json_path = out_base.with_suffix('.json')
    _write_npz_deterministic(npz_path, {
        'arm_q': arm_q,
        'arm_joint_names': np.array(CANONICAL_ARM_JOINTS),
        'left_hand_q20': left_hand_q20,
        'right_hand_q20': right_hand_q20,
        'target_fps': np.float64(target_fps),
        'k': np.int64(k),
    })
    json_path.write_text(
        json.dumps(meta, indent=1, sort_keys=True) + '\n', encoding='utf-8'
    )
    return npz_path, json_path


def _write_npz_deterministic(path: Path, arrays: dict) -> None:
    """np.savez with fixed zip-entry timestamps.

    np.savez stamps wall-clock time into every zip member, which breaks the
    spec's determinism requirement (same inputs, same output hashes). Same
    on-disk format (uncompressed npz), fixed 1980 epoch instead.
    """
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_STORED) as zf:
        for name, arr in arrays.items():
            buf = io.BytesIO()
            np.lib.format.write_array(buf, np.asanyarray(arr), allow_pickle=False)
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            zf.writestr(info, buf.getvalue())


def load_artifact(path) -> ConditionedClip:
    """Load and validate a conditioned clip. `path` is the .npz, the .json,
    or the shared basename."""
    path = Path(path)
    base = path.with_suffix('') if path.suffix in ('.npz', '.json') else path
    npz_path = base.with_suffix('.npz')
    json_path = base.with_suffix('.json')
    if not npz_path.exists() or not json_path.exists():
        raise ArtifactError(
            f"artifact needs both {npz_path.name} and {json_path.name} in "
            f"{base.parent} (missing: "
            f"{[p.name for p in (npz_path, json_path) if not p.exists()]})"
        )

    try:
        meta = json.loads(json_path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        raise ArtifactError(f"{json_path}: invalid JSON: {exc}") from exc
    if meta.get('schema_version') != SCHEMA_VERSION:
        raise ArtifactError(
            f"{json_path}: schema_version {meta.get('schema_version')}, "
            f"this loader understands {SCHEMA_VERSION}"
        )
    for key in ('verdict', 'max_allowed_speed_scale', 'audit'):
        if key not in meta:
            raise ArtifactError(f"{json_path}: missing required field '{key}'")
    if meta['verdict'] not in (VERDICT_PASS, VERDICT_FAIL):
        raise ArtifactError(f"{json_path}: verdict must be pass/fail")

    data = np.load(npz_path)
    required = ('arm_q', 'arm_joint_names', 'left_hand_q20',
                'right_hand_q20', 'target_fps', 'k')
    missing = [key for key in required if key not in data]
    if missing:
        raise ArtifactError(f"{npz_path}: missing keys {missing}")

    arm_q = np.asarray(data['arm_q'], dtype=np.float64)
    names = [str(n) for n in data['arm_joint_names']]
    left = np.asarray(data['left_hand_q20'], dtype=np.float64)
    right = np.asarray(data['right_hand_q20'], dtype=np.float64)
    fps = float(data['target_fps'])
    k = int(data['k'])

    t = arm_q.shape[0]
    if arm_q.ndim != 2 or arm_q.shape[1] != NUM_ARM_JOINTS or t < 1:
        raise ArtifactError(f"{npz_path}: arm_q shape {arm_q.shape}")
    if names != CANONICAL_ARM_JOINTS:
        raise ArtifactError(
            f"{npz_path}: arm_joint_names differ from the canonical G1_29 "
            f"table; refusing (got {names})"
        )
    for side, q in (('left', left), ('right', right)):
        if q.shape != (t, NUM_HAND_JOINTS):
            raise ArtifactError(f"{npz_path}: {side}_hand_q20 shape {q.shape}")
    if not np.isfinite(arm_q).all() or not np.isfinite(left).all() \
            or not np.isfinite(right).all():
        raise ArtifactError(f"{npz_path}: non-finite values")
    if fps <= 0 or k < 1:
        raise ArtifactError(f"{npz_path}: bad target_fps {fps} or k {k}")

    return ConditionedClip(
        arm_q=arm_q,
        arm_joint_names=names,
        left_hand_q20=left,
        right_hand_q20=right,
        target_fps=fps,
        k=k,
        meta=meta,
        npz_path=npz_path,
        json_path=json_path,
    )


def synthetic_artifact(
    out_base: Path,
    num_frames: int = 50,
    target_fps: float = 50.0,
    k: int = 1,
    verdict: str = VERDICT_PASS,
    max_allowed_speed_scale: float = 1.0,
    amplitude: float = 0.05,
    hands_conditioned: bool = True,
    sample: str = 'synthetic',
    method: str = 'GT',
) -> tuple:
    """Test/fixture factory: a slow, in-limits sinusoid clip.

    Shared by the conditioning, publisher, and supervisor tests so nobody
    hand-fakes the schema (plan-check amendment). Also the shape reused by
    the single-joint Stage B generator.
    """
    t = np.arange(num_frames) / target_fps
    arm_q = amplitude * np.sin(
        2 * np.pi * 0.25 * t[:, None] + np.linspace(0, 1, NUM_ARM_JOINTS)[None, :]
    )
    hand = 0.1 + amplitude * np.sin(2 * np.pi * 0.25 * t[:, None]
                                    + np.linspace(0, 1, NUM_HAND_JOINTS)[None, :])
    meta = {
        'schema_version': SCHEMA_VERSION,
        'sample': sample,
        'method': method,
        'source_dir': None,
        'input_sha256': {},
        'retargeter': None,
        'limits': {},
        'audit': {'synthetic': True},
        'max_allowed_speed_scale': max_allowed_speed_scale,
        'verdict': verdict,
        'verdict_reasons': [] if verdict == VERDICT_PASS else ['synthetic fail'],
        'hands_conditioned': hands_conditioned,
        'first_frame': {'arm_q': arm_q[0].tolist(),
                        'left_hand_q20': hand[0].tolist(),
                        'right_hand_q20': hand[0].tolist()},
        'last_frame': {'arm_q': arm_q[-1].tolist(),
                       'left_hand_q20': hand[-1].tolist(),
                       'right_hand_q20': hand[-1].tolist()},
        'tool_version': 'synthetic',
    }
    return save_artifact(out_base, arm_q, hand, hand, target_fps, k, meta)
