"""The two joint-space clip layouts, auto-detected on read, echoed on write.

    flat npz   one file, the RoboSTAR handoff layout: arm_q (N, 14) with an
               arm_joint_names (14,) column index, left_hand_q20 (N, 20),
               right_hand_q20 (N, 20), target_fps (). Every other key in the
               file is carried through untouched.
    clip dir   the replay pipeline's clip directory (docs/spec/spec1.md):
               arm_q.npz with 'left' and 'right' (N, 7), hand_q20.npz with
               'left' and 'right' (N, 20), clip.json with frames and rate_hz.

A read gives the same Clip either way; a write puts back the layout that was
read, so a sanitized flat npz stays a drop-in for whatever consumed the
original and a sanitized clip directory is immediately playable by
replay_publisher.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from clip_audit import ARM_JOINT_NAMES, SIDES

from .model import ARM_JOINTS_PER_SIDE, NUM_ARM_JOINTS, NUM_HAND_JOINTS

# Constants. Each one says where its value comes from.

# Layout tags, also written into sanitize.json so a report says which was read.
LAYOUT_FLAT = "flat_npz"
LAYOUT_DIR = "clip_dir"

# Flat-npz key names, as conditioned_clip_v1.npz and generate_sweep_sample.py.
FLAT_ARM_KEY = "arm_q"
FLAT_ARM_NAMES_KEY = "arm_joint_names"
FLAT_HAND_KEY = "{side}_hand_q20"
FLAT_FPS_KEY = "target_fps"

# Clip-directory file names (spec1, "Clip directory").
DIR_ARM_FILE = "arm_q.npz"
DIR_HAND_FILE = "hand_q20.npz"
DIR_META_FILE = "clip.json"

# The report written next to the output.
REPORT_FILE = "sanitize.json"

# clip.json key the sanitize report block is added under.
DIR_META_SANITIZE_KEY = "sanitize"

# Fallback rate for a flat npz without one; every bundle sample ships at this.
DEFAULT_RATE_HZ = 50.0


@dataclass
class Clip:
    """One joint-space clip, layout-independent.

    arm and hand are per side and in the clip's own column order
    (clip_audit.ARM_JOINT_NAMES / HAND_JOINT_NAMES). source records what has
    to be put back to reproduce the input layout on write.
    """
    arm: Dict[str, np.ndarray]
    hand: Dict[str, np.ndarray]
    rate_hz: float
    layout: str
    path: Path
    passthrough: Dict[str, np.ndarray] = field(default_factory=dict)
    meta: Dict = field(default_factory=dict)
    flat_arm_columns: Optional[List[str]] = None

    @property
    def frames(self) -> int:
        return int(self.arm["left"].shape[0])

    @property
    def duration_s(self) -> float:
        return self.frames / float(self.rate_hz)

    def time_of(self, frame: int) -> float:
        """Clip time of a frame index, in seconds, at unit playback speed."""
        return float(frame) / float(self.rate_hz)

    def arm_all(self) -> np.ndarray:
        """(N, 14): the left block then the right block, model DoF order."""
        return np.concatenate([self.arm[s] for s in SIDES], axis=1)

    def with_arm_all(self, arm_all: np.ndarray) -> "Clip":
        """A copy carrying new arm angles, given as the (N, 14) block form."""
        arm_all = np.asarray(arm_all, dtype=float)
        if arm_all.shape != (self.frames, NUM_ARM_JOINTS):
            raise ValueError(
                f"arm_all must be ({self.frames}, {NUM_ARM_JOINTS}), got {arm_all.shape}")
        arm = {side: arm_all[:, i * ARM_JOINTS_PER_SIDE:(i + 1) * ARM_JOINTS_PER_SIDE].copy()
               for i, side in enumerate(SIDES)}
        return Clip(arm=arm, hand={s: self.hand[s].copy() for s in SIDES},
                    rate_hz=self.rate_hz, layout=self.layout, path=self.path,
                    passthrough=dict(self.passthrough), meta=dict(self.meta),
                    flat_arm_columns=list(self.flat_arm_columns) if self.flat_arm_columns else None)


def detect_layout(path: Path) -> str:
    """Which layout a path holds, by structure only."""
    path = Path(path)
    if path.is_dir():
        missing = [f for f in (DIR_ARM_FILE, DIR_HAND_FILE) if not (path / f).is_file()]
        if missing:
            raise ValueError(f"{path} is a directory but has no {', '.join(missing)}")
        return LAYOUT_DIR
    if path.is_file():
        return LAYOUT_FLAT
    raise FileNotFoundError(path)


def _check_frames(name: str, array: np.ndarray, frames: int, width: int) -> np.ndarray:
    array = np.asarray(array, dtype=float)
    if array.ndim != 2 or array.shape[1] != width:
        raise ValueError(f"{name} must be (N, {width}), got {array.shape}")
    if array.shape[0] != frames:
        raise ValueError(f"{name} has {array.shape[0]} frames, expected {frames}")
    return array


def read_clip(path: Path) -> Clip:
    """Read either layout."""
    layout = detect_layout(Path(path))
    return _read_flat(Path(path)) if layout == LAYOUT_FLAT else _read_dir(Path(path))


def _read_flat(path: Path) -> Clip:
    with np.load(path, allow_pickle=False) as handle:
        keys = list(handle.files)
        for required in (FLAT_ARM_KEY, FLAT_ARM_NAMES_KEY):
            if required not in keys:
                raise ValueError(f"{path}: flat npz is missing '{required}'")
        arm_q = np.asarray(handle[FLAT_ARM_KEY], dtype=float)
        columns = [str(n) for n in handle[FLAT_ARM_NAMES_KEY]]
        if arm_q.ndim != 2 or arm_q.shape[1] != NUM_ARM_JOINTS:
            raise ValueError(f"{path}: {FLAT_ARM_KEY} must be (N, {NUM_ARM_JOINTS}), "
                             f"got {arm_q.shape}")
        if len(columns) != NUM_ARM_JOINTS:
            raise ValueError(f"{path}: {FLAT_ARM_NAMES_KEY} must have {NUM_ARM_JOINTS} "
                             f"entries, got {len(columns)}")
        frames = int(arm_q.shape[0])

        # Into the model's per-side DoF order; a missing name is an error.
        arm: Dict[str, np.ndarray] = {}
        for side in SIDES:
            index = []
            for name in ARM_JOINT_NAMES[side]:
                if name not in columns:
                    raise ValueError(f"{path}: {FLAT_ARM_NAMES_KEY} has no '{name}'")
                index.append(columns.index(name))
            arm[side] = arm_q[:, index].copy()

        hand: Dict[str, np.ndarray] = {}
        for side in SIDES:
            key = FLAT_HAND_KEY.format(side=side)
            if key not in keys:
                raise ValueError(f"{path}: flat npz is missing '{key}'")
            hand[side] = _check_frames(key, handle[key], frames, NUM_HAND_JOINTS)

        rate_hz = float(handle[FLAT_FPS_KEY]) if FLAT_FPS_KEY in keys else DEFAULT_RATE_HZ
        if not rate_hz > 0.0:
            raise ValueError(f"{path}: {FLAT_FPS_KEY} must be positive, got {rate_hz}")

        # Everything this tool does not own is written back byte-identical.
        consumed = {FLAT_ARM_KEY} | {FLAT_HAND_KEY.format(side=s) for s in SIDES}
        passthrough = {k: np.array(handle[k]) for k in keys if k not in consumed}

    return Clip(arm=arm, hand=hand, rate_hz=rate_hz, layout=LAYOUT_FLAT, path=Path(path),
                passthrough=passthrough, flat_arm_columns=columns)


def _read_dir(path: Path) -> Clip:
    with np.load(path / DIR_ARM_FILE, allow_pickle=False) as handle:
        arm_raw = {side: np.asarray(handle[side], dtype=float) for side in SIDES}
    frames = int(arm_raw["left"].shape[0])
    arm = {side: _check_frames(f"{DIR_ARM_FILE}[{side}]", arm_raw[side], frames,
                               ARM_JOINTS_PER_SIDE)
           for side in SIDES}
    with np.load(path / DIR_HAND_FILE, allow_pickle=False) as handle:
        hand = {side: _check_frames(f"{DIR_HAND_FILE}[{side}]", handle[side], frames,
                                    NUM_HAND_JOINTS)
                for side in SIDES}

    meta: Dict = {}
    meta_path = path / DIR_META_FILE
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text())
    rate_hz = float(meta.get("rate_hz", DEFAULT_RATE_HZ))
    if not rate_hz > 0.0:
        raise ValueError(f"{meta_path}: rate_hz must be positive, got {rate_hz}")
    if "frames" in meta and int(meta["frames"]) != frames:
        raise ValueError(f"{meta_path}: frames={meta['frames']} but the arrays "
                         f"carry {frames}")
    return Clip(arm=arm, hand=hand, rate_hz=rate_hz, layout=LAYOUT_DIR, path=path, meta=meta)


def write_clip(clip: Clip, out: Path, report: Optional[Dict] = None) -> List[Path]:
    """Write the clip back in its own layout. Returns the paths written."""
    out = Path(out)
    if clip.layout == LAYOUT_FLAT:
        return _write_flat(clip, out, report)
    return _write_dir(clip, out, report)


def _write_flat(clip: Clip, out: Path, report: Optional[Dict]) -> List[Path]:
    if out.suffix != ".npz":
        raise ValueError(f"flat-npz output must be a .npz path, got {out}")
    out.parent.mkdir(parents=True, exist_ok=True)

    # Back in the input's column order, for a consumer that indexes positionally.
    columns = clip.flat_arm_columns or (ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"])
    arm_q = np.zeros((clip.frames, NUM_ARM_JOINTS), dtype=float)
    for side in SIDES:
        for i, name in enumerate(ARM_JOINT_NAMES[side]):
            arm_q[:, columns.index(name)] = clip.arm[side][:, i]

    payload = dict(clip.passthrough)
    payload[FLAT_ARM_KEY] = arm_q
    for side in SIDES:
        payload[FLAT_HAND_KEY.format(side=side)] = clip.hand[side]
    np.savez(out, **payload)

    written = [out]
    if report is not None:
        report_path = out.with_name(out.stem + "_" + REPORT_FILE)
        report_path.write_text(json.dumps(report, indent=1, sort_keys=False) + "\n")
        written.append(report_path)
    return written


def _write_dir(clip: Clip, out: Path, report: Optional[Dict]) -> List[Path]:
    out.mkdir(parents=True, exist_ok=True)
    arm_path = out / DIR_ARM_FILE
    hand_path = out / DIR_HAND_FILE
    np.savez(arm_path, **{side: clip.arm[side] for side in SIDES})
    np.savez(hand_path, **{side: clip.hand[side] for side in SIDES})
    written = [arm_path, hand_path]

    meta = dict(clip.meta)
    if meta:
        # The copied verdict describes the input; the sanitize block says re-audit.
        if report is not None:
            meta[DIR_META_SANITIZE_KEY] = report
        meta_path = out / DIR_META_FILE
        meta_path.write_text(json.dumps(meta, indent=1, sort_keys=False) + "\n")
        written.append(meta_path)
    if report is not None:
        report_path = out / REPORT_FILE
        report_path.write_text(json.dumps(report, indent=1, sort_keys=False) + "\n")
        written.append(report_path)
    return written
