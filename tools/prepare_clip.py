#!/usr/bin/env python3
"""Turn a RobotSTAR bundle trajectory into a replay clip directory (spec1, offline half).

Reads a bundle sample's GT/ or Ours/ directory and produces one clip
directory (docs/spec/spec1.md, "Clip directory"):

    1. sanitize the 14 arm joints (trim, refuse orientation flips, zero-phase
       Butterworth, per-frame step clamp);
    2. retarget the hand keypoints to Wuji Hand 2 joint angles with the
       production retargeter and its configs, output in the driver's hardware
       order;
    3. replay the result dynamically in MuJoCo at every requested speed
       (tools/clip_audit.py, the model loaded once per process);
    4. judge each speed against the thresholds, write arm_q.npz, hand_q20.npz
       and clip.json under <out>/candidate/, then move the directory to
       <out>/safe/ or <out>/rejected/.

Every option is recorded in clip.json so a clip can be re-judged without
rerunning the simulation.

What this tool is not: it never talks to hardware or ROS, never writes legs
or waist (read, never written), and never uses the bundle's precomputed hand
joints (they target the legacy hand; DO_NOT_COMMAND_HAND2.txt). Hand joints
come from the keypoints only. The retargeter is imported lazily inside the
retarget stage and is injectable, so everything else here runs and is tested
without it.

Command line:
    python3 tools/prepare_clip.py --method-dir RobotSTAR_demos/samples/<sample>/Ours --out clips
    python3 tools/prepare_clip.py --all RobotSTAR_demos/samples --out clips    # writes clips/summary.md

Exit codes: 0 the run completed (rejected clips included); 2 a refused single
clip (a >= 90 deg single-frame arm step without --allow-flips) or bad
arguments; 1 an unexpected error.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.signal import butter, filtfilt

# tools/ is on sys.path when this file runs as a script; the tests put it
# there from their conftest.
import clip_audit
from clip_audit import ARM_JOINT_NAMES, HAND_JOINT_NAMES, SIDES, AuditResult, AuditRig, Thresholds, speed_key

# ---------------------------------------------------------------------------
# Constants. Each one says where its value comes from.
# ---------------------------------------------------------------------------

# Repo root is one directory above tools/ (same layout inside the container).
REPO_ROOT = Path(__file__).resolve().parents[1]

# clip.json "tool" field (spec1, "Clip directory").
TOOL_ID = "prepare_clip/1"

# Bundle layout (RobotSTAR_demos HANDOFF_README.md; brief). Paths are
# relative to a sample's GT/ or Ours/ directory.
BUNDLE_REFERENCE_DIR = "g1_reference"
BUNDLE_META_FILE = "target_meta.json"
BUNDLE_REFERENCE_FILE = "controller_reference_v7.npz"
BUNDLE_HAND_DIR = "hand2_input"
BUNDLE_HAND_GLOB = "*_human_targets_v5.npz"
BUNDLE_MANIFEST_FILE = "MANIFEST.sha256"
BUNDLE_METHODS = ("GT", "Ours")

# MediaPipe hand landmarks per hand, the keypoint file's second dimension.
NUM_KEYPOINTS = 21

# Wuji Hand 2 joints per side (starport_wuji_hand joint_map.NUM_JOINTS).
NUM_HAND_JOINTS = 20

# Arm joints per side and in total (G1 29-DoF, ARM_JOINT_NAMES).
ARM_JOINTS_PER_SIDE = 7
NUM_ARM_JOINTS = 2 * ARM_JOINTS_PER_SIDE

# Retargeter configs, one per side (spec1 step 2). The directory is the
# wujihand_output package's config dir; the pattern is recorded in clip.json.
DEFAULT_RETARGET_CONFIG_DIR = REPO_ROOT / "src" / "output_devices" / "wujihand_output" / "config"
RETARGET_CONFIG_PATTERN = "retarget_keypoints_topic_{side}.yaml"

# A retargeted value counts as clipped only when it lies outside the model's
# joint range by more than this. The optimizer pins joints to its own limits,
# which equal the model's, and returns them through float32, so a value at the
# limit comes back up to ~1e-7 rad past it. 1e-6 is above that noise and far
# below any angle that matters.
CLIP_TOLERANCE_RAD = 1e-6

# The retargeter's low-pass coefficient for the fingers: the fraction of each
# new solve that is kept, so LARGER means less smoothing (y += alpha * (x - y),
# applied once per body frame). The configs ship 0.2, a corner near 1.8 Hz at
# 50 Hz, which holds the gross finger envelope and flattens the fast detail
# that carries meaning in these clips: on 13_val_..._Ours it leaves 470
# direction reversals against 2130 at 0.8. 0.5 keeps roughly 1.6x the detail of
# 0.2 at an unchanged verdict and about 2 percent hand-servo saturation. It is
# not pushed higher because the hand driver slew-limits its own command to
# 2 rad/s, and 0.5 already commands 5.2 rad/s peaks: past here the extra
# detail is truncated at the driver and what is left is mostly monocular
# estimator noise. Arms are smoothed separately and much harder, by the
# Butterworth below.
DEFAULT_HAND_LP_ALPHA = 0.5

# Arm sanitizer (spec1 step 1; the earlier sanitize_robotstar_clip.py).
DEFAULT_CUTOFF_HZ = 6.0
DEFAULT_MAX_STEP_DEG = 15.0
BUTTER_ORDER = 2

# A single-frame step of this size or more is an estimator orientation flip;
# smoothing it sweeps the arm through the same wrong path more slowly.
FLIP_STEP_DEG = 90.0

# filtfilt pads each end with 3 * max(len(a), len(b)) samples (its default
# padlen) and needs strictly more samples than that: 10 for a 2nd-order filter.
MIN_FRAMES_TO_FILTER = 3 * (BUTTER_ORDER + 1) + 1

# --auto-trim keeps the longest passing window of at least this many seconds
# (spec1 "If every clip is rejected").
DEFAULT_MIN_SECONDS = 3.0

# Float slack when turning min_seconds * rate_hz into a frame count, so
# 3.0 * 50.0 is 150 frames and not 151.
MIN_FRAMES_EPS = 1e-9

# Clip filing (spec1 "Clip directory"). A clip is written under candidate/
# and moved to safe/ or rejected/ when complete.
SAFE_DIR = "safe"
REJECTED_DIR = "rejected"
CANDIDATE_DIR = "candidate"
ARM_FILE = "arm_q.npz"
HAND_FILE = "hand_q20.npz"
META_FILE = "clip.json"
SUMMARY_FILE = "summary.md"

# Verdicts. safe and rejected are clip.json values (spec1); refused and error
# appear only in summary.md and on stdout.
VERDICT_SAFE = "safe"
VERDICT_REJECTED = "rejected"
VERDICT_REFUSED = "refused"
VERDICT_ERROR = "error"

# Exit codes (module docstring).
EXIT_OK = 0
EXIT_ERROR = 1
EXIT_REFUSED = 2

# Speeds above this are never audited: the publisher refuses them.
MAX_SPEED = 1.0

# Column that says "no contact" or "not audited" in summary.md.
SUMMARY_EMPTY = "-"

RetargeterFactory = Callable[[Path, str], object]


# ---------------------------------------------------------------------------
# Errors and options
# ---------------------------------------------------------------------------

class PrepareError(Exception):
    """A bundle, argument, or retargeter problem that stops one trajectory."""


class FlipRefused(PrepareError):
    """A >= FLIP_STEP_DEG single-frame arm step without --allow-flips."""

    def __init__(self, max_step_deg: float):
        self.max_step_deg = float(max_step_deg)
        super().__init__(
            f"{self.max_step_deg:.0f} deg single-frame arm step is an estimator orientation "
            f"flip; re-solve the reference or pass --allow-flips to smooth through it anyway")


@dataclass(frozen=True)
class Options:
    """Every command-line option that shapes a clip. All are recorded in clip.json."""
    speeds: Tuple[float, ...] = tuple(clip_audit.DEFAULT_SPEEDS)
    cutoff_hz: float = DEFAULT_CUTOFF_HZ
    max_step_deg: float = DEFAULT_MAX_STEP_DEG
    trim_start: int = 0
    trim_end: int = 0
    auto_trim: bool = False
    min_seconds: float = DEFAULT_MIN_SECONDS
    allow_flips: bool = False
    max_arm_torque_ratio: float = clip_audit.DEFAULT_MAX_ARM_TORQUE_RATIO
    max_contact_force_n: float = clip_audit.DEFAULT_MAX_CONTACT_FORCE_N
    note: str = ""
    retarget_config_dir: Path = DEFAULT_RETARGET_CONFIG_DIR
    hand_lp_alpha: Optional[float] = DEFAULT_HAND_LP_ALPHA

    def thresholds(self) -> Thresholds:
        return Thresholds(max_arm_torque_ratio=float(self.max_arm_torque_ratio),
                          max_contact_force_n=float(self.max_contact_force_n))

    def validate(self) -> None:
        """Raise PrepareError on a value no clip can be prepared with."""
        if not self.speeds:
            raise PrepareError("--speeds needs at least one value")
        for s in self.speeds:
            if not (0.0 < float(s) <= MAX_SPEED):
                raise PrepareError(f"--speeds value {s} is not in (0, {MAX_SPEED}]")
        if len(set(speed_key(s) for s in self.speeds)) != len(self.speeds):
            raise PrepareError(f"--speeds has duplicates: {list(self.speeds)}")
        if self.cutoff_hz <= 0:
            raise PrepareError(f"--cutoff-hz must be > 0, got {self.cutoff_hz}")
        if self.max_step_deg <= 0:
            raise PrepareError(f"--max-step-deg must be > 0, got {self.max_step_deg}")
        if self.trim_start < 0 or self.trim_end < 0:
            raise PrepareError("--trim-start and --trim-end must be >= 0")
        if self.min_seconds <= 0:
            raise PrepareError(f"--min-seconds must be > 0, got {self.min_seconds}")
        if self.max_arm_torque_ratio < 0 or self.max_contact_force_n < 0:
            raise PrepareError("thresholds must be >= 0")
        if not Path(self.retarget_config_dir).is_dir():
            raise PrepareError(f"--retarget-config-dir {self.retarget_config_dir} is not a directory")
        if self.hand_lp_alpha is not None and not (0.0 < float(self.hand_lp_alpha) <= 1.0):
            raise PrepareError(f"--hand-lp-alpha must be in (0, 1], got {self.hand_lp_alpha}")


def log(message: str) -> None:
    """Progress and warnings go to stderr; stdout carries one line per clip."""
    print(message, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# 0. Read the bundle
# ---------------------------------------------------------------------------

@dataclass
class BundleTrajectory:
    """One GT or Ours trajectory, arms selected by name, keypoints as shipped.

    arm_q[side] is (frames, 7) float64 in ARM_JOINT_NAMES order;
    keypoints[side] is (source_frames, 21, 3) float32 meters.
    """
    sample: str
    method: str
    method_dir: Path
    frames: int
    source_frames: int
    target_fps: float
    time_scale: float
    arm_q: Dict[str, np.ndarray]
    keypoints: Dict[str, np.ndarray]
    manifest_sha256: Optional[str]

    @property
    def rate_hz(self) -> float:
        """The clip's frame rate: the nominal playback period is time_scale / target_fps."""
        return self.target_fps / self.time_scale

    @property
    def name(self) -> str:
        return trajectory_name(self.method_dir)


def trajectory_name(method_dir: Path) -> str:
    """<sample>_<method>, the clip directory name."""
    method_dir = Path(method_dir)
    return f"{method_dir.parent.name}_{method_dir.name}"


def find_manifest(method_dir: Path) -> Optional[Path]:
    """MANIFEST.sha256 at the bundle root: the first one found walking up."""
    method_dir = Path(method_dir).resolve()
    for directory in (method_dir, *method_dir.parents):
        candidate = directory / BUNDLE_MANIFEST_FILE
        if candidate.is_file():
            return candidate
    return None


def select_arm_columns(body_q: np.ndarray, body_actuators: Sequence[str]) -> Dict[str, np.ndarray]:
    """The 14 arm columns of body_q, by name, in ARM_JOINT_NAMES order per side.

    Never by index: the bundle's own order interleaves legs and waist, and a
    raw mj_name2id on the bundle's names (no _joint suffix) returns -1.
    """
    column = {name: i for i, name in enumerate(body_actuators)}
    out: Dict[str, np.ndarray] = {}
    for side in SIDES:
        missing = [n for n in ARM_JOINT_NAMES[side] if n not in column]
        if missing:
            raise PrepareError(f"body_actuators lacks arm joints {missing}")
        out[side] = np.asarray(body_q[:, [column[n] for n in ARM_JOINT_NAMES[side]]], dtype=np.float64)
    return out


def read_bundle(method_dir: Path) -> BundleTrajectory:
    """Read and validate one method directory. Legs and waist are read, not kept."""
    method_dir = Path(method_dir)
    ref_dir = method_dir / BUNDLE_REFERENCE_DIR
    meta_path = ref_dir / BUNDLE_META_FILE
    npz_path = ref_dir / BUNDLE_REFERENCE_FILE
    for path in (meta_path, npz_path):
        if not path.is_file():
            raise PrepareError(f"{path} is missing; --method-dir must be a sample's GT/ or Ours/ directory")

    try:
        meta = json.loads(meta_path.read_text())
        body_actuators = [str(n) for n in meta["joint_actuator_order"]["body_actuators"]]
        frames = int(meta["frames"])
        # source_frames is optional. The 15 bundle samples carry it because their
        # keypoints are on the source timeline; RobotSTAR_demos/sweep-test writes
        # its keypoints on the body frame grid and omits the key. The keypoint
        # array's own length is authoritative either way, so it is the fallback,
        # and the shape check below then compares the array against itself rather
        # than refusing a sample the tool can read perfectly well.
        source_frames = int(meta["source_frames"]) if "source_frames" in meta else None
        target_fps = float(meta["target_fps"])
        time_scale = float(meta.get("time_scale", 1))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise PrepareError(f"{meta_path}: cannot read ({exc!r})") from exc
    if target_fps <= 0 or time_scale <= 0 or frames < 1 or (source_frames is not None and source_frames < 1):
        raise PrepareError(f"{meta_path}: target_fps {target_fps}, time_scale {time_scale}, "
                           f"frames {frames}, source_frames {source_frames} must all be positive")

    with np.load(npz_path) as data:
        if "body_q" not in data.files:
            raise PrepareError(f"{npz_path}: no body_q (has {data.files})")
        body_q = np.asarray(data["body_q"], dtype=np.float64)
    if body_q.shape != (frames, len(body_actuators)):
        raise PrepareError(f"{npz_path}: body_q shape {body_q.shape}, target_meta says "
                           f"({frames}, {len(body_actuators)})")
    if not np.all(np.isfinite(body_q)):
        raise PrepareError(f"{npz_path}: body_q has non-finite values")
    arm_q = select_arm_columns(body_q, body_actuators)

    hand_files = sorted((method_dir / BUNDLE_HAND_DIR).glob(BUNDLE_HAND_GLOB))
    if len(hand_files) != 1:
        raise PrepareError(f"expected one {BUNDLE_HAND_DIR}/{BUNDLE_HAND_GLOB} in {method_dir}, "
                           f"found {[p.name for p in hand_files]}")
    keypoints: Dict[str, np.ndarray] = {}
    with np.load(hand_files[0]) as data:
        for side in SIDES:
            key = f"{side}_hand_keypoints21"
            if key not in data.files:
                raise PrepareError(f"{hand_files[0]}: no {key} (has {data.files})")
            kp = np.asarray(data[key], dtype=np.float32)
            if source_frames is None:
                # First side read sets the count the other side must match.
                source_frames = int(kp.shape[0])
            if kp.shape != (source_frames, NUM_KEYPOINTS, 3):
                raise PrepareError(f"{hand_files[0]}: {key} shape {kp.shape}, expected "
                                   f"({source_frames}, {NUM_KEYPOINTS}, 3)")
            if not np.all(np.isfinite(kp)):
                raise PrepareError(f"{hand_files[0]}: {key} has non-finite values")
            keypoints[side] = kp

    manifest = find_manifest(method_dir)
    if manifest is None:
        log(f"warning: no {BUNDLE_MANIFEST_FILE} above {method_dir}; bundle_manifest_sha256 is null")
    return BundleTrajectory(
        sample=method_dir.parent.name, method=method_dir.name, method_dir=method_dir,
        frames=frames, source_frames=source_frames, target_fps=target_fps, time_scale=time_scale,
        arm_q=arm_q, keypoints=keypoints,
        manifest_sha256=clip_audit.sha256_file(manifest) if manifest else None)


# ---------------------------------------------------------------------------
# 1. Sanitize arms
# ---------------------------------------------------------------------------

def stack_sides(per_side: Dict[str, np.ndarray]) -> np.ndarray:
    """(T, 14): left columns then right."""
    return np.concatenate([np.asarray(per_side[s], dtype=np.float64) for s in SIDES], axis=1)


def split_sides(arm14: np.ndarray) -> Dict[str, np.ndarray]:
    """Inverse of stack_sides."""
    return {"left": arm14[:, :ARM_JOINTS_PER_SIDE].copy(),
            "right": arm14[:, ARM_JOINTS_PER_SIDE:].copy()}


def trim_frames(arr: np.ndarray, trim_start: int, trim_end: int) -> np.ndarray:
    """Drop trim_start leading and trim_end trailing frames (axis 0)."""
    n = arr.shape[0]
    if trim_start < 0 or trim_end < 0:
        raise PrepareError("trim counts must be >= 0")
    if trim_start + trim_end >= n:
        raise PrepareError(f"trimming {trim_start} + {trim_end} frames leaves nothing of {n}")
    return arr[trim_start:n - trim_end]


def max_single_step_deg(arm14: np.ndarray) -> float:
    """Largest frame-to-frame change of any arm joint, in degrees."""
    if arm14.shape[0] < 2:
        return 0.0
    return float(np.degrees(np.abs(np.diff(arm14, axis=0)).max()))


def arm_stats(arm14: np.ndarray, fps: float) -> dict:
    """max step, peak velocity and peak acceleration (np.gradient at 1/fps)."""
    if arm14.shape[0] < 2:
        return {"max_step_deg": 0.0, "peak_vel_rad_s": 0.0, "peak_acc_rad_s2": 0.0}
    dt = 1.0 / fps
    dq = np.gradient(arm14, dt, axis=0)
    ddq = np.gradient(dq, dt, axis=0)
    return {"max_step_deg": max_single_step_deg(arm14),
            "peak_vel_rad_s": float(np.abs(dq).max()),
            "peak_acc_rad_s2": float(np.abs(ddq).max())}


def butter_zero_phase(arm14: np.ndarray, fps: float, cutoff_hz: float) -> np.ndarray:
    """Zero-phase 2nd-order Butterworth low-pass along axis 0."""
    nyquist = fps / 2.0
    if not 0.0 < cutoff_hz < nyquist:
        raise PrepareError(f"cutoff {cutoff_hz} Hz must be in (0, {nyquist}) Hz for {fps} fps")
    if arm14.shape[0] < MIN_FRAMES_TO_FILTER:
        raise PrepareError(f"{arm14.shape[0]} frames is too few to filter (need {MIN_FRAMES_TO_FILTER})")
    b, a = butter(BUTTER_ORDER, cutoff_hz / nyquist)
    return filtfilt(b, a, arm14, axis=0)


def step_clamp(arm14: np.ndarray, max_step_rad: float) -> np.ndarray:
    """Forward then backward per-frame step clamp, per column."""
    out = np.array(arm14, dtype=np.float64, copy=True)
    for i in range(1, out.shape[0]):
        out[i] = out[i - 1] + np.clip(out[i] - out[i - 1], -max_step_rad, max_step_rad)
    for i in range(out.shape[0] - 2, -1, -1):
        out[i] = out[i + 1] + np.clip(out[i] - out[i + 1], -max_step_rad, max_step_rad)
    return out


def sanitize_arms(arm14: np.ndarray, fps: float, cutoff_hz: float, max_step_deg: float,
                  allow_flips: bool) -> Tuple[np.ndarray, dict]:
    """spec1 step 1 on an already trimmed (T, 14) array.

    Returns the sanitized array and the stats part of clip.json's sanitize
    block: before, after, arm_rmse_rad, flip_max_step_deg. Raises FlipRefused
    on a >= FLIP_STEP_DEG single-frame step unless allow_flips.
    """
    arm14 = np.asarray(arm14, dtype=np.float64)
    if arm14.ndim != 2 or arm14.shape[1] != NUM_ARM_JOINTS:
        raise PrepareError(f"arm array shape {arm14.shape}, expected (T, {NUM_ARM_JOINTS})")
    flip_deg = max_single_step_deg(arm14)
    if flip_deg >= FLIP_STEP_DEG and not allow_flips:
        raise FlipRefused(flip_deg)
    before = arm_stats(arm14, fps)
    q = butter_zero_phase(arm14, fps, cutoff_hz)
    q = step_clamp(q, math.radians(max_step_deg))
    after = arm_stats(q, fps)
    stats = {"before": before, "after": after,
             "arm_rmse_rad": float(np.sqrt(np.mean((q - arm14) ** 2))),
             "flip_max_step_deg": flip_deg}
    return q, stats


# ---------------------------------------------------------------------------
# 2. Retarget hands
# ---------------------------------------------------------------------------

def keypoint_frame_index(body_index: int, n_body: int, n_hand: int) -> int:
    """Keypoint frame for body frame i: round(i * (T_hand - 1) / (T_body - 1)).

    The mapping the sim publisher used; T_body == 1 maps to keypoint frame 0.
    """
    if n_body <= 1:
        return 0
    return int(round(body_index * (n_hand - 1) / (n_body - 1)))


def keypoint_frame_indices(n_body: int, n_hand: int) -> np.ndarray:
    """keypoint_frame_index for every body frame of the untrimmed clip."""
    return np.array([keypoint_frame_index(i, n_body, n_hand) for i in range(n_body)], dtype=int)


def urdf_movable_joints(urdf_path: Path) -> List[str]:
    """Names of the URDF's non-fixed joints in declaration order.

    That order is the driver's hardware order (established fact, brief).
    """
    root = ET.parse(str(urdf_path)).getroot()
    return [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]


def retargeter_urdf_path(retargeter) -> Path:
    """optimizer.urdf_path from the retargeter's config, relative to the yaml's directory."""
    config = retargeter.config
    rel = (config.get("optimizer") or {}).get("urdf_path")
    if not rel:
        raise PrepareError("retarget config sets no optimizer.urdf_path; the Hand 2 configs must")
    path = Path(rel)
    if not path.is_absolute():
        path = (Path(config["__yaml_dir"]) / path).resolve()
    return path


def build_qpos_perm(optimizer_names: Sequence[str], urdf_names: Sequence[str]) -> np.ndarray:
    """Permutation taking optimizer order to URDF declaration order: q_urdf = q_opt[perm].

    Pinocchio orders joints by its own tree walk (index, middle, pinky, ring,
    thumb for Hand 2); publishing that raw would send thumb angles to the
    index finger. Raises when the two name lists cannot be aligned: an
    identity fallback would drive the wrong fingers silently.
    """
    optimizer_names = list(optimizer_names)
    urdf_names = list(urdf_names)
    index = {n: i for i, n in enumerate(optimizer_names)}
    missing = [n for n in urdf_names if n not in index]
    if missing or len(urdf_names) != len(optimizer_names) or len(index) != len(optimizer_names):
        raise PrepareError(f"cannot align optimizer joints {optimizer_names} with URDF joints "
                           f"{urdf_names}; unmatched {missing}")
    return np.array([index[n] for n in urdf_names], dtype=int)


def hardware_perm(retargeter, side: str) -> np.ndarray:
    """build_qpos_perm for a live retargeter, with the URDF order checked against HAND_JOINT_NAMES."""
    urdf_names = urdf_movable_joints(retargeter_urdf_path(retargeter))
    if urdf_names != HAND_JOINT_NAMES[side]:
        raise PrepareError(f"{side} URDF movable joints {urdf_names} differ from the hardware order "
                           f"{HAND_JOINT_NAMES[side]}")
    return build_qpos_perm(list(retargeter.optimizer.robot.dof_joint_names), urdf_names)


def default_retargeter_factory(config_path: Path, side: str):
    """The production retargeter. NLopt + Pinocchio: imported here, in the teleop container."""
    from wuji_retargeting import Retargeter
    return Retargeter.from_yaml(str(config_path), side)


def retarget_side(retargeter, perm: np.ndarray, keypoints: np.ndarray, kp_indices: np.ndarray) -> np.ndarray:
    """(T, 20) hardware-order joint angles, unclipped: reset() once, one retarget per body frame."""
    retargeter.reset()
    out = np.empty((len(kp_indices), NUM_HAND_JOINTS), dtype=np.float64)
    for t, k in enumerate(kp_indices):
        q = np.asarray(retargeter.retarget(np.asarray(keypoints[k], dtype=np.float32)), dtype=np.float64)
        if q.shape != (NUM_HAND_JOINTS,):
            raise PrepareError(f"retarget returned shape {q.shape} at frame {t}, expected ({NUM_HAND_JOINTS},)")
        out[t] = q[perm]
    return out


def set_lp_alpha(retargeter, alpha: Optional[float], side: str) -> Optional[float]:
    """Override the retargeter's low-pass coefficient, and report what it ended up as.

    The retargeter smooths its own output with a first-order filter,
    y += alpha * (x - y), applied once per body frame. The configs ship
    alpha 0.2, which at 50 Hz is a corner near 1.8 Hz: it keeps the gross
    finger envelope and flattens the fast detail. A larger alpha keeps more
    of the source motion and more of its estimator noise. The hand driver's
    own 2 rad/s slew limit bounds what any of it can reach the hardware as.

    None leaves the config's value alone. An override on a retargeter with no
    such filter raises rather than passing silently, so the flag can never
    look applied when it was not.
    """
    lp = getattr(retargeter, "lp_filter", None)
    if alpha is not None:
        if lp is None or not hasattr(lp, "alpha"):
            raise PrepareError(
                f"{side} retargeter has no lp_filter.alpha to override; "
                "drop --hand-lp-alpha or update this function for the new retargeter")
        lp.alpha = float(alpha)
    return None if lp is None else float(getattr(lp, "alpha"))


def retarget_hands(keypoints: Dict[str, np.ndarray], kp_indices: np.ndarray, config_dir: Path,
                   hand_jnt_range: Dict[str, np.ndarray],
                   retargeter_factory: Optional[RetargeterFactory] = None,
                   lp_alpha: Optional[float] = None) -> Tuple[Dict[str, np.ndarray], dict]:
    """spec1 step 2 for both sides.

    Returns hand_q20[side] (T, 20) float64 in HAND_JOINT_NAMES order, clipped
    into hand_jnt_range[side], and clip.json's hand_retarget block, whose
    clipped_fraction counts values more than CLIP_TOLERANCE_RAD outside the range.
    lp_alpha overrides the configs' low-pass coefficient; see set_lp_alpha.
    """
    factory = retargeter_factory or default_retargeter_factory
    hand_q20: Dict[str, np.ndarray] = {}
    config_sha: Dict[str, str] = {}
    clipped: Dict[str, float] = {}
    effective_alpha: Dict[str, Optional[float]] = {}
    for side in SIDES:
        config_path = Path(config_dir) / RETARGET_CONFIG_PATTERN.format(side=side)
        if not config_path.is_file():
            raise PrepareError(f"retarget config {config_path} is missing")
        retargeter = factory(config_path, side)
        effective_alpha[side] = set_lp_alpha(retargeter, lp_alpha, side)
        perm = hardware_perm(retargeter, side)
        raw = retarget_side(retargeter, perm, keypoints[side], kp_indices)
        if not np.all(np.isfinite(raw)):
            raise PrepareError(f"{side} retarget produced non-finite joint angles")
        lo = np.asarray(hand_jnt_range[side][:, 0], dtype=np.float64)
        hi = np.asarray(hand_jnt_range[side][:, 1], dtype=np.float64)
        clipped[side] = float(np.mean((raw < lo - CLIP_TOLERANCE_RAD) | (raw > hi + CLIP_TOLERANCE_RAD)))
        hand_q20[side] = np.clip(raw, lo, hi)
        config_sha[side] = clip_audit.sha256_file(config_path)
    block = {"config": RETARGET_CONFIG_PATTERN, "config_sha256": config_sha,
             "clipped_fraction": clipped, "lp_alpha": effective_alpha}
    return hand_q20, block


# ---------------------------------------------------------------------------
# 3. Audit, 5. auto-trim, 4. judge
# ---------------------------------------------------------------------------

def audit_speeds(rig: AuditRig, arm_q: Dict[str, np.ndarray], hand_q20: Dict[str, np.ndarray],
                 rate_hz: float, speeds: Sequence[float], thresholds: Thresholds,
                 name: str = "") -> Dict[float, AuditResult]:
    """One AuditResult per speed, in the order given."""
    results: Dict[float, AuditResult] = {}
    for speed in speeds:
        t0 = time.perf_counter()
        results[float(speed)] = rig.run(arm_q, hand_q20, rate_hz, float(speed), thresholds)
        log(f"{name}: audit {speed_key(speed)}x in {time.perf_counter() - t0:.1f} s")
    return results


def longest_passing_window(frame_torque_ratio: np.ndarray, frame_contact_force_n: np.ndarray,
                           thresholds: Thresholds) -> Tuple[int, int]:
    """[start, stop) of the longest run of frames within both thresholds.

    Ties go to the earliest run; (0, 0) when no frame passes.
    """
    ok = ((np.asarray(frame_torque_ratio) <= thresholds.max_arm_torque_ratio)
          & (np.asarray(frame_contact_force_n) <= thresholds.max_contact_force_n))
    best_start, best_stop = 0, 0
    run_start: Optional[int] = None
    for i, flag in enumerate(np.append(ok, False)):
        if flag and run_start is None:
            run_start = i
        elif not flag and run_start is not None:
            if i - run_start > best_stop - best_start:
                best_start, best_stop = run_start, i
            run_start = None
    return best_start, best_stop


def min_window_frames(min_seconds: float, rate_hz: float) -> int:
    """Frames in min_seconds at rate_hz, at least 1."""
    return max(1, int(math.ceil(min_seconds * rate_hz - MIN_FRAMES_EPS)))


def choose_auto_trim(results: Dict[float, AuditResult], thresholds: Thresholds,
                     min_frames: int) -> Optional[Tuple[float, int, int]]:
    """Decision 4: fastest speed with a passing window of at least min_frames.

    Returns (speed, start, stop) for that speed's longest window, or None.
    """
    for speed in sorted(results, reverse=True):
        result = results[speed]
        start, stop = longest_passing_window(result.frame_torque_ratio, result.frame_contact_force_n, thresholds)
        if stop - start >= min_frames:
            return speed, start, stop
    return None


def judge(per_speed: Dict[float, dict]) -> Tuple[str, List[float]]:
    """spec1 step 4: safe when any speed passes; safe_speeds descending."""
    safe = sorted((float(s) for s, summary in per_speed.items() if summary["pass"]), reverse=True)
    return (VERDICT_SAFE if safe else VERDICT_REJECTED), safe


def rejection_reason(per_speed: Dict[float, dict], thresholds: Thresholds) -> str:
    """One phrase per failing speed naming the number that failed."""
    parts = []
    for speed in sorted(per_speed, reverse=True):
        summary = per_speed[speed]
        if summary["pass"]:
            continue
        failed = []
        if summary["peak_arm_torque_ratio"] > thresholds.max_arm_torque_ratio:
            failed.append(f"torque ratio {summary['peak_arm_torque_ratio']:.2f} > {thresholds.max_arm_torque_ratio:g}")
        if summary["peak_contact_force_n"] > thresholds.max_contact_force_n:
            pair = "/".join(summary["peak_contact_pair"]) or "no pair"
            failed.append(f"contact {summary['peak_contact_force_n']:.1f} N > "
                          f"{thresholds.max_contact_force_n:g} ({pair})")
        parts.append(f"{speed_key(speed)}x: " + ", ".join(failed))
    return "; ".join(parts)


# ---------------------------------------------------------------------------
# 6. clip.json and filing
# ---------------------------------------------------------------------------

def to_jsonable(obj):
    """Recursively turn numpy scalars, arrays and Paths into JSON-native values."""
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [to_jsonable(v) for v in obj.tolist()]
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, Path):
        return str(obj)
    return obj


def build_clip_json(traj: BundleTrajectory, n_frames: int, sanitize_block: dict, hand_block: dict,
                    audit_block: dict, safe_speeds: List[float], verdict: str) -> dict:
    """clip.json in spec1's key order."""
    return {
        "tool": TOOL_ID,
        "source": {
            "sample": traj.sample,
            "method": traj.method,
            "bundle_manifest_sha256": traj.manifest_sha256,
            "method_dir": str(Path(traj.method_dir).resolve()),
            "time_scale": float(traj.time_scale),
        },
        "frames": int(n_frames),
        "rate_hz": float(traj.rate_hz),
        "arm_joint_names": {side: list(ARM_JOINT_NAMES[side]) for side in SIDES},
        "hand_joint_names": {side: list(HAND_JOINT_NAMES[side]) for side in SIDES},
        "sanitize": sanitize_block,
        "hand_retarget": hand_block,
        "audit": audit_block,
        "safe_speeds": [float(s) for s in safe_speeds],
        "verdict": verdict,
    }


def replace_dir(src: Path, dst: Path) -> None:
    """Rename src onto dst. An existing dst is moved aside first and removed after.

    dst is never half-written: it is either the old directory, absent for the
    instant between the two renames, or the new one.
    """
    old: Optional[Path] = None
    if dst.exists():
        old = src.parent / f"{dst.name}.replaced"
        if old.exists():
            shutil.rmtree(old)
        os.replace(dst, old)
    os.replace(src, dst)
    if old is not None:
        shutil.rmtree(old)


def write_clip_dir(out_root: Path, name: str, verdict: str, arm_q: Dict[str, np.ndarray],
                   hand_q20: Dict[str, np.ndarray], clip_json: dict) -> Path:
    """Write the three files under candidate/<name>, then move to safe/ or rejected/."""
    out_root = Path(out_root)
    candidate = out_root / CANDIDATE_DIR / name
    if candidate.exists():
        shutil.rmtree(candidate)
    candidate.mkdir(parents=True)
    np.savez(candidate / ARM_FILE, **{s: np.asarray(arm_q[s], dtype=np.float64) for s in SIDES})
    np.savez(candidate / HAND_FILE, **{s: np.asarray(hand_q20[s], dtype=np.float64) for s in SIDES})
    (candidate / META_FILE).write_text(json.dumps(to_jsonable(clip_json), indent=1, sort_keys=False) + "\n")
    final_dir = out_root / (SAFE_DIR if verdict == VERDICT_SAFE else REJECTED_DIR) / name
    final_dir.parent.mkdir(parents=True, exist_ok=True)
    replace_dir(candidate, final_dir)
    return final_dir


# ---------------------------------------------------------------------------
# One trajectory end to end
# ---------------------------------------------------------------------------

@dataclass
class Outcome:
    """What one trajectory came to: a summary.md row and a stdout line.

    The peak numbers are the fastest speed the row is about: for a safe clip
    the fastest speed that PASSED, for a rejected one the fastest audited.
    Reporting a safe clip at a speed it failed reads as a contradiction --
    05_test_G42xKICVj9U_5-5-rgb_front_GT is safe at 0.5 and its 1.0 audit has a
    torque ratio of 1.00, so the row said "safe" next to a saturated joint.
    ``reported_speed`` says which speed the numbers belong to.
    """
    name: str
    verdict: str
    safe_speeds: List[float] = field(default_factory=list)
    peak_contact_force_n: Optional[float] = None
    peak_contact_pair: List[str] = field(default_factory=list)
    peak_arm_torque_ratio: Optional[float] = None
    reported_speed: Optional[float] = None
    reason: str = ""
    clip_dir: Optional[Path] = None


def prepare_one(method_dir: Path, out_root: Path, opts: Options, rig: AuditRig,
                retargeter_factory: Optional[RetargeterFactory] = None) -> Outcome:
    """Bundle method dir -> clip directory. Raises FlipRefused or PrepareError."""
    t_start = time.perf_counter()
    traj = read_bundle(method_dir)
    name = traj.name
    rate_hz = traj.rate_hz
    thresholds = opts.thresholds()
    log(f"{name}: {traj.frames} frames at {rate_hz:g} Hz, {traj.source_frames} keypoint frames")

    # 1. Sanitize arms: trim first, then refuse flips, filter, clamp.
    arm14 = trim_frames(stack_sides(traj.arm_q), opts.trim_start, opts.trim_end)
    arm14, stats = sanitize_arms(arm14, rate_hz, opts.cutoff_hz, opts.max_step_deg, opts.allow_flips)
    if stats["flip_max_step_deg"] >= FLIP_STEP_DEG:
        log(f"{name}: warning: smoothing through a {stats['flip_max_step_deg']:.0f} deg step (--allow-flips)")
    arm_q = split_sides(arm14)

    # 2. Retarget hands on the same (trimmed) body frames.
    kp_indices = trim_frames(keypoint_frame_indices(traj.frames, traj.source_frames),
                             opts.trim_start, opts.trim_end)
    t0 = time.perf_counter()
    hand_q20, hand_block = retarget_hands(traj.keypoints, kp_indices, opts.retarget_config_dir,
                                          rig.hand_jnt_range, retargeter_factory,
                                          opts.hand_lp_alpha)
    log(f"{name}: retargeted {len(kp_indices)} frames x 2 hands in {time.perf_counter() - t0:.1f} s")

    # 3. Audit every speed; 5. --auto-trim re-audits the kept window.
    results = audit_speeds(rig, arm_q, hand_q20, rate_hz, opts.speeds, thresholds, name)
    trim_start, trim_end = opts.trim_start, opts.trim_end
    auto_trim_note = ""
    if opts.auto_trim:
        n_before = arm14.shape[0]
        min_frames = min_window_frames(opts.min_seconds, rate_hz)
        chosen = choose_auto_trim(results, thresholds, min_frames)
        if chosen is None:
            auto_trim_note = (f"no speed has a passing window of at least {min_frames} frames "
                              f"({opts.min_seconds:g} s at {rate_hz:g} Hz); clip left untrimmed")
        elif chosen[1:] == (0, n_before):
            auto_trim_note = f"the whole clip passes at {speed_key(chosen[0])}x; nothing trimmed"
        else:
            speed, start, stop = chosen
            arm_q = {s: arm_q[s][start:stop] for s in SIDES}
            hand_q20 = {s: hand_q20[s][start:stop] for s in SIDES}
            trim_start += start
            trim_end += n_before - stop
            auto_trim_note = (f"kept frames [{start}, {stop}) of {n_before} ({stop - start} frames, "
                              f"{(stop - start) / rate_hz:.2f} s), the longest passing window at "
                              f"{speed_key(speed)}x; re-audited")
            results = audit_speeds(rig, arm_q, hand_q20, rate_hz, opts.speeds, thresholds, name)
        log(f"{name}: auto-trim: {auto_trim_note}")

    # 4. Judge and 6. write.
    per_speed = {s: r.summary for s, r in results.items()}
    verdict, safe_speeds = judge(per_speed)
    n_frames = int(arm_q["left"].shape[0])
    sanitize_block = {
        "cutoff_hz": float(opts.cutoff_hz),
        "max_step_deg": float(opts.max_step_deg),
        "trim_start": int(trim_start),
        "trim_end": int(trim_end),
        "allow_flips": bool(opts.allow_flips),
        "auto_trim": bool(opts.auto_trim),
        "min_seconds": float(opts.min_seconds),
        "before": stats["before"],
        "after": stats["after"],
        "arm_rmse_rad": stats["arm_rmse_rad"],
        "flip_max_step_deg": stats["flip_max_step_deg"],
        "auto_trim_note": auto_trim_note,
    }
    audit_block = rig.audit_meta(opts.speeds, thresholds, opts.note)
    audit_block["per_speed"] = {speed_key(s): summary for s, summary in per_speed.items()}
    clip_json = build_clip_json(traj, n_frames, sanitize_block, hand_block, audit_block, safe_speeds, verdict)
    clip_dir = write_clip_dir(out_root, name, verdict, arm_q, hand_q20, clip_json)
    log(f"{name}: {verdict}, written to {clip_dir} in {time.perf_counter() - t_start:.1f} s total")

    reported_speed = max(safe_speeds) if safe_speeds else max(per_speed)
    reported = per_speed[reported_speed]
    return Outcome(
        name=name, verdict=verdict, safe_speeds=safe_speeds,
        peak_contact_force_n=float(reported["peak_contact_force_n"]),
        peak_contact_pair=list(reported["peak_contact_pair"]),
        peak_arm_torque_ratio=float(reported["peak_arm_torque_ratio"]),
        reported_speed=float(reported_speed),
        reason="" if verdict == VERDICT_SAFE else rejection_reason(per_speed, thresholds),
        clip_dir=clip_dir)


# ---------------------------------------------------------------------------
# --all and summary.md
# ---------------------------------------------------------------------------

def find_trajectories(samples_dir: Path) -> List[Path]:
    """Every <sample>/GT and <sample>/Ours directory under samples_dir, sorted."""
    samples_dir = Path(samples_dir)
    found = [p for method in BUNDLE_METHODS for p in samples_dir.glob(f"*/{method}") if p.is_dir()]
    return sorted(found)


def format_speeds(speeds: Sequence[float]) -> str:
    return ", ".join(speed_key(s) for s in speeds) if speeds else SUMMARY_EMPTY


def format_result_line(outcome: Outcome) -> str:
    """The one stdout line per clip."""
    if outcome.verdict in (VERDICT_SAFE, VERDICT_REJECTED):
        pair = "/".join(outcome.peak_contact_pair) if outcome.peak_contact_pair else "no contact"
        line = (f"{outcome.name}: {outcome.verdict} safe_speeds=[{format_speeds(outcome.safe_speeds)}] "
                f"peak_contact={outcome.peak_contact_force_n:.1f} N ({pair}) "
                f"peak_torque_ratio={outcome.peak_arm_torque_ratio:.2f} -> {outcome.clip_dir}")
        return line if not outcome.reason else f"{line} ({outcome.reason})"
    return f"{outcome.name}: {outcome.verdict} ({outcome.reason})"


def _cell(text: str) -> str:
    """Markdown table cell: no pipes or newlines."""
    return str(text).replace("|", "\\|").replace("\n", " ")


def summary_row(outcome: Outcome) -> str:
    force = (f"{outcome.peak_contact_force_n:.1f}" if outcome.peak_contact_force_n is not None
             else SUMMARY_EMPTY)
    pair = " / ".join(outcome.peak_contact_pair) if outcome.peak_contact_pair else SUMMARY_EMPTY
    torque = (f"{outcome.peak_arm_torque_ratio:.2f}" if outcome.peak_arm_torque_ratio is not None
              else SUMMARY_EMPTY)
    at = f"{outcome.reported_speed:g}x" if outcome.reported_speed is not None else SUMMARY_EMPTY
    cells = [outcome.name, outcome.verdict, format_speeds(outcome.safe_speeds), at, force, pair, torque,
             outcome.reason]
    return "| " + " | ".join(_cell(c) for c in cells) + " |"


def write_summary(path: Path, outcomes: Sequence[Outcome], opts: Options) -> None:
    """clips/summary.md: one row per trajectory (docs/replay.md)."""
    counts = {v: sum(1 for o in outcomes if o.verdict == v)
              for v in (VERDICT_SAFE, VERDICT_REJECTED, VERDICT_REFUSED, VERDICT_ERROR)}
    lines = [
        "# Clip summary",
        "",
        f"Written by `{TOOL_ID}` (`--all`). {len(outcomes)} trajectories: "
        + ", ".join(f"{n} {v}" for v, n in counts.items()) + ".",
        "",
        f"Options: speeds {format_speeds(opts.speeds)}; cutoff {opts.cutoff_hz:g} Hz; "
        f"max step {opts.max_step_deg:g} deg; trim {opts.trim_start}/{opts.trim_end}; "
        f"auto-trim {'on' if opts.auto_trim else 'off'} (min {opts.min_seconds:g} s); "
        f"allow-flips {'on' if opts.allow_flips else 'off'}; "
        f"max torque ratio {opts.max_arm_torque_ratio:g}; max contact {opts.max_contact_force_n:g} N."
        + (f" Note: {opts.note}" if opts.note else ""),
        "",
        "Peak contact force, pair and torque ratio are read at the speed in the `at` "
        "column: for a safe clip the fastest speed that passed, for any other the "
        "fastest audited. A safe clip's numbers therefore describe a speed it may "
        "actually be played at.",
        "",
        "| clip | verdict | safe speeds | at | peak contact (N) | pair | peak torque ratio | reason |",
        "|---|---|---|---|---|---|---|---|",
    ]
    lines.extend(summary_row(o) for o in outcomes)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def run_all(samples_dir: Path, out_root: Path, opts: Options, rig: AuditRig,
            retargeter_factory: Optional[RetargeterFactory] = None) -> List[Outcome]:
    """Every trajectory under samples_dir; failures become rows, not exits."""
    trajectories = find_trajectories(samples_dir)
    if not trajectories:
        raise PrepareError(f"no */GT or */Ours directories under {samples_dir}")
    outcomes: List[Outcome] = []
    for method_dir in trajectories:
        name = trajectory_name(method_dir)
        try:
            outcome = prepare_one(method_dir, out_root, opts, rig, retargeter_factory)
        except FlipRefused as exc:
            outcome = Outcome(name=name, verdict=VERDICT_REFUSED, reason=str(exc))
        except Exception as exc:  # one bad trajectory must not stop the other 29
            outcome = Outcome(name=name, verdict=VERDICT_ERROR, reason=f"{type(exc).__name__}: {exc}")
            log(f"{name}: error: {outcome.reason}")
        print(format_result_line(outcome), flush=True)
        outcomes.append(outcome)
    summary_path = Path(out_root) / SUMMARY_FILE
    write_summary(summary_path, outcomes, opts)
    log(f"wrote {summary_path}")
    return outcomes


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Turn a RobotSTAR bundle trajectory into a replay clip directory (spec1).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--method-dir", type=Path, metavar="DIR",
                        help="a sample's GT/ or Ours/ directory")
    source.add_argument("--all", type=Path, metavar="SAMPLES_DIR",
                        help="every */GT and */Ours under this directory; also writes <out>/summary.md")
    p.add_argument("--out", type=Path, required=True, metavar="CLIPS_DIR",
                   help="clips root; a clip lands in <out>/safe/ or <out>/rejected/")
    p.add_argument("--speeds", type=float, nargs="+", default=list(clip_audit.DEFAULT_SPEEDS),
                   help="speeds to audit; a clip is safe if any passes")
    p.add_argument("--cutoff-hz", type=float, default=DEFAULT_CUTOFF_HZ,
                   help="arm low-pass cutoff (zero-phase 2nd-order Butterworth)")
    p.add_argument("--max-step-deg", type=float, default=DEFAULT_MAX_STEP_DEG,
                   help="arm per-frame step clamp")
    p.add_argument("--trim-start", type=int, default=0, help="leading frames to drop first")
    p.add_argument("--trim-end", type=int, default=0, help="trailing frames to drop first")
    p.add_argument("--auto-trim", action="store_true",
                   help="keep the longest passing window of at least --min-seconds and re-audit")
    p.add_argument("--min-seconds", type=float, default=DEFAULT_MIN_SECONDS,
                   help="shortest window --auto-trim accepts")
    p.add_argument("--allow-flips", action="store_true",
                   help=f"smooth through a >= {FLIP_STEP_DEG:g} deg single-frame step instead of refusing")
    p.add_argument("--max-arm-torque-ratio", type=float, default=clip_audit.DEFAULT_MAX_ARM_TORQUE_RATIO,
                   help="pass threshold on peak arm torque as a fraction of the joint clamp")
    p.add_argument("--max-contact-force-n", type=float, default=clip_audit.DEFAULT_MAX_CONTACT_FORCE_N,
                   help="pass threshold on peak contact force")
    p.add_argument("--note", type=str, default="", help="reason, when a threshold was changed")
    p.add_argument("--model", type=Path, default=clip_audit.default_model_path(),
                   help="composed MJCF the audit replays on")
    p.add_argument("--retarget-config-dir", type=Path, default=DEFAULT_RETARGET_CONFIG_DIR,
                   help=f"directory holding {RETARGET_CONFIG_PATTERN}")
    p.add_argument("--hand-lp-alpha", type=float, default=DEFAULT_HAND_LP_ALPHA,
                   help="the retargeter's low-pass coefficient for the fingers, in (0, 1]: "
                        "the fraction of each new solve that is kept, so larger is less "
                        f"smoothing (default {DEFAULT_HAND_LP_ALPHA}; the configs' own value "
                        "is 0.2, which flattens fast finger motion)")
    return p.parse_args(argv)


def options_from_args(args: argparse.Namespace) -> Options:
    return Options(
        speeds=tuple(float(s) for s in args.speeds),
        cutoff_hz=float(args.cutoff_hz),
        max_step_deg=float(args.max_step_deg),
        trim_start=int(args.trim_start),
        trim_end=int(args.trim_end),
        auto_trim=bool(args.auto_trim),
        min_seconds=float(args.min_seconds),
        allow_flips=bool(args.allow_flips),
        max_arm_torque_ratio=float(args.max_arm_torque_ratio),
        max_contact_force_n=float(args.max_contact_force_n),
        note=str(args.note),
        retarget_config_dir=Path(args.retarget_config_dir),
        hand_lp_alpha=(None if args.hand_lp_alpha is None else float(args.hand_lp_alpha)))


def main(argv: Optional[Sequence[str]] = None,
         retargeter_factory: Optional[RetargeterFactory] = None) -> int:
    args = parse_args(argv)
    opts = options_from_args(args)
    try:
        opts.validate()
    except PrepareError as exc:
        log(f"error: {exc}")
        return EXIT_REFUSED

    rig = AuditRig(args.model)
    if args.method_dir is not None:
        try:
            outcome = prepare_one(args.method_dir, args.out, opts, rig, retargeter_factory)
        except FlipRefused as exc:
            print(format_result_line(Outcome(name=trajectory_name(args.method_dir),
                                             verdict=VERDICT_REFUSED, reason=str(exc))), flush=True)
            return EXIT_REFUSED
        except PrepareError as exc:
            log(f"error: {exc}")
            return EXIT_ERROR
        print(format_result_line(outcome), flush=True)
        return EXIT_OK

    try:
        run_all(args.all, args.out, opts, rig, retargeter_factory)
    except PrepareError as exc:
        log(f"error: {exc}")
        return EXIT_ERROR
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
