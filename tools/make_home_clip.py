#!/usr/bin/env python3
"""Generate and audit a rehome clip: a slow move from a measured pose to home.

Design: docs/spec/spec1_1.md. Operator command: docs/replay.md.

This writes an ordinary clip directory, the same file boundary every other clip
crosses (docs/spec/spec1.md, "Clip directory"), so the rehome motion goes
through the same publisher, the same G1 node interpolation and the same MuJoCo
audit as a recorded clip. There is no second motion path in the system and no
run-time layer anywhere: this tool runs to completion, files a clip, and exits
before anything is published.

Frame 0 is the start pose verbatim, so the first published frame is a no-op
against the pose the arms are already in. The rest is a half-cosine ease to
HOME_POSE_RAD, sized so peak joint velocity never exceeds HOME_PEAK_VEL_RAD_S.

The clip carries hand columns because the loader requires them, and they are
never published: scripts/replay.sh --home starts no hand driver. The hand pose
in those columns is what the audit assumed for contact geometry, and the audit
runs against two of them (open and curled), taking the worse.

    tools/make_home_clip.py --start-pose json:measured.json --out clips
    tools/make_home_clip.py --start-pose clip:clips/safe/90_sweep_joints_GT@last --out clips
    tools/make_home_clip.py --start-pose stand --home-pose zeros --out clips

Exit status: 0 the clip is filed under clips/home/ and may be played, 2 the
audit rejected it (filed under clips/rejected/) or the start pose was refused,
1 an error.

Importing this module pulls in numpy and the standard library only. MuJoCo is
imported inside the audit step, so the path arithmetic can be tested on a
machine that has no simulator.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants. Each one says where its value comes from.
# ---------------------------------------------------------------------------

TOOL_ID = "make_home_clip/1"

# Repo root is one directory above tools/ (same layout inside the container).
REPO_ROOT = Path(__file__).resolve().parents[1]

# The composed model the audit judges against, and the single source in this
# file for joint ranges, hand joint names and the stand keyframe. Same path
# tools/clip_audit.py uses, so the ranges this tool clamps to are the ranges
# the audit simulates.
MODEL_REL_PATH = "src/g1_wuji2_description/g1_29_wuji2_fixed.xml"

# MJCF joint and actuator names carry this suffix on the arm joints; the G1
# node's names and the clip's column names do not.
MJCF_JOINT_SUFFIX = "_joint"

# The model's rest keyframe, used only by --start-pose stand.
STAND_KEYFRAME = "stand"

SIDES = ("left", "right")

# 7 arm joints per side, the G1 node's order (robot_arm.py
# G1_29_ARM_JOINT_NAMES) and the arm_q.npz column order. Identical to
# clip_audit.ARM_JOINT_NAMES; tools/tests asserts that rather than trusting it.
ARM_JOINT_NAMES: Dict[str, List[str]] = {
    side: [
        f"{side}_shoulder_pitch",
        f"{side}_shoulder_roll",
        f"{side}_shoulder_yaw",
        f"{side}_elbow",
        f"{side}_wrist_roll",
        f"{side}_wrist_pitch",
        f"{side}_wrist_yaw",
    ]
    for side in SIDES
}
ARM_JOINTS_PER_SIDE = 7
HAND_JOINTS_PER_SIDE = 20

# MJCF hand joint names are the driver's hardware names behind this prefix.
# Stripping it gives hand_q20.npz's column order (clip_audit.HAND_JOINT_NAMES).
HAND_MJCF_PREFIX = {"left": "left_wuji_", "right": "right_wuji_"}

# Home. All-zeros, matching the vendor's "zero posture" in the arm_sdk example
# (unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py, stages
# 1 and 3 command q = (1 - ratio) * measured_q, then stage 4 ramps the arm_sdk
# weight to 0). Ending here leaves the arms in the state that vendor sequence
# leaves them, so G1ArmController.shutdown()'s weight ramp hands them back at
# the pose the onboard controller expects. The MJCF 'stand' keyframe was the
# rival and lost: it is a posed keyframe rather than a robot convention, its
# 1.28 rad elbow puts both hands in front of the abdomen where the audit's
# hand-to-hand contact pairs come from, and docs/architecture.md records that
# an arm with nothing publishing to it settles at all-zeros anyway. All-zeros
# also sits within 0.25 rad of where every committed clip starts and carries
# the lowest gravity load. Reasoning and the open audit: docs/spec/spec1_1.md.
HOME_POSE_RAD: Dict[str, Tuple[float, ...]] = {
    "left": (0.0,) * ARM_JOINTS_PER_SIDE,
    "right": (0.0,) * ARM_JOINTS_PER_SIDE,
}

# The candidates --home-pose can select. "zeros" is HOME_POSE_RAD above and the
# decision; "stand" is the rival, kept selectable so the audit matrix in
# docs/spec/spec1_1.md can be reproduced and re-run against a changed model
# rather than argued about. Selecting a candidate does not skip anything: every
# home pose goes through the same audit.
HOME_POSE_CHOICES = ("zeros", "stand")

# Peak joint velocity the ease is sized for. 40% of the 0.5 rad/s deploy
# screening velocity in tools/sweep_joint_limits.yaml. At the G1 node's wrist
# kd of 2 the damping term is then 0.4 Nm against a 5 Nm actuatorfrcrange (8%
# of the clamp), and 0.6 Nm against 25 Nm on the other arm joints. The hardware
# ceilings are 22 to 37 rad/s, two orders above this.
HOME_PEAK_VEL_RAD_S = 0.2

# Duration floor, so a short travel is still a visibly slow motion the operator
# can watch and interrupt rather than a twitch.
HOME_MIN_DURATION_S = 3.0

# Duration ceiling. Unreachable from any in-range start pose: the largest legal
# travel to all-zeros is 3.0892 rad (shoulder pitch at its lower limit), which
# sizes to 24.3 s. This only bounds a nonsense input.
HOME_MAX_DURATION_S = 30.0

# Frame rate. What every prepared clip uses, so the G1 node's one-publish-period
# interpolation adds the same 20 ms of latency it always does.
HOME_RATE_HZ = 50.0

# Speeds to audit. The duration is already in the clip, so there is nothing for
# a speed knob to do here except offer a second way to get it wrong. "Slower is
# not always safer" is established on this rig (docs/spec/spec1.md).
HOME_AUDIT_SPEEDS: Tuple[float, ...] = (1.0,)

# The audit's stand-in for a hand left mid-grasp: every flexion joint at this
# fraction of its upper range, abduction at zero. The real hand pose is not
# knowable offline and the hands are limp during a rehome anyway, so this is a
# repeatable worst case rather than a measurement. At 0.7 the finger PIP joints
# land near 1.47 rad, close to the curl the sweep clip's donor pose holds.
CURLED_FLEX_FRACTION = 0.7

# Hand joints whose name ends with this are abduction, not flexion, and stay at
# zero in the curled pose: spreading the fingers is not part of a grasp.
ABDUCTION_SUFFIX = "_abd"

# A retract waypoint (move shoulder roll outward first, then descend) was built
# and measured against the case it exists for, and dropped. From a pose folded
# against the torso the arms are already in contact before anything moves
# (42.6 N, torque ratio 1.00 at frame 0), and the dominant pair is
# shoulder_yaw_link against torso_link, which abduction does not relieve: with
# the waypoint 133.3 N, without it 133.4 N. The audit rejects that start either
# way, which is the right answer, and no path shape rescues a pose that is
# already jammed. docs/spec/spec1_1.md carries the table.

# Clip directory layout (docs/spec/spec1.md). A home clip that passes is filed
# under home/, which replay/clip.py accepts alongside safe/; one that fails is
# filed under rejected/, which it refuses.
HOME_DIR = "home"
REJECTED_DIR = "rejected"
CANDIDATE_DIR = "candidate"
ARM_FILE = "arm_q.npz"
HAND_FILE = "hand_q20.npz"
META_FILE = "clip.json"

VERDICT_SAFE = "safe"
VERDICT_REJECTED = "rejected"

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_REFUSED = 2

# Frames counted as "the start of the motion" when reporting whether contact
# force grew as the arms began to move. One second at HOME_RATE_HZ. A straight
# joint-space interpolation out of a contacting pose can press harder before it
# clears, and a peak that arrives here rather than later is that happening.
FIRST_SECOND_FRAMES = int(HOME_RATE_HZ)

# A joint within this of a range end counts as sitting on it when reporting a
# start pose the model would not allow. Far below any real pose difference and
# far above float noise.
RANGE_EPS = 1e-9


class HomeClipError(Exception):
    """A start pose or an option this tool refuses."""


def log(message: str) -> None:
    # stderr, so stdout carries only the filed clip path for the caller to read.
    print(f"make_home_clip: {message}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# The model tables: ranges, hand names, the stand keyframe
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ModelTables:
    """What this tool needs from the MJCF, read without a simulator.

    The model is fixed-base and every joint is a hinge, so a joint's index in
    worldbody declaration order is its qpos index. That is what lets the stand
    keyframe be read here rather than through mujoco.
    """

    path: Path
    sha256: str
    joint_names: Tuple[str, ...]
    ranges: Dict[str, Tuple[float, float]]
    stand_qpos: np.ndarray

    def index(self, joint: str) -> int:
        try:
            return self.joint_names.index(joint)
        except ValueError as exc:
            raise HomeClipError(f"{self.path}: no joint named {joint!r}") from exc

    def arm_columns(self, side: str) -> List[int]:
        return [self.index(f"{n}{MJCF_JOINT_SUFFIX}") for n in ARM_JOINT_NAMES[side]]

    def arm_ranges(self, side: str) -> Tuple[np.ndarray, np.ndarray]:
        rows = [self.ranges[f"{n}{MJCF_JOINT_SUFFIX}"] for n in ARM_JOINT_NAMES[side]]
        return (np.array([r[0] for r in rows]), np.array([r[1] for r in rows]))

    def hand_names(self, side: str) -> List[str]:
        prefix = HAND_MJCF_PREFIX[side]
        names = [n[len(prefix):] for n in self.joint_names if n.startswith(prefix)]
        if len(names) != HAND_JOINTS_PER_SIDE:
            raise HomeClipError(
                f"{self.path}: {len(names)} joints under {prefix!r}, expected {HAND_JOINTS_PER_SIDE}"
            )
        return names

    def stand_arm_pose(self) -> Dict[str, np.ndarray]:
        return {side: self.stand_qpos[self.arm_columns(side)].copy() for side in SIDES}


def default_model_path() -> Path:
    return REPO_ROOT / MODEL_REL_PATH


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_model_tables(model_path: Optional[Path] = None) -> ModelTables:
    """Read joint order, ranges and the stand keyframe out of the MJCF."""
    path = Path(model_path) if model_path else default_model_path()
    if not path.is_file():
        raise HomeClipError(f"{path}: model not found")
    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise HomeClipError(f"{path}: no <worldbody>")

    names: List[str] = []
    ranges: Dict[str, Tuple[float, float]] = {}
    for joint in worldbody.iter("joint"):
        kind = joint.get("type", "hinge")
        if kind != "hinge":
            # Every joint in the fixed-base composed model is a hinge, one qpos
            # each. Anything else would break the index mapping below, so say so
            # instead of computing a wrong keyframe.
            raise HomeClipError(f"{path}: joint {joint.get('name')!r} is type {kind!r}, not hinge")
        name = joint.get("name")
        if not name:
            raise HomeClipError(f"{path}: a joint has no name")
        names.append(name)
        raw = joint.get("range")
        if raw:
            lo, hi = (float(v) for v in raw.split())
            ranges[name] = (lo, hi)

    key = root.find(f"keyframe/key[@name='{STAND_KEYFRAME}']")
    if key is None:
        raise HomeClipError(f"{path}: no keyframe named {STAND_KEYFRAME!r}")
    qpos = np.array([float(v) for v in key.get("qpos").split()])
    if qpos.size != len(names):
        raise HomeClipError(
            f"{path}: keyframe has {qpos.size} qpos entries for {len(names)} hinge joints"
        )

    return ModelTables(
        path=path,
        sha256=sha256_file(path),
        joint_names=tuple(names),
        ranges=ranges,
        stand_qpos=qpos,
    )


# ---------------------------------------------------------------------------
# Hand poses for the audit
# ---------------------------------------------------------------------------

def open_hand_pose() -> Dict[str, np.ndarray]:
    """All-zeros: the pose the hand driver homes to on connect."""
    return {side: np.zeros(HAND_JOINTS_PER_SIDE) for side in SIDES}


def curled_hand_pose(model: ModelTables) -> Dict[str, np.ndarray]:
    """Every flexion joint at CURLED_FLEX_FRACTION of its upper range."""
    out: Dict[str, np.ndarray] = {}
    for side in SIDES:
        prefix = HAND_MJCF_PREFIX[side]
        values = []
        for name in model.hand_names(side):
            lo, hi = model.ranges[f"{prefix}{name}"]
            q = 0.0 if name.endswith(ABDUCTION_SUFFIX) else CURLED_FLEX_FRACTION * hi
            values.append(float(np.clip(q, lo, hi)))
        out[side] = np.array(values)
    return out


AUDIT_HAND_POSES = ("open", "curled")


def hand_pose(name: str, model: ModelTables) -> Dict[str, np.ndarray]:
    if name == "open":
        return open_hand_pose()
    if name == "curled":
        return curled_hand_pose(model)
    raise HomeClipError(f"unknown hand pose {name!r}; expected one of {list(AUDIT_HAND_POSES)}")


# ---------------------------------------------------------------------------
# The path
# ---------------------------------------------------------------------------

def duration_for(travel_rad: float) -> float:
    """Seconds for a half-cosine ease whose peak velocity is HOME_PEAK_VEL_RAD_S.

    s(t) = (1 - cos(pi t / T)) / 2 has peak velocity travel * pi / (2T), so
    T = travel * pi / (2 * v_peak), clamped to the floor and the ceiling.
    """
    unclamped = travel_rad * math.pi / (2.0 * HOME_PEAK_VEL_RAD_S)
    return float(min(max(unclamped, HOME_MIN_DURATION_S), HOME_MAX_DURATION_S))


def frame_count(duration_s: float) -> int:
    """Frames at HOME_RATE_HZ spanning at least ``duration_s``, ends included.

    Rounded up, not to nearest. A clip of n frames published at HOME_RATE_HZ
    takes (n - 1) / HOME_RATE_HZ seconds, so rounding the frame count down
    shortens the motion and raises its peak velocity above the limit the
    duration was chosen to respect. Measured on the stand pose: rounding to
    nearest gave 0.2003 rad/s against a 0.2 limit. Rounding up can only make
    the motion slower than asked.
    """
    return max(2, int(math.ceil(duration_s * HOME_RATE_HZ)) + 1)


def ease(n_frames: int) -> np.ndarray:
    """Half-cosine from 0 to 1 inclusive: zero velocity at both ends.

    Zero end velocity means the motion starts and stops without a commanded
    velocity step, at either end of the clip.
    """
    t = np.linspace(0.0, 1.0, n_frames)
    return (1.0 - np.cos(math.pi * t)) / 2.0


def peak_velocity(travel_rad: float, duration_s: float) -> float:
    return travel_rad * math.pi / (2.0 * duration_s)


def peak_acceleration(travel_rad: float, duration_s: float) -> float:
    return travel_rad * math.pi ** 2 / (2.0 * duration_s ** 2)


def segment(start: Dict[str, np.ndarray], goal: Dict[str, np.ndarray]) -> Tuple[Dict[str, np.ndarray], dict]:
    """One eased segment from start to goal, sized by its own largest travel."""
    travel = max(float(np.max(np.abs(goal[side] - start[side]))) for side in SIDES)
    n = frame_count(duration_for(travel))
    # The realised duration, which is what the publisher will take: n frames at
    # HOME_RATE_HZ span (n - 1) periods. Every number reported below is this
    # one, not the nominal duration it was sized from.
    duration = (n - 1) / HOME_RATE_HZ
    s = ease(n)[:, None]
    arm_q = {side: start[side][None, :] + s * (goal[side] - start[side])[None, :] for side in SIDES}
    # The ease's endpoints are exactly 0 and 1, but say so in the array rather
    # than relying on cos(pi) rounding: frame 0 must equal the measured pose to
    # the bit, because that is what makes the first published frame a no-op.
    for side in SIDES:
        arm_q[side][0] = start[side]
        arm_q[side][-1] = goal[side]
    block = {
        "frames": int(n),
        "duration_s": round(duration, 4),
        "travel_rad": round(travel, 6),
        "peak_vel_rad_s": round(peak_velocity(travel, duration), 4),
        "peak_acc_rad_s2": round(peak_acceleration(travel, duration), 4),
    }
    return arm_q, block


def home_pose(name: str, model: ModelTables) -> Dict[str, np.ndarray]:
    """The goal pose for a named candidate. 'zeros' is HOME_POSE_RAD."""
    if name == "zeros":
        return {side: np.array(HOME_POSE_RAD[side], dtype=np.float64) for side in SIDES}
    if name == "stand":
        return model.stand_arm_pose()
    raise HomeClipError(f"unknown home pose {name!r}; expected one of {list(HOME_POSE_CHOICES)}")


def build_path(start: Dict[str, np.ndarray], model: ModelTables,
               goal: str = "zeros") -> Tuple[Dict[str, np.ndarray], List[dict]]:
    """The whole motion: one eased segment from the start pose to the home pose."""
    arm_q, block = segment(start, home_pose(goal, model))
    return arm_q, [block]


def clamp_to_ranges(arm_q: Dict[str, np.ndarray], model: ModelTables) -> dict:
    """Clamp every frame after the first to the model's joint ranges, in place.

    Frame 0 is left alone on purpose: it is the measured pose, and a start pose
    outside the model range is a thing that happens on this rig (four wrist
    joints in the handoff note are commanded past their model limits). Making
    frame 0 anything other than measured would put a step back into the first
    published frame, which is the one thing this design buys.
    """
    report: Dict[str, dict] = {}
    for side in SIDES:
        lo, hi = model.arm_ranges(side)
        outside = {}
        for col, name in enumerate(ARM_JOINT_NAMES[side]):
            q0 = float(arm_q[side][0, col])
            if q0 < lo[col] - RANGE_EPS or q0 > hi[col] + RANGE_EPS:
                outside[name] = {"start": round(q0, 6),
                                 "range": [float(lo[col]), float(hi[col])]}
        arm_q[side][1:] = np.clip(arm_q[side][1:], lo, hi)
        if outside:
            report[side] = outside
    return report


# ---------------------------------------------------------------------------
# Start poses
# ---------------------------------------------------------------------------

def _as_arm_vector(values: Sequence[float], what: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (ARM_JOINTS_PER_SIDE,):
        raise HomeClipError(f"{what}: {arr.shape} values, expected {ARM_JOINTS_PER_SIDE}")
    if not np.all(np.isfinite(arr)):
        raise HomeClipError(f"{what}: non-finite values")
    return arr


def start_pose_from_json(path: Path) -> Dict[str, np.ndarray]:
    """Read what capture_arm_pose wrote: {side: {joint name: radians}}."""
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise HomeClipError(f"{path}: cannot read ({exc})") from exc
    out: Dict[str, np.ndarray] = {}
    for side in SIDES:
        if side not in data:
            raise HomeClipError(f"{path}: no {side!r} entry")
        block = data[side]
        missing = [n for n in ARM_JOINT_NAMES[side] if n not in block]
        if missing:
            raise HomeClipError(f"{path}: {side} is missing {missing}")
        out[side] = _as_arm_vector([block[n] for n in ARM_JOINT_NAMES[side]], f"{path} {side}")
    return out


def start_pose_from_clip(spec: str) -> Dict[str, np.ndarray]:
    """'clip:<dir>@first' or 'clip:<dir>@last': a frame of a prepared clip."""
    body = spec[len("clip:"):]
    if "@" in body:
        raw_dir, which = body.rsplit("@", 1)
    else:
        raw_dir, which = body, "last"
    if which not in ("first", "last"):
        raise HomeClipError(f"{spec}: frame must be 'first' or 'last', got {which!r}")
    clip_dir = Path(raw_dir).expanduser()
    if not clip_dir.is_absolute() and not (clip_dir / ARM_FILE).is_file():
        # Accept a repo-relative path from any working directory, the way
        # scripts/replay.sh writes clip arguments.
        clip_dir = REPO_ROOT / clip_dir
    arm_file = clip_dir / ARM_FILE
    if not arm_file.is_file():
        raise HomeClipError(f"{arm_file}: not found")
    with np.load(arm_file) as data:
        row = 0 if which == "first" else -1
        return {side: _as_arm_vector(np.asarray(data[side])[row], f"{arm_file} {side}")
                for side in SIDES}


def parse_start_pose(spec: str, model: ModelTables) -> Tuple[Dict[str, np.ndarray], str]:
    """One of: json:<path>, clip:<dir>[@first|@last], stand, zeros, 14 numbers."""
    if spec.startswith("json:"):
        path = Path(spec[len("json:"):]).expanduser()
        return start_pose_from_json(path), f"json:{path}"
    if spec.startswith("clip:"):
        return start_pose_from_clip(spec), spec
    if spec == "stand":
        return model.stand_arm_pose(), "stand keyframe"
    if spec == "zeros":
        return {side: np.zeros(ARM_JOINTS_PER_SIDE) for side in SIDES}, "zeros"
    parts = spec.replace(",", " ").split()
    if len(parts) == 2 * ARM_JOINTS_PER_SIDE:
        try:
            values = [float(p) for p in parts]
        except ValueError as exc:
            raise HomeClipError(f"start pose {spec!r}: not all numbers") from exc
        return ({"left": _as_arm_vector(values[:ARM_JOINTS_PER_SIDE], "start pose left"),
                 "right": _as_arm_vector(values[ARM_JOINTS_PER_SIDE:], "start pose right")},
                "explicit")
    raise HomeClipError(
        f"start pose {spec!r} not understood; expected json:<path>, clip:<dir>[@first|@last], "
        f"stand, zeros, or {2 * ARM_JOINTS_PER_SIDE} numbers"
    )


# ---------------------------------------------------------------------------
# Audit
# ---------------------------------------------------------------------------

def audit_path(arm_q: Dict[str, np.ndarray], model: ModelTables,
               speeds: Sequence[float]) -> Tuple[dict, str, List[float]]:
    """Audit the motion at each speed against both assumed hand poses.

    A speed passes only when both hand poses pass, because which one the hands
    are actually in is not knowable from here. MuJoCo is imported now rather
    than at module load so the path arithmetic above stays testable without it.
    """
    tools_dir = str(Path(__file__).resolve().parent)
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    import clip_audit  # noqa: E402

    if {s: list(v) for s, v in clip_audit.ARM_JOINT_NAMES.items()} != ARM_JOINT_NAMES:
        raise HomeClipError("arm joint names disagree with tools/clip_audit.py")

    rig = clip_audit.AuditRig(model.path)
    thresholds = clip_audit.Thresholds()
    frames = arm_q["left"].shape[0]

    per_speed: Dict[str, dict] = {}
    safe_speeds: List[float] = []
    for speed in speeds:
        per_pose = {}
        for pose_name in AUDIT_HAND_POSES:
            constant = hand_pose(pose_name, model)
            hand_q20 = {side: np.tile(constant[side], (frames, 1)) for side in SIDES}
            result = rig.run(arm_q, hand_q20, HOME_RATE_HZ, float(speed), thresholds)
            summary = result.summary
            # Did contact grow as the arms started moving, or was the peak just
            # the pose they were already in? This is the folded-start question.
            contact = np.asarray(result.frame_contact_force_n, dtype=np.float64)
            start_n = float(contact[0]) if contact.size else 0.0
            first_second_n = float(np.max(contact[:FIRST_SECOND_FRAMES])) if contact.size else 0.0
            summary["contact_at_start_n"] = round(start_n, 3)
            summary["peak_contact_first_second_n"] = round(first_second_n, 3)
            summary["contact_rise_first_second_n"] = round(first_second_n - start_n, 3)
            per_pose[pose_name] = summary
            log(f"audit {speed}x hands={pose_name}: "
                f"torque ratio {summary['peak_arm_torque_ratio']:.2f}, "
                f"contact {summary['peak_contact_force_n']:.1f} N "
                f"{summary.get('peak_contact_pair')}, "
                f"rise in the first second {summary['contact_rise_first_second_n']:+.1f} N, "
                f"pass={summary['pass']}")
        passed = all(per_pose[p]["pass"] for p in AUDIT_HAND_POSES)
        worst = max(AUDIT_HAND_POSES, key=lambda p: per_pose[p]["peak_arm_torque_ratio"])
        per_speed[clip_audit.speed_key(speed)] = {
            "pass": bool(passed), "worst_hand_pose": worst, "per_hand_pose": per_pose,
        }
        if passed:
            safe_speeds.append(float(speed))

    block = {
        "model": model.path.name,
        "model_sha256": model.sha256,
        "mujoco_version": getattr(clip_audit.mujoco, "__version__", "unknown"),
        "arm_gains": clip_audit.ArmGains().as_dict(),
        "thresholds": thresholds.as_dict(),
        "speeds": [float(s) for s in speeds],
        "hand_poses": list(AUDIT_HAND_POSES),
        "note": "hand columns are audited for contact geometry only; --home publishes none",
        "first_second_frames": FIRST_SECOND_FRAMES,
        "per_speed": per_speed,
    }
    verdict = VERDICT_SAFE if safe_speeds else VERDICT_REJECTED
    return block, verdict, safe_speeds


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def clip_name(now: Optional[datetime] = None) -> str:
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    return f"home_{stamp}"


def write_clip_dir(out_root: Path, name: str, verdict: str, arm_q: Dict[str, np.ndarray],
                   hand_q20: Dict[str, np.ndarray], clip_json: dict) -> Path:
    """Write the three files under candidate/<name>, then move them into place.

    Deliberately not tools/prepare_clip.py's write_clip_dir: that one files a
    safe clip under safe/, which is tracked in git and holds recordings. A
    rehome clip is generated per invocation and belongs under home/.
    """
    out_root = Path(out_root)
    candidate = out_root / CANDIDATE_DIR / name
    if candidate.exists():
        shutil.rmtree(candidate)
    candidate.mkdir(parents=True)
    np.savez(candidate / ARM_FILE, **{s: np.asarray(arm_q[s], dtype=np.float64) for s in SIDES})
    np.savez(candidate / HAND_FILE, **{s: np.asarray(hand_q20[s], dtype=np.float64) for s in SIDES})
    (candidate / META_FILE).write_text(json.dumps(clip_json, indent=1, sort_keys=False) + "\n")

    final_dir = out_root / (HOME_DIR if verdict == VERDICT_SAFE else REJECTED_DIR) / name
    final_dir.parent.mkdir(parents=True, exist_ok=True)
    if final_dir.exists():
        shutil.rmtree(final_dir)
    shutil.move(str(candidate), str(final_dir))
    return final_dir


def build_clip_json(model: ModelTables, start: Dict[str, np.ndarray],
                    start_description: str, segments: List[dict],
                    frames: int, outside_range: dict, audit_block: dict,
                    verdict: str, safe_speeds: List[float],
                    audit_hand_pose: str, goal: str = "zeros") -> dict:
    total_duration = sum(b["duration_s"] for b in segments)
    return {
        "tool": TOOL_ID,
        "source": {
            "kind": "home",
            "start_pose": start_description,
            "start_rad": {side: [round(float(v), 6) for v in start[side]] for side in SIDES},
            "home_pose": goal,
            "home_rad": {side: [round(float(v), 6) for v in home_pose(goal, model)[side]]
                         for side in SIDES},
        },
        "frames": int(frames),
        "rate_hz": HOME_RATE_HZ,
        "arm_joint_names": {side: list(ARM_JOINT_NAMES[side]) for side in SIDES},
        "hand_joint_names": {side: model.hand_names(side) for side in SIDES},
        "home": {
            "ease": "half_cosine",
            "segments": segments,
            "duration_s": round(total_duration, 4),
            "peak_vel_rad_s": max(b["peak_vel_rad_s"] for b in segments),
            "peak_acc_rad_s2": max(b["peak_acc_rad_s2"] for b in segments),
            "limits": {
                "peak_vel_rad_s": HOME_PEAK_VEL_RAD_S,
                "min_duration_s": HOME_MIN_DURATION_S,
                "max_duration_s": HOME_MAX_DURATION_S,
            },
            "start_outside_model_range": outside_range,
        },
        "hands": {
            "published": False,
            "columns": audit_hand_pose,
            "note": "scripts/replay.sh --home starts no hand driver and publishes no hand topic",
        },
        "audit": audit_block,
        "safe_speeds": safe_speeds,
        "verdict": verdict,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="make_home_clip.py",
        description="Generate and audit a rehome clip from a start pose to the home pose.",
    )
    parser.add_argument("--start-pose", required=True,
                        help="json:<path>, clip:<dir>[@first|@last], stand, zeros, or 14 numbers")
    parser.add_argument("--out", default="clips", help="clip root (default clips)")
    parser.add_argument("--home-pose", choices=HOME_POSE_CHOICES, default="zeros",
                        help="goal pose; 'zeros' is the decision (docs/spec/spec1_1.md), "
                             "'stand' is the rival kept selectable so the audit matrix is reproducible")
    parser.add_argument("--speeds", type=float, nargs="+", default=list(HOME_AUDIT_SPEEDS),
                        help=f"speeds to audit (default {list(HOME_AUDIT_SPEEDS)})")
    parser.add_argument("--name", default=None, help="clip directory name (default home_<UTC stamp>)")
    parser.add_argument("--model", default=None, help="MJCF to audit against")
    parser.add_argument("--columns", choices=AUDIT_HAND_POSES, default="open",
                        help="hand pose written into hand_q20.npz (never published; default open)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        model = load_model_tables(Path(args.model) if args.model else None)
        start, description = parse_start_pose(args.start_pose, model)

        arm_q, segments = build_path(start, model, goal=args.home_pose)
        outside = clamp_to_ranges(arm_q, model)
        frames = arm_q["left"].shape[0]

        for side in SIDES:
            travel = np.abs(arm_q[side][-1] - arm_q[side][0])
            worst = int(np.argmax(travel))
            log(f"{side}: largest travel {travel[worst]:.3f} rad on "
                f"{ARM_JOINT_NAMES[side][worst]}")
        log(f"start {description}, home {args.home_pose}, "
            f"{frames} frames at {HOME_RATE_HZ:g} Hz, "
            f"{sum(b['duration_s'] for b in segments):.1f} s, "
            f"peak {max(b['peak_vel_rad_s'] for b in segments):.3f} rad/s")
        if outside:
            log(f"start pose outside the model range: {outside}")

        audit_block, verdict, safe_speeds = audit_path(arm_q, model, args.speeds)

        columns = hand_pose(args.columns, model)
        hand_q20 = {side: np.tile(columns[side], (frames, 1)) for side in SIDES}
        clip_json = build_clip_json(model, start, description, segments, frames,
                                    outside, audit_block, verdict, safe_speeds, args.columns,
                                    goal=args.home_pose)
        name = args.name or clip_name()
        final_dir = write_clip_dir(Path(args.out), name, verdict, arm_q, hand_q20, clip_json)
    except HomeClipError as exc:
        print(f"make_home_clip: refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED

    if verdict != VERDICT_SAFE:
        log(f"REJECTED by the audit, filed at {final_dir}. Nothing will be published.")
        return EXIT_REFUSED
    log(f"safe at {safe_speeds}, filed at {final_dir}")
    print(final_dir)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
