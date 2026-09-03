"""Clip directory loader and the replay rules. Pure numpy and json, no ROS.

A clip directory is the file boundary between the offline tool
(tools/prepare_clip.py) and the online publisher (replay_publisher). This
module reads one and decides whether it may be played and at what speed.
Everything the publisher checks lives here so it can be tested without ROS.

Layout of a clip directory (docs/spec/spec1.md, "Clip directory"):

    clips/{safe,home}/<name>/arm_q.npz      keys left, right: (T, 7) float64 rad
    clips/{safe,home}/<name>/hand_q20.npz   keys left, right: (T, 20) float64 rad
    clips/{safe,home}/<name>/clip.json      verdict, safe_speeds, names, audit

Rules, in the order they are applied:

1. The resolved parent directory must be named ``safe`` or ``home``. Each is
   written only by a tool that has just audited the clip: prepare_clip.py
   files ``safe``, make_home_clip.py files ``home``. A copy elsewhere is not
   played, whatever its clip.json says.
2. clip.json ``verdict`` must be ``"safe"`` and ``safe_speeds`` non-empty.
3. Both npz files must carry both sides with the declared shapes, the same
   frame count as clip.json, and only finite values.
4. Each side has 7 arm names (the G1 node's names, no ``_joint`` suffix) and
   20 hand names (the driver's hardware order), with that side's prefix.
5. A requested speed must be in (0, 1] and at most the fastest safe speed.

Every refusal raises ClipError with a plain message that names the file or
value at fault. Nothing here is a run-time safety layer: it is a format check
on a file, done once before the first message is published.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# The two sides every clip carries. Both npz files have both keys even when
# the publisher plays one side.
SIDES = ("left", "right")

# 7 arm joints per side: shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw.
# The G1 29-DoF arm, matching G1_29_ARM_JOINT_NAMES in g1_world_output.
ARM_JOINTS_PER_SIDE = 7

# 20 hand joints per side: 5 fingers x 4 joints, the Wuji Hand 2 driver's
# hardware index count (starport_wuji_hand joint_map.NUM_JOINTS).
HAND_JOINTS_PER_SIDE = 20

# Directory names a clip may be played from. A clip lands in one of these only
# after an audit passed: clips/safe/ from tools/prepare_clip.py, clips/home/
# from tools/make_home_clip.py, which generates and audits a rehome motion
# seconds before it is played (docs/spec/spec1_1.md). Either tool files a clip
# that failed under clips/rejected/, which is not in this tuple. The publisher
# refuses any directory whose parent is not named one of these.
PLAYABLE_PARENT_DIR_NAMES = ("safe", "home")

# The clip.json verdict value that allows playback.
SAFE_VERDICT = "safe"

# File names inside a clip directory (spec1.md, "Clip directory").
ARM_FILE = "arm_q.npz"
HAND_FILE = "hand_q20.npz"
META_FILE = "clip.json"

# Speeds above 1.0 are never audited (the audit's speed list is capped at
# 1.0), so a faster request has no audit result behind it.
MAX_SPEED = 1.0

# A speed typed as 0.5 must equal an audited 0.5 after the json round trip.
# 1e-9 is far below any speed difference that matters and far above float
# representation noise.
SPEED_TOLERANCE = 1e-9

# Name prefixes per side. Arm names are the G1 node's names
# (left_shoulder_pitch, ...); hand names are the driver's (l_thumb_cmc_flex,
# ...). The hand driver refuses a command naming the other hand, so a wrong
# prefix is caught here, before the first message.
ARM_NAME_PREFIX = {"left": "left_", "right": "right_"}
HAND_NAME_PREFIX = {"left": "l_", "right": "r_"}

# Values accepted by --arms and --hands.
SIDE_CHOICES = ("none", "left", "right", "both")


class ClipError(Exception):
    """A clip directory or a speed request that the publisher refuses."""


@dataclass(frozen=True)
class Clip:
    """One loaded clip. Arrays are float64 and already validated.

    ``arm_names[side]`` orders the columns of ``arm_q[side]``;
    ``hand_names[side]`` orders the columns of ``hand_q20[side]``.
    ``meta`` is the parsed clip.json, kept whole so a caller can log the
    audit numbers without re-reading the file.
    """

    name: str
    rate_hz: float
    frames: int
    arm_names: dict[str, tuple[str, ...]]
    arm_q: dict[str, np.ndarray]
    hand_names: dict[str, tuple[str, ...]]
    hand_q20: dict[str, np.ndarray]
    safe_speeds: tuple[float, ...]
    verdict: str
    meta: dict = field(repr=False)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ClipError(message)


def _get(meta: dict, key: str, path: Path):
    """Read a required key from clip.json with a message naming the file."""
    if key not in meta:
        raise ClipError(f"{path}: missing key {key!r}")
    return meta[key]


def _names_per_side(raw, count: int, prefixes: dict[str, str], what: str, path: Path) -> dict[str, tuple[str, ...]]:
    """Validate a {side: [names]} block: both sides, ``count`` strings each, right prefix."""
    _require(isinstance(raw, dict), f"{path}: {what} must be a mapping of side to names")
    out: dict[str, tuple[str, ...]] = {}
    for side in SIDES:
        _require(side in raw, f"{path}: {what} has no {side!r} entry")
        names = raw[side]
        _require(
            isinstance(names, list) and all(isinstance(n, str) for n in names),
            f"{path}: {what}[{side!r}] must be a list of strings",
        )
        _require(
            len(names) == count,
            f"{path}: {what}[{side!r}] has {len(names)} names, expected {count}",
        )
        bad = [n for n in names if not n.startswith(prefixes[side])]
        _require(
            not bad,
            f"{path}: {what}[{side!r}] has names without the {prefixes[side]!r} prefix: {bad}",
        )
        _require(len(set(names)) == count, f"{path}: {what}[{side!r}] has duplicate names")
        out[side] = tuple(names)
    return out


def _arrays_per_side(path: Path, width: int, frames: int) -> dict[str, np.ndarray]:
    """Load an npz with both sides as (frames, width) finite float arrays."""
    try:
        with np.load(path) as data:
            out: dict[str, np.ndarray] = {}
            for side in SIDES:
                _require(side in data.files, f"{path}: missing key {side!r} (has {data.files})")
                arr = np.asarray(data[side], dtype=np.float64)
                _require(
                    arr.ndim == 2 and arr.shape[1] == width,
                    f"{path} key {side!r}: shape {arr.shape}, expected (T, {width})",
                )
                _require(
                    arr.shape[0] == frames,
                    f"{path} key {side!r}: {arr.shape[0]} frames, clip.json says {frames}",
                )
                _require(bool(np.all(np.isfinite(arr))), f"{path} key {side!r}: non-finite values")
                out[side] = arr
            return out
    except (OSError, ValueError) as exc:
        # np.load raises ValueError on a file that is not an npz archive.
        raise ClipError(f"{path}: cannot read ({exc})") from exc


def load_clip(path: str | Path) -> Clip:
    """Load and validate a clip directory. Raises ClipError on any refusal."""
    clip_dir = Path(path).expanduser().resolve()
    _require(clip_dir.is_dir(), f"{clip_dir} is not a directory")
    _require(
        clip_dir.parent.name in PLAYABLE_PARENT_DIR_NAMES,
        f"{clip_dir} is not under a directory named one of {list(PLAYABLE_PARENT_DIR_NAMES)}: "
        "the publisher plays only clips a tool filed after an audit, under clips/safe/ "
        "(tools/prepare_clip.py) or clips/home/ (tools/make_home_clip.py)",
    )
    for fname in (META_FILE, ARM_FILE, HAND_FILE):
        _require((clip_dir / fname).is_file(), f"{clip_dir / fname} is missing")

    meta_path = clip_dir / META_FILE
    try:
        meta = json.loads(meta_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ClipError(f"{meta_path}: cannot parse ({exc})") from exc
    _require(isinstance(meta, dict), f"{meta_path}: top level must be an object")

    verdict = _get(meta, "verdict", meta_path)
    _require(verdict == SAFE_VERDICT, f"{meta_path}: verdict is {verdict!r}, not {SAFE_VERDICT!r}")

    frames = _get(meta, "frames", meta_path)
    _require(isinstance(frames, int) and frames >= 1, f"{meta_path}: frames must be a positive integer, got {frames!r}")

    rate_hz = _get(meta, "rate_hz", meta_path)
    _require(
        isinstance(rate_hz, (int, float)) and math.isfinite(rate_hz) and rate_hz > 0,
        f"{meta_path}: rate_hz must be a positive number, got {rate_hz!r}",
    )

    raw_speeds = _get(meta, "safe_speeds", meta_path)
    _require(isinstance(raw_speeds, list) and raw_speeds, f"{meta_path}: safe_speeds is empty; a safe clip lists at least one speed")
    speeds: list[float] = []
    for s in raw_speeds:
        _require(
            isinstance(s, (int, float)) and math.isfinite(s) and 0.0 < s <= MAX_SPEED,
            f"{meta_path}: safe_speeds entry {s!r} is not in (0, {MAX_SPEED}]",
        )
        speeds.append(float(s))

    arm_names = _names_per_side(_get(meta, "arm_joint_names", meta_path), ARM_JOINTS_PER_SIDE, ARM_NAME_PREFIX, "arm_joint_names", meta_path)
    hand_names = _names_per_side(_get(meta, "hand_joint_names", meta_path), HAND_JOINTS_PER_SIDE, HAND_NAME_PREFIX, "hand_joint_names", meta_path)

    arm_q = _arrays_per_side(clip_dir / ARM_FILE, ARM_JOINTS_PER_SIDE, frames)
    hand_q20 = _arrays_per_side(clip_dir / HAND_FILE, HAND_JOINTS_PER_SIDE, frames)

    return Clip(
        name=clip_dir.name,
        rate_hz=float(rate_hz),
        frames=int(frames),
        arm_names=arm_names,
        arm_q=arm_q,
        hand_names=hand_names,
        hand_q20=hand_q20,
        safe_speeds=tuple(speeds),
        verdict=str(verdict),
        meta=meta,
    )


def default_speed(clip: Clip) -> float:
    """The fastest speed the audit passed. load_clip guarantees the list is non-empty."""
    return max(clip.safe_speeds)


def check_speed(clip: Clip, speed: float) -> float:
    """Return ``speed`` as a float, or raise ClipError if the clip may not be played at it.

    The speed must BE one of ``safe_speeds``, not merely at or below the largest.
    A slower speed is not always a safer speed, and the audit is the only thing
    that knows which is which: ``05_test_G42xKICVj9U_5-5-rgb_front_GT`` passes at
    0.5 (peak arm torque ratio 0.736) and fails at both 1.0 (1.00) and 0.25
    (0.846), because most of the load on the wrist joints is a hand-to-hand
    contact reaction that barely changes with speed. Under the old rule, which
    only refused a speed above the fastest safe one, ``--speed 0.25`` on that
    clip was accepted and would have played a speed the audit had rejected.
    Playing an unaudited speed is exactly what the offline gate exists to stop.
    """
    try:
        s = float(speed)
    except (TypeError, ValueError) as exc:
        raise ClipError(f"speed must be a number, got {speed!r}") from exc
    _require(math.isfinite(s) and s > 0.0, f"speed must be > 0, got {s}")
    _require(s <= MAX_SPEED + SPEED_TOLERANCE, f"speed must be <= {MAX_SPEED}, got {s}")
    _require(
        any(abs(s - audited) <= SPEED_TOLERANCE for audited in clip.safe_speeds),
        f"speed {s} is not one of the clip's audited safe speeds {list(clip.safe_speeds)}; "
        "every other speed either failed the audit or was never run. Re-run "
        "tools/prepare_clip.py with --speeds to audit the speed you want.",
    )
    return s


def frame_period(clip: Clip, speed: float) -> float:
    """Timer period in seconds: one clip frame every 1 / (rate_hz * speed)."""
    return 1.0 / (clip.rate_hz * speed)


def duration_s(clip: Clip, speed: float) -> float:
    """Wall-clock length of one playback at ``speed``, before the hold."""
    return clip.frames * frame_period(clip, speed)


def parse_sides(arg: str) -> tuple[str, ...]:
    """Map an --arms / --hands value onto the sides it selects, in left, right order."""
    if arg == "none":
        return ()
    if arg == "left":
        return ("left",)
    if arg == "right":
        return ("right",)
    if arg == "both":
        return SIDES
    raise ValueError(f"side selection must be one of {list(SIDE_CHOICES)}, got {arg!r}")
