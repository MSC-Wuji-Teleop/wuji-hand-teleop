"""Wrist re-clock: rotate the extracted wrist placement to this rig's hand mount.

The RobotSTAR bundle's arm joints were solved against the authors' own model,
which mounts the legacy Wuji hand on the G1 wrist at a different clock angle
than the adapter on this rig. Replaying those joints reproduces the bundle's
wrist link and puts the hand about 90 deg off about the forearm axis, left
and right in opposite senses (docs/issues/wrist-clock-2026-09-11.md).

The correction is a rotation of {side}_wrist_yaw_link about its own +x, the
forearm and mount axis, applied to the placement the source joints imply
(model.targets) before the arm is re-solved. It is not a roll offset: the G1
wrist is roll, pitch, yaw along that axis, so rotating the last link swaps
the roles of pitch and yaw as well. closed_form_wrist is the exact joint-space
answer when only the three wrist joints move; it shifts the wrist link on the
pitch joint's lever, which is why the IK, and not this function, produces the
clip. It seeds the first frame and pins the tests.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from clip_audit import SIDES

from .model import ARM_JOINTS_PER_SIDE, ArmTargets

# Constants. Each one says where its value comes from.

# target_meta.json "detected_hand_model" of a bundle solved against the legacy
# hand mount. prepare_clip.py copies it into clip.json "source".
BUNDLE_HAND_MODEL = "legacy_wuji"

# clip.json key, under "source", that carries the value above.
META_SOURCE_KEY = "source"
META_HAND_MODEL_KEY = "detected_hand_model"

# Degrees about {side}_wrist_yaw_link +x that take the bundle's wrist link to
# the one this rig's mount needs. Fitted 2026-09-11 over all 30 bundle
# trajectories at +86 (left) and -101 (right); 90 deg mirrored is the working
# value until the bundle authors' mount transform is read
# (docs/issues/wrist-clock-2026-09-11.md, "Evidence").
BUNDLE_WRIST_CLOCK_DEG: Dict[str, float] = {"left": 90.0, "right": -90.0}

# --wrist-clock values. auto applies the bundle clock when clip.json says the
# source is the legacy hand model; bundle and none force it; "L,R" gives the
# two angles in degrees.
CLOCK_AUTO = "auto"
CLOCK_BUNDLE = "bundle"
CLOCK_NONE = "none"

# Wrist columns inside a side's 7-vector (clip_audit.ARM_JOINT_NAMES order).
WRIST_LOCAL = (4, 5, 6)

# A clock this small, in degrees, is treated as no rotation at all.
ZERO_ANGLE_DEG = 1e-12


def rotation_about_x(deg: float) -> np.ndarray:
    """(3, 3) rotation about +x by deg."""
    return Rotation.from_rotvec([math.radians(deg), 0.0, 0.0]).as_matrix()


def parse_clock(value: str) -> Optional[Dict[str, float]]:
    """--wrist-clock text to per-side degrees; None means no re-clock.

    CLOCK_AUTO is resolved by clock_for_clip, not here.
    """
    text = value.strip().lower()
    if text == CLOCK_NONE:
        return None
    if text == CLOCK_BUNDLE:
        return dict(BUNDLE_WRIST_CLOCK_DEG)
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError(f"--wrist-clock must be {CLOCK_AUTO}, {CLOCK_BUNDLE}, {CLOCK_NONE} "
                         f"or 'LEFT_DEG,RIGHT_DEG', got {value!r}")
    try:
        left, right = (float(p) for p in parts)
    except ValueError as error:
        raise ValueError(f"--wrist-clock angles must be numbers, got {value!r}") from error
    if not (math.isfinite(left) and math.isfinite(right)):
        raise ValueError(f"--wrist-clock angles must be finite, got {value!r}")
    return {"left": left, "right": right}


def clip_hand_model(meta: Dict) -> Optional[str]:
    """clip.json source.detected_hand_model, or None."""
    source = meta.get(META_SOURCE_KEY) if isinstance(meta, dict) else None
    if not isinstance(source, dict):
        return None
    value = source.get(META_HAND_MODEL_KEY)
    return str(value) if value is not None else None


def clock_for_clip(flag: str, meta: Dict) -> tuple:
    """Resolve the flag against the clip's provenance.

    Returns (per-side degrees or None, one-line reason for the report).
    """
    if flag.strip().lower() == CLOCK_AUTO:
        hand_model = clip_hand_model(meta)
        if hand_model == BUNDLE_HAND_MODEL:
            return dict(BUNDLE_WRIST_CLOCK_DEG), (
                f"clip.json {META_SOURCE_KEY}.{META_HAND_MODEL_KEY} is {BUNDLE_HAND_MODEL}")
        if hand_model is None:
            return None, f"no {META_SOURCE_KEY}.{META_HAND_MODEL_KEY} in clip.json"
        return None, f"clip.json {META_SOURCE_KEY}.{META_HAND_MODEL_KEY} is {hand_model}"
    clock = parse_clock(flag)
    return clock, f"--wrist-clock {flag}"


def reclock_placement(pin, wrist, deg: float):
    """The wrist placement rotated about its own +x by deg; translation kept."""
    if abs(deg) < ZERO_ANGLE_DEG:
        return wrist.copy()
    return pin.SE3(np.asarray(wrist.rotation) @ rotation_about_x(deg),
                   np.asarray(wrist.translation).copy())


def reclock_targets(pin, targets: Dict[str, ArmTargets],
                    clock: Optional[Dict[str, float]]) -> Dict[str, ArmTargets]:
    """Per-side re-clock of one frame's targets; identity when clock is None."""
    if clock is None:
        return targets
    return {side: ArmTargets(wrist=reclock_placement(pin, targets[side].wrist, clock[side]),
                             elbow=np.asarray(targets[side].elbow).copy())
            for side in SIDES}


def closed_form_wrist(q7: Sequence[float], deg: float) -> np.ndarray:
    """The three wrist joints that give R_x(r) R_y(p) R_z(y) R_x(deg) exactly.

    Shoulder and elbow columns are copied. Not clamped to any range: the
    caller decides. The wrist link's origin moves when pitch changes, so this
    is the orientation answer, not the placement answer.
    """
    q = np.array(q7, dtype=float).copy()
    if q.shape != (ARM_JOINTS_PER_SIDE,):
        raise ValueError(f"q7 must be ({ARM_JOINTS_PER_SIDE},), got {q.shape}")
    roll, pitch, yaw = (q[i] for i in WRIST_LOCAL)
    # Intrinsic X, then Y, then Z: the wrist chain's roll, pitch, yaw axes all
    # lie along the same line at zero, and each joint's origin is offset along
    # its parent's x, so orientations compose as this product.
    combined = Rotation.from_euler("XYZ", [roll, pitch, yaw]) * Rotation.from_rotvec(
        [math.radians(deg), 0.0, 0.0])
    q[list(WRIST_LOCAL)] = combined.as_euler("XYZ")
    return q


def closed_form_seed(arm_all: np.ndarray, clock: Optional[Dict[str, float]],
                     lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """A (14,) seed for the first frame: the source wrists re-clocked in closed form, clamped.

    Seeding the first solve with the source joints would start it 90 deg
    from the target; this starts it on the right branch.
    """
    seed = np.array(arm_all, dtype=float).copy()
    if clock is None:
        return seed
    for i, side in enumerate(SIDES):
        cols = slice(i * ARM_JOINTS_PER_SIDE, (i + 1) * ARM_JOINTS_PER_SIDE)
        seed[cols] = closed_form_wrist(seed[cols], clock[side])
    return np.clip(seed, lower, upper)
