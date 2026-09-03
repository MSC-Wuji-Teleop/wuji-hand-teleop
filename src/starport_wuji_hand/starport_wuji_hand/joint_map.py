"""Joint naming and command resolution for the Wuji Hand 2, 20 DoF, either hand.

One naming system, shared with the Hand 2 URDFs (``src/wujihand_urdf``), the composed G1 + hands
MJCF (``src/g1_wuji2_description``) and the clip directories ``prepare_clip.py`` writes:
anatomical names under a chirality prefix, ``r_thumb_cmc_flex`` on the right and
``l_thumb_cmc_flex`` on the left.

The order below IS the hardware index contract: position *i* in ``joint_names(side)`` is the
index the SDK reads and writes for that joint. The SDK itself names nothing -- it takes twenty
array slots -- so nothing downstream can recover the mapping if this order is wrong. It is a
literal rather than a file read at import so the package carries its own contract;
``test_joint_map.py`` fails if it drifts from the URDF's movable-joint declaration order or from
the composed MJCF's hand joint order.

Pure module: no ROS, no SDK. Everything here is unit-testable on any machine.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER

# Right first: the anchor side.
HAND_SIDES: tuple[str, ...] = ("right", "left")

# The chirality prefix.
SIDE_PREFIX: dict[str, str] = {"right": "r", "left": "l"}

# Hardware index order, pinned here and held to the URDF and the composed MJCF by
# test_joint_map.py. Fingers run thumb, index, middle, ring, pinky; each contributes four joints
# in the model's own order.
JOINT_NAMES_RIGHT: tuple[str, ...] = (
    "r_thumb_cmc_flex",
    "r_thumb_cmc_abd",
    "r_thumb_mcp",
    "r_thumb_ip",
    "r_index_finger_mcp_flex",
    "r_index_finger_mcp_abd",
    "r_index_finger_pip",
    "r_index_finger_dip",
    "r_middle_finger_mcp_flex",
    "r_middle_finger_mcp_abd",
    "r_middle_finger_pip",
    "r_middle_finger_dip",
    "r_ring_finger_mcp_flex",
    "r_ring_finger_mcp_abd",
    "r_ring_finger_pip",
    "r_ring_finger_dip",
    "r_pinky_mcp_flex",
    "r_pinky_mcp_abd",
    "r_pinky_pip",
    "r_pinky_dip",
)


# Each finger's bus segment carries its four joints plus a tactile node, so the firmware's node
# ids run 1-4, 6-9, 11-14, 16-19, 21-24 and skip every multiple of five. The state stream reports
# those ids, NOT joint indices: a reader that treats one as the other addresses the wrong joint on
# every finger but the thumb. Verified against the device's own joint labels for all twenty.
NODES_PER_FINGER = JOINTS_PER_FINGER + 1


def nid_to_index(nid: int) -> int:
    """Dense 0..19 joint index for a firmware bus-node id."""
    finger, within = divmod(nid - 1, NODES_PER_FINGER)
    if not (0 <= finger < NUM_FINGERS) or not (0 <= within < JOINTS_PER_FINGER):
        raise ValueError(f"{nid} is not a joint node id (it may be a tactile node)")
    return finger * JOINTS_PER_FINGER + within


def index_to_nid(index: int) -> int:
    """Firmware bus-node id for a dense 0..19 joint index."""
    if not 0 <= index < NUM_JOINTS:
        raise ValueError(f"joint index must be 0..{NUM_JOINTS - 1}, got {index}")
    finger, within = divmod(index, JOINTS_PER_FINGER)
    return finger * NODES_PER_FINGER + within + 1


def _check_side(side: str) -> str:
    if side not in SIDE_PREFIX:
        raise ValueError(f"side must be one of {list(HAND_SIDES)}, got {side!r}")
    return side


def mirror_to_side(name: str, side: str) -> str:
    """Rewrite a right-hand (``r_``) joint name into ``side``'s namespace.

    The two hands are a mirror pair: same twenty joints, same order, same ranges, differing only
    in geometry and prefix. Verified against the composed MJCF by ``test_joint_map.py``.
    """
    _check_side(side)
    if side == "left" and name.startswith("r_"):
        return "l_" + name[2:]
    return name


def joint_names(side: str = "right") -> tuple[str, ...]:
    """The twenty joint names for ``side``, in hardware index order."""
    _check_side(side)
    return tuple(mirror_to_side(name, side) for name in JOINT_NAMES_RIGHT)


def side_of(name: str) -> str | None:
    """The hand a joint name belongs to, or ``None`` if it names neither."""
    for side, prefix in SIDE_PREFIX.items():
        if name.startswith(f"{prefix}_"):
            return side
    return None


def name_to_index(side: str = "right") -> dict[str, int]:
    """``{joint_name: hardware_index}`` for ``side``."""
    return {name: i for i, name in enumerate(joint_names(side))}


def index_of(name: str, side: str = "right") -> int:
    """Hardware index of a joint name on ``side``. Raises ``KeyError`` if unknown."""
    return name_to_index(side)[name]


def resolve_command(
    names: Sequence[str] | None,
    positions: Sequence[float],
    current: np.ndarray,
    side: str = "right",
) -> np.ndarray:
    """Resolve a JointState-shaped command into a full ``(20,)`` target vector for ``side``.

    The result is always a freshly allocated array, never a view onto ``positions`` or
    ``current``, so a caller may keep publishing from its own buffer while the guard chain
    works on the returned one.

    ``names=None`` means ``positions`` is already a bare 20-array in hardware order. With names,
    only the listed joints are updated and every other joint keeps its value from ``current`` --
    holding is the right default for an unnamed joint; snapping it to zero would fling a finger.

    A name belonging to the OTHER hand is refused by side rather than reported as unknown. With
    both hands on one ``/joint_states`` that is the likeliest way a command goes astray, and the
    two diagnoses send an operator to different places.

    Every rejection raises rather than degrading: a partially-valid command is an upstream bug,
    not a request to be partially honored.
    """
    _check_side(side)
    current = np.asarray(current, dtype=np.float64)
    if current.shape != (NUM_JOINTS,):
        raise ValueError(f"current must be ({NUM_JOINTS},), got {current.shape}")

    if names is None:
        # np.array, not np.asarray: asarray is a no-op on an existing float64 array and would
        # hand back a view the caller still owns, so a later in-place guard stage would corrupt
        # the publisher's own buffer. Both paths return a buffer the caller cannot alias.
        try:
            target = np.array(positions, dtype=np.float64)
        except TypeError as exc:
            raise ValueError(f"unnamed command carries a non-numeric position: {exc}") from exc
        if target.shape != (NUM_JOINTS,):
            raise ValueError(f"unnamed command must carry exactly {NUM_JOINTS} positions, got {target.shape}")
        return target

    if len(names) != len(positions):
        raise ValueError(f"name/position length mismatch: {len(names)} names, {len(positions)} positions")
    if len(set(names)) != len(names):
        duplicates = sorted({n for n in names if list(names).count(n) > 1})
        raise ValueError(f"duplicate joint name(s) in command: {duplicates}")

    lookup = name_to_index(side)
    target = current.copy()
    for name, value in zip(names, positions, strict=True):
        # The lookup is guarded alone so a KeyError raised anywhere else is never mislabelled
        # as an unknown joint name.
        try:
            index = lookup[name]
        except KeyError:
            other = side_of(name)
            if other is not None and other != side:
                raise ValueError(
                    f"command names a {other}-hand joint {name!r}, but this driver is the {side} hand"
                ) from None
            raise ValueError(f"unknown joint name in command: {name!r}") from None
        try:
            target[index] = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"non-numeric position for joint {name!r}: {value!r}") from exc
    return target
