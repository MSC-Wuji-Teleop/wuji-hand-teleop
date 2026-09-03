"""The joint-order contract: the pinned hardware order, and the two hands' mirror relationship.

The order in ``joint_map`` IS the hardware index contract: position *i* is the slot the SDK
reads and writes, and the SDK names nothing, so nothing downstream can recover the mapping if
it is wrong. It is pinned as a literal in the module and checked here against two in-repo
sources: the Hand 2 URDFs in ``src/wujihand_urdf`` (the retargeter's model; its movable-joint
declaration order is the order clips are written in) and the composed G1 + hands MJCF in
``src/g1_wuji2_description`` (the model the offline audit runs). A reorder in either becomes a
test failure here instead of a silent rewiring that sends every finger's targets to the wrong
joint.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from starport_wuji_hand.joint_map import (
    HAND_SIDES,
    JOINT_NAMES_RIGHT,
    NUM_JOINTS,
    index_of,
    index_to_nid,
    joint_names,
    mirror_to_side,
    nid_to_index,
    resolve_command,
    side_of,
)

# This file lives at src/starport_wuji_hand/test/, so parents[2] is the workspace src/ directory
# that holds the two sibling packages read below.
_SRC = Path(__file__).resolve().parents[2]
# The Hand 2 URDFs the retargeter solves on (Beta 2; same 20 joints and order as Beta 1).
URDF_DIR = _SRC / "wujihand_urdf"
# The composed G1 + two hands model the offline audit runs.
COMPOSED_MJCF = _SRC / "g1_wuji2_description" / "g1_29_wuji2_fixed.xml"

FINGER_STEMS = ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky")


def _urdf_movable_joints(side: str) -> tuple[str, ...]:
    """Movable (``type != fixed``) joint names of ``side``'s URDF, in declaration order."""
    root = ET.parse(URDF_DIR / f"wujihand_{side}.urdf").getroot()
    return tuple(joint.get("name") for joint in root.findall("joint") if joint.get("type") != "fixed")


def _mjcf_hand_joints(side: str) -> dict[str, tuple[float, float]]:
    """``{driver joint name: (lower, upper)}`` for ``side``'s hand in the composed MJCF, in document order.

    In the composed model each hand joint is the driver's name under a ``{side}_wuji_`` prefix:
    ``left_wuji_l_thumb_cmc_flex`` for the driver's ``l_thumb_cmc_flex``.
    """
    prefix = f"{side}_wuji_"
    out: dict[str, tuple[float, float]] = {}
    for joint in ET.parse(COMPOSED_MJCF).getroot().iter("joint"):
        name = joint.get("name") or ""
        if not name.startswith(prefix):
            continue
        lower, upper = (float(v) for v in joint.get("range").split())
        out[name[len(prefix) :]] = (lower, upper)
    return out


@pytest.mark.parametrize("side", HAND_SIDES)
def test_the_pinned_order_matches_the_urdf_declaration_order(side):
    # The drift gate. The module holds the order as a literal so the package carries its own
    # contract and reads nothing across the repo at import; this keeps that literal honest against
    # the URDF the retargeter solves on, whose movable-joint order is the order clips are written in.
    assert joint_names(side) == _urdf_movable_joints(side)


@pytest.mark.parametrize("side", HAND_SIDES)
def test_the_pinned_order_matches_the_composed_mjcf(side):
    # The audit model's hand joints, in document order, are the same twenty names in the same order.
    assert tuple(_mjcf_hand_joints(side)) == joint_names(side)


def test_the_pinned_order_is_finger_major_and_20_long():
    assert len(JOINT_NAMES_RIGHT) == NUM_JOINTS
    # Pinning the whole order is what catches a swap WITHIN one finger -- abduction landing on the
    # MCP joint, say -- which finger-block membership alone cannot see.
    for finger_id, stem in enumerate(FINGER_STEMS):
        block = JOINT_NAMES_RIGHT[finger_id * 4 : finger_id * 4 + 4]
        assert all(stem in name for name in block), (finger_id, stem, block)


def test_index_is_finger_major():
    for finger_id in range(5):
        for joint_id in range(4):
            name = JOINT_NAMES_RIGHT[finger_id * 4 + joint_id]
            assert index_of(name) == finger_id * 4 + joint_id


def test_left_is_the_right_order_mirrored():
    left = joint_names("left")
    assert len(left) == NUM_JOINTS
    assert left == tuple("l_" + name[2:] for name in JOINT_NAMES_RIGHT)
    # Same slot, same joint, either hand: the index contract does not depend on the side.
    for i, name in enumerate(left):
        assert index_of(name, "left") == i


def test_the_two_hands_share_no_joint_name():
    assert not set(joint_names("right")) & set(joint_names("left"))


def test_side_of_names_the_hand_or_nothing():
    assert side_of("r_thumb_ip") == "right"
    assert side_of("l_thumb_ip") == "left"
    assert side_of("left_robotiq_85_left_knuckle_joint") is None
    assert side_of("shoulder_pan_joint") is None


def test_mirror_to_side_is_identity_on_its_own_side():
    assert mirror_to_side("r_thumb_ip", "right") == "r_thumb_ip"
    assert mirror_to_side("r_thumb_ip", "left") == "l_thumb_ip"


@pytest.mark.parametrize("side", ["middle", "", "RIGHT"])
def test_an_unknown_side_is_refused(side):
    with pytest.raises(ValueError, match="side must be one of"):
        joint_names(side)


def test_the_two_hands_are_a_mirror_pair_in_the_composed_mjcf():
    """The assumption the whole side-parameterisation rests on, checked against the audit model.

    If the hands ever stop being a mirror -- a different joint order, a different range -- then
    mirroring names is no longer sound and each side needs its own inventory.
    """
    right, left = _mjcf_hand_joints("right"), _mjcf_hand_joints("left")
    assert len(right) == NUM_JOINTS and len(left) == NUM_JOINTS
    assert list(left) == [mirror_to_side(n, "left") for n in right], "joint ORDER differs between the hands"
    for name, span in right.items():
        assert left[mirror_to_side(name, "left")] == span, f"{name} range differs between the hands"


def test_hand_sides_lists_right_first():
    # Right is the anchor side.
    assert HAND_SIDES == ("right", "left")


def test_index_of_rejects_unknown_name():
    with pytest.raises(KeyError):
        index_of("right_finger9_joint1")


def test_resolve_command_refuses_the_other_hands_joint_by_side():
    # With both hands on one /joint_states this is the likeliest way a command goes astray, and it
    # must not read as an unknown joint: the two diagnoses send an operator to different places.
    with pytest.raises(ValueError, match="names a left-hand joint"):
        resolve_command(["l_thumb_ip"], [0.1], np.zeros(NUM_JOINTS), side="right")
    with pytest.raises(ValueError, match="names a right-hand joint"):
        resolve_command(["r_thumb_ip"], [0.1], np.zeros(NUM_JOINTS), side="left")


def test_resolve_command_places_a_left_joint_at_its_index_on_the_left_hand():
    out = resolve_command(["l_pinky_dip"], [0.25], np.zeros(NUM_JOINTS), side="left")
    assert out[index_of("l_pinky_dip", "left")] == 0.25
    assert np.count_nonzero(out) == 1


def test_resolve_command_accepts_a_bare_20_array():
    current = np.zeros(NUM_JOINTS)
    out = resolve_command(None, list(range(NUM_JOINTS)), current)
    assert out.shape == (NUM_JOINTS,)
    assert out.dtype == np.float64
    np.testing.assert_allclose(out, np.arange(NUM_JOINTS, dtype=np.float64))


def test_resolve_command_rejects_wrong_length_bare_array():
    with pytest.raises(ValueError, match=str(NUM_JOINTS)):
        resolve_command(None, [0.0] * (NUM_JOINTS - 1), np.zeros(NUM_JOINTS))


def test_resolve_command_named_subset_holds_unnamed_joints():
    current = np.full(NUM_JOINTS, 0.5)
    out = resolve_command(["r_index_finger_pip"], [1.25], current)
    assert out[6] == pytest.approx(1.25)
    # Every other joint keeps its current value rather than snapping to zero.
    assert out[np.arange(NUM_JOINTS) != 6].tolist() == [0.5] * (NUM_JOINTS - 1)


def test_resolve_command_rejects_unknown_joint_name():
    with pytest.raises(ValueError, match="unknown joint"):
        resolve_command(["not_a_joint"], [0.0], np.zeros(NUM_JOINTS))


def test_resolve_command_rejects_duplicate_joint_names():
    with pytest.raises(ValueError, match="duplicate joint"):
        resolve_command(["r_thumb_cmc_flex"] * 2, [0.0, 1.0], np.zeros(NUM_JOINTS))


def test_resolve_command_rejects_name_position_length_mismatch():
    with pytest.raises(ValueError, match="length"):
        resolve_command(["r_thumb_cmc_flex", "r_thumb_cmc_abd"], [0.0], np.zeros(NUM_JOINTS))


def test_resolve_command_never_returns_a_view_of_its_input():
    # A publisher keeps the array it passed in; the guard chain mutates what it gets back. If the
    # two were the same buffer, clamping or rate-limiting would corrupt the publisher's own copy.
    bare = np.zeros(NUM_JOINTS)
    out = resolve_command(None, bare, np.zeros(NUM_JOINTS))
    assert out is not bare
    out[3] = 9.0
    assert bare[3] == 0.0

    current = np.zeros(NUM_JOINTS)
    named = resolve_command(["r_thumb_cmc_flex"], [1.0], current)
    assert named is not current
    named[5] = 9.0
    assert current[5] == 0.0


def test_resolve_command_rejects_non_numeric_positions_as_valueerror():
    # Callers refuse a malformed command by catching ValueError, so a TypeError must not escape
    # from either path -- it would take a live command callback down instead of being refused.
    # The annotation is deliberately violated here: the runtime values arrive off the wire, where
    # nothing enforces it, so the type-checker suppressions are the point of the test, not a wart.
    with pytest.raises(ValueError):
        resolve_command(["r_thumb_cmc_flex"], [[0.5]], np.zeros(NUM_JOINTS))  # ty: ignore[invalid-argument-type]
    with pytest.raises(ValueError):
        resolve_command(["r_thumb_cmc_flex"], ["nan-ish"], np.zeros(NUM_JOINTS))  # ty: ignore[invalid-argument-type]
    with pytest.raises(ValueError):
        resolve_command(None, [{}] * NUM_JOINTS, np.zeros(NUM_JOINTS))  # ty: ignore[invalid-argument-type]


def test_bus_node_ids_round_trip_to_joint_indices():
    # The stream reports firmware node ids, not joint indices, because each finger's bus carries a
    # tactile node alongside its four joints. Treating one as the other addresses the wrong joint
    # on every finger but the thumb.
    assert [index_to_nid(i) for i in range(NUM_JOINTS)] == [
        1,
        2,
        3,
        4,
        6,
        7,
        8,
        9,
        11,
        12,
        13,
        14,
        16,
        17,
        18,
        19,
        21,
        22,
        23,
        24,
    ]
    assert all(nid_to_index(index_to_nid(i)) == i for i in range(NUM_JOINTS))


@pytest.mark.parametrize("nid", [0, 5, 10, 15, 20, 25])
def test_a_tactile_node_id_is_refused_rather_than_mapped(nid):
    # Every multiple of five is a tactile node. Silently mapping one would land a tactile reading
    # on a joint.
    with pytest.raises(ValueError):
        nid_to_index(nid)
