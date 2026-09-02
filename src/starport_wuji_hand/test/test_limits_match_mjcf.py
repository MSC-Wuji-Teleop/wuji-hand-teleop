"""The committed limits YAML is the safety artifact: it is what the guard chain clamps to.

Two independent gates per hand. ``EXPECTED_LIMITS`` pins all twenty pairs as literals, so every
bound is asserted with no model on disk. ``test_committed_yaml_matches_the_composed_mjcf`` then
holds each hand's YAML to the ``range`` attribute of that hand's joints in the composed G1 + hands
MJCF in ``src/g1_wuji2_description``: the model the offline audit runs, and the envelope
``prepare_clip.py`` clamps retargeted hand targets to. The two must agree, or the audit would judge
a clip against one envelope and the driver would clamp it to another.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from starport_wuji_hand.joint_map import HAND_SIDES, NUM_JOINTS, joint_names, mirror_to_side
from starport_wuji_hand.limits_io import limits_filename, load_limits_mapping
from starport_wuji_hand.safety import Limits

# Resolved from this file rather than the working directory, so the paths hold wherever pytest is
# invoked from. This file lives at src/starport_wuji_hand/test/: parents[1] is the package,
# parents[2] the workspace src/.
CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
COMPOSED_MJCF = Path(__file__).resolve().parents[2] / "g1_wuji2_description" / "g1_29_wuji2_fixed.xml"

# Both files write each bound as a three-decimal literal, so a real disagreement is at least
# 1e-3 rad; 1e-6 absorbs float parsing and nothing else.
RANGE_TOLERANCE_RAD = 1e-6

# The driver's default limit_margin (hand_node.py), so the shrink check below runs on the value
# the guard chain really uses.
DEFAULT_LIMIT_MARGIN_RAD = 0.02

# Every right-hand joint's (lower, upper) in radians, transcribed from the composed MJCF's
# right_wuji_r_* joint ranges. Per finger, the first joint is the base flex, the second the
# abduction, the third the middle flex and the fourth the tip. Fingers 2-5 are identical to each
# other; the thumb shares only its tip bounds with them, and its middle flex travels less than
# their middle flex, not more -- do not "correct" it upward to match. The left hand mirrors these
# name for name.
EXPECTED_LIMITS: dict[str, tuple[float, float]] = {
    "r_thumb_cmc_flex": (-1.187, 1.291),
    "r_thumb_cmc_abd": (-1.484, 0.698),
    "r_thumb_mcp": (-1.047, 1.57),
    "r_thumb_ip": (-1.047, 1.57),
    "r_index_finger_mcp_flex": (-1.047, 1.57),
    "r_index_finger_mcp_abd": (-0.698, 0.698),
    "r_index_finger_pip": (-1.047, 2.094),
    "r_index_finger_dip": (-1.047, 1.57),
    "r_middle_finger_mcp_flex": (-1.047, 1.57),
    "r_middle_finger_mcp_abd": (-0.698, 0.698),
    "r_middle_finger_pip": (-1.047, 2.094),
    "r_middle_finger_dip": (-1.047, 1.57),
    "r_ring_finger_mcp_flex": (-1.047, 1.57),
    "r_ring_finger_mcp_abd": (-0.698, 0.698),
    "r_ring_finger_pip": (-1.047, 2.094),
    "r_ring_finger_dip": (-1.047, 1.57),
    "r_pinky_mcp_flex": (-1.047, 1.57),
    "r_pinky_mcp_abd": (-0.698, 0.698),
    "r_pinky_pip": (-1.047, 2.094),
    "r_pinky_dip": (-1.047, 1.57),
}


def _committed(side: str) -> str:
    return str(CONFIG_DIR / limits_filename(side))


def _mjcf_ranges(side: str) -> dict[str, tuple[float, float]]:
    """``{driver joint name: (lower, upper)}`` from the composed MJCF's ``range`` attributes.

    The composed model names each hand joint ``{side}_wuji_<driver name>``. Only ``range`` is
    read, never ``actuatorfrcrange``: that is a force bound that also ends in "range".
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
def test_committed_yaml_covers_all_twenty_joints(side):
    mapping = load_limits_mapping(_committed(side))
    assert sorted(mapping) == sorted(joint_names(side))


@pytest.mark.parametrize("side", HAND_SIDES)
def test_committed_yaml_builds_valid_limits(side):
    names = joint_names(side)
    limits = Limits.from_mapping(load_limits_mapping(_committed(side)), margin=DEFAULT_LIMIT_MARGIN_RAD, names=names)
    # The margin has to land on both sides and shrink the envelope, never widen it -- a sign slip
    # here would hand the guard chain soft limits wider than the hardware's own.
    np.testing.assert_allclose(limits.lower, limits.raw_lower + DEFAULT_LIMIT_MARGIN_RAD)
    np.testing.assert_allclose(limits.upper, limits.raw_upper - DEFAULT_LIMIT_MARGIN_RAD)
    assert np.all(limits.lower > limits.raw_lower)
    assert np.all(limits.upper < limits.raw_upper)


@pytest.mark.parametrize("side", HAND_SIDES)
def test_committed_yaml_matches_the_composed_mjcf(side):
    # All twenty joints, to RANGE_TOLERANCE_RAD, against the model the audit judges clips on.
    modelled = _mjcf_ranges(side)
    committed = load_limits_mapping(_committed(side))
    assert len(modelled) == NUM_JOINTS, f"the composed MJCF carries {len(modelled)} {side} hand joints, not {NUM_JOINTS}"
    assert sorted(modelled) == sorted(committed)
    for name in joint_names(side):
        np.testing.assert_allclose(
            committed[name],
            modelled[name],
            rtol=0.0,
            atol=RANGE_TOLERANCE_RAD,
            err_msg=f"{name}: {limits_filename(side)} disagrees with {COMPOSED_MJCF.name}",
        )


def test_malformed_yaml_names_the_file_and_the_key(tmp_path):
    no_joints = tmp_path / "no_joints.yaml"
    no_joints.write_text("something_else: 1\n")
    with pytest.raises(ValueError, match="no_joints"):
        load_limits_mapping(str(no_joints))

    no_bound = tmp_path / "no_bound.yaml"
    no_bound.write_text("joints:\n  r_thumb_cmc_flex:\n    lower: -1.0\n")
    with pytest.raises(ValueError, match="upper"):
        load_limits_mapping(str(no_bound))

    # A bare `joints:` loads as None and a list-shaped one has no .items(); both would otherwise
    # escape as an AttributeError naming neither the file nor the key.
    empty_joints = tmp_path / "empty_joints.yaml"
    empty_joints.write_text("joints:\n")
    with pytest.raises(ValueError, match="empty_joints"):
        load_limits_mapping(str(empty_joints))

    list_joints = tmp_path / "list_joints.yaml"
    list_joints.write_text("joints:\n  - r_thumb_cmc_flex\n")
    with pytest.raises(ValueError, match="list_joints"):
        load_limits_mapping(str(list_joints))


@pytest.mark.parametrize("side", HAND_SIDES)
def test_known_values_are_present(side):
    # All twenty pairs as literals, model-independent: a regeneration against the wrong model, or
    # one that gave a joint another joint's class, is caught here even with no MJCF on disk.
    committed = load_limits_mapping(_committed(side))
    expected = {mirror_to_side(name, side): span for name, span in EXPECTED_LIMITS.items()}
    assert sorted(committed) == sorted(expected)
    for name, span in expected.items():
        assert committed[name] == pytest.approx(span), name
