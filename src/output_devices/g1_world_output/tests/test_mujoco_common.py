#!/usr/bin/env python3
"""Tests for the viewer's joint-name -> ctrl mapping (scripts/_mujoco_common.py).

Loads the composed models from src/g1_wuji2_description next to this package
and pins: the 20 hardware-order hand names of each side resolve to the 20
hand actuators hand_actuator_ids() returns, in the same order; the 7 G1_29 arm
names per side resolve to their `_joint` actuators; unknown names are skipped
and reported, never written; the named and positional forms write the same
ctrl entries.

_mujoco_common imports rclpy.qos, so this runs where rclpy is installed (the
teleop container) and is skipped elsewhere.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy")

# Package root and its scripts/ directory: _mujoco_common is a script-side
# helper, not part of the g1_world_output Python package.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

import _mujoco_common as mc  # noqa: E402
import mujoco  # noqa: E402

MODEL_DIR = mc.find_g1_wuji2_description_dir(Path(__file__))
# Replay runs on the 29-DoF model; teleop's default is the 23.
MODEL_29 = MODEL_DIR / "g1_29_wuji2_fixed.xml"
MODEL_23 = MODEL_DIR / "g1_23_wuji2_fixed.xml"

# Hand joint names per side in the hand driver's hardware order (the
# JointState names replay_publisher sends; starport_wuji_hand joint_map.py).
HAND_JOINT_NAMES = {
    side: [
        f"{p}_thumb_cmc_flex", f"{p}_thumb_cmc_abd", f"{p}_thumb_mcp", f"{p}_thumb_ip",
        f"{p}_index_finger_mcp_flex", f"{p}_index_finger_mcp_abd",
        f"{p}_index_finger_pip", f"{p}_index_finger_dip",
        f"{p}_middle_finger_mcp_flex", f"{p}_middle_finger_mcp_abd",
        f"{p}_middle_finger_pip", f"{p}_middle_finger_dip",
        f"{p}_ring_finger_mcp_flex", f"{p}_ring_finger_mcp_abd",
        f"{p}_ring_finger_pip", f"{p}_ring_finger_dip",
        f"{p}_pinky_mcp_flex", f"{p}_pinky_mcp_abd", f"{p}_pinky_pip", f"{p}_pinky_dip",
    ]
    for side, p in (("left", "l"), ("right", "r"))
}

# Arm joint names per side as the G1 node publishes them with arm_type G1_29
# (G1_29_ARM_JOINT_NAMES in g1_world_output/robot_arm.py, no `_joint` suffix).
ARM_JOINT_NAMES = {
    side: [
        f"{side}_shoulder_pitch", f"{side}_shoulder_roll", f"{side}_shoulder_yaw",
        f"{side}_elbow", f"{side}_wrist_roll", f"{side}_wrist_pitch", f"{side}_wrist_yaw",
    ]
    for side in ("left", "right")
}


def _actuator(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert aid >= 0, name
    return aid


@pytest.fixture(scope="module")
def model29() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MODEL_29))


@pytest.fixture(scope="module")
def maps29(model29) -> mc.CtrlMaps:
    return mc.CtrlMaps(model29)


@pytest.fixture
def data29(model29) -> mujoco.MjData:
    return mujoco.MjData(model29)


@pytest.mark.parametrize("side", ["left", "right"])
def test_hand_names_resolve_to_hardware_order_actuators(model29, maps29, side):
    ids = [maps29.hand_actuator(side, n) for n in HAND_JOINT_NAMES[side]]
    assert len(ids) == 20
    assert min(ids) >= 0
    assert len(set(ids)) == 20
    assert ids == list(mc.hand_actuator_ids(model29, side))


@pytest.mark.parametrize("side", ["left", "right"])
def test_arm_names_resolve_to_joint_actuators(model29, maps29, side):
    ids = [maps29.arm_actuator(n) for n in ARM_JOINT_NAMES[side]]
    assert ids == [_actuator(model29, n + "_joint") for n in ARM_JOINT_NAMES[side]]
    assert len(set(ids)) == 7


def test_every_actuator_in_the_model_is_a_joint_drive(model29):
    jnt_to_act = mc.joint_actuator_map(model29)
    assert len(jnt_to_act) == model29.nu
    assert sorted(jnt_to_act.values()) == list(range(model29.nu))


def test_unknown_names_resolve_to_minus_one(maps29):
    assert maps29.arm_actuator("left_wrist_flap") == -1
    assert maps29.hand_actuator("left", "l_sixth_finger_pip") == -1
    # A right-hand name on the left side is not a left joint.
    assert maps29.hand_actuator("left", "r_thumb_mcp") == -1
    # The bare MJCF name without the mapping rule is not a command name either.
    assert maps29.arm_actuator("left_elbow_joint") == -1


def test_apply_hand_named_writes_the_named_ctrl_entries(model29, maps29, data29):
    before = data29.ctrl.copy()
    names = ("l_index_finger_mcp_flex", "l_thumb_ip")
    unknown = mc.apply_hand(data29, "left", (names, [1.2, 0.3]), maps29)
    assert unknown == ()
    assert data29.ctrl[_actuator(model29, "left_wuji_l_FFJ0")] == pytest.approx(1.2)
    assert data29.ctrl[_actuator(model29, "left_wuji_l_THJ3")] == pytest.approx(0.3)
    changed = np.flatnonzero(data29.ctrl != before)
    assert sorted(changed) == sorted(
        [_actuator(model29, "left_wuji_l_FFJ0"), _actuator(model29, "left_wuji_l_THJ3")]
    )


def test_apply_hand_positional_writes_hardware_order(model29, maps29, data29):
    values = 0.01 * np.arange(20)
    unknown = mc.apply_hand(data29, "right", list(values), maps29)
    assert unknown == ()
    assert data29.ctrl[mc.hand_actuator_ids(model29, "right")] == pytest.approx(values)
    # Same entries the named form would have written.
    named = mujoco.MjData(model29)
    mc.apply_hand(named, "right", (tuple(HAND_JOINT_NAMES["right"]), list(values)), maps29)
    assert named.ctrl == pytest.approx(data29.ctrl)


def test_apply_hand_skips_unknown_names_and_reports_them(model29, maps29, data29):
    before = data29.ctrl.copy()
    names = ("l_thumb_mcp", "r_thumb_mcp", "l_no_such_joint")
    unknown = mc.apply_hand(data29, "left", (names, [0.5, 0.6, 0.7]), maps29)
    assert unknown == ("r_thumb_mcp", "l_no_such_joint")
    assert data29.ctrl[_actuator(model29, "left_wuji_l_THJ2")] == pytest.approx(0.5)
    assert np.count_nonzero(data29.ctrl != before) == 1


def test_apply_arm_named_writes_seven_g1_29_joints(model29, maps29, data29):
    values = 0.1 * np.arange(1, 8)
    unknown = mc.apply_arm(data29, "right", (tuple(ARM_JOINT_NAMES["right"]), list(values)), maps29)
    assert unknown == ()
    for name, value in zip(ARM_JOINT_NAMES["right"], values):
        assert data29.ctrl[_actuator(model29, name + "_joint")] == pytest.approx(value)


def test_apply_arm_positional_writes_ik_order(model29, maps29, data29):
    values = [0.1, 0.2, 0.3, 0.4, 0.5]
    assert mc.apply_arm(data29, "left", values, maps29) == ()
    for joint, value in zip(mc.ARM_JOINTS_IK, values):
        assert data29.ctrl[_actuator(model29, f"left_{joint}_joint")] == pytest.approx(value)


def test_apply_none_writes_nothing(maps29, data29):
    data29.ctrl[:] = 0.42
    assert mc.apply_hand(data29, "left", None, maps29) == ()
    assert mc.apply_arm(data29, "left", None, maps29) == ()
    assert np.all(data29.ctrl == 0.42)


def test_g1_29_wrist_names_are_skipped_on_the_23_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_23))
    maps = mc.CtrlMaps(model)
    data = mujoco.MjData(model)
    values = 0.1 * np.arange(1, 8)
    unknown = mc.apply_arm(data, "left", (tuple(ARM_JOINT_NAMES["left"]), list(values)), maps)
    assert unknown == ("left_wrist_pitch", "left_wrist_yaw")
    for name, value in zip(ARM_JOINT_NAMES["left"][:5], values[:5]):
        assert data.ctrl[_actuator(model, name + "_joint")] == pytest.approx(value)
    # The hands are the same 40 actuators on both models.
    for side in ("left", "right"):
        ids = [maps.hand_actuator(side, n) for n in HAND_JOINT_NAMES[side]]
        assert ids == list(mc.hand_actuator_ids(model, side))


def test_unknown_name_log_reports_each_set_once():
    lines: list[str] = []
    log = mc.UnknownNameLog(lines.append)
    log.note("left arm", ())
    log.note("left arm", ("left_wrist_pitch", "left_wrist_yaw"))
    log.note("left arm", ("left_wrist_yaw", "left_wrist_pitch"))
    log.note("right arm", ("right_wrist_pitch", "right_wrist_yaw"))
    log.note("left arm", ("left_wrist_pitch",))
    assert len(lines) == 3
    assert "left_wrist_pitch" in lines[0] and "left arm" in lines[0]
    assert lines[1].startswith("right arm")
