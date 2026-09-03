#!/usr/bin/env python3
"""
Shared MuJoCo plumbing for the g1_world_output sim tooling (sweep_and_visualize.py,
mujoco_visualizer.py): actuator lookups, joint-name -> ctrl mapping, MJCF loading, and
the render loop. Neither script solves IK or drives real hardware -- see each script's
own docstring. This module publishes nothing and checks nothing; it only writes
`data.ctrl` from whatever the caller's node last received.

Snapshot contract. `run_viewer` calls `node.snapshot()` once per physics step and
expects `(left_hand, right_hand, left_arm, right_arm)`. Each element is one of:

    None                 nothing received yet; that ctrl is left where it is
                         (the 'stand' keyframe until something moves it).
    list / array         positional. Hands: 20 values in the hand driver's hardware
                         order (the glove controller's /{side}_hand/joint_commands,
                         sweep_and_visualize.py's own sweep). Arms: 5 values in
                         ARM_JOINTS_IK order (G1_23 pose-IK output).
    (names, positions)   named, as carried by the JointState itself. Each name is
                         matched to the loaded model: an arm name `left_elbow` is
                         the MJCF joint `left_elbow_joint`; a hand name `l_thumb_ip`
                         on the left is the MJCF joint `left_wuji_l_thumb_ip`. The
                         ctrl written is the actuator whose transmission is that
                         joint (CtrlMaps, built once from model.actuator_trnid).
                         Names the model has no actuated joint for are skipped, so
                         5- and 7-joint arm commands work on either composed model.

A Python `tuple` means "named"; anything else is positional. Positional values are
written by index, so their length must match (20 hands, 5 arms) -- the same
contract the topics they come from already carry.

Import this before your own `import mujoco` so LIBGL_ALWAYS_SOFTWARE takes effect
before MuJoCo touches OpenGL. On a host GPU newer than the container's Mesa build
supports, hardware GL context creation can fail (`libGL error: failed to load
driver: iris`) and fall back to a slow/blocking software path that also swallows
Ctrl-C/SIGTERM. Set the env var to "0" beforehand to force a hardware-context
attempt instead.
"""

from __future__ import annotations

import os
import threading
import time
from functools import partial
from typing import Callable, Sequence

os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

SIDES = ("left", "right")

# 20 hand actuator codes in the driver's hardware order (finger1..5, joint1..4):
# thumb, index, middle, ring, pinky x {mcp_flex/cmc_flex, mcp_abd/cmc_abd, pip/mcp, dip/ip}.
# MJCF hand actuators are `{side}_wuji_{l|r}_{code}` (same table as HAND_ACTUATOR_CODES
# in tools/clip_audit.py).
HAND_CODES = [
    "THJ0", "THJ1", "THJ2", "THJ3",
    "FFJ0", "FFJ1", "FFJ2", "FFJ3",
    "MFJ0", "MFJ1", "MFJ2", "MFJ3",
    "RFJ0", "RFJ1", "RFJ2", "RFJ3",
    "LFJ0", "LFJ1", "LFJ2", "LFJ3",
]

# MJCF hand joints are the hand driver's hardware names (`l_thumb_ip`, `r_thumb_ip`;
# starport_wuji_hand joint_map.py) under this per-side prefix.
HAND_MJCF_PREFIX = {"left": "left_wuji_", "right": "right_wuji_"}

# MJCF arm joints and actuators are the G1 node's joint names (G1_29_ARM_JOINT_NAMES
# in g1_world_output/robot_arm.py) plus this suffix.
ARM_MJCF_SUFFIX = "_joint"

# G1_23 IK solves and the 23 MJCF actuates exactly these 5 DoF/arm; the positional
# arm form is in this order. The G1_29 body's 7-DoF arm (wrist_pitch/wrist_yaw in
# addition to these) only ever arrives named -- see the module docstring.
ARM_JOINTS_IK = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"]

# Matches g1_world_output_node's publishers on /left_arm/joint_commands etc.
ARM_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Matches wujihand_output/_internal/hand_interface.py::get_sensor_data_qos(),
# used by the real /left_hand|right_hand/joint_commands publishers. A default
# (RELIABLE) subscription is QoS-incompatible with that BEST_EFFORT publisher
# -- rclpy silently drops every message with an "incompatible QoS" warning
# instead of erroring, so this is easy to miss until real hardware is in the
# loop. A BEST_EFFORT subscriber matches RELIABLE publishers too, so the same
# profile also receives replay_publisher's RELIABLE depth-10
# /{side}/wuji_hand/joint_command and sweep_and_visualize.py's own (RELIABLE)
# hand publisher: one profile for every hand source.
HAND_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def find_g1_wuji2_description_dir(start: Path) -> Path:
    """Walk up from `start` for a directory named g1_wuji2_description containing
    g1_23_wuji2_fixed.xml -- robust to where this script/repo sits on disk (same idea
    as config_loader.py::G1Config._default_urdf_dir, kept local here rather than
    imported so this script doesn't depend on g1_world_output being colcon-built
    wherever it happens to run)."""
    for parent in start.resolve().parents:
        candidate = parent / "g1_wuji2_description"
        if (candidate / "g1_23_wuji2_fixed.xml").exists():
            return candidate
    raise FileNotFoundError(
        f"Could not find g1_wuji2_description/g1_23_wuji2_fixed.xml near {start}. "
        "Pass --mjcf explicitly."
    )


def default_mjcf_path() -> Path:
    """Resolve g1_23_wuji2_fixed.xml: ament package share dir first (correct in
    both the source tree and an install tree), falling back to the by-name
    crawl above. Unlike config_loader.py -- which always runs under
    `ros2 launch` in a sourced workspace and treats a missing package as a
    hard error -- this script is commonly invoked directly as
    `python3 scripts/foo.py` without install/setup.bash sourced, so the
    crawl fallback is kept here rather than dropped.

    The 23 model is the default for glove/PICO teleop. Clip replay (`--sim`
    in docs/replay.md) passes `--mjcf .../g1_29_wuji2_fixed.xml` explicitly.
    """
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
        try:
            return Path(get_package_share_directory('g1_wuji2_description')) / "g1_23_wuji2_fixed.xml"
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    return find_g1_wuji2_description_dir(Path(__file__)) / "g1_23_wuji2_fixed.xml"


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise KeyError(f"actuator not found in MJCF: {name}")
    return aid


def hand_actuator_ids(model: mujoco.MjModel, side: str) -> np.ndarray:
    """The 20 hand actuators of `side` in the driver's hardware order (the positional form)."""
    prefix = f"{side}_wuji_{side[0]}_"
    return np.array([actuator_id(model, prefix + code) for code in HAND_CODES])


def joint_actuator_map(model: mujoco.MjModel) -> dict[int, int]:
    """joint id -> id of the actuator whose transmission is that joint.

    Every actuator in both composed models is a joint transmission (69 on the
    29-DoF model, 63 on the 23; all 40 hand servos among them), so this covers
    every ctrl the viewer can write. An actuator with another transmission
    type is not a joint drive and is left out.
    """
    out: dict[int, int] = {}
    for aid in range(model.nu):
        if model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT:
            out[int(model.actuator_trnid[aid, 0])] = aid
    return out


class CtrlMaps:
    """Per-model lookup tables for writing snapshot values into ctrl, built once.

    `hand_ids[side]` / `arm_ik_ids[side]` serve the positional forms;
    `arm_actuator` / `hand_actuator` resolve one command name to the actuator
    driving the MJCF joint it names, or -1 when the loaded model has no such
    joint (or no actuator drives it). Resolutions are cached by MJCF joint name.
    """

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.hand_ids = {side: hand_actuator_ids(model, side) for side in SIDES}
        self.arm_ik_ids = {
            side: np.array([actuator_id(model, f"{side}_{j}{ARM_MJCF_SUFFIX}") for j in ARM_JOINTS_IK])
            for side in SIDES
        }
        self._joint_to_actuator = joint_actuator_map(model)
        self._by_joint_name: dict[str, int] = {}

    def actuator_for_joint(self, mjcf_joint: str) -> int:
        """Actuator driving the MJCF joint `mjcf_joint`, or -1."""
        aid = self._by_joint_name.get(mjcf_joint)
        if aid is None:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, mjcf_joint)
            aid = self._joint_to_actuator.get(jid, -1) if jid >= 0 else -1
            self._by_joint_name[mjcf_joint] = aid
        return aid

    def arm_actuator(self, name: str) -> int:
        """`left_elbow` -> actuator of MJCF joint `left_elbow_joint`, or -1."""
        return self.actuator_for_joint(name + ARM_MJCF_SUFFIX)

    def hand_actuator(self, side: str, name: str) -> int:
        """`l_thumb_ip` on `left` -> actuator of MJCF joint `left_wuji_l_thumb_ip`, or -1.

        A name of the other hand (`r_thumb_ip` on the left) has no such joint
        and resolves to -1 like any other unknown name.
        """
        return self.actuator_for_joint(HAND_MJCF_PREFIX[side] + name)


def _apply_named(
    data: mujoco.MjData,
    resolve: Callable[[str], int],
    names: Sequence[str],
    positions: Sequence[float],
) -> tuple[str, ...]:
    """Write each (name, position) whose name resolves; return the names that did not."""
    unknown = []
    for name, position in zip(names, positions):
        aid = resolve(name)
        if aid < 0:
            unknown.append(name)
        else:
            data.ctrl[aid] = position
    return tuple(unknown)


def apply_hand(data: mujoco.MjData, side: str, val, maps: CtrlMaps) -> tuple[str, ...]:
    """Write one hand's snapshot value (see the module docstring) into data.ctrl.

    Returns the names that were skipped because the model does not have them
    (named form only; empty otherwise), so the caller can log them.
    """
    if val is None:
        return ()
    if isinstance(val, tuple):
        names, positions = val
        return _apply_named(data, partial(maps.hand_actuator, side), names, positions)
    data.ctrl[maps.hand_ids[side]] = val
    return ()


def apply_arm(data: mujoco.MjData, side: str, val, maps: CtrlMaps) -> tuple[str, ...]:
    """Write one arm's snapshot value (see the module docstring) into data.ctrl.

    Same return as apply_hand. On the 23 model a G1_29 command's wrist_pitch and
    wrist_yaw come back as skipped.
    """
    if val is None:
        return ()
    if isinstance(val, tuple):
        names, positions = val
        return _apply_named(data, maps.arm_actuator, names, positions)
    data.ctrl[maps.arm_ik_ids[side]] = val
    return ()


class UnknownNameLog:
    """Reports each distinct set of skipped names once per source.

    A publisher repeats the same names at its publish rate, so logging every
    frame would flood; a new set (a different publisher, a changed layout) is
    still reported. `info` is the logger call to use, e.g. `node.get_logger().info`.
    """

    def __init__(self, info: Callable[[str], None]):
        self._info = info
        self._seen: set[tuple[str, frozenset[str]]] = set()

    def note(self, source: str, unknown: tuple[str, ...]) -> None:
        if not unknown:
            return
        key = (source, frozenset(unknown))
        if key in self._seen:
            return
        self._seen.add(key)
        self._info(
            f"{source}: skipping {len(unknown)} joint name(s) the loaded model does not "
            f"actuate: {sorted(unknown)}"
        )


def load_model(mjcf_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load the MJCF, reset to the 'stand' keyframe, and forward-kinematics once."""
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


# How long run_viewer waits for the GLFW render thread to tear the window
# down after the loop ends. It takes a frame or two; this is a backstop.
VIEWER_EXIT_TIMEOUT_S = 2.0

# Initial camera pose, keyed by name so a caller can pick one with --focus.
# "full": whole robot. "hands": closer/lower, framing the forearms+hands
# rather than the whole body -- useful when only glove input is driving the
# hands and the G1 arms are sitting at their keyframe pose.
CAMERAS = {
    "full": {"azimuth": 90, "elevation": -15, "distance": 1.6, "lookat": [0.0, 0.0, 1.0]},
    "hands": {"azimuth": 90, "elevation": -5, "distance": 0.8, "lookat": [0.0, 0.0, 1.0]},
}


def run_viewer(node, model: mujoco.MjModel, data: mujoco.MjData, camera: str = "full") -> None:
    """Generic render loop shared by sweep_and_visualize.py and mujoco_visualizer.py.

    Each physics step calls `node.snapshot()`, expected to return
    `(left_hand, right_hand, left_arm, right_arm)`; each element is None
    (nothing received yet -- ctrl left at its current value), a positional
    list/array (20 hand values in hardware order; 5 arm values in
    ARM_JOINTS_IK order), or a `(names, positions)` tuple matched by name to
    the loaded model's joints (arms: `{name}_joint`; hands:
    `{side}_wuji_{name}`) -- the full contract is in the module docstring.
    Names the model does not actuate are skipped, and each distinct set of
    skipped names is logged once per source at info level through
    `node.get_logger()`, so the same caller works on both the 23-DoF model
    (5 actuated arm joints/side) and the 29-DoF one (7/side, clip replay).
    """
    maps = CtrlMaps(model)
    unknown_log = UnknownNameLog(node.get_logger().info)

    cam = CAMERAS[camera]
    # launch_passive runs the GLFW window on a daemon thread it does not hand
    # back; remember which thread(s) it started so they can be joined below.
    threads_before = set(threading.enumerate())
    viewer_threads: list[threading.Thread] = []
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer_threads = [t for t in threading.enumerate() if t not in threads_before]
            viewer.cam.azimuth = cam["azimuth"]
            viewer.cam.elevation = cam["elevation"]
            viewer.cam.distance = cam["distance"]
            viewer.cam.lookat[:] = cam["lookat"]

            while viewer.is_running():
                left_hand, right_hand, left_arm, right_arm = node.snapshot()
                unknown_log.note("left hand", apply_hand(data, "left", left_hand, maps))
                unknown_log.note("right hand", apply_hand(data, "right", right_hand, maps))
                unknown_log.note("left arm", apply_arm(data, "left", left_arm, maps))
                unknown_log.note("right arm", apply_arm(data, "right", right_arm, maps))

                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)
    finally:
        # Leaving the `with` only *requests* the window to exit. On Ctrl-C the
        # interpreter then runs glfw's atexit terminate() while the render
        # thread is still inside its loop, and the two race to a segfault
        # (seen with both composed models, software GL). Wait for the thread
        # to destroy the window first.
        for thread in viewer_threads:
            thread.join(timeout=VIEWER_EXIT_TIMEOUT_S)
