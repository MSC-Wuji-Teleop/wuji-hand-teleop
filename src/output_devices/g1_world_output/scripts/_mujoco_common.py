#!/usr/bin/env python3
"""
Shared MuJoCo plumbing for the g1_world_output sim tooling (sweep_and_visualize.py,
mujoco_visualizer.py): actuator lookups, MJCF loading, and the render loop. Neither
script solves IK or drives real hardware -- see each script's own docstring.

Import this before your own `import mujoco` so LIBGL_ALWAYS_SOFTWARE takes effect
before MuJoCo touches OpenGL. On a host GPU newer than the container's Mesa build
supports, hardware GL context creation can fail (`libGL error: failed to load
driver: iris`) and fall back to a slow/blocking software path that also swallows
Ctrl-C/SIGTERM. Set the env var to "0" beforehand to force a hardware-context
attempt instead.
"""

from __future__ import annotations

import os
import time

os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# 20 hand actuator codes in wujihandros2 index order (finger1..5, joint1..4):
# thumb, index, middle, ring, pinky x {mcp_flex/cmc_flex, mcp_abd/cmc_abd, pip/mcp, dip/ip}
HAND_CODES = [
    "THJ0", "THJ1", "THJ2", "THJ3",
    "FFJ0", "FFJ1", "FFJ2", "FFJ3",
    "MFJ0", "MFJ1", "MFJ2", "MFJ3",
    "RFJ0", "RFJ1", "RFJ2", "RFJ3",
    "LFJ0", "LFJ1", "LFJ2", "LFJ3",
]

# G1_23 IK solves 5 DoF/arm; wrist_pitch/wrist_yaw exist in the MJCF (G1 has
# 7 DoF/arm) but aren't produced by this IK, so they're held at 0.
ARM_JOINTS_IK = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"]
ARM_JOINTS_FIXED = ["wrist_pitch", "wrist_yaw"]

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
# loop. sweep_and_visualize.py's own (RELIABLE) hand publisher is compatible
# with a BEST_EFFORT subscriber either way, so using this QoS everywhere
# works for both the synthetic sweep and real teleop.
HAND_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def find_g1_wuji2_description_dir(start: Path) -> Path:
    """Walk up from `start` for a directory named g1_wuji2_description containing
    g1_wuji2_fixed.xml -- robust to where this script/repo sits on disk (same idea
    as config_loader.py::G1Config._default_urdf_dir, kept local here rather than
    imported so this script doesn't depend on g1_world_output being colcon-built
    wherever it happens to run)."""
    for parent in start.resolve().parents:
        candidate = parent / "g1_wuji2_description"
        if (candidate / "g1_wuji2_fixed.xml").exists():
            return candidate
    raise FileNotFoundError(
        f"Could not find g1_wuji2_description/g1_wuji2_fixed.xml near {start}. "
        "Pass --mjcf explicitly."
    )


def default_mjcf_path() -> Path:
    """Resolve g1_wuji2_fixed.xml: ament package share dir first (correct in
    both the source tree and an install tree), falling back to the by-name
    crawl above. Unlike config_loader.py -- which always runs under
    `ros2 launch` in a sourced workspace and treats a missing package as a
    hard error -- this script is commonly invoked directly as
    `python3 scripts/foo.py` without install/setup.bash sourced, so the
    crawl fallback is kept here rather than dropped.
    """
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
        try:
            return Path(get_package_share_directory('g1_wuji2_description')) / "g1_wuji2_fixed.xml"
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    return find_g1_wuji2_description_dir(Path(__file__)) / "g1_wuji2_fixed.xml"


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise KeyError(f"actuator not found in MJCF: {name}")
    return aid


def hand_actuator_ids(model: mujoco.MjModel, side: str) -> np.ndarray:
    prefix = f"{side}_wuji_{side[0]}_"
    return np.array([actuator_id(model, prefix + code) for code in HAND_CODES])


def load_model(mjcf_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load the MJCF, reset to the 'stand' keyframe, and forward-kinematics once."""
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


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

    Each frame calls `node.snapshot()`, expected to return
    `(left_hand, right_hand, left_arm_q, right_arm_q)` where each element is either
    None (nothing received yet -- ctrl left at its current value) or an array-like
    of the right length (20 for hands, 5 for arms). `camera` selects an initial
    viewpoint from CAMERAS (still freely orbitable/zoomable once open).
    """
    left_hand_ids = hand_actuator_ids(model, "left")
    right_hand_ids = hand_actuator_ids(model, "right")
    left_arm_ik_ids = np.array([actuator_id(model, f"left_{j}_joint") for j in ARM_JOINTS_IK])
    right_arm_ik_ids = np.array([actuator_id(model, f"right_{j}_joint") for j in ARM_JOINTS_IK])
    for j in ARM_JOINTS_FIXED:
        data.ctrl[actuator_id(model, f"left_{j}_joint")] = 0.0
        data.ctrl[actuator_id(model, f"right_{j}_joint")] = 0.0

    cam = CAMERAS[camera]
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = cam["azimuth"]
        viewer.cam.elevation = cam["elevation"]
        viewer.cam.distance = cam["distance"]
        viewer.cam.lookat[:] = cam["lookat"]

        while viewer.is_running():
            left_hand, right_hand, left_arm_q, right_arm_q = node.snapshot()
            if left_hand is not None:
                data.ctrl[left_hand_ids] = left_hand
            if right_hand is not None:
                data.ctrl[right_hand_ids] = right_hand
            if left_arm_q is not None:
                data.ctrl[left_arm_ik_ids] = left_arm_q
            if right_arm_q is not None:
                data.ctrl[right_arm_ik_ids] = right_arm_q

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
