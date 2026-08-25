#!/usr/bin/env python3
"""
Sweep test for the G1 + dual Wuji Hand 2 pipeline: drives hand joints and arm
target poses through continuous sweeps, publishes them on the standard
wuji-hand-teleop topics, and mirrors the resulting robot state live in
MuJoCo (g1_wuji2_description/g1_wuji2_fixed.xml).

Topic contract (see wuji-hand-teleop/README.md Appendix > Topic Interface and
g1_world_output/README.md):
    pub  /left_hand/joint_commands   sensor_msgs/JointState  (20 floats, position-only)
    pub  /right_hand/joint_commands  sensor_msgs/JointState  (20 floats, position-only)
    pub  /left_arm_target_pose       geometry_msgs/PoseStamped  (frame_id world_left, chest frame)
    pub  /right_arm_target_pose      geometry_msgs/PoseStamped  (frame_id world_right, chest frame)
    sub  /left_arm/joint_commands    sensor_msgs/JointState  (5 floats: shoulder pitch/roll/yaw, elbow, wrist roll)
    sub  /right_arm/joint_commands   sensor_msgs/JointState  (5 floats, same layout)

This script does NOT solve arm IK itself -- that is g1_world_output_node's
job (Pinocchio/CasADi, see g1_world_output/README.md). Hands are visualized
directly from the swept command since they need no IK. For the arms to move
in the viewer, run g1_world_output_node alongside this script so it can
consume /left_arm_target_pose and /right_arm_target_pose and publish the
solved joints back on /left_arm/joint_commands and /right_arm/joint_commands.
It needs its own container (Pinocchio+CasADi vs. this script's plain
rclpy+mujoco deps don't coexist in one env -- see g1_world_output/README.md
"Why this package has its own Docker image"); --dry-run skips DDS/hardware
so it runs off the pose topics alone:

    cd docker && docker compose run --rm g1_world_output \
        ros2 run g1_world_output g1_world_output_node --dry-run

Then, in the main teleop container (or any env with rclpy + mujoco):

    python3 sweep_and_visualize.py

If you just want to watch real teleop (real hand/arm input, not a synthetic
sweep) in MuJoCo instead, use mujoco_visualizer.py in this same directory --
it mirrors the same topics without publishing anything itself.
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import _mujoco_common as mc
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState

# Chest-frame centers for the /left_arm_target_pose, /right_arm_target_pose
# sweep: the chest-frame equivalent of config/g1_robot.yaml's
# reset_wrist_pose (a pose g1_world_output's own config asserts is
# IK-solvable), obtained by inverting chest_pose_to_pelvis with
# world_to_chest_quat / chest_origin_in_pelvis from that same file.
POSE_CENTER = {
    "left": {"position": np.array([0.296, 0.242, 0.150]), "quat": np.array([0.7071, 0.0, 0.0, 0.7071])},
    "right": {"position": np.array([0.296, -0.242, 0.150]), "quat": np.array([0.7071, 0.0, 0.0, -0.7071])},
}


class SweepNode(Node):
    def __init__(self, args: argparse.Namespace, hand_ranges: dict[str, np.ndarray]):
        super().__init__("g1_sweep_and_visualize")
        self._args = args
        self._hand_ranges = hand_ranges
        self._lock = threading.Lock()
        self._t0 = time.monotonic()

        self.left_hand_ctrl = np.zeros(20)
        self.right_hand_ctrl = np.zeros(20)
        self.left_arm_q: list[float] | None = None
        self.right_arm_q: list[float] | None = None
        self._warned_no_arm_ik = False

        self.left_hand_pub = self.create_publisher(JointState, "/left_hand/joint_commands", 10)
        self.right_hand_pub = self.create_publisher(JointState, "/right_hand/joint_commands", 10)
        self.left_pose_pub = self.create_publisher(PoseStamped, "/left_arm_target_pose", 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, "/right_arm_target_pose", 10)

        self.create_subscription(
            JointState, "/left_arm/joint_commands", self._on_left_arm_cmd, mc.ARM_JOINT_QOS
        )
        self.create_subscription(
            JointState, "/right_arm/joint_commands", self._on_right_arm_cmd, mc.ARM_JOINT_QOS
        )

        self.create_timer(1.0 / args.hz, self._tick)
        self.get_logger().info(
            f"Sweeping hands + arm poses at {args.hz} Hz, period {args.period}s. "
            "Waiting for /left_arm/joint_commands, /right_arm/joint_commands "
            "(published by g1_world_output_node) to animate the arms..."
        )

    def _on_left_arm_cmd(self, msg: JointState) -> None:
        with self._lock:
            self.left_arm_q = list(msg.position)

    def _on_right_arm_cmd(self, msg: JointState) -> None:
        with self._lock:
            self.right_arm_q = list(msg.position)

    def _hand_sweep(self, side: str, t: float) -> np.ndarray:
        lo = self._hand_ranges[side][:, 0]
        hi = self._hand_ranges[side][:, 1]
        phase = np.linspace(0.0, 2 * np.pi * 1.5, 20)  # rolling wave across fingers
        s = 0.5 + 0.5 * np.sin(2 * np.pi * t / self._args.period + phase)
        return lo + (hi - lo) * s

    def _pose_sweep(self, side: str, t: float) -> tuple[np.ndarray, np.ndarray]:
        center = POSE_CENTER[side]
        T = self._args.period
        amp = self._args.pos_amplitude
        pos = center["position"] + amp * np.array([
            np.sin(2 * np.pi * t / T),
            np.sin(2 * np.pi * t / (T * 1.3) + 1.0),
            np.sin(2 * np.pi * t / (T * 0.7) + 2.0),
        ])
        euler_deg = self._args.rot_amplitude_deg * np.array([
            np.sin(2 * np.pi * t / (T * 1.1)),
            np.sin(2 * np.pi * t / (T * 0.9) + 1.5),
            np.sin(2 * np.pi * t / (T * 1.4) + 3.0),
        ])
        rot = Rot.from_quat(center["quat"]) * Rot.from_euler("xyz", euler_deg, degrees=True)
        return pos, rot.as_quat()

    @staticmethod
    def _publish_hand(pub, positions: np.ndarray, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.position = [float(v) for v in positions]  # position-only: wujihandros2 parses by index
        pub.publish(msg)

    @staticmethod
    def _publish_pose(pub, frame_id: str, position: np.ndarray, quat: np.ndarray, stamp) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = (
            float(v) for v in quat
        )
        pub.publish(msg)

    def _tick(self) -> None:
        t = time.monotonic() - self._t0
        stamp = self.get_clock().now().to_msg()

        left_hand = self._hand_sweep("left", t)
        right_hand = self._hand_sweep("right", t)
        self._publish_hand(self.left_hand_pub, left_hand, stamp)
        self._publish_hand(self.right_hand_pub, right_hand, stamp)

        left_pos, left_quat = self._pose_sweep("left", t)
        right_pos, right_quat = self._pose_sweep("right", t)
        self._publish_pose(self.left_pose_pub, "world_left", left_pos, left_quat, stamp)
        self._publish_pose(self.right_pose_pub, "world_right", right_pos, right_quat, stamp)

        with self._lock:
            self.left_hand_ctrl = left_hand
            self.right_hand_ctrl = right_hand
            if self.left_arm_q is None and self.right_arm_q is None and not self._warned_no_arm_ik and t > 3.0:
                self._warned_no_arm_ik = True
                self.get_logger().warn(
                    "No /left_arm/joint_commands or /right_arm/joint_commands received yet -- "
                    "arms will stay at the 'stand' keyframe pose. Is g1_world_output_node running?"
                )

    def snapshot(self):
        with self._lock:
            return (
                self.left_hand_ctrl.copy(),
                self.right_hand_ctrl.copy(),
                self.left_arm_q,
                self.right_arm_q,
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mjcf", type=Path, default=None, help="Path to g1_wuji2_fixed.xml (auto-detected if omitted)")
    parser.add_argument("--hz", type=float, default=50.0, help="Publish rate for the sweep (Hz)")
    parser.add_argument("--period", type=float, default=6.0, help="Sweep period (s)")
    parser.add_argument("--pos-amplitude", type=float, default=0.06, help="Arm target position sweep amplitude (m)")
    parser.add_argument("--rot-amplitude-deg", type=float, default=15.0, help="Arm target orientation sweep amplitude (deg)")
    parser.add_argument("--no-viewer", action="store_true", help="Publish only; skip opening the MuJoCo viewer window")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mjcf_path = args.mjcf or mc.default_mjcf_path()
    model, data = mc.load_model(mjcf_path)

    hand_ranges = {
        "left": model.actuator_ctrlrange[mc.hand_actuator_ids(model, "left")].copy(),
        "right": model.actuator_ctrlrange[mc.hand_actuator_ids(model, "right")].copy(),
    }

    rclpy.init()
    node = SweepNode(args, hand_ranges)

    def _spin():
        try:
            rclpy.spin(node)
        except Exception:
            # A shutdown mid-callback (e.g. a timer tick publishing while the
            # main thread tears down the context) can surface as a plain
            # RCLError rather than ExternalShutdownException. Only re-raise
            # if the context is still valid -- that's a real bug, not a
            # shutdown race.
            if rclpy.ok():
                raise

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        if args.no_viewer:
            spin_thread.join()
        else:
            mc.run_viewer(node, model, data)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
