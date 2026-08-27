#!/usr/bin/env python3
"""
MuJoCo mirror of real teleop -- this is the "watch the sim instead of the
physical robot" tool. Unlike sweep_and_visualize.py, it publishes nothing:
it only subscribes to the joint-command topics real teleop already produces
and mirrors them live in MuJoCo (g1_wuji2_description/g1_23_wuji2_fixed.xml).

Topics (all subscribe-only; each is independent -- whatever is or isn't
publishing just determines what moves in the viewer):
    sub  /left_arm/joint_commands    sensor_msgs/JointState  (5 floats, from g1_world_output_node)
    sub  /right_arm/joint_commands   sensor_msgs/JointState  (5 floats, from g1_world_output_node)
    sub  /left_hand/joint_commands   sensor_msgs/JointState  (20 floats, from wujihand_controller)
    sub  /right_hand/joint_commands  sensor_msgs/JointState  (20 floats, from wujihand_controller)

This is the sim side of the G1 hardware/sim toggle: run g1_world_output_node
with --dry-run (no DDS/hardware) and this script together to exercise the
full teleop pipeline -- real Wuji Glove/PICO input if you have it, or
sweep_and_visualize.py's synthetic sweep if you don't -- without touching a
physical G1. See g1_world_output/README.md "Sim mode vs. hardware mode".

    # terminal 1 -- sim mode, no DDS/hardware
    cd docker && docker compose run --rm g1_world_output \
        ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true

    # terminal 2 -- watch it (main teleop container: rclpy + mujoco)
    python3 mujoco_visualizer.py

The hand side of this works the same way, independent of G1 entirely: the
Wuji Glove -> retargeting -> `/left_hand/joint_commands` publish
(`wujihand_controller`, one process per side) never touches the physical
Wuji Hand SDK itself -- that only happens in the separate `wujihand_driver`
process. So real glove input can drive this viewer with the Wuji Hand
never plugged in at all; just don't launch the driver:

    # terminal 1 -- real glove input, no physical hand
    ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py enable_hand_driver:=false

    # terminal 2 -- watch it
    python3 mujoco_visualizer.py --focus hands

`--focus hands` just frames the camera closer for a hands-only session;
`--focus full` (default) is the whole-body G1 framing. Either topic pair
(arms, hands) is independent -- whatever is or isn't publishing determines
what moves, so this also works with only hands, or only arms, running.
"""

from __future__ import annotations

import argparse
import threading
from pathlib import Path

import _mujoco_common as mc
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MujocoVisualizerNode(Node):
    def __init__(self):
        super().__init__("g1_mujoco_visualizer")
        self._lock = threading.Lock()
        self.left_hand: list[float] | None = None
        self.right_hand: list[float] | None = None
        self.left_arm_q: list[float] | None = None
        self.right_arm_q: list[float] | None = None

        self.create_subscription(
            JointState, "/left_hand/joint_commands", self._on_left_hand, mc.HAND_JOINT_QOS
        )
        self.create_subscription(
            JointState, "/right_hand/joint_commands", self._on_right_hand, mc.HAND_JOINT_QOS
        )
        self.create_subscription(
            JointState, "/left_arm/joint_commands", self._on_left_arm, mc.ARM_JOINT_QOS
        )
        self.create_subscription(
            JointState, "/right_arm/joint_commands", self._on_right_arm, mc.ARM_JOINT_QOS
        )

        self.get_logger().info(
            "Mirroring /left_arm/joint_commands, /right_arm/joint_commands, "
            "/left_hand/joint_commands, /right_hand/joint_commands into MuJoCo. "
            "Nothing published yet on any of those topics is fine -- the model "
            "just sits at its 'stand' keyframe until something moves it."
        )

    def _on_left_hand(self, msg: JointState) -> None:
        with self._lock:
            self.left_hand = list(msg.position)

    def _on_right_hand(self, msg: JointState) -> None:
        with self._lock:
            self.right_hand = list(msg.position)

    def _on_left_arm(self, msg: JointState) -> None:
        with self._lock:
            # (names, positions) tuple: run_viewer maps by name, so 5-joint
            # (G1_23) and 7-joint (G1_29 replay) commands both work.
            self.left_arm_q = (tuple(msg.name), list(msg.position))

    def _on_right_arm(self, msg: JointState) -> None:
        with self._lock:
            self.right_arm_q = (tuple(msg.name), list(msg.position))

    def snapshot(self):
        with self._lock:
            return (self.left_hand, self.right_hand, self.left_arm_q, self.right_arm_q)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mjcf", type=Path, default=None, help="Path to g1_23_wuji2_fixed.xml (auto-detected if omitted)")
    parser.add_argument("--focus", choices=["full", "hands"], default="full", help="Initial camera framing (still freely orbitable once open)")
    parser.add_argument("--no-viewer", action="store_true", help="Subscribe only; skip opening the MuJoCo viewer window (headless smoke test)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mjcf_path = args.mjcf or mc.default_mjcf_path()
    model, data = mc.load_model(mjcf_path)

    rclpy.init()
    node = MujocoVisualizerNode()

    def _spin():
        try:
            rclpy.spin(node)
        except Exception:
            if rclpy.ok():
                raise

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        if args.no_viewer:
            spin_thread.join()
        else:
            mc.run_viewer(node, model, data, camera=args.focus)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
