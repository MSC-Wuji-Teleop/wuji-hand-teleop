#!/usr/bin/env python3
"""
MuJoCo mirror of real teleop and of clip replay -- the "watch the sim instead of
the physical robot" tool. Unlike sweep_and_visualize.py, it publishes nothing
and checks nothing: it only subscribes to the joint-command topics the rest of
the stack already produces and mirrors them live in MuJoCo on a composed
g1_wuji2_description model.

Topics (all subscribe-only; each is independent -- whatever is or isn't
publishing just determines what moves in the viewer):

    sub  /left_arm/joint_commands        JointState, named (5 names G1_23, 7 names G1_29)  g1_world_output_node
    sub  /right_arm/joint_commands       JointState, named                                  g1_world_output_node
    sub  /left_hand/joint_commands       JointState, 20 positional, hardware order          wujihand_controller (glove teleop)
    sub  /right_hand/joint_commands      JointState, 20 positional, hardware order          wujihand_controller (glove teleop)
    sub  /left/wuji_hand/joint_command   JointState, 20 named (l_*), the left hand_node's own command topic    replay_publisher
    sub  /right/wuji_hand/joint_command  JointState, 20 named (r_*), the right hand_node's own command topic   replay_publisher

Named messages are matched by joint name against the loaded model (arm name
`left_elbow` -> MJCF joint `left_elbow_joint`; hand name `l_thumb_ip` on the
left -> MJCF joint `left_wuji_l_thumb_ip`; then the actuator driving that
joint). Names the model does not have are skipped and reported once per
distinct set at info level -- G1_29 wrist pitch/yaw on the 23 model, for
example. The two hand sources share one slot per side (last message wins);
they are never both live, glove teleop uses the controller topic and replay
the driver topic. Contract details: _mujoco_common.py.

Models. The default is g1_23_wuji2_fixed.xml (glove/PICO teleop, 5-DoF arms).
Clip replay runs on the 29-DoF model and passes it explicitly:

    python3 mujoco_visualizer.py --mjcf src/g1_wuji2_description/g1_29_wuji2_fixed.xml

This is the sim side of the G1 hardware/sim toggle: run g1_world_output_node
with dry_run (no DDS/hardware) and this script together to exercise the
full teleop pipeline -- real Wuji Glove/PICO input if you have it, or
sweep_and_visualize.py's synthetic sweep if you don't -- without touching a
physical G1. See g1_world_output/README.md "Sim mode vs. hardware mode".

    # terminal 1 -- sim mode, no DDS/hardware
    cd docker && docker compose run --rm g1_world_output \
        ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true

    # terminal 2 -- watch it (main teleop container: rclpy + mujoco)
    python3 mujoco_visualizer.py

Clip replay (`scripts/replay.sh <clip> --sim`, docs/replay.md): the G1 node
runs `mode:=joint_replay arm_type:=G1_29 dry_run:=true` in its container,
replay_publisher publishes the clip, no hand driver starts, and this viewer
mirrors the G1 node's arm commands and the publisher's hand commands on the
29-DoF model:

    # teleop container, alongside replay_publisher
    python3 mujoco_visualizer.py --mjcf src/g1_wuji2_description/g1_29_wuji2_fixed.xml

The glove-teleop hand side works the same way, independent of G1 entirely:
the Wuji Glove -> retargeting -> `/left_hand/joint_commands` publish
(`wujihand_controller`, one process per side) never touches the physical
Wuji Hand SDK itself -- only the separate hand driver process does. So real
glove input can drive this viewer with the Wuji Hand never plugged in at
all; just don't launch the driver:

    # terminal 1 -- real glove input, no physical hand
    ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py enable_hand_driver:=false

    # terminal 2 -- watch it
    python3 mujoco_visualizer.py --focus hands

`--focus hands` just frames the camera closer for a hands-only session;
`--focus full` (default) is the whole-body G1 framing. `--no-viewer`
subscribes without opening a window (headless smoke test).
"""

from __future__ import annotations

import argparse
import threading
from functools import partial
from pathlib import Path

import _mujoco_common as mc
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Glove teleop: wujihand_controller's positional 20-vector, hardware order.
CONTROLLER_HAND_TOPIC = "/{side}_hand/joint_commands"
# Replay: the starport_wuji_hand hand_node's own command topic (namespace
# /{side}, node wuji_hand), 20 named; replay_publisher publishes it.
DRIVER_HAND_TOPIC = "/{side}/wuji_hand/joint_command"
# The G1 node's command echo, named per arm_type.
ARM_TOPIC = "/{side}_arm/joint_commands"


class MujocoVisualizerNode(Node):
    def __init__(self):
        super().__init__("g1_mujoco_visualizer")
        self._lock = threading.Lock()
        # Newest value per side; forms as in _mujoco_common's snapshot contract.
        self.hand: dict[str, object] = {"left": None, "right": None}
        self.arm: dict[str, object] = {"left": None, "right": None}

        for side in mc.SIDES:
            self.create_subscription(
                JointState, CONTROLLER_HAND_TOPIC.format(side=side),
                partial(self._on_controller_hand, side), mc.HAND_JOINT_QOS,
            )
            # replay_publisher publishes this topic RELIABLE depth 10 (what the
            # hand driver subscribes). HAND_JOINT_QOS is BEST_EFFORT: a
            # BEST_EFFORT subscriber matches RELIABLE and BEST_EFFORT publishers
            # alike, so the one profile serves this topic and the glove
            # controller's BEST_EFFORT one above; a RELIABLE subscriber would
            # never match the latter.
            self.create_subscription(
                JointState, DRIVER_HAND_TOPIC.format(side=side),
                partial(self._on_driver_hand, side), mc.HAND_JOINT_QOS,
            )
            self.create_subscription(
                JointState, ARM_TOPIC.format(side=side),
                partial(self._on_arm, side), mc.ARM_JOINT_QOS,
            )

        topics = [
            t.format(side=side)
            for t in (ARM_TOPIC, CONTROLLER_HAND_TOPIC, DRIVER_HAND_TOPIC)
            for side in mc.SIDES
        ]
        self.get_logger().info(
            "Mirroring " + ", ".join(topics) + " into MuJoCo. Nothing published "
            "yet on any of those topics is fine -- the model just sits at its "
            "'stand' keyframe until something moves it."
        )

    def _on_controller_hand(self, side: str, msg: JointState) -> None:
        # Positional 20 in hardware order.
        with self._lock:
            self.hand[side] = list(msg.position)

    def _on_driver_hand(self, side: str, msg: JointState) -> None:
        # Named, as the publisher sends it. A bare 20-vector is also what the
        # driver accepts on this topic, so it is taken positionally here too.
        val = (tuple(msg.name), list(msg.position)) if msg.name else list(msg.position)
        with self._lock:
            self.hand[side] = val

    def _on_arm(self, side: str, msg: JointState) -> None:
        # (names, positions): run_viewer maps by name, so 5-joint (G1_23) and
        # 7-joint (G1_29 replay) commands both work.
        with self._lock:
            self.arm[side] = (tuple(msg.name), list(msg.position))

    def snapshot(self):
        with self._lock:
            return (self.hand["left"], self.hand["right"], self.arm["left"], self.arm["right"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--mjcf", type=Path, default=None,
        help="Composed MJCF to load (default: g1_23_wuji2_fixed.xml, auto-detected). "
        "Clip replay uses src/g1_wuji2_description/g1_29_wuji2_fixed.xml.",
    )
    parser.add_argument("--focus", choices=["full", "hands"], default="full", help="Initial camera framing (still freely orbitable once open)")
    parser.add_argument("--no-viewer", action="store_true", help="Subscribe only; skip opening the MuJoCo viewer window (headless smoke test)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mjcf_path = args.mjcf or mc.default_mjcf_path()
    model, data = mc.load_model(mjcf_path)

    rclpy.init()
    node = MujocoVisualizerNode()
    node.get_logger().info(f"Model: {mjcf_path} ({model.nu} actuators)")

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
