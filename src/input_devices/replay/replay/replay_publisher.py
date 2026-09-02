#!/usr/bin/env python3
"""Replay publisher: plays one clip directory to the G1 node and the hand drivers.

Reads ``clips/safe/<clip>/`` (see replay/clip.py for the format and the
refusals) and publishes it on one timer, so arms and hands stay time-aligned:

    arms  -> /left_arm/joint_targets, /right_arm/joint_targets
             sensor_msgs/JointState, 7 named joints per side, radians.
             Consumed by g1_world_output in mode:=joint_replay, which matches
             joints by name.
    hands -> /left/wuji_hand/joint_command, /right/wuji_hand/joint_command
             sensor_msgs/JointState, 20 named joints per side, radians.
             Consumed by the starport_wuji_hand driver (one node per side),
             which matches by name and refuses names of the other hand.

All four publishers are RELIABLE, KEEP_LAST, depth 10: both consumers
subscribe with that profile, and a BEST_EFFORT publisher never matches a
RELIABLE subscriber.

Behaviour: one timer with period ``1 / (rate_hz * speed)``. Each tick
publishes frame ``i`` for every selected side. When the last frame is
reached the node keeps publishing it until killed (the driver's idle release
and the G1 node's hold are the consumers' own business). Nothing else runs
here: no approach ramp, no run-time checks, no loop. Clip quality is decided
offline by tools/prepare_clip.py; this node refuses a directory that is not
under ``safe/`` with a safe verdict, and a speed the audit did not pass.

Usage:

    ros2 run replay replay_publisher -- --clip clips/safe/<clip> \\
        [--arms none|left|right|both] [--hands none|left|right|both] [--speed S]

Defaults: arms both, hands both, speed = the fastest entry in safe_speeds.
Refused: arms none together with hands none; speed <= 0, > 1, or above the
fastest safe speed.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from replay.clip import (
    SIDE_CHOICES,
    Clip,
    ClipError,
    check_speed,
    default_speed,
    duration_s,
    frame_period,
    load_clip,
    parse_sides,
)

# Depth 10: the G1 node subscribes /{side}_arm/joint_targets with depth 10
# and the hand driver subscribes ~/joint_command with depth 10. Matching the
# consumer's depth keeps a short burst from being dropped on either side.
COMMAND_QOS_DEPTH = 10

# RELIABLE because both consumers subscribe RELIABLE (the rclpy default).
# A BEST_EFFORT publisher would never match them and every message would be
# dropped with only a QoS warning in the log.
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=COMMAND_QOS_DEPTH,
)

# Topic patterns. The arm pattern is g1_world_output's joint_replay input;
# the hand pattern follows starport hand.launch.py, which names each driver
# node wuji_hand in the /{side} namespace.
ARM_TOPIC = "/{side}_arm/joint_targets"
HAND_TOPIC = "/{side}/wuji_hand/joint_command"

# Exit status for a refused clip or speed. 2 is argparse's own status for a
# bad argument; a clip that may not be played is the same class of error.
EXIT_REFUSED = 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="replay_publisher",
        description="Play one clip directory to the G1 node and the hand drivers.",
    )
    parser.add_argument("--clip", required=True, help="Clip directory under clips/safe/")
    parser.add_argument("--arms", choices=SIDE_CHOICES, default="both", help="Arm topics to publish (default both)")
    parser.add_argument("--hands", choices=SIDE_CHOICES, default="both", help="Hand topics to publish (default both)")
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help="Playback speed in (0, 1]; default: the fastest entry in the clip's safe_speeds",
    )
    return parser


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse the command line. Refuses --arms none together with --hands none."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.arms == "none" and args.hands == "none":
        parser.error("nothing to publish: --arms none and --hands none together")
    return args


class ReplayPublisher(Node):
    """One timer, four optional publishers, no state beyond the frame index."""

    def __init__(self, clip: Clip, speed: float, arm_sides: tuple[str, ...], hand_sides: tuple[str, ...]):
        super().__init__("replay_publisher")
        self._clip = clip
        self._arm_sides = arm_sides
        self._hand_sides = hand_sides
        self._idx = 0
        self._last_frame_logged = False

        self._arm_pubs = {
            side: self.create_publisher(JointState, ARM_TOPIC.format(side=side), COMMAND_QOS)
            for side in arm_sides
        }
        self._hand_pubs = {
            side: self.create_publisher(JointState, HAND_TOPIC.format(side=side), COMMAND_QOS)
            for side in hand_sides
        }

        period = frame_period(clip, speed)
        self.get_logger().info(
            f"clip {clip.name}: {clip.frames} frames at {clip.rate_hz:g} Hz, speed {speed:g} "
            f"(period {period * 1000:.1f} ms), arms {'+'.join(arm_sides) or 'none'}, "
            f"hands {'+'.join(hand_sides) or 'none'}, duration {duration_s(clip, speed):.1f} s, "
            "then holding the last frame"
        )
        self._timer = self.create_timer(period, self._tick)

    def _publish(self, pub, names: tuple[str, ...], positions, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(names)
        msg.position = positions.tolist()
        pub.publish(msg)

    def _tick(self) -> None:
        i = self._idx
        stamp = self.get_clock().now().to_msg()
        for side in self._arm_sides:
            self._publish(self._arm_pubs[side], self._clip.arm_names[side], self._clip.arm_q[side][i], stamp)
        for side in self._hand_sides:
            self._publish(self._hand_pubs[side], self._clip.hand_names[side], self._clip.hand_q20[side][i], stamp)

        if i < self._clip.frames - 1:
            self._idx = i + 1
        elif not self._last_frame_logged:
            self._last_frame_logged = True
            self.get_logger().info(f"last frame ({i}) reached; holding it until killed")


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ["replay_publisher", *argv]
    args = parse_args(remove_ros_args(raw_argv)[1:])

    try:
        clip = load_clip(args.clip)
        speed = default_speed(clip) if args.speed is None else check_speed(clip, args.speed)
    except ClipError as exc:
        print(f"replay_publisher: refused: {exc}", file=sys.stderr)
        sys.exit(EXIT_REFUSED)

    rclpy.init(args=raw_argv)
    node = ReplayPublisher(clip, speed, parse_sides(args.arms), parse_sides(args.hands))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
