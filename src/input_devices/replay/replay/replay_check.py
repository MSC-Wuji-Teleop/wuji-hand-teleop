#!/usr/bin/env python3
"""Connection check: wait for state from the selected device nodes, print the rates, exit 0 or 1.

What ``scripts/replay.sh --check`` runs in place of ``replay_publisher`` once
the G1 node and the hand drivers are up (docs/replay.md section 2). It
publishes nothing. It subscribes to the state each selected device node
reports and, on a 0.5 s poll, asks replay/check.py whether every source has
been heard from:

    --arms  side  ->  /{side}_arm/joint_states       (G1 node, 250 Hz on the rig)
    --hands side  ->  /joint_states                   (driver, that side's 20 names)
                      /{side}/wuji_hand/connected     (driver, must have been true once)

When all have reported it prints the table and exits 0. At ``--timeout``
(default 30 s, matching the publisher's ready wait: two-hand scan plus the
driver's blocking 3 s home) it prints the table with the missing rows
marked and exits 1. The table goes to stdout with print(),
so it reads the same under ``ros2 run`` and ``ros2 launch``.

Subscriptions are BEST_EFFORT, KEEP_LAST, depth 10. The G1 node publishes its
state BEST_EFFORT depth 1 and a RELIABLE subscription would never match it;
BEST_EFFORT also matches the hand driver's RELIABLE publishers.

Usage:

    ros2 run replay replay_check -- [--arms none|left|right|both] \\
        [--hands none|left|right|both] [--timeout S]

Defaults: arms both, hands both, timeout 30. Refused: arms none together with
hands none; a timeout that is not positive. Exit status: 0 when every selected
source reported, 1 when one did not (timeout or Ctrl-C), 2 for a bad argument.
"""

from __future__ import annotations

import argparse
import sys
from functools import partial
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from replay.check import (
    ARM_STATE_TOPIC,
    DEFAULT_TIMEOUT_S,
    HAND_CONNECTED_TOPIC,
    HAND_STATE,
    HAND_STATE_TOPIC,
    ConnectionCheck,
    Verdict,
)
from replay.clip import SIDE_CHOICES

# BEST_EFFORT: the G1 node publishes /{side}_arm/joint_states BEST_EFFORT
# depth 1 (ARM_JOINT_QOS in g1_world_output_node.py); a RELIABLE subscription
# never matches a BEST_EFFORT publisher, while a BEST_EFFORT subscription
# matches both it and the hand driver's RELIABLE publishers. Depth 10: the
# check counts arrivals, so a short burst between polls must not be dropped.
STATE_QOS_DEPTH = 10
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=STATE_QOS_DEPTH,
)

# How often the verdict is re-evaluated. Coarse enough to cost nothing next to
# 250 Hz state, fine enough that the exit follows the last arrival, or the
# timeout, within half a second.
POLL_PERIOD_S = 0.5

# Exit statuses. 2 is argparse's own for a bad argument.
EXIT_OK = 0
EXIT_NOT_REPORTED = 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="replay_check",
        description="Wait for state from the selected device nodes and print the topic rates.",
    )
    parser.add_argument("--arms", choices=SIDE_CHOICES, default="both", help="Arm state topics to wait for (default both)")
    parser.add_argument("--hands", choices=SIDE_CHOICES, default="both", help="Hand driver topics to wait for (default both)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=(
            f"Seconds to wait before giving up (default {DEFAULT_TIMEOUT_S:g}, "
            "matching the publisher's ready wait)"
        ),
    )
    return parser


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse the command line. Refuses --arms none with --hands none, and a non-positive timeout."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.arms == "none" and args.hands == "none":
        parser.error("nothing to check: --arms none and --hands none together")
    if not args.timeout > 0.0:
        parser.error(f"--timeout must be > 0, got {args.timeout}")
    return args


class ReplayCheck(Node):
    """Subscriptions for the selected sources, one poll timer, a Future that completes with the verdict."""

    def __init__(self, arms: str, hands: str, timeout_s: float):
        super().__init__("replay_check")
        self.done: Future = Future()
        self.last_verdict: Optional[Verdict] = None
        self._check = ConnectionCheck(arms, hands, timeout_s, start_s=self._now())

        for side in self._check.arm_sides:
            self.create_subscription(JointState, ARM_STATE_TOPIC.format(side=side), partial(self._on_arm_state, side), STATE_QOS)
        if self._check.hand_sides:
            self.create_subscription(JointState, HAND_STATE_TOPIC, self._on_joint_states, STATE_QOS)
            for side in self._check.hand_sides:
                self.create_subscription(Bool, HAND_CONNECTED_TOPIC.format(side=side), partial(self._on_connected, side), STATE_QOS)

        self.get_logger().info(
            f"waiting up to {timeout_s:g} s for {len(self._check.sources)} sources: "
            + ", ".join(sorted({s.topic for s in self._check.sources}))
        )
        self._timer = self.create_timer(POLL_PERIOD_S, self._poll)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_arm_state(self, side: str, msg: JointState) -> None:
        self._check.record_arm_state(side, self._now())

    def _on_joint_states(self, msg: JointState) -> None:
        self._check.record_joint_states(msg.name, self._now())

    def _on_connected(self, side: str, msg: Bool) -> None:
        self._check.record_hand_connected(side, bool(msg.data), self._now())

    def _poll(self) -> None:
        verdict = self._check.verdict(self._now())
        self.last_verdict = verdict
        if not (verdict.complete or verdict.timed_out):
            return
        self._timer.cancel()
        n = len(self._check.sources)
        if verdict.complete:
            self.get_logger().info(f"all {n} sources reported in {verdict.elapsed_s:.1f} s")
        else:
            topics = ", ".join(f"{s.topic} ({s.side})" if s.kind == HAND_STATE else s.topic for s in verdict.missing)
            self.get_logger().error(f"timeout after {verdict.elapsed_s:.1f} s: {len(verdict.missing)} of {n} sources missing: {topics}")
        if not self.done.done():
            self.done.set_result(verdict)


def report(verdict: Optional[Verdict], interrupted: bool) -> int:
    """Print the table and return the exit status."""
    if verdict is None:
        print("replay_check: interrupted before the first poll; nothing reported", file=sys.stderr)
        return EXIT_NOT_REPORTED
    print(verdict.table, flush=True)
    if interrupted and not verdict.complete:
        print(f"replay_check: interrupted after {verdict.elapsed_s:.1f} s with {len(verdict.missing)} sources not reported", file=sys.stderr)
    return EXIT_OK if verdict.complete else EXIT_NOT_REPORTED


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ["replay_check", *argv]
    args = parse_args(remove_ros_args(raw_argv)[1:])

    rclpy.init(args=raw_argv)
    node = ReplayCheck(args.arms, args.hands, args.timeout)
    interrupted = False
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    except Exception:
        # Humble: a SIGINT/SIGTERM that lands between the executor's context
        # check and its wait-set creation surfaces as RCLError ("context is not
        # valid") instead of ExternalShutdownException. Same shutdown; anything
        # else is a real error and propagates.
        if rclpy.ok():
            raise
        interrupted = True
    finally:
        verdict = node.done.result() if node.done.done() else node.last_verdict
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(report(verdict, interrupted))


if __name__ == "__main__":
    main()
