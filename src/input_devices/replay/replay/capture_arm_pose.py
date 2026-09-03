#!/usr/bin/env python3
"""Read the measured arm pose off the graph, write it as JSON, and exit.

Step 2 of the rehome program (docs/spec/spec1_1.md). It subscribes to the G1
node's state topics, takes one settled reading per side, writes it where
tools/make_home_clip.py can read it, and stops. It creates no publisher and
commands nothing: this process cannot move the robot.

    ros2 run replay capture_arm_pose -- --out measured.json [--arms both]
                                        [--timeout 20] [--samples 5]

Output, in the G1 node's joint names and radians:

    {"captured_utc": "...", "arms": "both",
     "left":  {"left_shoulder_pitch": 0.01, ...},
     "right": {"right_shoulder_pitch": 0.02, ...}}

A side that is not selected is absent from the file, and make_home_clip.py
refuses such a file: a rehome needs both arms, because a clip carries both.
--arms is here so a one-armed check can still be written and read by hand.

Exit status: 0 the file is written, 1 a selected side did not report within
the timeout, 2 a bad argument.

Why a median rather than one message: /{side}_arm/joint_states is BEST_EFFORT
at 100 Hz off the robot's lowstate, and a single dropped or torn sample would
become frame 0 of a clip. The median of SAMPLES consecutive readings costs
50 ms and removes that. It is not filtering for its own sake: the arms are
holding still while this runs, so any spread across the samples is noise.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from functools import partial
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from replay.clip import ARM_JOINTS_PER_SIDE, SIDE_CHOICES, SIDES, parse_sides

# The G1 node publishes state BEST_EFFORT (ARM_JOINT_QOS in
# g1_world_output_node.py). A RELIABLE subscriber never matches it and would
# wait out the timeout with no error. Same profile replay_check uses.
STATE_QOS_DEPTH = 10
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=STATE_QOS_DEPTH,
)

ARM_STATE_TOPIC = "/{side}_arm/joint_states"

# Same wait replay_check gives every source, for the same reason: the G1 node
# spends up to 30 s waiting for its first lowstate before it publishes anything.
DEFAULT_TIMEOUT_S = 20.0

# Readings to take per side before writing. At the node's fixed 100 Hz state
# timer this is 50 ms, short enough that the operator does not notice and long
# enough that one torn sample cannot decide the pose.
DEFAULT_SAMPLES = 5

EXIT_OK = 0
EXIT_NOT_REPORTED = 1
EXIT_REFUSED = 2

# Spread across the samples above this is not sensor noise on a held arm; it
# means something is moving the arms while the pose is being captured, and a
# clip built on it would start with a step. 0.01 rad is about 0.6 degrees.
MAX_SAMPLE_SPREAD_RAD = 0.01


class CapturedPose:
    """Collects readings per side and reports when every selected side is done."""

    def __init__(self, sides: tuple[str, ...], samples: int):
        self._sides = sides
        self._samples = samples
        self._readings: dict[str, list[list[float]]] = {s: [] for s in sides}
        self._names: dict[str, list[str]] = {}
        self._errors: list[str] = []

    @property
    def errors(self) -> list[str]:
        return list(self._errors)

    def add(self, side: str, names: list[str], positions: list[float]) -> None:
        if side not in self._sides or self.done:
            return
        if len(names) != ARM_JOINTS_PER_SIDE or len(positions) != len(names):
            self._errors.append(
                f"{ARM_STATE_TOPIC.format(side=side)}: {len(names)} names and "
                f"{len(positions)} positions, expected {ARM_JOINTS_PER_SIDE} of each"
            )
            return
        known = self._names.setdefault(side, list(names))
        if list(names) != known:
            self._errors.append(
                f"{ARM_STATE_TOPIC.format(side=side)}: joint names changed mid-capture"
            )
            return
        self._readings[side].append([float(p) for p in positions])

    @property
    def done(self) -> bool:
        return all(len(self._readings[s]) >= self._samples for s in self._sides)

    def missing(self) -> tuple[str, ...]:
        return tuple(s for s in self._sides if len(self._readings[s]) < self._samples)

    def spread(self) -> dict[str, float]:
        out = {}
        for side in self._sides:
            arr = np.asarray(self._readings[side], dtype=np.float64)
            out[side] = float(np.max(arr.max(axis=0) - arr.min(axis=0))) if arr.size else 0.0
        return out

    def pose(self) -> dict[str, dict[str, float]]:
        out: dict[str, dict[str, float]] = {}
        for side in self._sides:
            arr = np.asarray(self._readings[side], dtype=np.float64)
            median = np.median(arr, axis=0)
            out[side] = {n: float(q) for n, q in zip(self._names[side], median)}
        return out


class CaptureArmPose(Node):
    """Subscribes, collects, and completes a future. No publisher, ever."""

    def __init__(self, sides: tuple[str, ...], samples: int, timeout_s: float):
        super().__init__("capture_arm_pose")
        self._captured = CapturedPose(sides, samples)
        self.done = rclpy.task.Future()
        for side in sides:
            self.create_subscription(
                JointState, ARM_STATE_TOPIC.format(side=side),
                partial(self._on_state, side), STATE_QOS,
            )
        self.create_timer(timeout_s, self._on_timeout)
        self.get_logger().info(
            f"waiting up to {timeout_s:.0f} s for {samples} samples on "
            + ", ".join(ARM_STATE_TOPIC.format(side=s) for s in sides)
        )

    @property
    def captured(self) -> CapturedPose:
        return self._captured

    def _on_state(self, side: str, msg: JointState) -> None:
        """Collect, and finish as soon as there is an answer either way.

        Resolving here rather than on a poll timer is what rclpy's
        spin_until_future_complete expects: the future completes on the event
        that decided it, and the only timer left is the timeout.
        """
        self._captured.add(side, list(msg.name), list(msg.position))
        if not self.done.done() and (self._captured.done or self._captured.errors):
            self.done.set_result(self._captured.done and not self._captured.errors)

    def _on_timeout(self) -> None:
        if not self.done.done():
            missing = ", ".join(ARM_STATE_TOPIC.format(side=s) for s in self._captured.missing())
            self.get_logger().error(f"timeout: no complete reading from {missing}")
            self.done.set_result(False)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="capture_arm_pose",
        description="Write the measured G1 arm pose to a JSON file and exit.",
    )
    parser.add_argument("--out", required=True, help="file to write")
    parser.add_argument("--arms", choices=SIDE_CHOICES, default="both",
                        help="sides to read (default both)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"seconds to wait (default {DEFAULT_TIMEOUT_S:g})")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES,
                        help=f"readings per side to take the median of (default {DEFAULT_SAMPLES})")
    return parser


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.arms == "none":
        parser.error("nothing to capture: --arms none")
    if args.samples < 1:
        parser.error(f"--samples must be at least 1, got {args.samples}")
    if args.timeout <= 0:
        parser.error(f"--timeout must be positive, got {args.timeout}")
    return args


def write_pose(path: Path, captured: CapturedPose, arms: str) -> dict:
    document = {
        "captured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "arms": arms,
        **captured.pose(),
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=1, sort_keys=False) + "\n")
    return document


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ["capture_arm_pose", *argv]
    args = parse_args(remove_ros_args(raw_argv)[1:])
    sides = parse_sides(args.arms)

    rclpy.init(args=raw_argv)
    node = CaptureArmPose(sides, args.samples, args.timeout)
    ok = False
    try:
        rclpy.spin_until_future_complete(node, node.done)
        ok = bool(node.done.done() and node.done.result())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble: a SIGINT/SIGTERM landing between the executor's context check
        # and its wait-set creation surfaces as RCLError instead. Same shutdown.
        if rclpy.ok():
            raise
    finally:
        captured = node.captured
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    for message in captured.errors:
        print(f"capture_arm_pose: {message}", file=sys.stderr)
    if not ok:
        print("capture_arm_pose: no pose captured; nothing written", file=sys.stderr)
        sys.exit(EXIT_NOT_REPORTED)

    spread = captured.spread()
    for side, value in spread.items():
        if value > MAX_SAMPLE_SPREAD_RAD:
            print(f"capture_arm_pose: {side} moved {value:.4f} rad across the samples "
                  f"(limit {MAX_SAMPLE_SPREAD_RAD}); the arms are not holding still",
                  file=sys.stderr)
            sys.exit(EXIT_NOT_REPORTED)

    write_pose(Path(args.out), captured, args.arms)
    print(f"capture_arm_pose: wrote {args.out} "
          f"(spread {max(spread.values()):.5f} rad)", file=sys.stderr)
    print(args.out)
    sys.exit(EXIT_OK)


if __name__ == "__main__":
    main()
