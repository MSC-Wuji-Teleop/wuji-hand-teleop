#!/usr/bin/env python3
"""Replay publisher: plays one clip directory to the G1 node and the hand drivers.

Reads ``clips/safe/<clip>/`` (see replay/clip.py for the format and the
refusals) and publishes selected sides on one 100 Hz timer, so arms and
hands stay time-aligned:

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

Behaviour, in order:

1. Wait until every selected consumer has reported state (the same sources
   ``replay_check`` uses). Default ``--ready-timeout`` is 30 s so a hand
   still scanning and homing is not a missed clip. ``0`` skips the wait
   (required for ``sim:=true``, which starts no drivers).
2. Approach frame 0 from the measured pose over ``--ramp`` seconds (default
   2) with a rest-to-start-velocity quintic (zero acceleration at both
   ends; rest-to-rest is min-jerk). Matching the clip's first-frame
   velocity keeps the join C1. No measurement, or ``--ramp 0``: skip.
3. Play the clip at 100 Hz, linearly interpolating between adjacent frames
   so ``--speed 0.25`` is not a 12.5 Hz staircase. Hold the last frame
   until killed.

Nothing else runs here: no run-time trip conditions, no loop. Clip quality
is decided offline by tools/prepare_clip.py; this node refuses a directory
that is not under ``safe/`` with a safe verdict, and a speed the audit did
not pass.

Usage:

    ros2 run replay replay_publisher -- --clip clips/safe/<clip> \\
        [--arms none|left|right|both] [--hands none|left|right|both] \\
        [--speed S] [--ready-timeout S] [--ramp S]

Defaults: arms both, hands both, speed = the fastest entry in safe_speeds,
ready-timeout 30, ramp 2. Refused: arms none together with hands none;
speed <= 0, > 1, or not an audited safe speed; a negative timeout or ramp.
"""

from __future__ import annotations

import argparse
import sys
from functools import partial

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from replay.check import (
    ARM_STATE_TOPIC,
    HAND_CONNECTED_TOPIC,
    HAND_STATE_TOPIC,
    ConnectionCheck,
    hand_sides_in,
)
from replay.clip import (
    SIDE_CHOICES,
    Clip,
    ClipError,
    check_speed,
    default_speed,
    duration_s,
    load_clip,
    parse_sides,
)
from replay.motion import clip_start_velocity, lerp_clip, named_positions, quintic_blend
from replay.replay_check import STATE_QOS

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

# Command stream rate. Independent of clip rate and --speed: interpolation
# fills the frames in between, so a slow replay is still a continuous
# command rather than a staircase at rate_hz * speed.
PUBLISH_HZ = 100.0

# Seconds to wait for selected consumers before the first command. Hand
# scan plus the 3 s home can exceed replay_check's 20 s give-up.
DEFAULT_READY_TIMEOUT_S = 30.0

# Seconds of quintic approach from the measured pose to clip frame 0.
DEFAULT_RAMP_S = 2.0

# Exit status for a refused clip or speed. 2 is argparse's own status for a
# bad argument; a clip that may not be played is the same class of error.
EXIT_REFUSED = 2

# Consumers never reported within --ready-timeout. Launch on_exit=Shutdown
# takes the drivers down with the publisher.
EXIT_NOT_READY = 1

# Phase names. Wait publishes nothing; ramp and play share the 100 Hz timer.
PHASE_WAIT = "wait"
PHASE_RAMP = "ramp"
PHASE_PLAY = "play"


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
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=DEFAULT_READY_TIMEOUT_S,
        help=(
            f"Seconds to wait for selected consumers before frame 0 "
            f"(default {DEFAULT_READY_TIMEOUT_S:g}; 0 skips the wait)"
        ),
    )
    parser.add_argument(
        "--ramp",
        type=float,
        default=DEFAULT_RAMP_S,
        help=f"Seconds of min-jerk approach from the measured pose to frame 0 (default {DEFAULT_RAMP_S:g}; 0 skips)",
    )
    return parser


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse the command line. Refuses --arms none together with --hands none."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.arms == "none" and args.hands == "none":
        parser.error("nothing to publish: --arms none and --hands none together")
    if args.ready_timeout < 0.0:
        parser.error(f"--ready-timeout must be >= 0, got {args.ready_timeout}")
    if args.ramp < 0.0:
        parser.error(f"--ramp must be >= 0, got {args.ramp}")
    return args


def sides_arg(sides: tuple[str, ...]) -> str:
    """Inverse of parse_sides: a tuple of sides back to none|left|right|both."""
    if not sides:
        return "none"
    if len(sides) == 2:
        return "both"
    return sides[0]


class ReplayPublisher(Node):
    """Wait for consumers, approach frame 0, then interpolate the clip at 100 Hz."""

    def __init__(
        self,
        clip: Clip,
        speed: float,
        arm_sides: tuple[str, ...],
        hand_sides: tuple[str, ...],
        *,
        ready_timeout_s: float = DEFAULT_READY_TIMEOUT_S,
        ramp_s: float = DEFAULT_RAMP_S,
        publish_hz: float = PUBLISH_HZ,
    ):
        super().__init__("replay_publisher")
        self._clip = clip
        self._speed = float(speed)
        self._arm_sides = arm_sides
        self._hand_sides = hand_sides
        self._ready_timeout_s = float(ready_timeout_s)
        self._ramp_s = float(ramp_s)
        self._last_frame_logged = False
        self.ready_failed = False

        self._arm_measured: dict[str, tuple[list[str], list[float]]] = {}
        self._hand_measured: tuple[list[str], list[float]] | None = None
        self._q0_arm: dict[str, np.ndarray] = {}
        self._qd0_arm: dict[str, np.ndarray] = {}
        self._q1_arm: dict[str, np.ndarray] = {}
        self._qd1_arm: dict[str, np.ndarray] = {}
        self._q0_hand: dict[str, np.ndarray] = {}
        self._qd0_hand: dict[str, np.ndarray] = {}
        self._q1_hand: dict[str, np.ndarray] = {}
        self._qd1_hand: dict[str, np.ndarray] = {}
        self._ramp_t0 = 0.0
        self._play_t0 = 0.0

        self._arm_pubs = {
            side: self.create_publisher(JointState, ARM_TOPIC.format(side=side), COMMAND_QOS)
            for side in arm_sides
        }
        self._hand_pubs = {
            side: self.create_publisher(JointState, HAND_TOPIC.format(side=side), COMMAND_QOS)
            for side in hand_sides
        }

        self._check: ConnectionCheck | None
        if self._ready_timeout_s > 0.0:
            self._check = ConnectionCheck(
                sides_arg(arm_sides),
                sides_arg(hand_sides),
                timeout_s=self._ready_timeout_s,
                start_s=self._now(),
            )
            self._phase = PHASE_WAIT
        else:
            self._check = None
            self._phase = PHASE_WAIT  # first tick captures an optional measurement, then starts

        for side in arm_sides:
            self.create_subscription(
                JointState, ARM_STATE_TOPIC.format(side=side), partial(self._on_arm_state, side), STATE_QOS
            )
        if hand_sides:
            self.create_subscription(JointState, HAND_STATE_TOPIC, self._on_joint_states, STATE_QOS)
            for side in hand_sides:
                self.create_subscription(
                    Bool, HAND_CONNECTED_TOPIC.format(side=side), partial(self._on_connected, side), STATE_QOS
                )

        wait_note = f"wait up to {self._ready_timeout_s:g} s" if self._ready_timeout_s > 0.0 else "no wait"
        ramp_note = f"{self._ramp_s:g} s min-jerk approach" if self._ramp_s > 0.0 else "no approach"
        self.get_logger().info(
            f"clip {clip.name}: {clip.frames} frames at {clip.rate_hz:g} Hz, speed {speed:g}, "
            f"publish {publish_hz:g} Hz ({wait_note}, {ramp_note}), "
            f"arms {'+'.join(arm_sides) or 'none'}, "
            f"hands {'+'.join(hand_sides) or 'none'}, duration {duration_s(clip, speed):.1f} s, "
            "then holding the last frame"
        )
        self._timer = self.create_timer(1.0 / publish_hz, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_arm_state(self, side: str, msg: JointState) -> None:
        self._arm_measured[side] = (list(msg.name), [float(p) for p in msg.position])
        if self._check is not None:
            self._check.record_arm_state(side, self._now())

    def _on_joint_states(self, msg: JointState) -> None:
        self._hand_measured = (list(msg.name), [float(p) for p in msg.position])
        if self._check is not None:
            self._check.record_joint_states(msg.name, self._now())

    def _on_connected(self, side: str, msg: Bool) -> None:
        if self._check is not None:
            self._check.record_hand_connected(side, bool(msg.data), self._now())

    def _publish(self, pub, names: tuple[str, ...], positions, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(names)
        msg.position = np.asarray(positions, dtype=np.float64).tolist()
        pub.publish(msg)

    def _publish_targets(self, arm_q: dict[str, np.ndarray], hand_q: dict[str, np.ndarray], stamp) -> None:
        for side in self._arm_sides:
            self._publish(self._arm_pubs[side], self._clip.arm_names[side], arm_q[side], stamp)
        for side in self._hand_sides:
            self._publish(self._hand_pubs[side], self._clip.hand_names[side], hand_q[side], stamp)

    def _have_start_pose(self) -> bool:
        if any(side not in self._arm_measured for side in self._arm_sides):
            return False
        if not self._hand_sides:
            return True
        if self._hand_measured is None:
            return False
        covered = set(hand_sides_in(self._hand_measured[0]))
        return all(side in covered for side in self._hand_sides)

    def _arm_at_frame(self, frame_f: float) -> dict[str, np.ndarray]:
        return {side: lerp_clip(self._clip.arm_q[side], frame_f) for side in self._arm_sides}

    def _hand_at_frame(self, frame_f: float) -> dict[str, np.ndarray]:
        return {side: lerp_clip(self._clip.hand_q20[side], frame_f) for side in self._hand_sides}

    def _begin_motion(self, now: float) -> None:
        """Leave wait: start the approach if we have a measured pose, else play."""
        if self._ramp_s > 0.0 and self._have_start_pose():
            self._phase = PHASE_RAMP
            self._ramp_t0 = now
            for side in self._arm_sides:
                names, positions = self._arm_measured[side]
                q1 = np.array(self._clip.arm_q[side][0], dtype=np.float64, copy=True)
                v1 = clip_start_velocity(self._clip.arm_q[side], self._clip.rate_hz, self._speed)
                self._q0_arm[side] = named_positions(self._clip.arm_names[side], names, positions, q1)
                self._qd0_arm[side] = np.zeros_like(q1)
                self._q1_arm[side] = q1
                self._qd1_arm[side] = v1 * self._ramp_s
            if self._hand_sides:
                names, positions = self._hand_measured  # type: ignore[misc]
                for side in self._hand_sides:
                    q1 = np.array(self._clip.hand_q20[side][0], dtype=np.float64, copy=True)
                    v1 = clip_start_velocity(self._clip.hand_q20[side], self._clip.rate_hz, self._speed)
                    self._q0_hand[side] = named_positions(self._clip.hand_names[side], names, positions, q1)
                    self._qd0_hand[side] = np.zeros_like(q1)
                    self._q1_hand[side] = q1
                    self._qd1_hand[side] = v1 * self._ramp_s
            self.get_logger().info(
                f"approaching frame 0 over {self._ramp_s:g} s (min-jerk, matching clip start velocity)"
            )
            return
        if self._ramp_s > 0.0:
            self.get_logger().warning("no measured pose; skipping approach ramp")
        self._phase = PHASE_PLAY
        self._play_t0 = now

    def _fail_ready(self, elapsed_s: float, table: str) -> None:
        self.ready_failed = True
        self._timer.cancel()
        self.get_logger().error(f"timeout after {elapsed_s:.1f} s waiting for consumers:\n{table}")
        if rclpy.ok():
            rclpy.shutdown()

    def _tick(self) -> None:
        now = self._now()
        if self._phase == PHASE_WAIT:
            if self._check is not None:
                verdict = self._check.verdict(now)
                if not verdict.complete:
                    if verdict.timed_out:
                        self._fail_ready(verdict.elapsed_s, verdict.table)
                    return
                self.get_logger().info(f"consumers ready in {verdict.elapsed_s:.1f} s")
            self._begin_motion(now)
        stamp = self.get_clock().now().to_msg()
        if self._phase == PHASE_RAMP:
            u = (now - self._ramp_t0) / self._ramp_s
            if u < 1.0:
                arm_q = {
                    side: quintic_blend(
                        self._q0_arm[side], self._qd0_arm[side], self._q1_arm[side], self._qd1_arm[side], u
                    )
                    for side in self._arm_sides
                }
                hand_q = {
                    side: quintic_blend(
                        self._q0_hand[side], self._qd0_hand[side], self._q1_hand[side], self._qd1_hand[side], u
                    )
                    for side in self._hand_sides
                }
                self._publish_targets(arm_q, hand_q, stamp)
                return
            self._phase = PHASE_PLAY
            self._play_t0 = now
        elapsed = now - self._play_t0
        frame_f = elapsed * self._clip.rate_hz * self._speed
        last = self._clip.frames - 1
        if frame_f >= last:
            frame_f = float(last)
            if not self._last_frame_logged:
                self._last_frame_logged = True
                self.get_logger().info(f"last frame ({last}) reached; holding it until killed")
        self._publish_targets(self._arm_at_frame(frame_f), self._hand_at_frame(frame_f), stamp)


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
    node = ReplayPublisher(
        clip,
        speed,
        parse_sides(args.arms),
        parse_sides(args.hands),
        ready_timeout_s=args.ready_timeout,
        ramp_s=args.ramp,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble: a SIGINT/SIGTERM that lands between the executor's context
        # check and its wait-set creation surfaces as RCLError ("context is not
        # valid") instead of ExternalShutdownException. Same shutdown; anything
        # else is a real error and propagates.
        if rclpy.ok():
            raise
    finally:
        failed = node.ready_failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        sys.exit(EXIT_NOT_READY)


if __name__ == "__main__":
    main()
