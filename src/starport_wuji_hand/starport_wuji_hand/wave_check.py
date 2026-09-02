"""Slow per-finger curl, published at the driver's command topic.

Deliberately a PUBLISHER rather than logic inside the driver, so it exercises the identical path a
real trajectory replay takes -- guard chain included. This is what finds a sign flip or a zero
offset, which no amount of documentation reading will.

Its blind spot is inside a finger: all four joints of a finger are given the same phase, so a
crossed pair WITHIN one finger (joint3 <-> joint4) moves indistinguishably from a correct one. What
this check establishes is finger identity, direction of travel and zero offset.

    ros2 run starport_wuji_hand wave_check --ros-args -p amplitude:=0.4
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .first_frame_gate import FirstFrameGate, declare_and_validate
from .joint_map import HAND_SIDES, JOINTS_PER_FINGER, NUM_FINGERS, NUM_JOINTS, joint_names

DEFAULT_AMPLITUDE = 0.4  # rad, well inside every joint's range
# Sized against the driver's slew-rate guard, not for resolution. The per-frame step is
# amplitude/steps, and the driver grants max_joint_velocity times its MEASURED tick gap of travel;
# because commands arrive slower than the driver ticks, a whole step lands in a single tick and is
# measured against a single tick's budget. At 20 steps the step equals one NOMINAL tick's budget and
# the guard trips on float rounding alone, which turns a signal test into a guard test. This keeps
# the step at half of that budget, which is headroom for measured gaps down to half nominal and no
# further -- there is no floor on the gap (see hand_node._MAX_TICK_FACTOR), so no choice of steps
# makes the guard silent against an arbitrarily short one. test_wave_check.py pins the ratio.
DEFAULT_STEPS = 40
DEFAULT_FRAME_RATE = 20.0  # Hz -- slow enough to watch
# The driver runs one node per hand under its own namespace, so a tool's topic follows the side
# it was told to drive. A fixed default here would publish into the void for one of the two hands.
COMMAND_TOPIC_TEMPLATE = "/{side}/wuji_hand/joint_command"


def curl_sequence(amplitude: float, steps: int) -> list[tuple[str, np.ndarray]]:
    """Frames curling each finger in turn: zero, in, out, zero, next finger.

    One finger at a time is the whole point -- a simultaneous curl cannot tell you WHICH finger
    responded to which command block.

    Both arguments are validated here rather than in the caller so the refusals are covered by the
    pure tests. A non-positive ``steps`` or a zero ``amplitude`` would otherwise produce the worst
    outcome this tool has: a run that publishes nothing but zero poses, reports success, and leaves
    the operator inspecting wiring for a hand that was never asked to move.
    """
    if not math.isfinite(amplitude) or amplitude == 0.0:
        raise ValueError(f"amplitude must be finite and non-zero, got {amplitude}")
    if steps <= 0:
        raise ValueError(f"steps must be positive, got {steps}")

    frames: list[tuple[str, np.ndarray]] = [("home", np.zeros(NUM_JOINTS))]
    for finger in range(NUM_FINGERS):
        lo = finger * JOINTS_PER_FINGER
        for step in range(steps + 1):
            phase = amplitude * step / steps
            target = np.zeros(NUM_JOINTS)
            target[lo : lo + JOINTS_PER_FINGER] = phase
            frames.append((f"finger{finger + 1}_in_{step}", target))
        for step in range(steps + 1):
            phase = amplitude * (1.0 - step / steps)
            target = np.zeros(NUM_JOINTS)
            target[lo : lo + JOINTS_PER_FINGER] = phase
            frames.append((f"finger{finger + 1}_out_{step}", target))
    frames.append(("home", np.zeros(NUM_JOINTS)))
    return frames


class WaveCheck(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("wave_check", **kwargs)
        self.declare_parameter("amplitude", DEFAULT_AMPLITUDE)
        self.declare_parameter("steps", DEFAULT_STEPS)
        self.declare_parameter("frame_rate", DEFAULT_FRAME_RATE)
        self.declare_parameter("hand_side", "right")
        side = str(self.get_parameter("hand_side").value)
        if side not in HAND_SIDES:
            raise ValueError(f"hand_side must be one of {list(HAND_SIDES)}, got {side!r}")
        self._joint_names = joint_names(side)
        self.declare_parameter("command_topic", COMMAND_TOPIC_TEMPLATE.format(side=side))

        self._topic = str(self.get_parameter("command_topic").value)
        frame_rate = float(self.get_parameter("frame_rate").value)
        if not math.isfinite(frame_rate) or frame_rate <= 0.0:
            raise ValueError(f"frame_rate must be finite and positive, got {frame_rate}")
        wait_s = declare_and_validate(self)

        self._pub = self.create_publisher(JointState, self._topic, 10)
        self._gate = FirstFrameGate(self, self._pub, self._topic, wait_s)
        self._frames = curl_sequence(
            amplitude=float(self.get_parameter("amplitude").value),
            steps=int(self.get_parameter("steps").value),
        )
        self._i = 0
        self._finger = ""
        self.create_timer(1.0 / frame_rate, self._publish_next)

    def _publish_next(self) -> None:
        if not self._gate.ready():
            return
        if self._i >= len(self._frames):
            self.get_logger().info("wave check complete")
            raise SystemExit(0)
        label, target = self._frames[self._i]
        self._i += 1
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Named rather than a bare 20-vector, so the driver resolves by name instead of trusting
        # this tool's column order.
        msg.name = list(self._joint_names)
        msg.position = target.tolist()
        self._pub.publish(msg)

        # rclpy throttles per call site regardless of the message, so sharing one site would let a
        # finger transition be swallowed by the per-step spam -- the one label that must not be
        # missed, because it is what ties an observed motion to a commanded finger.
        finger = label.split("_", 1)[0]
        if finger != self._finger:
            self._finger = finger
            self.get_logger().info(label)
        else:
            self.get_logger().info(label, throttle_duration_sec=0.5)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WaveCheck()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
