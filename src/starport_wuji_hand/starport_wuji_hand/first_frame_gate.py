"""The bring-up tools' shared gate on their first published frame.

Both tools that command a real hand hold their first frame until something is subscribed to the
command topic. That gate is one mechanism, so it lives in one module -- split the way safety.py is
split from hand_node.py: a pure decision function with no ROS in it at all, and a thin holder that
owns the clock read, the latch and the two operator-facing log lines. Two copies of a first-frame
check is a shape this package has already been bitten by, where one copy was fixed and the other
was not.

WHY IT WAITS. A subscription that has not finished matching yet is not a missing driver: discovery
on a warm graph costs tens of milliseconds, which at 100 Hz is several frames, and a cold
participant's first exchange costs more. The default below is many multiples of the former and
still covers the latter. Waiting is strictly safer than not waiting, because nothing is published
while it waits.

WHY EXPIRY PUBLISHES ANYWAY rather than refusing. Publishing into a topic nobody holds is inert,
so there is nothing to protect the hand from; refusing would put a bench run at the mercy of DDS
matching and would throw away a trace that still diagnoses the run. Zero disables the wait, which
keeps a deliberate publish-into-nothing run available without editing a file.

WHY IT IS POLLED from the caller's timer rather than slept out in a constructor: the caller stays
under rclpy.spin for the whole wait, so Ctrl-C behaves the same during the wait as during the run,
and a tool that writes a trace on the way out still reaches it.

WHAT A MATCH DOES NOT PROVE. Any subscriber satisfies the gate -- an echo, a bag recorder, a
visualiser, or a stale one from an earlier session. The gate de-noises the head of a run; it says
nothing about who is listening.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.publisher import Publisher

# One parameter name and one default for both tools, so an operator's muscle memory carries
# between them and there is no pair of numbers that can drift apart.
WAIT_PARAM = "wait_for_subscriber_s"
DEFAULT_WAIT_FOR_SUBSCRIBER_S = 2.0


class GateDecision(Enum):
    """What the gate does with the first frame on one poll."""

    WAIT = "wait"
    MATCHED = "matched"
    EXPIRED = "expired"


def gate_decision(subscriber_count: int, elapsed_s: float, wait_s: float) -> GateDecision:
    """Whether the first frame may go out yet, and on what grounds.

    A matched subscriber wins outright, whatever the elapsed time and even when the wait is zero:
    the wait exists to give matching a chance, so evidence that matching happened ends it.

    Pure, and the whole decision: everything else in this module is plumbing around it.
    """
    if subscriber_count > 0:
        return GateDecision.MATCHED
    if elapsed_s < wait_s:
        return GateDecision.WAIT
    return GateDecision.EXPIRED


def declare_and_validate(node: Node) -> float:
    """Declare the wait parameter on ``node`` and return it, refusing an unusable value.

    A negative or non-finite wait is refused rather than coerced, so there is no wait-forever
    setting to reach for by accident; a large finite value is how you ask for one. One function
    rather than one per tool, so the refusal is a single string.
    """
    node.declare_parameter(WAIT_PARAM, DEFAULT_WAIT_FOR_SUBSCRIBER_S)
    wait_s = float(node.get_parameter(WAIT_PARAM).value)
    if not math.isfinite(wait_s) or wait_s < 0.0:
        raise ValueError(f"{WAIT_PARAM} must be finite and non-negative, got {wait_s}")
    return wait_s


class FirstFrameGate:
    """Holds a tool's first frame until a subscriber matches or the wait runs out.

    Latched: once open it stays open. A subscriber that drops out mid-run does not stop a run that
    is already going, and each log line is emitted once rather than per frame.
    """

    def __init__(
        self,
        node: Node,
        publisher: Publisher,
        topic: str,
        wait_s: float,
        require_matched=(),
        require_ready=None,
    ) -> None:
        self._node = node
        self._publisher = publisher
        self._topic = topic
        self._wait_s = wait_s
        # Topics that must have a publisher before the first frame goes out. A caller
        # that is also RECORDING needs its own inputs live: a subscriber on the command topic says
        # someone is listening, not that this node can hear anything back. Publishing early loses
        # the measurement silently, and intermittently -- discovery is fast when the far side has
        # been up a while and slow when it has just appeared.
        self._require_matched = tuple(require_matched)
        # An optional predicate for "the far side is ready to ACT on this", which having a
        # subscriber does not establish. The driver reports false while its motors are released,
        # and a command sent then is dropped rather than queued.
        self._require_ready = require_ready
        # Counted from construction, which is the moment discovery can begin.
        self._started_ns = node.get_clock().now().nanoseconds
        self._open = False

    def ready(self) -> bool:
        """True once the first frame may go out. Call it before publishing anything."""
        if self._open:
            return True
        elapsed_ns = self._node.get_clock().now().nanoseconds - self._started_ns
        unmatched = [t for t in self._require_matched if self._node.count_publishers(t) == 0]
        not_ready = self._require_ready is not None and not self._require_ready()
        if (unmatched or not_ready) and elapsed_ns * 1e-9 < self._wait_s:
            return False
        if not_ready:
            self._node.get_logger().warning(
                f"the driver never reported ready in {self._wait_s:.1f} s; publishing anyway, and "
                "anything sent before it is will be dropped"
            )
        if unmatched:
            self._node.get_logger().warning(
                f"{len(unmatched)} trace subscription(s) still have no publisher after "
                f"{self._wait_s:.1f} s; the recorded trace will be incomplete"
            )
        decision = gate_decision(self._publisher.get_subscription_count(), elapsed_ns * 1e-9, self._wait_s)
        if decision is GateDecision.WAIT:
            return False
        if decision is GateDecision.MATCHED:
            self._node.get_logger().info(
                f"a subscriber matched {self._topic} after {elapsed_ns * 1e-6:.0f} ms; any subscriber "
                "counts (an echo, a recorder, a visualiser), so this is not evidence the driver is up"
            )
        else:
            self._node.get_logger().warning(
                f"nothing is subscribed to {self._topic} after waiting {self._wait_s:.1f} s; publishing "
                f"anyway, and the hand will not move. Check that the driver is running and that "
                f"command_topic matches the namespace it runs under."
            )
        self._open = True
        return True
