#!/usr/bin/env python3
"""Watch the hand's DERIVED joint velocity at a rate a person can read.

`/joint_states` from the hand arrives at ~100 Hz, so `ros2 topic echo` is either one frozen
printout or a blur. This subscribes at full rate and redraws a summary a few times a second, which
is what you need to answer the three questions the derivation actually raises:

* does a joint you move by hand read the RIGHT SIGN and roughly the right rate;
* does it settle to ~0 AT REST, or buzz (the filter cutoff is too high);
* does it LAG a quick motion (the cutoff is too low).

    hand_velocity_watch.py                 # live table, Ctrl-C to stop
    hand_velocity_watch.py --top 5         # only the five fastest joints
    hand_velocity_watch.py --once          # a single reading

TUNING WHAT IT SHOWS. The velocity is differenced from positions and low-passed at
`measured_velocity_filter_hz` (default 20 Hz) in the driver, NOT measured -- this hand reports
position and current only. If the numbers are hashy at rest, relaunch the driver lower; if they lag,
higher. The peak-since-start column is there because a lagging filter shows up as a peak that never
reaches the rate you know you moved at.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

#: Hand joints are the ones named for a side's fingers; the arms share this topic.
_HAND_PREFIXES = ("r_", "l_")


def _is_hand(name: str) -> bool:
    return name.startswith(_HAND_PREFIXES)


def _render(latest, peak, top):
    """One frame of the table: the joints, sorted by how fast they are moving right now."""
    rows = sorted(latest.items(), key=lambda item: -abs(item[1][1]))
    if top:
        rows = rows[:top]
    lines = ["%-26s %10s %12s %12s" % ("joint", "pos(rad)", "vel(rad/s)", "peak|vel|")]
    for name, (position, velocity) in rows:
        lines.append("%-26s %+10.4f %+12.4f %12.4f" % (name, position, velocity, peak.get(name, 0.0)))
    fastest = max((abs(v) for _, v in latest.values()), default=0.0)
    lines.append("max |vel| now: %.4f rad/s   (at rest this should settle to ~0)" % fastest)
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=float, default=2.0, help="redraws per second (default: 2)")
    parser.add_argument("--top", type=int, default=0, help="show only the N fastest joints (default: all)")
    parser.add_argument("--once", action="store_true", help="print one reading and exit")
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("hand_velocity_watch")
    latest: dict[str, tuple[float, float]] = {}
    peak: dict[str, float] = {}
    missing_velocity = []

    def on_state(msg):
        names = [n for n in msg.name if _is_hand(n)]
        if not names:
            return  # an arm's message; it shares this topic
        if not msg.velocity:
            # The whole point of the tool: say so rather than drawing a table of zeros.
            missing_velocity.append(len(msg.name))
            return
        for i, name in enumerate(msg.name):
            if not _is_hand(name) or i >= len(msg.velocity):
                continue
            velocity = float(msg.velocity[i])
            latest[name] = (float(msg.position[i]), velocity)
            peak[name] = max(peak.get(name, 0.0), abs(velocity))

    node.create_subscription(JointState, "/joint_states", on_state, 50)

    deadline = time.monotonic() + 5.0
    while not latest and not missing_velocity and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    try:
        if missing_velocity and not latest:
            print(
                "the hand is publishing positions but NO velocities. The driver derives them; a "
                "driver that predates that change publishes an empty velocity field.",
                file=sys.stderr,
            )
            return 1
        if not latest:
            print("no hand joints on /joint_states -- is the hand driver up?", file=sys.stderr)
            return 1
        period = 1.0 / args.rate
        while True:
            print("\n" + _render(latest, peak, args.top), flush=True)
            if args.once:
                return 0
            end = time.monotonic() + period
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
