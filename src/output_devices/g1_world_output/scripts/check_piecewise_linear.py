#!/usr/bin/env python3
"""Stage 0 piecewise-linear assert (spec_1): record + analyze.

record (container, during a sim replay run):
    python3 scripts/check_piecewise_linear.py record \
        --topic /left_arm/joint_commands --duration 20 --out /tmp/left.npz

analyze (anywhere with numpy; the judgment is pure and unit-tested in
tests/test_command_stream_check.py):
    python3 scripts/check_piecewise_linear.py analyze /tmp/left.npz \
        --vel-limit 0.5

Exit codes: 0 = piecewise-linear, 2 = failed the assert, 1 = error.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_world_output.command_stream_check import analyze_command_stream  # noqa: E402


def record(topic: str, duration: float, out: Path) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('command_stream_recorder')
    rows = []

    qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=10)
    node.create_subscription(
        JointState, topic,
        lambda m: rows.append((time.monotonic(), list(m.position))), qos)

    node.get_logger().info(f'recording {topic} for {duration:.0f} s...')
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if len(rows) < 10:
        print(f'only {len(rows)} messages heard on {topic}; is the replay '
              f'running?', file=sys.stderr)
        return 1
    t = np.array([r[0] for r in rows])
    q = np.array([r[1] for r in rows])
    np.savez(out, t=t, q=q, topic=topic)
    print(f'{len(rows)} samples -> {out}')
    return 0


def analyze(npz_path: Path, vel_limit: float) -> int:
    data = np.load(npz_path, allow_pickle=True)
    check = analyze_command_stream(data['t'], data['q'], vel_limit)
    print(f"topic:              {data.get('topic')}")
    print(f"ticks (moving/all): {check.moving_ticks}/{check.total_ticks}")
    print(f"duplicate_fraction: {check.duplicate_fraction:.3f}")
    print(f"max_tick_step_rad:  {check.max_tick_step_rad:.5f}")
    print(f"piecewise_linear:   {check.piecewise_linear}")
    for r in check.reasons:
        print(f"  - {r}")
    return 0 if check.piecewise_linear else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p_rec = sub.add_parser('record')
    p_rec.add_argument('--topic', default='/left_arm/joint_commands')
    p_rec.add_argument('--duration', type=float, default=20.0)
    p_rec.add_argument('--out', type=Path, required=True)
    p_an = sub.add_parser('analyze')
    p_an.add_argument('npz', type=Path)
    p_an.add_argument('--vel-limit', type=float, default=0.5,
                      help='deploy velocity (rad/s) for the step threshold')
    args = parser.parse_args(argv)
    if args.command == 'record':
        return record(args.topic, args.duration, args.out)
    return analyze(args.npz, args.vel_limit)


if __name__ == '__main__':
    sys.exit(main())
