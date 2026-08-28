#!/usr/bin/env python3
"""run_ctl: operator CLI for the replay supervisor (spec_1 component 6).

Every motion-initiating transition is an operator service call; every stop
is automatic or operator. Solo operation: one launch terminal, one run_ctl
terminal, one hand on the physical e-stop.

    run_ctl load CLIP [--speed 0.5] [--arms left,right] [--hands left,right]
                      [--override-gt-gate] [--override-first-clip]
                      [--operator NAME]
    run_ctl arm            # publish_first -> engage -> approach -> barrier
    run_ctl start          # begin the clip (requires ARMED)
    run_ctl stop           # operator stop = the fault path; no resume
    run_ctl park           # end of run: devices approach their snapshots
    run_ctl release        # weight down; closes the run dir and the bag
    run_ctl clear-fault    # operator-only; next step is a fresh load
    run_ctl status [-w]    # print /run/status once (or watch)
"""

from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger

SUPERVISOR = '/replay_supervisor'
TIMEOUT_S = 5.0


class _Ctl(Node):
    def __init__(self):
        super().__init__('run_ctl')

    def call_trigger(self, service: str) -> int:
        client = self.create_client(Trigger, f'{SUPERVISOR}/{service}')
        if not client.wait_for_service(timeout_sec=TIMEOUT_S):
            print(f'error: {SUPERVISOR}/{service} not available '
                  f'(is the supervisor running?)', file=sys.stderr)
            return 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_S)
        if future.result() is None:
            print('error: service call timed out', file=sys.stderr)
            return 1
        resp = future.result()
        print(('ok:      ' if resp.success else 'refused: ') + resp.message)
        return 0 if resp.success else 2

    def set_load_request(self, payload: str) -> int:
        client = self.create_client(SetParameters, f'{SUPERVISOR}/set_parameters')
        if not client.wait_for_service(timeout_sec=TIMEOUT_S):
            print(f'error: {SUPERVISOR} parameter service not available',
                  file=sys.stderr)
            return 1
        p = Parameter()
        p.name = 'load_request'
        p.value = ParameterValue(type=4, string_value=payload)
        future = client.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_S)
        if future.result() is None or not future.result().results[0].successful:
            print('error: setting load_request failed', file=sys.stderr)
            return 1
        return 0

    def print_status(self, watch: bool) -> int:
        done = {'n': 0}

        def cb(msg):
            print(json.dumps(json.loads(msg.data), indent=1, sort_keys=True))
            done['n'] += 1

        self.create_subscription(String, '/run/status', cb, 10)
        import time
        deadline = time.monotonic() + TIMEOUT_S
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if not watch and done['n'] > 0:
                return 0
            if not watch and time.monotonic() > deadline:
                print('error: no /run/status heard (supervisor down?)',
                      file=sys.stderr)
                return 1
        return 0


def main(argv=None) -> int:
    raw_argv = sys.argv if argv is None else ['run_ctl', *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    p_load = sub.add_parser('load')
    p_load.add_argument('clip')
    p_load.add_argument('--speed', type=float, default=1.0)
    p_load.add_argument('--arms', default='left,right')
    p_load.add_argument('--hands', default='left,right')
    p_load.add_argument('--override-gt-gate', action='store_true')
    p_load.add_argument('--override-first-clip', action='store_true')
    p_load.add_argument('--operator', default='')
    for cmd in ('arm', 'start', 'stop', 'park', 'release', 'clear-fault'):
        sub.add_parser(cmd)
    p_status = sub.add_parser('status')
    p_status.add_argument('-w', '--watch', action='store_true')

    args = parser.parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = _Ctl()
    try:
        if args.command == 'load':
            payload = json.dumps({
                'clip': args.clip,
                'speed_scale': args.speed,
                'arms': [s for s in args.arms.split(',') if s],
                'hands': [s for s in args.hands.split(',') if s],
                'override_gt_gate': args.override_gt_gate,
                'override_first_clip': args.override_first_clip,
                'operator': args.operator,
            })
            rc = node.set_load_request(payload)
            if rc == 0:
                rc = node.call_trigger('load')
            return rc
        if args.command == 'status':
            return node.print_status(args.watch)
        return node.call_trigger(args.command.replace('-', '_'))
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
