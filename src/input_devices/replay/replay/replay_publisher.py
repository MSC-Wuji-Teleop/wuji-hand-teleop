#!/usr/bin/env python3
"""Conditioned-clip replay publisher (spec_1 component 2).

Paces one conditioned clip artifact onto both devices' target streams from
one timer, so arms and hands play the same clip by construction:

  arms  -> /left_arm/joint_targets, /right_arm/joint_targets
           (sensor_msgs/JointState, named q7/side, stamped)
  hands -> /left_hand/joint_targets, /right_hand/joint_targets
           (sensor_msgs/JointState, named q20, stamped)

Hand angles come from the artifact, retargeted OFFLINE by condition_clip
(Retargeter reset per clip, PCHIP-retimed onto the arm grid; TUITION 3.1).
This node never publishes keypoints21: that topic is teleop-only, and the
hardware replay pipeline carries joint targets end to end.

Service-gated; publishes nothing on spin. The full state machine and every
gate live in replay/pacer.py (ROS-free, unit-tested); this file is the
rclpy adapter. Surface (docs/spec/spec_1_interfaces.md):

  param    load_request     JSON {"clip", "speed_scale", "arms", "hands"}
  service  ~/load           consume load_request; refuse fail verdicts
                            (--force-sim overrides, simulation only)
  service  ~/publish_first  repeat frame 0, advancing stamps, no advance
  service  ~/start          begin advancing; stamp series continues
  service  ~/fault          freeze the tick; no resume (TUITION section 9)
  topic    /replay/status   String JSON

Stamps: tick j is stamped t0 + j * dt_play with dt_play =
k / (target_fps * speed_scale). speed_scale is time redistribution only,
layered on the baked k; there is no amplitude scaling anywhere.

No pause, no mid-clip resume, no loop: a faulted or finished run is parked,
inspected, and rerun from the start.

Usage:
    ros2 run replay replay_publisher              # hardware profile
    ros2 run replay replay_publisher -- --force-sim   # sim only
"""

from __future__ import annotations

import argparse
import json
import sys

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from replay.clip_artifact import ArtifactError, load_artifact
from replay.pacer import LoadError, LoadRequest, ReplayPacer

# Pinned for both target streams and both subscribers (plan amendment A5):
# RELIABLE so scoped runs cannot silently drop frames into the hand branch.
TARGET_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

STATUS_PERIOD_S = 0.1


class ReplayPublisherNode(Node):
    def __init__(self, force_sim: bool = False):
        super().__init__('replay_publisher')
        self.pacer = ReplayPacer(force_sim=force_sim)
        self.declare_parameter('load_request', '')

        self._t0_s: float = 0.0
        self._arm_pubs = {}
        self._hand_pubs = {}
        self._tick_timer = None
        self._last_status_json = None

        self._status_pub = self.create_publisher(String, '/replay/status', 10)
        self.create_timer(STATUS_PERIOD_S, self._publish_status)

        self.create_service(Trigger, '~/load', self._srv_load)
        self.create_service(Trigger, '~/publish_first', self._srv_publish_first)
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/fault', self._srv_fault)

        if force_sim:
            self.get_logger().warning(
                '--force-sim: verdict/scale/scope load gates are BYPASSED. '
                'Simulation only; never run this flag against hardware.'
            )
        self.get_logger().info(
            'replay_publisher ready (gated: nothing publishes until '
            'load + publish_first)'
        )

    # ---------------------------------------------------------- services

    def _reply(self, ok: bool, payload: dict) -> Trigger.Response:
        resp = Trigger.Response()
        resp.success = ok
        resp.message = json.dumps(payload, sort_keys=True)
        return resp

    def _srv_load(self, request, response) -> Trigger.Response:
        raw = str(self.get_parameter('load_request').value)
        try:
            req = LoadRequest.from_json(raw)
            clip = load_artifact(req.clip)
            self.pacer.load(clip, req)
        except (LoadError, ArtifactError, OSError) as exc:
            self.get_logger().error(f'load refused: {exc}')
            return self._reply(False, {'error': str(exc)})

        self._recreate_publishers()
        self._recreate_tick_timer()
        self.get_logger().info(
            f"loaded {req.clip} (sample={clip.meta.get('sample')}, "
            f"method={clip.meta.get('method')}, k={clip.k}, "
            f"speed_scale={req.speed_scale}, dt_play={self.pacer.dt_play * 1e3:.1f} ms, "
            f"arms={list(req.arms)}, hands={list(req.hands)})"
        )
        return self._reply(True, self.pacer.status())

    def _srv_publish_first(self, request, response) -> Trigger.Response:
        try:
            self.pacer.publish_first()
        except LoadError as exc:
            return self._reply(False, {'error': str(exc)})
        self._t0_s = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info('publish_first: repeating frame 0')
        return self._reply(True, self.pacer.status())

    def _srv_start(self, request, response) -> Trigger.Response:
        try:
            self.pacer.start()
        except LoadError as exc:
            return self._reply(False, {'error': str(exc)})
        self.get_logger().info('start: advancing')
        return self._reply(True, self.pacer.status())

    def _srv_fault(self, request, response) -> Trigger.Response:
        self.pacer.fault()
        self.get_logger().error('FAULT: tick frozen; a fresh load is required')
        return self._reply(True, self.pacer.status())

    # -------------------------------------------------------- publishing

    def _recreate_publishers(self) -> None:
        for pub in list(self._arm_pubs.values()) + list(self._hand_pubs.values()):
            self.destroy_publisher(pub)
        req = self.pacer.request
        self._arm_pubs = {
            side: self.create_publisher(
                JointState, f'/{side}_arm/joint_targets', TARGET_QOS)
            for side in req.arms
        }
        self._hand_pubs = {
            side: self.create_publisher(
                JointState, f'/{side}_hand/joint_targets', TARGET_QOS)
            for side in req.hands
        }

    def _recreate_tick_timer(self) -> None:
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self.destroy_timer(self._tick_timer)
        self._tick_timer = self.create_timer(self.pacer.dt_play, self._tick)

    @staticmethod
    def _stamp_from_seconds(t: float) -> TimeMsg:
        sec = int(t)
        return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))

    def _tick(self) -> None:
        out = self.pacer.tick()
        if out is None:
            return
        stamp = self._stamp_from_seconds(self._t0_s + out.stamp_offset_s)
        # One construction site for both device streams: keeping the two
        # streams identical by construction is this node's whole job.
        for targets, pubs in ((out.arm_targets, self._arm_pubs),
                              (out.hand_targets, self._hand_pubs)):
            for side, (names, q) in targets.items():
                msg = JointState()
                msg.header.stamp = stamp
                msg.name = list(names)
                msg.position = [float(v) for v in q]
                pubs[side].publish(msg)

    def _publish_status(self) -> None:
        payload = json.dumps(self.pacer.status(), sort_keys=True)
        if payload != self._last_status_json:
            self.get_logger().info(f'status: {payload}')
            self._last_status_json = payload
        msg = String()
        msg.data = payload
        self._status_pub.publish(msg)


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ['replay_publisher', *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--force-sim', action='store_true',
        help='Bypass the load gates (fail verdicts, allowed-scale, hand '
             'scope). SIMULATION ONLY.',
    )
    args = parser.parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = ReplayPublisherNode(force_sim=args.force_sim)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
