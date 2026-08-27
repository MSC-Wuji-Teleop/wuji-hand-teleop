#!/usr/bin/env python3
"""
Publishes a SOT bundle's G1 arm reference trajectory as named JointState
targets for g1_world_output_node's 'joint_replay' mode.

Publishes:
    /left_arm/joint_targets  (sensor_msgs/JointState)
    /right_arm/joint_targets (sensor_msgs/JointState)

Reads the 29-DoF body_q trajectory from controller_reference_v7.npz and the
name/order metadata from target_meta.json (joint_actuator_order.body_actuators).
For each side, it publishes every {side}_shoulder*/elbow/wrist* joint found
in that metadata -- 7 per arm for this bundle's native 29-DoF layout. It does
NOT filter that down to a specific rig's DoF count: which of those names a
given consumer actually uses is g1_world_output_node's job (it matches by
name against G1CartesianController.joint_names() and ignores/warns on
unrecognized extras), so this script keeps working unchanged whether the
consumer is the current 23-DoF controller (5/arm) or a future 29-DoF one
(7/arm). Leg and waist joints are never published (not per-arm; out of
scope for this topic). Per TUITION.md Sec. 3.2: "Map joints by joint name.
Do not map only by array index." -- selection here is by name for the same
reason.

Deliberately imports only rclpy/numpy/sensor_msgs so this can run in the
plain teleop container (NumPy 2, no Pinocchio): it never talks to DDS and
must NOT import g1_controller/robot_arm/robot_arm_ik/unitree_sdk2py.

Usage:
    python3 joint_replay_publisher.py \\
        --npz .../GT/g1_reference/controller_reference_v7.npz \\
        --meta .../GT/g1_reference/target_meta.json \\
        [--rate HZ] [--loop] [--side {left,right,both}]

Per HANDOFF_README.md/TUITION.md: run one low-contact, low-motion sample
first, and start g1_world_output_node with --dry-run before ever pointing
this at real DDS.
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_ARM_KEYWORDS = ("shoulder", "elbow", "wrist")


def _arm_joint_names(actuator_order: list, side: str) -> list:
    """All of this side's arm joint names, in actuator_order's own order.

    Not hardcoded to a DoF count: returns whatever {side}_shoulder*/elbow/
    wrist* names exist in the metadata, so it picks up 5 (G1_23) or 7
    (G1_29) automatically. Legs and waist are excluded by construction
    (they don't match the side prefix + arm keyword).
    """
    prefix = f"{side}_"
    return [
        n for n in actuator_order
        if n.startswith(prefix) and any(k in n for k in _ARM_KEYWORDS)
    ]


class JointReplayPublisher(Node):
    def __init__(self, npz_path: str, meta_path: str, rate: float, loop: bool, sides: list):
        super().__init__('joint_replay_publisher')

        with open(meta_path) as f:
            meta = json.load(f)
        actuator_order = meta['joint_actuator_order']['body_actuators']

        data = np.load(npz_path)
        if 'body_q' not in data.files:
            raise ValueError(f"{npz_path} has no 'body_q' array; wrong npz?")
        body_q = np.asarray(data['body_q'], dtype=np.float64)
        if body_q.shape[1] != len(actuator_order):
            raise ValueError(
                f"body_q has {body_q.shape[1]} columns but target_meta.json lists "
                f"{len(actuator_order)} joint names -- npz/meta mismatch"
            )
        name_to_col = {name: i for i, name in enumerate(actuator_order)}

        self._frames = {}
        for side in sides:
            joint_names = _arm_joint_names(actuator_order, side)
            if not joint_names:
                raise ValueError(f"No {side} arm joints found in {meta_path}")
            cols = [name_to_col[n] for n in joint_names]
            self._frames[side] = (joint_names, body_q[:, cols])
            self.get_logger().info(f"{side}: publishing {joint_names}")

        source_fps = float(data['target_fps']) if 'target_fps' in data.files \
            else float(meta.get('target_fps', 50.0))
        time_scale = float(data['time_scale']) if 'time_scale' in data.files else 1.0
        step_dt = (time_scale / source_fps) if rate <= 0 else (1.0 / rate)

        self._loop = loop
        self._num_frames = body_q.shape[0]
        self._idx = 0

        self._pubs = {
            side: self.create_publisher(JointState, f'/{side}_arm/joint_targets', 10)
            for side in sides
        }

        self.get_logger().info(
            f"Replaying {self._num_frames} frames, source_fps={source_fps:.1f} "
            f"time_scale={time_scale:.3f} -> publishing every {step_dt * 1000:.1f} ms "
            f"(loop={loop})"
        )
        self.timer = self.create_timer(step_dt, self._tick)

    def _tick(self) -> None:
        if self._idx >= self._num_frames:
            if self._loop:
                self._idx = 0
            else:
                # Hold the final frame -- never snap to neutral (TUITION.md Sec 8:
                # "hold the final target ... never abruptly zero the command").
                self._idx = self._num_frames - 1

        stamp = self.get_clock().now().to_msg()
        for side, (names, q_arr) in self._frames.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = names
            msg.position = [float(v) for v in q_arr[self._idx]]
            self._pubs[side].publish(msg)

        self._idx += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--npz', required=True, help='Path to controller_reference_v7.npz')
    parser.add_argument('--meta', required=True, help='Path to target_meta.json')
    parser.add_argument(
        '--rate', type=float, default=0.0,
        help='Publish rate in Hz. Default: source_fps/time_scale from the npz '
             '(redistributes time, same as TUITION.md Sec 7 Stage E playback-speed '
             'guidance -- never scales joint amplitude).',
    )
    parser.add_argument('--loop', action='store_true', help='Loop back to frame 0 at the end')
    parser.add_argument('--side', choices=['left', 'right', 'both'], default='both')
    args = parser.parse_args()

    sides = ['left', 'right'] if args.side == 'both' else [args.side]

    rclpy.init()
    node = JointReplayPublisher(args.npz, args.meta, args.rate, args.loop, sides)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
