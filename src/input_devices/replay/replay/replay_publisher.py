#!/usr/bin/env python3
"""
SOT bundle replay publisher -- an "input device" that replays one sample.

Reads a sample method directory (GT/ or Ours/) from the
RobotSTAR_demos handoff bundle and publishes, on one shared
timer so arms and hands stay time-aligned:

  arms  -> /left_arm/joint_targets, /right_arm/joint_targets
           (sensor_msgs/JointState, named)
           from g1_reference/controller_reference_v7.npz body_q, using
           joint names from target_meta.json. Every {side}_shoulder*/elbow/
           wrist* joint present in the bundle is published (7/arm for the
           native 29-DoF layout); the consumer (g1_world_output_node in
           mode:=joint_replay) matches by name and ignores joints its rig
           lacks, so this works for both the current 23-DoF controller and
           a future 29-DoF one.

  hands -> /left_hand/keypoints21, /right_hand/keypoints21
           (std_msgs/Float64MultiArray, 63 = 21x3 floats, meters,
           MediaPipe order) from hand2_input/*_human_targets_v5.npz.
           These are the HUMAN keypoints, not joint angles: retargeting to
           Hand 2's 20 DoF happens live in wujihand_controller
           (input_source: "keypoints_topic"), i.e. through the exact same
           production path glove teleop uses. This is deliberate: the
           bundle's precomputed hand joint columns target the legacy hand
           model and must never be used; hand joints are regenerated from
           these keypoints.

Timeline: body_q is the retimed reference (frames @ target_fps); the hand
keypoints are on the source timeline (source_frames @ source_fps) spanning
the same wall-clock duration (all 15 bundle samples are a uniform resample;
asserted at load). Each body tick i maps to hand frame
round(i * (T_hand-1)/(T_body-1)). At clip end the last frame is held
(bundle semantics: hold_last_target), unless --loop.

Pure rclpy + numpy: runs in the main teleop container, next to the hand
controllers it feeds. Never touches DDS or hand/arm SDKs.

Usage:
    ros2 run replay replay_publisher -- \
        --method-dir <sample>/GT [--rate HZ] [--loop] [--no-arms] [--no-hands]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

_ARM_KEYWORDS = ("shoulder", "elbow", "wrist")


def _arm_joint_names(actuator_order: list, side: str) -> list:
    """All of one side's arm joint names, in the bundle's own order."""
    prefix = f"{side}_"
    return [
        n for n in actuator_order
        if n.startswith(prefix) and any(k in n for k in _ARM_KEYWORDS)
    ]


class ReplayPublisher(Node):
    def __init__(self, method_dir: Path, rate: float, loop: bool,
                 arms: bool, hands: bool):
        super().__init__('replay_publisher')
        if not (arms or hands):
            raise ValueError("Nothing to publish: both --no-arms and --no-hands given")

        meta_path = method_dir / 'g1_reference' / 'target_meta.json'
        if not meta_path.exists():
            raise FileNotFoundError(
                f"{meta_path} not found. --method-dir must be a sample's GT/ or "
                "Ours/ directory. In the teleop container the bundle is mounted "
                "read-only at /home/wuji/ros2_ws/RobotSTAR_demos "
                "(docker-compose.yml); a container created before that mount "
                "was added needs `docker compose up -d teleop` to recreate it."
            )
        with open(meta_path) as f:
            meta = json.load(f)
        frames = int(meta['frames'])
        source_frames = int(meta['source_frames'])
        ratio = meta['target_fps'] / meta['source_fps']
        if frames != round(source_frames * ratio):
            raise ValueError(
                f"Non-uniform retime (frames={frames}, source_frames={source_frames}, "
                f"fps ratio={ratio}) -- the normalized-time arm/hand alignment "
                "below would be wrong for this sample."
            )

        self._num_frames = frames
        self._idx = 0
        self._loop = loop
        self._arm_frames = {}
        self._hand_frames = {}

        if arms:
            npz = method_dir / 'g1_reference' / 'controller_reference_v7.npz'
            data = np.load(npz)
            body_q = np.asarray(data['body_q'], dtype=np.float64)
            actuator_order = meta['joint_actuator_order']['body_actuators']
            if body_q.shape != (frames, len(actuator_order)):
                raise ValueError(
                    f"body_q shape {body_q.shape} vs meta "
                    f"({frames}, {len(actuator_order)}) -- npz/meta mismatch"
                )
            name_to_col = {n: i for i, n in enumerate(actuator_order)}
            for side in ('left', 'right'):
                names = _arm_joint_names(actuator_order, side)
                if not names:
                    raise ValueError(f"No {side} arm joints in {meta_path}")
                self._arm_frames[side] = (names, body_q[:, [name_to_col[n] for n in names]])
                self.get_logger().info(f"arms/{side}: {names}")

        if hands:
            kp_candidates = sorted(method_dir.glob('hand2_input/*_human_targets_v5.npz'))
            if len(kp_candidates) != 1:
                raise FileNotFoundError(
                    f"Expected one hand2_input/*_human_targets_v5.npz in "
                    f"{method_dir}, found {kp_candidates}"
                )
            kp_data = np.load(kp_candidates[0])
            for side in ('left', 'right'):
                kp = np.asarray(kp_data[f'{side}_hand_keypoints21'], dtype=np.float64)
                if kp.shape != (source_frames, 21, 3):
                    raise ValueError(
                        f"{side}_hand_keypoints21 shape {kp.shape}, expected "
                        f"({source_frames}, 21, 3)"
                    )
                self._hand_frames[side] = kp
            self.get_logger().info(
                f"hands: {source_frames} keypoint frames from {kp_candidates[0].name} "
                f"(retargeted live by wujihand_controller, input_source=keypoints_topic)"
            )

        self._arm_pubs = {
            side: self.create_publisher(JointState, f'/{side}_arm/joint_targets', 10)
            for side in self._arm_frames
        }
        self._hand_pubs = {
            side: self.create_publisher(Float64MultiArray, f'/{side}_hand/keypoints21', 10)
            for side in self._hand_frames
        }

        step_dt = (meta['time_scale'] / meta['target_fps']) if rate <= 0 else (1.0 / rate)
        self.get_logger().info(
            f"Replaying {method_dir} -- {frames} frames every {step_dt * 1000:.1f} ms "
            f"(loop={loop}, arms={sorted(self._arm_frames)}, hands={sorted(self._hand_frames)})"
        )
        self.timer = self.create_timer(step_dt, self._tick)

    def _hand_index(self, body_idx: int, hand_len: int) -> int:
        if self._num_frames <= 1:
            return 0
        return round(body_idx * (hand_len - 1) / (self._num_frames - 1))

    def _tick(self) -> None:
        if self._idx >= self._num_frames:
            if self._loop:
                self._idx = 0
            else:
                # Hold the final frame (bundle end behavior: hold_last_target).
                self._idx = self._num_frames - 1

        stamp = self.get_clock().now().to_msg()
        for side, (names, q_arr) in self._arm_frames.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = names
            msg.position = [float(v) for v in q_arr[self._idx]]
            self._arm_pubs[side].publish(msg)

        for side, kp in self._hand_frames.items():
            msg = Float64MultiArray()
            msg.data = [float(v) for v in kp[self._hand_index(self._idx, kp.shape[0])].ravel()]
            self._hand_pubs[side].publish(msg)

        self._idx += 1


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ['replay_publisher', *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--method-dir', required=True, type=Path,
        help="A sample's GT/ or Ours/ directory (contains g1_reference/ and hand2_input/)",
    )
    parser.add_argument(
        '--rate', type=float, default=0.0,
        help='Publish rate in Hz. Default: target_fps/time_scale from the bundle '
             '(slower playback redistributes time, never scales amplitude).',
    )
    parser.add_argument('--loop', action='store_true', help='Loop back to frame 0 at clip end')
    parser.add_argument('--no-arms', action='store_true', help='Skip arm joint targets')
    parser.add_argument('--no-hands', action='store_true', help='Skip hand keypoints')
    args = parser.parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = ReplayPublisher(
        args.method_dir, args.rate, args.loop,
        arms=not args.no_arms, hands=not args.no_hands,
    )
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
