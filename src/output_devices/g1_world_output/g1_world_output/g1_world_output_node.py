"""
ROS2 node that subscribes to pose topics and controls Unitree G1 arms.

Data Flow:
  pico_input -> /left_arm_target_pose  (chest frame)
            -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd
  pico_input -> /right_arm_target_pose (chest frame)
            -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd

Topic contract matches tianji_world_output so pico_input / Monitor can swap
output devices without remapping publishers.
"""

from __future__ import annotations

import argparse
import signal
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from g1_world_output.config_loader import G1Config
from g1_world_output.g1_controller import G1CartesianController
from g1_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge

LOG_DIR = Path.home() / ".g1_teleop_logs"

ARM_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class G1WorldOutputNode(Node):
    """ROS2 node: chest-frame pose topics -> G1 pelvis remap -> IK -> DDS."""

    def __init__(
        self,
        motion_mode: bool | None = None,
        simulation_mode: bool | None = None,
        dry_run: bool = False,
    ):
        super().__init__("g1_world_output")
        setup_ros2_logging_bridge(self.get_logger())

        # Load YAML first so it can seed the ROS parameter defaults below --
        # declaring these with a literal False would make the parameter's
        # resolved value never be None, so G1CartesianController's own
        # "YAML if None" fallback (g1_controller.py) could never fire.
        # Precedence ends up: CLI flag > ROS launch param > YAML > False.
        self._cfg = G1Config.load()

        self.declare_parameter('control_rate', 90.0)
        self.declare_parameter('motion_mode', self._cfg.motion_mode)
        self.declare_parameter('simulation_mode', self._cfg.simulation_mode)
        self.declare_parameter('dry_run', False)

        control_rate = float(self.get_parameter('control_rate').value)
        if motion_mode is None:
            motion_mode = bool(self.get_parameter('motion_mode').value)
        if simulation_mode is None:
            simulation_mode = bool(self.get_parameter('simulation_mode').value)
        if not dry_run:
            dry_run = bool(self.get_parameter('dry_run').value)

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._detail_log_path = LOG_DIR / f'g1_output_{ts}.log'
        self._detail_log = None
        try:
            self._detail_log = open(self._detail_log_path, 'w', buffering=1)
            self._detail_log.write(f"# G1 World Output detailed log - {ts}\n")
            self.get_logger().info(f'Detailed log: {self._detail_log_path}')
        except OSError as e:
            self.get_logger().error(f'Cannot create detailed log file: {e}')

        logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self.get_logger().info(
            f"Initializing G1 controller (motion={motion_mode} sim={simulation_mode} dry_run={dry_run})..."
        )
        self.controller = G1CartesianController(
            config=self._cfg,
            motion_mode=motion_mode,
            simulation_mode=simulation_mode,
            logger=logger_adapter,
            connect=not dry_run,
        )
        self.controller.move_to_init(wait=True, timeout=2.0)

        self.left_arm_pose = None
        self.right_arm_pose = None
        self.left_arm_direction = self._cfg.get_default_zsp_direction('left')
        self.right_arm_direction = self._cfg.get_default_zsp_direction('right')

        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/left_arm_target_pose', self.left_pose_callback, 10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/right_arm_target_pose', self.right_pose_callback, 10
        )
        self.left_elbow_sub = self.create_subscription(
            Vector3Stamped, '/left_arm_elbow_direction', self.left_elbow_callback, 10
        )
        self.right_elbow_sub = self.create_subscription(
            Vector3Stamped, '/right_arm_elbow_direction', self.right_elbow_callback, 10
        )

        self.left_state_pub = self.create_publisher(
            JointState, '/left_arm/joint_states', ARM_JOINT_QOS
        )
        self.right_state_pub = self.create_publisher(
            JointState, '/right_arm/joint_states', ARM_JOINT_QOS
        )
        self.left_cmd_pub = self.create_publisher(
            JointState, '/left_arm/joint_commands', ARM_JOINT_QOS
        )
        self.right_cmd_pub = self.create_publisher(
            JointState, '/right_arm/joint_commands', ARM_JOINT_QOS
        )
        self.left_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/zsp_para', ARM_JOINT_QOS
        )
        self.right_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/zsp_para', ARM_JOINT_QOS
        )

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.joint_publish_timer = self.create_timer(0.01, self._publish_joint_state)

        self.get_logger().info("G1 World Output node initialized (Topic-based, no TF).")
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /left_arm_target_pose")
        self.get_logger().info("  - /right_arm_target_pose")
        self.get_logger().info("  - /left_arm_elbow_direction (optional, echo only)")
        self.get_logger().info("  - /right_arm_elbow_direction (optional, echo only)")

        self._first_pose_received = False
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    def left_pose_callback(self, msg: PoseStamped):
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.left_arm_pose = self._pose_to_matrix(msg.pose)

    def right_pose_callback(self, msg: PoseStamped):
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.right_arm_pose = self._pose_to_matrix(msg.pose)

    def left_elbow_callback(self, msg: Vector3Stamped):
        self.left_arm_direction = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def right_elbow_callback(self, msg: Vector3Stamped):
        self.right_arm_direction = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _make_arm_joint_state(self, stamp, side: str, joints, frame_id: str) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = self.controller.joint_names(side)
        msg.position = [float(j) for j in joints]
        return msg

    def _publish_joint_state(self) -> None:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
        except Exception:
            return
        stamp = self.get_clock().now().to_msg()
        if left_joints is not None:
            self.left_state_pub.publish(
                self._make_arm_joint_state(stamp, 'left', left_joints, 'left_base_state')
            )
        if right_joints is not None:
            self.right_state_pub.publish(
                self._make_arm_joint_state(stamp, 'right', right_joints, 'right_base_state')
            )

    def _publish_joint_command(self, left_joints, right_joints) -> None:
        stamp = self.get_clock().now().to_msg()
        if left_joints is not None:
            self.left_cmd_pub.publish(
                self._make_arm_joint_state(stamp, 'left', left_joints, 'left_base_cmd')
            )
        if right_joints is not None:
            self.right_cmd_pub.publish(
                self._make_arm_joint_state(stamp, 'right', right_joints, 'right_base_cmd')
            )

    def _publish_zsp_para(self) -> None:
        for zsp, pub in (
            (getattr(self.controller, 'left_zsp_para', None), self.left_zsp_para_pub),
            (getattr(self.controller, 'right_zsp_para', None), self.right_zsp_para_pub),
        ):
            if zsp:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in zsp]
                pub.publish(msg)

    def control_loop(self) -> None:
        if self.left_arm_pose is None and self.right_arm_pose is None:
            return

        self.controller.left_zsp_para = [
            self.left_arm_direction[0],
            self.left_arm_direction[1],
            self.left_arm_direction[2],
            0, 0, 0,
        ]
        self.controller.right_zsp_para = [
            self.right_arm_direction[0],
            self.right_arm_direction[1],
            self.right_arm_direction[2],
            0, 0, 0,
        ]

        l_success, r_success, l_joints, r_joints = self.controller.move_to_pose_direct(
            left_pose=self.left_arm_pose,
            right_pose=self.right_arm_pose,
            unit='matrix',
        )
        self._publish_joint_command(l_joints, r_joints)
        self._publish_zsp_para()

        self._debug_counter += 1
        if self._debug_counter >= self._debug_interval:
            self._debug_counter = 0
            self._log_control_status(l_success, r_success, l_joints, r_joints)

    @staticmethod
    def _pose_to_matrix(pose) -> np.ndarray:
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return T

    def _write_detail_log(self, msg: str) -> None:
        if self._detail_log:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self._detail_log.write(f"{ts} {msg}\n")

    def _log_control_status(self, l_success, r_success, l_joints, r_joints) -> None:
        ld = self.left_arm_direction
        rd = self.right_arm_direction
        line1 = (
            f"IK: L={l_success} R={r_success} | "
            f"zsp L=[{ld[0]:.2f},{ld[1]:.2f},{ld[2]:.2f}] "
            f"R=[{rd[0]:.2f},{rd[1]:.2f},{rd[2]:.2f}]"
        )
        self.get_logger().info(line1)

        a_line = "  L:"
        if self.left_arm_pose is not None:
            lp = self.left_arm_pose[:3, 3]
            le = R.from_matrix(self.left_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            a_line += (
                f" chest_pos=[{lp[0]:.3f},{lp[1]:.3f},{lp[2]:.3f}] "
                f"euler=[{le[0]:.1f},{le[1]:.1f},{le[2]:.1f}] deg"
            )
        else:
            a_line += " no_pose"
        if l_joints:
            a_line += f" j_rad=[{','.join(f'{j:.2f}' for j in l_joints)}]"
        elif not l_success and self.left_arm_pose is not None:
            a_line += " j=FAIL"
        self.get_logger().info(a_line)

        b_line = "  R:"
        if self.right_arm_pose is not None:
            rp = self.right_arm_pose[:3, 3]
            re_ = R.from_matrix(self.right_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            b_line += (
                f" chest_pos=[{rp[0]:.3f},{rp[1]:.3f},{rp[2]:.3f}] "
                f"euler=[{re_[0]:.1f},{re_[1]:.1f},{re_[2]:.1f}] deg"
            )
        else:
            b_line += " no_pose"
        if r_joints:
            b_line += f" j_rad=[{','.join(f'{j:.2f}' for j in r_joints)}]"
        elif not r_success and self.right_arm_pose is not None:
            b_line += " j=FAIL"
        self.get_logger().info(b_line)

        self._write_detail_log(line1)
        self._write_detail_log(a_line)
        self._write_detail_log(b_line)


def main(argv: list[str] | None = None) -> None:
    program_name = sys.argv[0] if sys.argv else "g1_world_output_node"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(description="Unitree G1 arm control node (Topic-based)")
    parser.add_argument('--motion-mode', action='store_true', help='Use rt/arm_sdk instead of rt/lowcmd')
    parser.add_argument('--simulation-mode', action='store_true', help='DDS sim domain')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Simulation mode: solve real IK from the pose topics but never connect to DDS/hardware. '
             'Still needs Pinocchio+CasADi -- pair with scripts/mujoco_visualizer.py in the teleop '
             'container to see the result without a physical G1.',
    )
    args = parser.parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = G1WorldOutputNode(
        motion_mode=True if args.motion_mode else None,
        simulation_mode=True if args.simulation_mode else None,
        dry_run=args.dry_run,
    )

    _shutdown_done = False

    def _cleanup():
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True

        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if hasattr(node, '_detail_log') and node._detail_log:
                node._detail_log.write(
                    f"# === Log ended ({datetime.now().strftime('%H:%M:%S')}) ===\n"
                )
                node._detail_log.flush()
                node._detail_log.close()
            sys.stdout.flush()
            sys.stderr.flush()
            try:
                node.controller.disable_and_release()
            except Exception as e:
                print(f'[WARNING] G1 shutdown error: {e}', file=sys.stderr)
        finally:
            signal.signal(signal.SIGINT, prev_sigint)

    def _signal_handler(signum, frame):
        sig_name = signal.Signals(signum).name
        print(f'\n[{sig_name}] Received exit signal, shutting down safely...', file=sys.stderr)
        _cleanup()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
