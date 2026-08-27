"""
ROS2 node that controls the Unitree G1 arms. This is the ONLY node that may
construct G1_23_ArmController -- see robot_arm.py's writer lockfile, which
turns a second instance into a startup failure instead of two processes
silently fighting over rt/lowcmd/rt/arm_sdk.

Three modes, switchable at runtime via the 'mode' ROS parameter:

  pose (default)
    pico_input -> /left_arm_target_pose  (chest frame)
              -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd
    pico_input -> /right_arm_target_pose (chest frame)
              -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd

  joint_replay
    joint_replay_publisher.py -> /left_arm/joint_targets  (sensor_msgs/JointState)
                              -> /right_arm/joint_targets (sensor_msgs/JointState)
              -> g1_world_output (interpolate by arrival time) -> DDS LowCmd
    For sources that ship joint angles directly (e.g. a replayed reference
    trajectory) rather than end-effector poses -- no IK involved. See
    TUITION.md/HANDOFF_README.md: a 50 FPS offline reference must not be
    treated as 50 Hz step commands, so this mode interpolates between the
    two most recently received samples rather than holding/jumping.

  idle
    Holds the arms at their current measured position. Used as a safe
    parking mode between the other two.

Messages arriving for a topic that doesn't match the active mode are
dropped with a throttled warning, not queued. Switching modes seeds the
next command from the arm's current measured position first (bumpless
transfer), so a mode change itself can never produce a step.

Topic contract for 'pose' matches tianji_world_output so pico_input /
Monitor can swap output devices without remapping publishers.
"""

from __future__ import annotations

import argparse
import signal
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
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

VALID_MODES = ('pose', 'joint_replay', 'idle')


class _SideBuffer:
    """Two most-recent (timestamp, q) samples for one arm, for interpolation.

    seed() is for a mode switch (or startup): both prev and next are set to
    the same (measured) value so interpolate() holds there until a real
    sample arrives via push(), which then ramps from that measured position
    to the first real target instead of stepping.
    """

    def __init__(self):
        self.prev_t: Optional[float] = None
        self.prev_q: Optional[list] = None
        self.next_t: Optional[float] = None
        self.next_q: Optional[list] = None

    def seed(self, t: float, q: list) -> None:
        self.prev_t, self.prev_q = t, list(q)
        self.next_t, self.next_q = t, list(q)

    def push(self, t: float, q: list) -> None:
        if self.next_q is not None:
            self.prev_t, self.prev_q = self.next_t, self.next_q
        else:
            self.prev_t, self.prev_q = t, q
        self.next_t, self.next_q = t, q

    def interpolate(self, now: float) -> Optional[list]:
        if self.next_q is None:
            return None
        if self.prev_q is None or self.next_t <= self.prev_t:
            return self.next_q
        alpha = (now - self.prev_t) / (self.next_t - self.prev_t)
        alpha = min(max(alpha, 0.0), 1.0)
        return [
            (1.0 - alpha) * p + alpha * n
            for p, n in zip(self.prev_q, self.next_q)
        ]


class G1WorldOutputNode(Node):
    """ROS2 node: pose or joint-angle topics -> DDS. Sole DDS writer."""

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
        self.declare_parameter('mode', 'pose')
        # 'G1_23' (real rig, full pose+DDS) or 'G1_29' (joint_replay/sim only
        # -- 7-DoF-arm joint names for the SOT bundle; no DDS/IK yet).
        self.declare_parameter('arm_type', self._cfg.arm_type)

        arm_type = str(self.get_parameter('arm_type').value)
        control_rate = float(self.get_parameter('control_rate').value)
        if motion_mode is None:
            motion_mode = bool(self.get_parameter('motion_mode').value)
        if simulation_mode is None:
            simulation_mode = bool(self.get_parameter('simulation_mode').value)
        if not dry_run:
            dry_run = bool(self.get_parameter('dry_run').value)

        initial_mode = str(self.get_parameter('mode').value)
        if initial_mode not in VALID_MODES:
            self.get_logger().warning(
                f"Unknown mode '{initial_mode}', defaulting to 'pose'"
            )
            initial_mode = 'pose'
        if initial_mode == 'pose' and arm_type != 'G1_23':
            raise ValueError(
                f"mode=pose requires arm_type=G1_23 (IK/DDS are G1_23-only); "
                f"got arm_type={arm_type}. Use mode:=joint_replay or idle."
            )

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
            arm_type=arm_type,
        )

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
        self.left_joint_targets_sub = self.create_subscription(
            JointState, '/left_arm/joint_targets', self._left_joint_targets_callback, 10
        )
        self.right_joint_targets_sub = self.create_subscription(
            JointState, '/right_arm/joint_targets', self._right_joint_targets_callback, 10
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

        self._joint_buffers = {'left': _SideBuffer(), 'right': _SideBuffer()}
        self._bad_target_warned = {'left': False, 'right': False}
        self._first_joint_target_received = False

        # mode starts as 'pose' as a placeholder so _enter_mode's transition
        # logging/logic is well-defined even when initial_mode is 'pose'.
        self.mode = 'pose'
        self._enter_mode(initial_mode)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.joint_publish_timer = self.create_timer(0.01, self._publish_joint_state)

        self.get_logger().info(
            f"G1 World Output node initialized (mode={self.mode}, Topic-based, no TF)."
        )
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /left_arm_target_pose, /right_arm_target_pose (mode=pose)")
        self.get_logger().info("  - /left_arm_elbow_direction, /right_arm_elbow_direction (optional, echo only)")
        self.get_logger().info("  - /left_arm/joint_targets, /right_arm/joint_targets (mode=joint_replay)")

        self._first_pose_received = False
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    # ==================== Mode management ====================

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _enter_mode(self, new_mode: str) -> None:
        """Switch active mode with bumpless transfer.

        Seeds the joint-replay interpolation buffers from the arm's current
        measured position before switching, so whichever mode is entered
        next starts from where the arm actually is. For 'pose', also runs
        the existing move_to_init() reset-pose IK solve (unchanged
        behavior); for 'joint_replay'/'idle' it is skipped so those modes
        never need Pinocchio+CasADi.
        """
        old_mode = self.mode
        if new_mode == 'pose':
            self.controller.move_to_init(wait=True, timeout=2.0)

        now = self._now()
        left_q, right_q = self.controller.get_current_joints()
        if left_q is not None:
            self._joint_buffers['left'].seed(now, left_q)
        if right_q is not None:
            self._joint_buffers['right'].seed(now, right_q)

        self.mode = new_mode
        self.get_logger().info(f"Mode: {old_mode} -> {new_mode}")

    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'mode' and p.value not in VALID_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f"mode must be one of {VALID_MODES}, got '{p.value}'",
                )
            if p.name == 'mode' and p.value == 'pose' \
                    and self.controller.arm_type != 'G1_23':
                return SetParametersResult(
                    successful=False,
                    reason="mode=pose requires arm_type=G1_23 (IK/DDS are G1_23-only)",
                )
        for p in params:
            if p.name == 'mode' and p.value != self.mode:
                self._enter_mode(p.value)
        return SetParametersResult(successful=True)

    # ==================== 'pose' mode callbacks ====================

    def left_pose_callback(self, msg: PoseStamped):
        if self.mode != 'pose':
            self.get_logger().warning(
                f"Ignoring left pose target: mode is '{self.mode}', not 'pose'",
                throttle_duration_sec=5.0,
            )
            return
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.left_arm_pose = self._pose_to_matrix(msg.pose)

    def right_pose_callback(self, msg: PoseStamped):
        if self.mode != 'pose':
            self.get_logger().warning(
                f"Ignoring right pose target: mode is '{self.mode}', not 'pose'",
                throttle_duration_sec=5.0,
            )
            return
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.right_arm_pose = self._pose_to_matrix(msg.pose)

    def left_elbow_callback(self, msg: Vector3Stamped):
        self.left_arm_direction = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def right_elbow_callback(self, msg: Vector3Stamped):
        self.right_arm_direction = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    # ==================== 'joint_replay' mode callbacks ====================

    def _left_joint_targets_callback(self, msg: JointState) -> None:
        self._handle_joint_targets(msg, 'left')

    def _right_joint_targets_callback(self, msg: JointState) -> None:
        self._handle_joint_targets(msg, 'right')

    def _handle_joint_targets(self, msg: JointState, side: str) -> None:
        if self.mode != 'joint_replay':
            self.get_logger().warning(
                f"Ignoring {side} joint target: mode is '{self.mode}', not 'joint_replay'",
                throttle_duration_sec=5.0,
            )
            return

        names = self.controller.joint_names(side)
        by_name = dict(zip(msg.name, msg.position))
        missing = [n for n in names if n not in by_name]
        if missing:
            if not self._bad_target_warned[side]:
                self.get_logger().warning(
                    f"{side} joint target missing {missing}; dropping until a "
                    "complete message arrives",
                    throttle_duration_sec=5.0,
                )
                self._bad_target_warned[side] = True
            return
        self._bad_target_warned[side] = False

        extra = [n for n in msg.name if n not in names]
        if extra:
            self.get_logger().warning(
                f"{side} joint target has unrecognized joints {extra}; ignoring them",
                throttle_duration_sec=5.0,
            )

        if not self._first_joint_target_received:
            self._first_joint_target_received = True
            self.get_logger().info("First joint-replay target received, starting control...")

        q = [float(by_name[n]) for n in names]
        self._joint_buffers[side].push(self._now(), q)

    # ==================== Publishing ====================

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

    # ==================== Control loop ====================

    def control_loop(self) -> None:
        if self.mode == 'pose':
            self._control_loop_pose()
        elif self.mode == 'joint_replay':
            self._control_loop_joint_replay()
        elif self.mode == 'idle':
            self._control_loop_idle()

    def _control_loop_pose(self) -> None:
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
            self._log_pose_status(l_success, r_success, l_joints, r_joints)

    def _control_loop_joint_replay(self) -> None:
        now = self._now()
        left_q = self._joint_buffers['left'].interpolate(now)
        right_q = self._joint_buffers['right'].interpolate(now)
        if left_q is None and right_q is None:
            return

        l_success, r_success = self.controller.move_to_joints_direct(
            left_q=left_q, right_q=right_q,
        )
        self._publish_joint_command(
            left_q if l_success else None, right_q if r_success else None
        )

        self._debug_counter += 1
        if self._debug_counter >= self._debug_interval:
            self._debug_counter = 0
            line = f"joint_replay: L_active={l_success} R_active={r_success}"
            if left_q:
                line += f" | L_q=[{','.join(f'{j:.2f}' for j in left_q)}]"
            if right_q:
                line += f" | R_q=[{','.join(f'{j:.2f}' for j in right_q)}]"
            self.get_logger().info(line)
            self._write_detail_log(line)

    def _control_loop_idle(self) -> None:
        left_q, right_q = self.controller.get_current_joints()
        if left_q is None and right_q is None:
            return
        self.controller.move_to_joints_direct(left_q=left_q, right_q=right_q)
        self._publish_joint_command(left_q, right_q)

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

    def _log_pose_status(self, l_success, r_success, l_joints, r_joints) -> None:
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
             'Still needs Pinocchio+CasADi in "pose" mode -- pair with scripts/mujoco_visualizer.py '
             'in the teleop container to see the result without a physical G1.',
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
