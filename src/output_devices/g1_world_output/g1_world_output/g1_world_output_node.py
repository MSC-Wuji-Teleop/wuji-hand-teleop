"""
ROS2 node that controls the Unitree G1 arms. This is the ONLY node that may
construct G1ArmController -- see robot_arm.py's writer lockfile, which
turns a second instance into a startup failure instead of two processes
silently fighting over rt/lowcmd/rt/arm_sdk.

Three modes, switchable at runtime via the 'mode' ROS parameter:

  pose (default)
    pico_input -> /left_arm_target_pose  (chest frame)
              -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd
    pico_input -> /right_arm_target_pose (chest frame)
              -> g1_world_output (chest->pelvis remap) -> G1 IK -> DDS LowCmd

  joint_replay
    replay_publisher -> /left_arm/joint_targets  (sensor_msgs/JointState,
                     -> /right_arm/joint_targets  named, stamped)
              -> StreamBuffer (ramp toward newest over one stamped period)
              -> device FSM (engage/approach/track/end_hold/release,
                 spec_1 section 8) + safety chain (position clamp, per-joint
                 rate limit, staleness hold, divergence fault)
              -> DDS LowCmd under the arm-slots+weight slot policy
    Service-gated: ~/engage ~/approach ~/track ~/end_hold ~/park ~/release
    ~/fault ~/clear_fault (std_srvs/Trigger; non-blocking accept/reject,
    progress in the control loop, completion on /g1/status). See
    docs/spec/spec_1_interfaces.md.

  idle
    Holds the arms at their current measured position. Used as a safe
    parking mode between the other two (teleop legacy; the replay path
    parks via the FSM instead).

Publications: /left,right_arm/joint_states (position, velocity, effort),
/left,right_arm/joint_commands, /g1/imu (sensor_msgs/Imu), /g1/status
(String JSON: fsm fields + mode_machine, tick, lowstate age, max motor
temperature, min arm bus voltage).

--read-only (Stage A, TUITION 7A): subscribe lowstate, publish
joint_states/imu/status; never construct the DDS writer, never touch the
weight, refuse every motion service.

Messages arriving for a topic that doesn't match the active mode are
dropped with a throttled warning, not queued. Switching modes seeds the
next command from the arm's current measured position first (bumpless
transfer); switching away from joint_replay is refused while the device
FSM holds any weight.
"""

from __future__ import annotations

import argparse
import json
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
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_world_output.config_loader import G1Config
from g1_world_output.device_fsm import ArmDeviceFSM, DeviceState, FsmConfig, TickInputs
from g1_world_output.g1_controller import G1CartesianController
from g1_world_output.replay_safety import ArmLimits, ReplaySafetyChain
from g1_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge
from g1_world_output.stream_buffer import StreamBuffer

LOG_DIR = Path.home() / ".g1_teleop_logs"

ARM_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# joint_targets streams are RELIABLE end to end (plan amendment A5): the
# publisher pins the same profile, and a BEST_EFFORT mismatch would silently
# drop frames into the interpolators.
TARGET_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

VALID_MODES = ('pose', 'joint_replay', 'idle')

STATUS_PERIOD_S = 0.1


class G1WorldOutputNode(Node):
    """ROS2 node: pose or joint-angle topics -> DDS. Sole DDS writer."""

    def __init__(
        self,
        motion_mode: bool | None = None,
        simulation_mode: bool | None = None,
        dry_run: bool = False,
        read_only: bool = False,
    ):
        super().__init__("g1_world_output")
        setup_ros2_logging_bridge(self.get_logger())

        # Load YAML first so it can seed the ROS parameter defaults below --
        # precedence: CLI flag > ROS launch param > YAML > False.
        self._cfg = G1Config.load()

        # 90 Hz default serves the pose (teleop) path, whose per-tick IK
        # solve is expensive; the replay runbook passes 250.0 explicitly.
        self.declare_parameter('control_rate', 90.0)
        self.declare_parameter('motion_mode', self._cfg.motion_mode)
        self.declare_parameter('simulation_mode', self._cfg.simulation_mode)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('read_only', False)
        self.declare_parameter('mode', 'pose')
        # 'G1_29' (the rig's robot, 7-DoF arms; DDS since aae4638) or
        # 'G1_23' (secondary, 5-DoF arms). Pose IK remains G1_23-only.
        self.declare_parameter('arm_type', self._cfg.arm_type)
        # Replay safety-chain / FSM thresholds (see replay_safety.py,
        # device_fsm.py; defaults are the spec's numbers).
        self.declare_parameter('target_staleness_s', 0.25)
        self.declare_parameter('divergence_rad', 0.35)
        self.declare_parameter('divergence_ticks', 10)
        self.declare_parameter('position_margin_rad', 0.0)
        self.declare_parameter('lowstate_staleness_s', 0.2)
        self.declare_parameter('engage_ramp_s', 2.0)
        self.declare_parameter('release_ramp_s', 2.0)
        self.declare_parameter('engage_fresh_ticks', 50)
        # Completion thresholds, tunable on-site without a code edit:
        # approach_done_err must exceed the arm's static tracking error
        # under gravity at the configured gains, or approach never
        # completes and the barrier times out.
        self.declare_parameter('engage_dq_max', 0.05)
        self.declare_parameter('approach_done_err', 0.05)
        self.declare_parameter('approach_done_dq', 0.05)
        self.declare_parameter('end_hold_dq', 0.05)
        self.declare_parameter('end_hold_confirm_s', 1.0)

        arm_type = str(self.get_parameter('arm_type').value)
        control_rate = float(self.get_parameter('control_rate').value)
        if motion_mode is None:
            motion_mode = bool(self.get_parameter('motion_mode').value)
        if simulation_mode is None:
            simulation_mode = bool(self.get_parameter('simulation_mode').value)
        if not dry_run:
            dry_run = bool(self.get_parameter('dry_run').value)
        if not read_only:
            read_only = bool(self.get_parameter('read_only').value)
        self._dry_run = dry_run
        self._read_only = read_only

        initial_mode = str(self.get_parameter('mode').value)
        if initial_mode not in VALID_MODES:
            self.get_logger().warning(
                f"Unknown mode '{initial_mode}', defaulting to 'pose'"
            )
            initial_mode = 'pose'
        if read_only:
            # Stage A: observation only. joint_replay's loop is inert in
            # read-only, and pose would solve IK toward motion.
            initial_mode = 'joint_replay'
        if initial_mode == 'pose' and arm_type != 'G1_23':
            raise ValueError(
                f"mode=pose requires arm_type=G1_23 (the pose IK is G1_23-only); "
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
            f"Initializing G1 controller (motion={motion_mode} sim={simulation_mode} "
            f"dry_run={dry_run} read_only={read_only})..."
        )
        self.controller = G1CartesianController(
            config=self._cfg,
            motion_mode=motion_mode,
            simulation_mode=simulation_mode,
            logger=logger_adapter,
            connect=not dry_run,
            arm_type=arm_type,
            read_only=read_only,
        )
        self._dof_side = len(self.controller.joint_names('left'))
        self._side_names = {'left': self.controller.joint_names('left'),
                            'right': self.controller.joint_names('right')}
        self._joint_names_all = (self._side_names['left']
                                 + self._side_names['right'])

        # ---- replay machinery (FSM + chains + buffers), built regardless
        # of mode so a runtime switch into joint_replay is ready.
        limits_all = ArmLimits.from_yaml(self._cfg.limits_file, self._joint_names_all)
        chains = {}
        for side in ('left', 'right'):
            side_names = self.controller.joint_names(side)
            side_limits = ArmLimits.from_yaml(self._cfg.limits_file, side_names)
            chains[side] = ReplaySafetyChain(
                side_limits,
                control_dt=1.0 / control_rate,
                position_margin=float(self.get_parameter('position_margin_rad').value),
                staleness_timeout_s=float(self.get_parameter('target_staleness_s').value),
                divergence_threshold_rad=float(self.get_parameter('divergence_rad').value),
                divergence_ticks=int(self.get_parameter('divergence_ticks').value),
            )
        self._fsm = ArmDeviceFSM(
            self._joint_names_all,
            chains,
            limits_all.deploy_velocity,
            FsmConfig(
                control_dt=1.0 / control_rate,
                engage_ramp_s=max(2.0, float(self.get_parameter('engage_ramp_s').value)),
                release_ramp_s=max(2.0, float(self.get_parameter('release_ramp_s').value)),
                engage_fresh_ticks=int(self.get_parameter('engage_fresh_ticks').value),
                engage_dq_max=float(self.get_parameter('engage_dq_max').value),
                lowstate_staleness_s=float(self.get_parameter('lowstate_staleness_s').value),
                approach_done_err=float(self.get_parameter('approach_done_err').value),
                approach_done_dq=float(self.get_parameter('approach_done_dq').value),
                end_hold_dq=float(self.get_parameter('end_hold_dq').value),
                end_hold_confirm_s=float(self.get_parameter('end_hold_confirm_s').value),
                sim=dry_run,
            ),
        )
        self._buffers = {'left': StreamBuffer(), 'right': StreamBuffer()}
        self._bad_target_warned = {'left': False, 'right': False}
        self._first_joint_target_received = False

        # ---- pose-mode state
        self.left_arm_pose = None
        self.right_arm_pose = None
        self.left_arm_direction = self._cfg.get_default_zsp_direction('left')
        self.right_arm_direction = self._cfg.get_default_zsp_direction('right')

        # ---- subscriptions
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
            JointState, '/left_arm/joint_targets',
            self._left_joint_targets_callback, TARGET_QOS
        )
        self.right_joint_targets_sub = self.create_subscription(
            JointState, '/right_arm/joint_targets',
            self._right_joint_targets_callback, TARGET_QOS
        )

        # ---- publications
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
        self.imu_pub = self.create_publisher(Imu, '/g1/imu', 10)
        self.status_pub = self.create_publisher(String, '/g1/status', 10)

        # ---- FSM transition services (spec_1 interfaces doc). Handlers
        # validate and return immediately; motion progresses in the loop.
        for name, handler in (
            ('engage', self._srv_engage), ('approach', self._srv_approach),
            ('track', self._srv_track), ('end_hold', self._srv_end_hold),
            ('park', self._srv_park), ('release', self._srv_release),
            ('fault', self._srv_fault), ('clear_fault', self._srv_clear_fault),
        ):
            self.create_service(Trigger, f'~/{name}', handler)

        # mode starts as 'pose' as a placeholder so _enter_mode's transition
        # logging/logic is well-defined even when initial_mode is 'pose'.
        self.mode = 'pose'
        self._enter_mode(initial_mode)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.joint_publish_timer = self.create_timer(0.01, self._publish_joint_state)
        self.status_timer = self.create_timer(STATUS_PERIOD_S, self._publish_status)

        self.get_logger().info(
            f"G1 World Output node initialized (mode={self.mode}, "
            f"read_only={read_only}, arm_type={arm_type})."
        )

        self._first_pose_received = False
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    # ==================== Mode management ====================

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _enter_mode(self, new_mode: str) -> None:
        """Switch active mode with bumpless transfer.

        Seeds the replay stream buffers from the arm's current measured
        position so joint_replay starts from where the arm actually is.
        'pose' runs the existing move_to_init() reset IK and takes arm_sdk
        authority (weight 1, legacy teleop behavior); joint_replay leaves
        the weight to the device FSM (0 until an operator engage).
        """
        old_mode = self.mode
        if self._read_only:
            self.mode = 'joint_replay'
            return
        if new_mode == 'pose':
            self.controller.move_to_init(wait=True, timeout=2.0)

        left_q, right_q = self.controller.get_current_joints()
        if left_q is not None:
            self._buffers['left'].seed(left_q)
        if right_q is not None:
            self._buffers['right'].seed(right_q)

        if new_mode == 'pose' and not self._dry_run:
            # Teleop parity: the pose path has always run at full arm_sdk
            # authority. (The replay path ramps via the FSM instead.)
            self.controller.set_weight(1.0)

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
                    reason="mode=pose requires arm_type=G1_23 (the pose IK is G1_23-only)",
                )
            if p.name == 'mode' and self.mode == 'joint_replay' \
                    and p.value != 'joint_replay' \
                    and (self._fsm.weight > 0.0
                         or self._fsm.state is not DeviceState.READY):
                return SetParametersResult(
                    successful=False,
                    reason=f"device FSM is {self._fsm.state.value} at weight "
                           f"{self._fsm.weight:.2f}; park and release before "
                           "leaving joint_replay",
                )
            if p.name == 'mode' and p.value == 'joint_replay' \
                    and self.mode != 'joint_replay' \
                    and self.controller.get_weight() > 0.0:
                # pose mode holds arm_sdk weight 1 outside the FSM. Entering
                # joint_replay would hand the FSM (weight attr 0) the write
                # path, and the next control tick would step the hardware
                # weight 1 -> 0 with no ramp: an instant authority handback
                # the >= 2 s release rule exists to prevent. Restarting the
                # node releases properly (shutdown ramps the weight down).
                return SetParametersResult(
                    successful=False,
                    reason=f"arm_sdk weight is "
                           f"{self.controller.get_weight():.2f} (pose/idle "
                           "authority); restart the node to enter "
                           "joint_replay -- an in-place switch would drop "
                           "the weight without a ramp",
                )
        for p in params:
            if p.name == 'mode' and p.value != self.mode:
                self._enter_mode(p.value)
        return SetParametersResult(successful=True)

    # ==================== FSM services ====================

    def _fsm_reply(self, ok_msg) -> Trigger.Response:
        ok, msg = ok_msg
        resp = Trigger.Response()
        resp.success = bool(ok)
        resp.message = json.dumps(
            {'result': msg, **self._fsm.status()}, sort_keys=True)
        return resp

    def _motion_service_allowed(self):
        if self._read_only:
            return False, 'read-only node: motion services are disabled (7A)'
        if self.mode != 'joint_replay':
            return False, f"mode is '{self.mode}', not joint_replay"
        return True, ''

    def _srv_engage(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_engage() if ok else (False, why))

    def _srv_approach(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_approach() if ok else (False, why))

    def _srv_track(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_track() if ok else (False, why))

    def _srv_end_hold(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_end_hold() if ok else (False, why))

    def _srv_park(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_park() if ok else (False, why))

    def _srv_release(self, request, response):
        ok, why = self._motion_service_allowed()
        return self._fsm_reply(self._fsm.request_release() if ok else (False, why))

    def _srv_fault(self, request, response):
        # Fault latching is allowed in every mode, read-only included: it
        # only ever freezes.
        return self._fsm_reply(self._fsm.fault('external fault trigger'))

    def _srv_clear_fault(self, request, response):
        # Allowed in every mode, read-only included (clearing only unlatches).
        return self._fsm_reply(self._fsm.request_clear_fault())

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
        if self.mode != 'joint_replay' or self._read_only:
            self.get_logger().warning(
                f"Ignoring {side} joint target: mode is '{self.mode}'"
                + (' (read-only)' if self._read_only else ''),
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
            self.get_logger().info("First joint-replay target received.")

        q = np.array([by_name[n] for n in names], dtype=float)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self._now()
        sl = slice(0, self._dof_side) if side == 'left' \
            else slice(self._dof_side, 2 * self._dof_side)
        current_cmd = None if self._fsm.cmd is None else self._fsm.cmd[sl]
        self._buffers[side].push(now, stamp_s, q, current_cmd=current_cmd)
        self._fsm.chains[side].mark_input(now)

    # ==================== Publishing ====================

    def _make_arm_joint_state(self, stamp, side: str, positions, frame_id: str,
                              velocities=None, efforts=None) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = self._side_names[side]
        msg.position = [float(j) for j in positions]
        if velocities is not None:
            msg.velocity = [float(v) for v in velocities]
        if efforts is not None:
            msg.effort = [float(e) for e in efforts]
        return msg

    def _publish_joint_state(self) -> None:
        try:
            q, dq, tau = self.controller.get_measured_state()
        except Exception:
            return
        if q is None:
            return
        d = self._dof_side
        stamp = self.get_clock().now().to_msg()
        self.left_state_pub.publish(self._make_arm_joint_state(
            stamp, 'left', q[:d], 'left_base_state', dq[:d], tau[:d]))
        self.right_state_pub.publish(self._make_arm_joint_state(
            stamp, 'right', q[d:], 'right_base_state', dq[d:], tau[d:]))

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

    def _publish_status(self) -> None:
        payload = {
            **self._fsm.status(),
            **self.controller.status_snapshot(),
            'mode': self.mode,
            'read_only': self._read_only,
            'dry_run': self._dry_run,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

        imu = self.controller.get_imu()
        if imu is not None:
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'g1_pelvis'
            w, x, y, z = imu['quaternion']  # Unitree order (w, x, y, z)
            m.orientation.w, m.orientation.x = float(w), float(x)
            m.orientation.y, m.orientation.z = float(y), float(z)
            gx, gy, gz = imu['gyroscope']
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
                float(gx), float(gy), float(gz)
            ax, ay, az = imu['accelerometer']
            m.linear_acceleration.x, m.linear_acceleration.y, \
                m.linear_acceleration.z = float(ax), float(ay), float(az)
            self.imu_pub.publish(m)

    # ==================== Control loop ====================

    def control_loop(self) -> None:
        if self._read_only:
            return  # observation only: no commands, ever (7A)
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
        q, dq, tau = self.controller.get_measured_state()
        age = self.controller.lowstate_age()
        stream = {side: self._buffers[side].interpolate(now)
                  for side in ('left', 'right')}
        out = self._fsm.tick(TickInputs(
            now=now, measured_q=q, measured_dq=dq, lowstate_age=age,
            stream=stream,
        ))
        for event in self._fsm.events:
            self.get_logger().info(f'FSM: {event}')
            self._write_detail_log(f'FSM: {event}')
        self._fsm.events.clear()

        d = self._dof_side
        if out.cmd is not None:
            self.controller.move_to_joints_direct(
                left_q=out.cmd[:d], right_q=out.cmd[d:])
            self._publish_joint_command(out.cmd[:d], out.cmd[d:])
        self.controller.set_weight(out.weight)

        self._debug_counter += 1
        if self._debug_counter >= self._debug_interval:
            self._debug_counter = 0
            line = (f"joint_replay: fsm={self._fsm.state.value} "
                    f"weight={out.weight:.2f}")
            if out.cmd is not None:
                line += f" | cmd=[{','.join(f'{j:.2f}' for j in out.cmd)}]"
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
        help='No DDS at all: the replay FSM runs in sim (measured := command). '
             'Pair with scripts/mujoco_visualizer.py in the teleop container.',
    )
    parser.add_argument(
        '--read-only', action='store_true',
        help='Stage A (TUITION 7A): subscribe lowstate and publish '
             'joint_states/imu/status; never write DDS, never touch the '
             'arm_sdk weight, refuse every motion service.',
    )
    args = parser.parse_args(cli_argv)
    if args.dry_run and args.read_only:
        parser.error('--dry-run (no DDS) and --read-only (DDS, no writer) conflict')

    rclpy.init(args=raw_argv)
    node = G1WorldOutputNode(
        motion_mode=True if args.motion_mode else None,
        simulation_mode=True if args.simulation_mode else None,
        dry_run=args.dry_run,
        read_only=args.read_only,
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
