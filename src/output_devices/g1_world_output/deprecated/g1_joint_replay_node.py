"""
DEPRECATED -- kept for reference only, not built/installed (moved out of the
g1_world_output/ package dir, no setup.py entry point).

Superseded by g1_world_output_node's 'joint_replay' mode: this file made a
second G1CartesianController/G1_23_ArmController, i.e. a second DDS writer,
mutually exclusive with live teleop only by convention (don't run both).
That's now enforced mechanically by a writer lockfile in robot_arm.py, and
there is exactly one node that can talk to DDS. Its interpolation logic and
dry_run handling were ported into g1_world_output_node.py; the wire format
also changed from a bare Float64MultiArray (no names) to a named
sensor_msgs/JointState matched against G1CartesianController.joint_names(),
published on /left_arm/joint_targets and /right_arm/joint_targets by
scripts/joint_replay_publisher.py.

Original docstring, for context on what this used to do:

ROS2 node that takes target joint angles (not wrist poses) and drives the
Unitree G1 arms directly, bypassing IK.

Data Flow:
  <trajectory source, e.g. g1_npz_arm_replay_publisher or IK teleop> ->
      /left_arm_target_joints  (std_msgs/Float64MultiArray)
      /right_arm_target_joints (std_msgs/Float64MultiArray)
          -> g1_joint_replay_node -> DDS LowCmd

Both topics carry a plain joint-angle vector (radians), one element per
joint, in the order of G1CartesianController.joint_names(side) -- no names
on the wire, so publisher and subscriber must agree on that order out of
band (both sides import it from the same place: robot_arm.py).

This is an alternative to g1_world_output_node's headset -> wrist-pose -> IK
path, not a replacement: pico_input and g1_world_output_node are untouched.
Run one or the other, not both -- they share the same DDS arm channel via
G1CartesianController/G1_23_ArmController.

An offline reference is typically
sampled far slower (e.g. 50 FPS) than the hardware control loop needs
("do not treat a 50 FPS reference as 50 Hz step commands" / "do not jump
directly to the next vector at every source frame"). This node linearly
interpolates between the two most recently received samples using the wall
-clock time each arrived, so the commanded position is continuous even when
the publisher is much slower than this node's control loop. Once no newer
sample has arrived past the latest one, it holds that target (matches the
bundle's "hold_last_target" end behavior).
"""

from __future__ import annotations

import argparse
import signal
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray

from g1_world_output.config_loader import G1Config
from g1_world_output.g1_controller import G1CartesianController
from g1_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge

LOG_DIR = Path.home() / ".g1_teleop_logs"

# Matches G1_23_ArmController.control_dt (robot_arm.py): the DDS write
# thread rate-limits toward whatever target this node last set, so this
# node's own timer needs to update that target at least as fast to avoid
# being the bottleneck between two interpolated samples.
DEFAULT_CONTROL_RATE_HZ = 250.0


class _SideBuffer:
    """Two most-recent (timestamp, q) samples for one arm, for interpolation."""

    def __init__(self):
        self.prev_t: Optional[float] = None
        self.prev_q: Optional[list] = None
        self.next_t: Optional[float] = None
        self.next_q: Optional[list] = None

    def push(self, t: float, q: list) -> None:
        if self.next_q is not None:
            self.prev_t, self.prev_q = self.next_t, self.next_q
        else:
            # First sample ever: nothing to interpolate from yet, so treat
            # it as already "arrived" rather than ramping from zero.
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


class G1JointReplayNode(Node):
    """ROS2 node: joint-angle vector topics -> interpolate -> DDS (no IK)."""

    def __init__(
        self,
        motion_mode: bool | None = None,
        simulation_mode: bool | None = None,
        dry_run: bool = False,
    ):
        super().__init__("g1_joint_replay_node")
        setup_ros2_logging_bridge(self.get_logger())

        self._cfg = G1Config.load()

        self.declare_parameter('control_rate', DEFAULT_CONTROL_RATE_HZ)
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
        self._detail_log_path = LOG_DIR / f'g1_joint_replay_{ts}.log'
        self._detail_log = None
        try:
            self._detail_log = open(self._detail_log_path, 'w', buffering=1)
            self._detail_log.write(f"# G1 Joint Replay detailed log - {ts}\n")
            self.get_logger().info(f'Detailed log: {self._detail_log_path}')
        except OSError as e:
            self.get_logger().error(f'Cannot create detailed log file: {e}')

        logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self.get_logger().info(
            f"Initializing G1 controller (motion={motion_mode} sim={simulation_mode} "
            f"dry_run={dry_run})..."
        )
        self.controller = G1CartesianController(
            config=self._cfg,
            motion_mode=motion_mode,
            simulation_mode=simulation_mode,
            logger=logger_adapter,
            connect=not dry_run,
        )
        self.controller.move_to_init(wait=True, timeout=2.0)

        self._buffers = {'left': _SideBuffer(), 'right': _SideBuffer()}
        self._bad_length_warned = {'left': False, 'right': False}

        self.left_joint_sub = self.create_subscription(
            Float64MultiArray, '/left_arm_target_joints', self._left_joint_callback, 10
        )
        self.right_joint_sub = self.create_subscription(
            Float64MultiArray, '/right_arm_target_joints', self._right_joint_callback, 10
        )

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f"G1 Joint Replay node initialized (no IK, control_rate={control_rate:.0f} Hz)."
        )
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /left_arm_target_joints  (Float64MultiArray, rad)")
        self.get_logger().info("  - /right_arm_target_joints (Float64MultiArray, rad)")

        self._first_target_received = False
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _expected_len(self, side: str) -> int:
        return len(self.controller.joint_names(side))

    def _handle_joint_msg(self, msg: Float64MultiArray, side: str) -> None:
        expected = self._expected_len(side)
        if len(msg.data) != expected:
            if not self._bad_length_warned[side]:
                self.get_logger().warning(
                    f"{side} arm target has {len(msg.data)} elements, expected "
                    f"{expected} ({self.controller.joint_names(side)}); dropping "
                    "until a correctly-sized message arrives"
                )
                self._bad_length_warned[side] = True
            return
        self._bad_length_warned[side] = False

        if not self._first_target_received:
            self._first_target_received = True
            self.get_logger().info("First joint target received, starting control...")

        self._buffers[side].push(self._now(), list(msg.data))

    def _left_joint_callback(self, msg: Float64MultiArray) -> None:
        self._handle_joint_msg(msg, 'left')

    def _right_joint_callback(self, msg: Float64MultiArray) -> None:
        self._handle_joint_msg(msg, 'right')

    def control_loop(self) -> None:
        now = self._now()
        left_q = self._buffers['left'].interpolate(now)
        right_q = self._buffers['right'].interpolate(now)
        if left_q is None and right_q is None:
            return

        l_success, r_success = self.controller.move_to_joints_direct(
            left_q=left_q, right_q=right_q,
        )

        self._debug_counter += 1
        if self._debug_counter >= self._debug_interval:
            self._debug_counter = 0
            self._log_control_status(l_success, r_success, left_q, right_q)

    def _write_detail_log(self, msg: str) -> None:
        if self._detail_log:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self._detail_log.write(f"{ts} {msg}\n")

    def _log_control_status(self, l_success, r_success, left_q, right_q) -> None:
        line = f"joint-direct: L_active={l_success} R_active={r_success}"
        if left_q:
            line += f" | L_q=[{','.join(f'{j:.2f}' for j in left_q)}]"
        if right_q:
            line += f" | R_q=[{','.join(f'{j:.2f}' for j in right_q)}]"
        self.get_logger().info(line)
        self._write_detail_log(line)


def main(argv: list[str] | None = None) -> None:
    program_name = sys.argv[0] if sys.argv else "g1_joint_replay_node"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(
        description="Unitree G1 arm control node (joint-angle vector topics, no IK)"
    )
    parser.add_argument('--motion-mode', action='store_true', help='Use rt/arm_sdk instead of rt/lowcmd')
    parser.add_argument('--simulation-mode', action='store_true', help='DDS sim domain')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Simulation mode: accept joint targets but never connect to DDS/hardware.',
    )
    args = parser.parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = G1JointReplayNode(
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
