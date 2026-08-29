"""Wuji-hand controller node (one process per hand, multi-core parallelism).

input_source is selected by wujihand_ik.yaml:

  'wuji_glove'       UDP, in-process via wuji_sdk (teleop)
  'keypoints_topic'  subscribes /{hand_name}/keypoints21 -- 63-float
                     MediaPipe (21,3) keypoints in meters; retargeted live
                     (legacy sim replay path, teleop-only)
  'q20_topic'        subscribes /{hand_name}/joint_targets -- named, stamped
                     q20 from the conditioned-clip replay publisher; NO
                     retargeter is constructed (spec_1 component 4). Runs
                     the hand device FSM (hold/approach/track/end_hold) with
                     Layer-1 clamps and feedback watchdogs, service-gated
                     (~/approach ~/track ~/end_hold ~/park ~/fault
                     ~/clear_fault), status on /{hand_name}/status.

Every mode publishes /{hand_name}/joint_commands (unnamed, position-only,
exactly 20 elements -- the only command shape that is safe against the
driver's named-path zero-fill), so this node keeps sole ownership of the
topic and a replay/teleop double-writer is structurally impossible.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args

from wujihand_output import WujiHandController
from .common import (
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# joint_targets stream QoS, pinned RELIABLE end to end (plan amendment A5).
# get_default_qos() is BEST_EFFORT depth 1 and must NOT be used here: it
# would silently drop frames into the interpolator.
TARGET_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Default control-loop rate (Hz). Override with the `control_rate` ROS2 param.
# 120Hz matches the upper bound of wuji_glove skeleton frames; higher
# adds no new input. The wujihand C++ driver on the host runs 1000Hz down to
# the firmware, so publishing faster from the controller is pointless.
DEFAULT_CONTROL_RATE_HZ = 120.0

# keypoints_topic branch: a stream gap longer than this is treated as a
# clip boundary and resets the retargeter (TUITION 3.1 requires a reset per
# clip; the legacy live-keypoint path has no explicit clip-start signal, so
# the gap is a proxy). Deliberately its own constant: _RECV_TIMEOUT_SEC
# below is the glove-UDP disconnect watchdog and may be retuned for
# reconnect reasons that have nothing to do with clip boundaries.
_KEYPOINT_CLIP_GAP_RESET_SEC = 2.0

# wuji_glove reconnect behavior: if the main loop receives no skeleton frame
# for _RECV_TIMEOUT_SEC seconds in a row, treat the underlying connection as
# lost (unplugged glove, network drop, power loss, etc.) and call
# manager.connect() to reconnect.
# Default ConnectOptions: timeout_ms=1000, retry_count=3 — when offline, a
# single reconnect blocks ~3s worst-case (i.e. the main loop misses ~3s of
# joint_commands; output resumes naturally once the link is back).
_RECV_TIMEOUT_SEC = 2.0


def _extract_wuji_glove_keypoints(skeleton) -> Optional[np.ndarray]:
    """Convert a wuji_sdk HandSkeleton to MediaPipe-style (21, 3) float32.

    Returns None if the skeleton does not have exactly 21 joints
    (caller should skip the frame).
    """
    joints = skeleton.joints
    if len(joints) != 21:
        return None
    kp = np.array(
        [j.pose.position for j in joints],
        dtype=np.float32,
    )
    if kp.shape != (21, 3):
        return None
    return kp


class WujiHandControllerNode(Node):
    """Per-hand wujihand controller node.

    Dispatches at __init__ on cfg['input_source']:
      - 'wuji_glove':      connect via wuji_sdk (UDP), subscribe hand_skeleton
      - 'keypoints_topic': subscribe /{hand_name}/keypoints21
                           (std_msgs/Float64MultiArray, 63 floats = MediaPipe
                           (21,3) in meters; retargeted here live --
                           legacy sim replay path, teleop-only)
      - 'q20_topic':       subscribe /{hand_name}/joint_targets (named,
                           stamped q20 from the conditioned-clip publisher);
                           no retargeter; hand device FSM + Layer-1 clamps
                           (spec_1 component 4)
    """

    def __init__(self, side: str, hand_name: str, cfg: dict,
                 glove_config_path: Optional[str] = None,
                 retarget_config_dir: Optional[str] = None):
        super().__init__(f"wujihand_controller_{side}")

        self._side = side
        self._hand_name = hand_name
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._input_source = cfg.get("input_source", "wuji_glove")
        self._cfg = cfg
        # keypoints_topic state: written by the subscription callback,
        # consumed by the timer-driven control loop on another thread.
        self._latest_keypoints: Optional[np.ndarray] = None
        self._keypoints_lock = threading.Lock()
        self._last_keypoints_time: float = 0.0
        # wuji_glove connection state
        self._sdk_device = None
        self._sdk_sub = None
        # Stash connect params for reconnects (populated by _setup_wuji_glove).
        self._glove_sn: Optional[str] = None
        self._glove_device_name: Optional[str] = None
        self._glove_config_path: Optional[str] = None
        # recv watchdog: main loop treats (now - _last_recv_time) > _RECV_TIMEOUT_SEC as a disconnect.
        self._last_recv_time: float = 0.0
        self._reconnect_log_counter: int = 0

        # control_rate is resolved ONCE, before the input dispatch, and this
        # single value feeds both the timer and the q20 FSM's control_dt.
        # Deriving them separately would let a launch override change the
        # tick rate but not the per-tick rate-limit budget, silently scaling
        # the Layer-1 velocity cap. Config file may override the default
        # (q20 replay configs set 200.0 per TUITION section 5; teleop 120).
        default_rate = float(cfg.get('control_rate', DEFAULT_CONTROL_RATE_HZ))
        self.declare_parameter('control_rate', default_rate)
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        if self._control_rate_hz <= 0.0:
            raise ValueError(
                f"control_rate must be > 0, got {self._control_rate_hz}")

        # Controller (drives retargeter + wujihand driver). The q20_topic
        # branch never constructs a Retargeter (enable_ik=False): replay
        # neither pays for nor depends on NLopt (spec_1 component 4).
        self.get_logger().info(
            f"Initializing {side}-hand controller (input_source={self._input_source})..."
        )
        self.controller = WujiHandController(
            side=side,
            hand_name=hand_name,
            input_source=self._input_source,
            node=self,
            logger=self._logger_adapter,
            retarget_config_dir=retarget_config_dir,
            enable_ik=(self._input_source != "q20_topic"),
        )
        self.get_logger().info("Controller initialized")

        # Dispatch on input_source
        if self._input_source == "wuji_glove":
            self._setup_wuji_glove(glove_config_path)
        elif self._input_source == "keypoints_topic":
            self._setup_keypoints_topic(hand_name)
        elif self._input_source == "q20_topic":
            self._setup_q20_topic(hand_name)
        else:
            raise ValueError(
                f"unknown input_source: {self._input_source!r} "
                f"(expected 'wuji_glove', 'keypoints_topic', or 'q20_topic')"
            )

        self.create_timer(1.0 / self._control_rate_hz, self._teleop_loop)

        self.get_logger().info(
            f"Ready: side={side}, source={self._input_source}, "
            f"rate={self._control_rate_hz:.1f}Hz -> /{hand_name}/joint_commands"
        )

    # ==================== input_source=wuji_glove ====================

    def _setup_wuji_glove(self, glove_config_path: Optional[str]) -> None:
        """Load the glove config and try the first connection. A first-connect
        failure does NOT raise (the main loop keeps retrying); configuration
        errors such as a hand_side mismatch still fail-fast.
        """
        # Resolve glove config path: prefer caller-supplied (from launch), fall
        # back to ament index.
        if glove_config_path is None:
            glove_config_path = get_package_config_path(
                "wuji_glove", "wuji_glove.yaml"
            )
        glove_cfg = load_yaml_config(glove_config_path)[f"{self._side}_glove"]
        self._glove_sn = glove_cfg["serial_number"]
        self._glove_device_name = glove_cfg.get(
            "device_name", f"{self._side}_glove"
        )
        self._glove_config_path = glove_config_path

        # First connect: do not raise on failure (e.g. glove not powered at
        # startup); the main loop keeps retrying.
        self._connect_glove()

    def _connect_glove(self) -> bool:
        """Connect or reconnect a Wuji Glove.

        Returns True on success. A transient failure (device offline, network
        drop) returns False; the main loop tries again next tick.
        An SN / hand_side mismatch still raises RuntimeError — that is a
        configuration error and should not be papered over by retries.
        """
        from wuji_sdk import SdkManager, ConnectOptions  # lazy: only loaded on wuji_glove path

        # Release any prior handles (harmless if already None).
        self._sdk_sub = None
        self._sdk_device = None

        try:
            manager = SdkManager.instance()
            # enable_bridge=False keeps the glove off the zenoh device-bridge.
            # The default (True) would call declare_node_token + start_bridge_for
            # after a successful direct connect, advertising this glove on the
            # LAN so any peer could discover and take it over. Direct-connect
            # failure also falls back to zenoh discovery — both paths leak.
            opts = ConnectOptions(enable_bridge=False)
            device = manager.connect(
                sn=self._glove_sn,
                device_name=self._glove_device_name,
                options=opts,
            )
        except Exception as e:
            # Transient failure: throttle logs (one per 10 attempts) so we
            # do not flood the console.
            self._reconnect_log_counter += 1
            if self._reconnect_log_counter == 1 or self._reconnect_log_counter % 10 == 0:
                self.get_logger().warn(
                    f"wuji_sdk connect attempt #{self._reconnect_log_counter} failed: {e}"
                )
            return False

        # SN / side mismatch is a config error — do not paper over it with retries.
        actual_side = device.hand_side().get().lower()
        if actual_side != self._side:
            raise RuntimeError(
                f"{self._side}_glove SN={self._glove_sn} reports hand_side={actual_side}; "
                f"swap left_glove/right_glove SNs in wuji_glove.yaml."
            )

        self._sdk_device = device
        self._sdk_sub = device.hand_skeleton().subscribe()
        self._last_recv_time = time.monotonic()
        was_retry = self._reconnect_log_counter > 0
        self._reconnect_log_counter = 0
        self.get_logger().info(
            f"wuji_sdk {'re' if was_retry else ''}connected: SN={self._glove_sn} "
            f"side={actual_side} device_name={self._glove_device_name} "
            f"(config={self._glove_config_path})"
        )
        return True

    def _teleop_loop_wuji_glove(self) -> None:
        now = time.monotonic()

        # No active connection -> (re)connect.
        if self._sdk_sub is None:
            self._connect_glove()
            return

        skeleton = self._sdk_sub.recv()
        if skeleton is None:
            # No frame this tick — has the link been quiet too long?
            if now - self._last_recv_time > _RECV_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"wuji_sdk: no skeleton frame for {now - self._last_recv_time:.1f}s, "
                    f"reconnecting..."
                )
                self._connect_glove()  # release old sub + reconnect
            return

        # Frame received — refresh the watchdog timestamp.
        self._last_recv_time = now

        # Drain queue: keep only the latest frame to prevent lag buildup
        # when the SDK pushes faster than 120Hz.
        while True:
            newer = self._sdk_sub.recv()
            if newer is None:
                break
            skeleton = newer

        kp = _extract_wuji_glove_keypoints(skeleton)
        if kp is None:
            return
        self.controller.set_keypoints(kp)

    # ==================== input_source=keypoints_topic ====================

    def _setup_keypoints_topic(self, hand_name: str) -> None:
        from std_msgs.msg import Float64MultiArray  # lazy: only on this path
        qos = get_default_qos()
        topic = f"/{hand_name}/keypoints21"
        self.create_subscription(Float64MultiArray, topic, self._keypoints_callback, qos)
        self.get_logger().info(
            f"Subscribed to {topic} (63-float MediaPipe (21,3) keypoints, meters)"
        )

    def _keypoints_callback(self, msg) -> None:
        if len(msg.data) != 63:
            self.get_logger().warn(
                f"keypoints21 message has {len(msg.data)} floats, expected 63; dropping",
                throttle_duration_sec=5.0,
            )
            return
        kp = np.array(msg.data, dtype=np.float32).reshape(21, 3)
        with self._keypoints_lock:
            self._latest_keypoints = kp

    def _teleop_loop_topic(self) -> None:
        # Same consume-once pattern as manus: latest frame wins, retarget in
        # the timer thread via the production controller path.
        with self._keypoints_lock:
            kp = self._latest_keypoints
            self._latest_keypoints = None
        if kp is None:
            return
        # Reset the retargeter's filter/warm-start state when the stream
        # resumes after a gap (clip-boundary proxy): TUITION 3.1 requires a
        # reset per clip, and this branch previously leaked state across
        # clips (spec_1 known defect).
        now = time.monotonic()
        if (self._last_keypoints_time > 0.0
                and now - self._last_keypoints_time > _KEYPOINT_CLIP_GAP_RESET_SEC
                and self.controller.retargeter is not None):
            self.controller.retargeter.reset()
            self.get_logger().info(
                f'keypoint stream resumed after '
                f'{now - self._last_keypoints_time:.1f}s gap; retargeter reset '
                f'(per-clip warm-start/filter state cleared)'
            )
        self._last_keypoints_time = now
        self.controller.set_keypoints(kp)

    # ==================== input_source=q20_topic ====================

    def _setup_q20_topic(self, hand_name: str) -> None:
        """Conditioned-clip replay branch (spec_1 component 4)."""
        from sensor_msgs.msg import JointState  # lazy: only on this path
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from wujihand_output.hand_fsm import (HandDeviceFSM, HandFsmConfig,
                                              HandTickInputs)
        from wujihand_output.hand_safety import HandLimits
        from wujihand_output.stream_buffer import StreamBuffer

        limits_path = self._cfg.get('hand_limits_file') or get_package_config_path(
            'wujihand_output', 'hand_limits.yaml')
        if limits_path is None or not Path(limits_path).exists():
            raise FileNotFoundError(
                'hand_limits.yaml not found (wujihand_output config); the '
                'q20 branch cannot clamp without it'
            )
        self._hand_limits = HandLimits.from_yaml(limits_path)
        self._expected_target_names = self._hand_limits.side_names(self._side)

        require_feedback = bool(self._cfg.get('require_feedback', True))
        neutral = self._cfg.get('neutral_pose')
        # Layer-1 thresholds are config-file-tunable like the arm node's ROS
        # parameters, so Stage A retuning is a config edit on both devices,
        # not a source edit here. Defaults live in HandFsmConfig.
        fsm_kw = {
            key: float(self._cfg[key])
            for key in ('target_staleness_s', 'state_staleness_s',
                        'diagnostics_staleness_s', 'approach_done_err',
                        'temperature_trip_c', 'effort_guard_scale')
            if key in self._cfg
        }
        if 'effort_guard_ticks' in self._cfg:
            fsm_kw['effort_guard_ticks'] = int(self._cfg['effort_guard_ticks'])
        self._fsm = HandDeviceFSM(
            self._hand_limits,
            HandFsmConfig(
                control_dt=1.0 / self._control_rate_hz,
                require_feedback=require_feedback,
                neutral_pose=None if neutral is None else np.asarray(neutral, float),
                **fsm_kw,
            ),
        )
        self._buffer = StreamBuffer()
        self._HandTickInputs = HandTickInputs  # bound once, hot-loop use
        self._diag: Optional[dict] = None
        self._diag_time: float = 0.0
        self._name_mismatch_logged = False

        self.create_subscription(
            JointState, f'/{hand_name}/joint_targets',
            self._q20_target_callback, TARGET_QOS,
        )

        # hand_diagnostics feeds the fault inputs and the runtime effort
        # limits (amps). The message package lives in the wujihandros2
        # submodule; a hardware profile cannot run without it.
        try:
            from wujihand_msgs.msg import HandDiagnostics
            self.create_subscription(
                HandDiagnostics, f'/{hand_name}/hand_diagnostics',
                self._diagnostics_callback, 10,
            )
        except ImportError:
            if require_feedback:
                raise RuntimeError(
                    'wujihand_msgs not available but require_feedback=true; '
                    'build wujihandros2 or set require_feedback: false (sim only)'
                )
            self.get_logger().warning(
                'wujihand_msgs unavailable; diagnostics watchdog disabled (sim)'
            )

        self._status_pub = self.create_publisher(String, f'/{hand_name}/status', 10)
        self.create_timer(0.1, self._publish_q20_status)

        for name, handler in (
            ('approach', self._srv_approach), ('track', self._srv_track),
            ('end_hold', self._srv_end_hold), ('park', self._srv_park),
            ('release', self._srv_release),
            ('fault', self._srv_fault), ('clear_fault', self._srv_clear_fault),
        ):
            self.create_service(Trigger, f'~/{name}', handler)

        self._write_joint_mapping()
        self.get_logger().info(
            f'q20_topic: subscribed /{hand_name}/joint_targets (RELIABLE), '
            f'require_feedback={require_feedback}, limits={limits_path}'
        )

    def _q20_target_callback(self, msg) -> None:
        # Preflight name contract: the stream's names must equal the
        # side-prefixed URDF declaration order EXACTLY (same 20 strings,
        # same order). This is a software-to-software check between the
        # publisher and this node; the driver never sees names (we publish
        # unnamed), and driver-name comparison is impossible by design
        # (finger{i}_joint{j} vs anatomical names share no strings).
        names = list(msg.name)
        if names != self._expected_target_names:
            if not self._name_mismatch_logged:
                self.get_logger().error(
                    f'joint_targets names do not match the expected device '
                    f'order; dropping. expected={self._expected_target_names} '
                    f'got={names}'
                )
                self._name_mismatch_logged = True
            return
        self._name_mismatch_logged = False
        if len(msg.position) != len(names):
            return
        q = np.asarray(msg.position, dtype=float)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        self._buffer.push(now, stamp_s, q, current_cmd=self._fsm.cmd)
        self._fsm.mark_target_input(now)

    def _diagnostics_callback(self, msg) -> None:
        self._diag = {
            'handedness': str(msg.handedness),
            'error_codes': list(msg.error_codes),
            'enabled': list(msg.enabled),
            'joint_temperatures': list(msg.joint_temperatures),
            'effort_limits': list(msg.effort_limits),
        }
        first = self._diag_time == 0.0
        self._diag_time = time.monotonic()
        if first:
            reported = self._diag['handedness'].lower()
            if reported != self._side:
                # Firmware handedness vs topic namespace: a mismatch means
                # the drivers are namespaced onto the wrong sides.
                self._fsm.fault(
                    f'driver reports handedness={reported} under the '
                    f'{self._side} namespace; sides are mis-assigned'
                )
                self.get_logger().error(self._fsm.fault_info['reason'])
            self._write_joint_mapping(handedness=reported)

    def _teleop_loop_q20(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        hand = self.controller.hand
        if hand is not None:
            measured, effort, state_age = hand.state_snapshot()
        else:
            measured, effort, state_age = None, None, None
        diag_age = (None if self._diag_time == 0.0
                    else time.monotonic() - self._diag_time)
        out = self._fsm.tick(self._HandTickInputs(
            now=now,
            measured_q=None if measured is None else np.asarray(measured, float),
            measured_effort=None if effort is None else np.asarray(effort, float),
            state_age=state_age,
            diagnostics=self._diag,
            diagnostics_age=diag_age,
            stream=self._buffer.interpolate(now),
        ))
        for event in self._fsm.events:
            self.get_logger().info(f'FSM: {event}')
        self._fsm.events.clear()
        if out.cmd is not None:
            # Unnamed full-20 position-only arrays, every cycle. Exactly 20:
            # a shorter unnamed array zero-fills the tail in the driver.
            assert out.cmd.shape == (20,), out.cmd.shape
            self.controller.set_joint_positions(out.cmd)

    def _publish_q20_status(self) -> None:
        from std_msgs.msg import String
        msg = String()
        payload = self._fsm.status()
        payload['handedness'] = (self._diag or {}).get('handedness')
        payload['joints_online'] = (
            int(sum((self._diag or {}).get('enabled', []))) if self._diag else None
        )
        temps = (self._diag or {}).get('joint_temperatures') or []
        payload['max_joint_temp_c'] = max(temps) if temps else None
        msg.data = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(msg)

    def _write_joint_mapping(self, handedness: Optional[str] = None) -> None:
        """joint_mapping.json: the name tables side by side, so the intended
        flat-index correspondence is written down rather than assumed
        (spec_1 component 4). The physical half -- that driver finger1
        really is the thumb -- is a Stage A check, not software."""
        out_dir = Path(self._cfg.get('mapping_out_dir', '~/wuji_runs')).expanduser()
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f'joint_mapping_{self._side}.json'
            payload = {
                'side': self._side,
                'topic_namespace': self._hand_name,
                'expected_target_names': self._expected_target_names,
                'driver_names': self._hand_limits.driver_names,
                'correspondence': (
                    'flat index i names the same physical joint in both '
                    'columns: 5 fingers x 4 joints [flex, abd, pip/mcp, '
                    'dip/ip], thumb to pinky; commands are published '
                    'unnamed, exactly 20 elements, driver reads element i '
                    'as finger i/4, joint i%4'
                ),
                'driver_reported_handedness': handedness,
                'handedness_matches_namespace': (
                    None if handedness is None else handedness == self._side
                ),
                'physical_confirmation': (
                    'pending Stage A: command one distinguishable joint on '
                    'the finger believed to be the thumb and watch which '
                    'finger moves (the only check that catches an '
                    'off-by-one across all five fingers)'
                ),
                'limits_file': str(self._hand_limits.source_path),
            }
            path.write_text(json.dumps(payload, indent=1, sort_keys=True) + '\n')
        except OSError as exc:
            self.get_logger().warning(f'could not write joint_mapping: {exc}')

    # ---- q20 FSM services (validate + return immediately; progress in loop)

    def _fsm_reply(self, ok_msg):
        from std_srvs.srv import Trigger
        ok, message = ok_msg
        resp = Trigger.Response()
        resp.success = bool(ok)
        resp.message = json.dumps({'result': message, **self._fsm.status()},
                                  sort_keys=True)
        return resp

    def _srv_approach(self, request, response):
        return self._fsm_reply(self._fsm.request_approach())

    def _srv_track(self, request, response):
        return self._fsm_reply(self._fsm.request_track())

    def _srv_end_hold(self, request, response):
        return self._fsm_reply(self._fsm.request_end_hold())

    def _srv_park(self, request, response):
        return self._fsm_reply(self._fsm.request_park())

    def _srv_release(self, request, response):
        return self._fsm_reply(self._fsm.request_release())

    def _srv_fault(self, request, response):
        return self._fsm_reply(self._fsm.fault('external fault trigger'))

    def _srv_clear_fault(self, request, response):
        return self._fsm_reply(self._fsm.request_clear_fault())

    # ==================== shared ====================

    def _teleop_loop(self) -> None:
        if self._input_source == "keypoints_topic":
            self._teleop_loop_topic()
        elif self._input_source == "q20_topic":
            self._teleop_loop_q20()
        else:
            self._teleop_loop_wuji_glove()

    # ==================== lifecycle ====================

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        if self._sdk_sub is not None:
            self._sdk_sub = None  # SDK has no explicit unsubscribe; release ref
        if self._sdk_device is not None:
            self._sdk_device = None
        self.controller.disable_and_release()
        self.get_logger().info("Exited cleanly")


# -------------------- Entry point --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wuji-hand controller (per-hand)")
    parser.add_argument("--side", required=True, choices=["left", "right"],
                        help="which hand to drive")
    parser.add_argument("--hand-name", help="wujihandros2 driver namespace")
    parser.add_argument("-c", "--config", help="wujihand_ik.yaml path")
    parser.add_argument(
        "--glove-config",
        help="wuji_glove.yaml path (used when input_source=wuji_glove; "
             "falls back to the wuji_glove package default via ament_index)",
    )
    parser.add_argument(
        "--retarget-config-dir",
        help="Directory containing retarget yaml (overrides the wujihand_output "
             "package's default config/). Lookup order: "
             "retarget_{input_source}_{side}.yaml -> retarget_{input_source}.yaml. "
             "Use for cross-host deployments where launch passes an explicit "
             "override directory so retarget params follow the deploy host "
             "rather than the in-package default config/.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "wujihand_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    side = args.side
    default_hand_name = "left_hand" if side == "left" else "right_hand"
    hand_name = args.hand_name or default_hand_name

    config_path = args.config or get_package_config_path(
        "wujihand_output", "wujihand_ik.yaml"
    )
    cfg = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    node = WujiHandControllerNode(
        side=side, hand_name=hand_name, cfg=cfg,
        glove_config_path=args.glove_config,
        retarget_config_dir=args.retarget_config_dir,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
