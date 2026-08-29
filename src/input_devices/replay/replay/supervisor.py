#!/usr/bin/env python3
"""Replay supervisor (spec_1 component 6): run state machine, load gates,
alignment barrier, Layer-3 monitors, fault latch, logging.

One node, stock interfaces only (std_msgs/String JSON + std_srvs/Trigger).
All decision logic lives in replay/run_gates.py (ROS-free, unit-tested);
this file subscribes status topics, executes decided actions as ASYNC
service calls (never a synchronous call inside a callback), spawns the
rosbag2 recorder, and writes the run directory.

Run state machine (spec_1):

    IDLE -> ARMED -> RUNNING -> IDLE
                        |
       any powered state -> FAULT (latched)

Surface (docs/spec/spec_1_interfaces.md):
  param    load_request   JSON: clip, speed_scale, arms, hands,
                          override_gt_gate, override_first_clip, operator
  service  ~/load ~/arm ~/start ~/stop ~/park ~/release ~/clear_fault
  topics   /run/status /run/events /run/fault (String JSON)

The devices stay safe with this node dead (Layer 1 holds); everything here
is gates, orchestration, and evidence.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from replay.clip_artifact import ArtifactError, load_artifact
from replay.hand_pipeline import sha256_file
from replay.run_gates import (
    ArmSeqState,
    ArmSequence,
    GateError,
    Layer3Monitor,
    MonitorConfig,
    RunState,
    SupervisorLoadRequest,
    check_load_gates,
    fault_actions,
    load_run_history,
    release_gate_problems,
    start_actions,
)

TICK_PERIOD_S = 0.1

# Service names per target (docs/spec/spec_1_interfaces.md). Node names are
# fixed by the launch profile: replay_publisher, g1_world_output,
# wujihand_controller_{left,right}.
TARGET_NODES = {
    'replay': '/replay_publisher',
    'g1': '/g1_world_output',
    'left_hand': '/wujihand_controller_left',
    'right_hand': '/wujihand_controller_right',
}

# Section 10 bag allowlist, verbatim from spec_1.
BAG_TOPICS = [
    '/left_arm/joint_targets', '/right_arm/joint_targets',
    '/left_arm/joint_commands', '/right_arm/joint_commands',
    '/left_arm/joint_states', '/right_arm/joint_states',
    '/left_hand/joint_commands', '/right_hand/joint_commands',
    '/left_hand/joint_states', '/right_hand/joint_states',
    '/left_hand/hand_diagnostics', '/right_hand/hand_diagnostics',
    '/g1/imu', '/g1/status', '/replay/status',
    '/run/status', '/run/events', '/run/fault',
    '/rosout',
]


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('replay_supervisor')
        self.declare_parameter('load_request', '')
        self.declare_parameter('runs_dir', str(Path.home() / 'wuji_runs'))
        self.declare_parameter('arm_type', 'G1_29')
        self.declare_parameter('record_bag', True)
        self.declare_parameter('barrier_timeout_s', 30.0)
        self.declare_parameter('liveness_timeout_s', 1.0)
        self.declare_parameter('temp_warn_c', 50.0)
        self.declare_parameter('temp_trip_c', 60.0)
        # False for sim profiles (no wujihand driver): hand_diagnostics
        # liveness is not demanded by Layer 3.
        self.declare_parameter('expect_hand_diagnostics', True)
        # Sim-only fault-injection drills (mirrors the publisher's
        # --force-sim): load-gate problems are logged and bypassed instead
        # of refused. NEVER with hardware attached.
        self.declare_parameter('force_sim', False)

        self.run_state = RunState.IDLE
        self.request: Optional[SupervisorLoadRequest] = None
        self.clip_meta: Optional[dict] = None
        self.scope: dict = {'arms': [], 'hands': []}
        self.monitor: Optional[Layer3Monitor] = None
        self.arm_seq: Optional[ArmSequence] = None
        self.fault_reason: Optional[str] = None
        self._loaded = False
        self._release_pending = False

        self._run_dir: Optional[Path] = None
        self._events_file = None
        self._bag_proc: Optional[subprocess.Popen] = None

        # Status caches: name -> {'data': dict|list, 'mono': float}
        self._cache = {}

        self.create_subscription(String, '/replay/status',
                                 lambda m: self._cache_json('replay', m), 10)
        self.create_subscription(String, '/g1/status',
                                 lambda m: self._cache_json('g1', m), 10)
        for side in ('left', 'right'):
            self.create_subscription(
                String, f'/{side}_hand/status',
                lambda m, s=side: self._cache_json(f'{s}_hand', m), 10)
            self.create_subscription(
                JointState, f'/{side}_hand/joint_states',
                lambda m, s=side: self._cache_effort(f'{s}_hand_effort', m),
                QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=10))
        try:
            from wujihand_msgs.msg import HandDiagnostics
            for side in ('left', 'right'):
                self.create_subscription(
                    HandDiagnostics, f'/{side}_hand/hand_diagnostics',
                    lambda m, s=side: self._cache_diag(f'{s}_hand_diag', m), 10)
        except ImportError:
            self.get_logger().warning(
                'wujihand_msgs unavailable; hand-diagnostics Layer-3 '
                'detectors are inactive (sim only)')

        self.status_pub = self.create_publisher(String, '/run/status', 10)
        self.events_pub = self.create_publisher(String, '/run/events', 10)
        self.fault_pub = self.create_publisher(String, '/run/fault', 10)

        for name, handler in (
            ('load', self._srv_load), ('arm', self._srv_arm),
            ('start', self._srv_start), ('stop', self._srv_stop),
            ('park', self._srv_park), ('release', self._srv_release),
            ('clear_fault', self._srv_clear_fault),
        ):
            self.create_service(Trigger, f'~/{name}', handler)

        # Async Trigger clients, created lazily per (target, service).
        # NOT named _clients: that attribute is rclpy.Node's own client
        # list, and shadowing it breaks create_client.
        self._trigger_clients = {}
        self._param_client = self.create_client(
            SetParameters, f"{TARGET_NODES['replay']}/set_parameters")

        self.create_timer(TICK_PERIOD_S, self._tick)
        self.get_logger().info('supervisor ready (IDLE)')

    # ------------------------------------------------------------- caches

    def _cache_json(self, key: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._cache[key] = {'data': data, 'mono': time.monotonic()}

    def _cache_effort(self, key: str, msg: JointState) -> None:
        if msg.effort:
            self._cache[key] = {'data': list(msg.effort), 'mono': time.monotonic()}

    def _cache_diag(self, key: str, msg) -> None:
        self._cache[key] = {'data': {
            'handedness': str(msg.handedness),
            'error_codes': list(msg.error_codes),
            'enabled': list(msg.enabled),
            'joint_temperatures': list(msg.joint_temperatures),
            'effort_limits': list(msg.effort_limits),
        }, 'mono': time.monotonic()}

    def _snapshots(self) -> dict:
        now = time.monotonic()
        out = {}
        for key, entry in self._cache.items():
            out[key] = {'age': now - entry['mono'], 'data': entry['data']}
        return out

    # ------------------------------------------------------------- events

    def _event(self, severity: str, event: str, detail=None) -> None:
        record = {
            't_wall': datetime.now(timezone.utc).isoformat(),
            't_mono': time.monotonic(),
            'severity': severity,
            'event': event,
            'detail': detail,
        }
        line = json.dumps(record, sort_keys=True)
        log = self.get_logger()
        text = f'{severity}: {event}' + (f' -- {detail}' if detail else '')
        # Two call sites on purpose: rclpy pins a fixed severity per call
        # site, so one shared line raises "Logger severity cannot be
        # changed between calls" the first time severity flips.
        if severity in ('fault', 'error'):
            log.error(text)
        else:
            log.info(text)
        msg = String()
        msg.data = line
        self.events_pub.publish(msg)
        if self._events_file is not None:
            try:
                self._events_file.write(line + '\n')
            except OSError:
                pass

    # ------------------------------------------------------ service calls

    def _call(self, target: str, service: str) -> None:
        """Fire an async Trigger call; result arrives as an event."""
        key = (target, service)
        if key not in self._trigger_clients:
            self._trigger_clients[key] = self.create_client(
                Trigger, f'{TARGET_NODES[target]}/{service}')
        client = self._trigger_clients[key]
        if not client.service_is_ready():
            self._event('warn', f'{target}/{service} service not ready')
            return
        future = client.call_async(Trigger.Request())

        def done(fut, target=target, service=service):
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self._event('error', f'{target}/{service} call failed', str(exc))
                return
            sev = 'info' if resp.success else 'error'
            self._event(sev, f'{target}/{service} -> {resp.success}',
                        resp.message[:400])

        future.add_done_callback(done)

    def _execute(self, actions) -> None:
        for target, service in actions:
            self._call(target, service)

    # ----------------------------------------------------------- services

    def _reply(self, ok: bool, message: str) -> Trigger.Response:
        resp = Trigger.Response()
        resp.success = ok
        resp.message = json.dumps({'result': message, 'run_state': self.run_state.value},
                                  sort_keys=True)
        return resp

    def _srv_load(self, request, response) -> Trigger.Response:
        if self.run_state is RunState.FAULT:
            return self._reply(False, 'FAULT latched: clear_fault before loading (9)')
        if self.run_state is RunState.RUNNING:
            return self._reply(False, 'RUNNING: stop or finish before loading')
        raw = str(self.get_parameter('load_request').value)
        runs_dir = Path(str(self.get_parameter('runs_dir').value)).expanduser()
        force_sim = bool(self.get_parameter('force_sim').value)
        try:
            req = SupervisorLoadRequest.from_json(raw)
            clip = load_artifact(req.clip)
            bypassed = check_load_gates(
                req, clip.meta, str(self.get_parameter('arm_type').value),
                load_run_history(runs_dir), force_sim=force_sim,
            )
        except (GateError, ArtifactError, OSError) as exc:
            self._event('error', 'load refused', str(exc))
            return self._reply(False, str(exc))
        if bypassed:
            self._event('warn', 'force_sim: load gates BYPASSED (sim only, '
                                'never with hardware attached)', bypassed)

        self.request = req
        self.clip_meta = clip.meta
        self.scope = {'arms': sorted(req.arms), 'hands': sorted(req.hands)}
        self._loaded = False
        self._open_run_dir(req, clip)

        # Forward to the publisher: set its load_request param, then call
        # its load Trigger. Both async; progress lands in /run/events.
        payload = req.publisher_payload()

        if not self._param_client.service_is_ready():
            self._event('error', 'replay_publisher set_parameters not ready')
            return self._reply(False, 'replay_publisher parameter service not ready')
        pmsg = Parameter()
        pmsg.name = 'load_request'
        pmsg.value = ParameterValue(type=4, string_value=payload)  # 4 = string
        preq = SetParameters.Request(parameters=[pmsg])
        future = self._param_client.call_async(preq)

        def after_param(fut):
            try:
                fut.result()
            except Exception as exc:  # noqa: BLE001
                self._event('error', 'set load_request failed', str(exc))
                return
            self._call('replay', 'load')
            self._loaded = True

        future.add_done_callback(after_param)
        self._event('info', 'load accepted',
                    {'clip': req.clip, 'speed_scale': req.speed_scale,
                     'scope': self.scope})
        return self._reply(True, 'load accepted; watch /run/events')

    def _srv_arm(self, request, response) -> Trigger.Response:
        if self.run_state is not RunState.IDLE or not self._loaded:
            return self._reply(False, f'arm requires IDLE with a loaded clip '
                                      f'(state {self.run_state.value})')
        self.monitor = Layer3Monitor(
            MonitorConfig(
                liveness_timeout_s=float(self.get_parameter('liveness_timeout_s').value),
                temp_warn_c=float(self.get_parameter('temp_warn_c').value),
                temp_trip_c=float(self.get_parameter('temp_trip_c').value),
                barrier_timeout_s=float(self.get_parameter('barrier_timeout_s').value),
                expect_hand_diagnostics=bool(
                    self.get_parameter('expect_hand_diagnostics').value),
            ),
            self.scope,
        )
        self.monitor.record_mode_machine(self._snapshots())
        self.arm_seq = ArmSequence(
            self.scope, now=time.monotonic(),
            timeout_s=float(self.get_parameter('barrier_timeout_s').value),
        )
        self._event('info', 'arm sequence started', self.scope)
        return self._reply(True, 'arming; barrier progress on /run/events')

    def _srv_start(self, request, response) -> Trigger.Response:
        if self.run_state is not RunState.ARMED:
            return self._reply(False, f'start requires ARMED (state '
                                      f'{self.run_state.value})')
        self._execute(start_actions(self.scope))
        self.run_state = RunState.RUNNING
        self._event('info', 'START', {'scope': self.scope})
        return self._reply(True, 'running')

    def _srv_stop(self, request, response) -> Trigger.Response:
        # Operator stop IS the fault path: one stop path, not three (spec_1
        # safety chain). No resume.
        self._do_fault('operator stop (run_ctl)')
        return self._reply(True, 'fault latched (operator stop)')

    def _srv_park(self, request, response) -> Trigger.Response:
        actions = []
        if self.scope.get('arms'):
            actions.append(('g1', 'park'))
        for side in self.scope.get('hands', []):
            actions.append((f'{side}_hand', 'park'))
        self._execute(actions)
        self._event('info', 'park requested')
        return self._reply(True, 'parking')

    def _srv_release(self, request, response) -> Trigger.Response:
        # Refuse until every in-scope device is parked (arm at its engage
        # snapshot, hands holding after park), so the reply reflects what
        # will actually happen instead of deferring the refusal to an
        # async device error on /run/events.
        snapshots = self._snapshots()
        problems = release_gate_problems(self.scope, snapshots)
        if problems:
            msg = '; '.join(problems)
            self._event('error', 'release refused', msg)
            return self._reply(False, msg)
        actions = []
        g1 = (snapshots.get('g1') or {}).get('data') or {}
        if self.scope.get('arms') and g1.get('fsm_state') != 'ready':
            actions.append(('g1', 'release'))
        for side in self.scope.get('hands', []):
            actions.append((f'{side}_hand', 'release'))
        self._execute(actions)
        self._event('info', 'release requested')
        # The run dir and bag close when the release COMPLETES (arm back
        # to ready, weight 0), observed in the tick -- closing here would
        # truncate the bag before the >= 2 s ramp and stop recording while
        # the arm is still powered. Hands have no weight: their release is
        # an immediate acknowledgment and never delays the close.
        self._release_pending = True
        if not self.scope.get('arms') or g1.get('fsm_state') == 'ready':
            self._finish_release()
        return self._reply(True, 'releasing; bag closes when the arm is back to ready')

    def _finish_release(self) -> None:
        self._release_pending = False
        self.monitor = None   # powered window over; detectors stand down
        self._event('info', 'release complete; run directory closed')
        self._close_run_dir()

    def _srv_clear_fault(self, request, response) -> Trigger.Response:
        if self.run_state is not RunState.FAULT:
            return self._reply(True, 'no fault latched')
        actions = []
        if self.scope.get('arms'):
            actions.append(('g1', 'clear_fault'))
        for side in self.scope.get('hands', []):
            actions.append((f'{side}_hand', 'clear_fault'))
        self._execute(actions)
        self.run_state = RunState.IDLE
        self._loaded = False   # 9: rerun from the start, fresh load required
        self.fault_reason = None
        self.monitor = None
        self._release_pending = False
        self._event('info', 'fault cleared by operator; fresh load required')
        return self._reply(True, 'fault cleared; load again to continue')

    # ---------------------------------------------------------------- tick

    def _do_fault(self, reason: str) -> None:
        if self.run_state is RunState.FAULT:
            return
        self.run_state = RunState.FAULT
        self.fault_reason = reason
        self._execute(fault_actions(self.scope))
        self._event('fault', 'FAULT_HOLD', reason)
        msg = String()
        msg.data = json.dumps({'reason': reason,
                               't_wall': datetime.now(timezone.utc).isoformat()},
                              sort_keys=True)
        self.fault_pub.publish(msg)

    def _tick(self) -> None:
        snapshots = self._snapshots()

        if self.arm_seq is not None and self.run_state is RunState.IDLE:
            actions = self.arm_seq.step(time.monotonic(), snapshots)
            self._execute(actions)
            if self.arm_seq.state is ArmSeqState.DONE:
                self.run_state = RunState.ARMED
                self._event('info', 'ARMED: all in-scope devices at frame-0 hold')
                self.arm_seq = None
            elif self.arm_seq.state is ArmSeqState.FAILED:
                reason = self.arm_seq.reason
                self.arm_seq = None
                self._do_fault(reason)

        # Active from the arm sequence (engage/approach move the robot)
        # through ARMED/RUNNING and the post-clip powered window
        # (end_hold/park/release); cleared on release completion.
        if self.monitor is not None:
            faults, warnings = self.monitor.update(time.monotonic(), snapshots)
            for w in warnings:
                self._event('warn', w)
            if faults:
                self._do_fault('; '.join(faults))

        if self._release_pending:
            g1 = (snapshots.get('g1') or {}).get('data') or {}
            if g1.get('fsm_state') == 'ready':
                self._finish_release()

        if self.run_state is RunState.RUNNING:
            replay = (snapshots.get('replay') or {}).get('data') or {}
            # The RUNNING -> IDLE transition is the once-per-run latch.
            if replay.get('clip_done'):
                self.run_state = RunState.IDLE
                self._event('info', 'clip end: publisher holding last frame; '
                                    'end_hold/park/release when ready')
                if self.scope.get('arms'):
                    self._call('g1', 'end_hold')
                for side in self.scope.get('hands', []):
                    self._call(f'{side}_hand', 'end_hold')

        self._publish_run_status(snapshots)

    def _publish_run_status(self, snapshots: dict) -> None:
        device_fields = {}
        for key in ('replay', 'g1', 'left_hand', 'right_hand'):
            snap = snapshots.get(key) or {}
            data = snap.get('data') or {}
            device_fields[key] = {
                'age_s': None if snap.get('age') is None else round(snap['age'], 3),
                'state': data.get('state') or data.get('fsm_state'),
                'approach_done': data.get('approach_done'),
                'fault': data.get('fault'),
                'clip_done': data.get('clip_done'),
            }
        payload = {
            'run_state': self.run_state.value,
            'fault_reason': self.fault_reason,
            'scope': self.scope,
            'clip': None if self.clip_meta is None else {
                'sample': self.clip_meta.get('sample'),
                'method': self.clip_meta.get('method'),
            },
            'speed_scale': None if self.request is None else self.request.speed_scale,
            'run_dir': None if self._run_dir is None else str(self._run_dir),
            'devices': device_fields,
            'arm_seq': None if self.arm_seq is None else self.arm_seq.state.value,
            'force_sim': bool(self.get_parameter('force_sim').value),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    # ------------------------------------------------------ run directory

    def _open_run_dir(self, req: SupervisorLoadRequest, clip) -> None:
        self._close_run_dir()
        runs_dir = Path(str(self.get_parameter('runs_dir').value)).expanduser()
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        scope_tag = 'full' if (len(req.arms) == 2 and len(req.hands) == 2) else \
            'a' + ''.join(s[0] for s in req.arms) + '_h' + ''.join(s[0] for s in req.hands)
        name = (f"{stamp}_{clip.meta.get('sample')}_{clip.meta.get('method')}"
                f"_{req.speed_scale}_{scope_tag}")
        self._run_dir = runs_dir / name
        self._run_dir.mkdir(parents=True, exist_ok=True)

        try:
            git_sha = subprocess.run(
                ['git', '-C', str(Path(__file__).resolve().parents[4]),
                 'rev-parse', 'HEAD'],
                capture_output=True, text=True, timeout=10,
            ).stdout.strip() or None
        except (subprocess.SubprocessError, FileNotFoundError):
            git_sha = None

        manifest = {
            'clip': str(clip.npz_path),
            'clip_sha256': {'npz': sha256_file(clip.npz_path),
                            'json': sha256_file(clip.json_path)},
            'sample': clip.meta.get('sample'),
            'method': clip.meta.get('method'),
            'speed_scale': req.speed_scale,
            'scope': self.scope,
            'operator': req.operator,
            'git_sha': git_sha,
            'image_digest': None,  # recorded by the operator until the
                                   # containers expose digests at runtime
            'arm_type': str(self.get_parameter('arm_type').value),
            'force_sim': bool(self.get_parameter('force_sim').value),
            'threshold_profile': {
                'liveness_timeout_s': float(self.get_parameter('liveness_timeout_s').value),
                'temp_warn_c': float(self.get_parameter('temp_warn_c').value),
                'temp_trip_c': float(self.get_parameter('temp_trip_c').value),
                'barrier_timeout_s': float(self.get_parameter('barrier_timeout_s').value),
            },
            'bag_topics': BAG_TOPICS,
        }
        (self._run_dir / 'run_manifest.json').write_text(
            json.dumps(manifest, indent=1, sort_keys=True) + '\n')

        self._events_file = open(self._run_dir / 'events.jsonl', 'w', buffering=1)
        self._event('info', 'run directory opened', str(self._run_dir))

        if bool(self.get_parameter('record_bag').value):
            bag_dir = self._run_dir / 'bag'
            cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir),
                   *BAG_TOPICS]
            try:
                self._bag_proc = subprocess.Popen(cmd)
                self._event('info', 'bag recording', str(bag_dir))
            except (OSError, subprocess.SubprocessError) as exc:
                self._event('error', 'bag record failed to start', str(exc))
                self._bag_proc = None

    def _close_run_dir(self) -> None:
        if self._bag_proc is not None:
            self._bag_proc.send_signal(signal.SIGINT)
            try:
                self._bag_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._bag_proc.kill()
            self._bag_proc = None
        if self._events_file is not None:
            self._events_file.close()
            self._events_file = None
        self._run_dir = None


def main(argv=None) -> None:
    raw_argv = sys.argv if argv is None else ['supervisor', *argv]
    rclpy.init(args=raw_argv)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_run_dir()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
