"""Supervisor core: run FSM, load gates, Layer-3 detectors, arm sequence.

ROS-free (spec_1 component 6 + safety envelope Layer 3). The supervisor
node adapts: it feeds status snapshots in and executes the actions this
module decides. Everything here is unit-testable in a bare venv.

Snapshots shape (all entries optional; 'age' is seconds since last message,
None = never seen; 'data' is the parsed JSON payload / field dict):

    {
      'replay':          {'age': float|None, 'data': dict|None},
      'g1':              {'age', 'data'},           # /g1/status
      'left_hand':       {'age', 'data'},           # /left_hand/status
      'right_hand':      {'age', 'data'},
      'left_hand_diag':  {'age', 'data'},           # raw hand_diagnostics
      'right_hand_diag': {'age', 'data'},
      'left_hand_effort':  {'age', 'data': [20 floats]},   # joint_states effort
      'right_hand_effort': {'age', 'data': [20 floats]},
    }

Layer 3 detects only what the supervisor alone can see (cross-device
liveness, barrier timeout, hand joint offline / error codes, effort
saturation > 1 s, temperature warn and trip, mode_machine change).
Divergence and per-device staleness stay at Layer 1 where they act with the
supervisor dead. Response is always FAULT_HOLD: the caller latches FAULT
and fans out fault triggers; nothing here zeroes anything.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

from replay.clip_artifact import BANNED_FIRST_SAMPLE_PREFIXES, VERDICT_PASS
from replay.pacer import LoadError as PacerLoadError
from replay.pacer import LoadRequest as PacerLoadRequest
from replay.pacer import PacerState


class RunState(Enum):
    IDLE = 'idle'
    ARMED = 'armed'
    RUNNING = 'running'
    FAULT = 'fault'


class GateError(ValueError):
    """A load/arm/start gate refused; the message says which and why."""


# ------------------------------------------------------------- load gates


@dataclass
class SupervisorLoadRequest:
    """Parsed supervisor load_request JSON (superset of the publisher's)."""

    clip: str
    speed_scale: float = 1.0
    arms: tuple = ('left', 'right')
    hands: tuple = ('left', 'right')
    override_gt_gate: bool = False
    override_first_clip: bool = False
    operator: str = ''

    @classmethod
    def from_json(cls, text: str) -> 'SupervisorLoadRequest':
        try:
            raw = json.loads(text)
        except json.JSONDecodeError as exc:
            raise GateError(f'load_request is not valid JSON: {exc}') from exc
        if not isinstance(raw, dict) or 'clip' not in raw:
            raise GateError("load_request must be an object with a 'clip' field")
        known = {'clip', 'speed_scale', 'arms', 'hands', 'override_gt_gate',
                 'override_first_clip', 'operator'}
        unknown = set(raw) - known
        if unknown:
            raise GateError(f'load_request has unknown fields {sorted(unknown)}')
        # Shared-field validation (side names, empty scope, speed_scale
        # bounds) is the pacer's parser -- composed, not re-implemented,
        # so the gate layer can never accept what the publisher refuses.
        shared = {k: raw[k] for k in ('clip', 'speed_scale', 'arms', 'hands')
                  if k in raw}
        try:
            base = PacerLoadRequest.from_json(json.dumps(shared))
        except PacerLoadError as exc:
            raise GateError(str(exc)) from exc
        return cls(
            clip=base.clip,
            speed_scale=base.speed_scale,
            arms=base.arms,
            hands=base.hands,
            override_gt_gate=bool(raw.get('override_gt_gate', False)),
            override_first_clip=bool(raw.get('override_first_clip', False)),
            operator=str(raw.get('operator', '')),
        )

    def publisher_payload(self) -> str:
        return json.dumps({
            'clip': self.clip,
            'speed_scale': self.speed_scale,
            'arms': list(self.arms),
            'hands': list(self.hands),
        }, sort_keys=True)


def check_load_gates(
    request: SupervisorLoadRequest,
    artifact_meta: dict,
    rig_arm_type: str,
    run_history: list,
) -> None:
    """7D / 7F load preconditions. run_history is a list of dicts from
    prior runs' tracking_summary.json files:
    {'sample', 'method', 'scope', 'speed_scale', 'pass'}."""
    problems = []

    if artifact_meta.get('verdict') != VERDICT_PASS:
        problems.append(
            f"artifact verdict is {artifact_meta.get('verdict')!r}: "
            f"{artifact_meta.get('verdict_reasons')}"
        )
    allowed = float(artifact_meta.get('max_allowed_speed_scale', 0.0))
    if request.speed_scale > allowed:
        problems.append(
            f'speed_scale {request.speed_scale} exceeds the artifact '
            f'max_allowed_speed_scale {allowed}'
        )
    if request.hands and not artifact_meta.get('hands_conditioned', True):
        problems.append('hand scope requested on an arm-only artifact')
    if rig_arm_type != 'G1_29':
        problems.append(
            f'rig arm_type is {rig_arm_type}; the conditioned 14-joint '
            f'artifact targets G1_29 (the 23-DoF path is out of scope for '
            f'this campaign)'
        )

    sample = str(artifact_meta.get('sample') or '')
    method = artifact_meta.get('method')

    any_prior_pass = any(h.get('pass') for h in run_history)
    if (sample.startswith(BANNED_FIRST_SAMPLE_PREFIXES) and not any_prior_pass
            and not request.override_first_clip):
        problems.append(
            f'sample {sample} is banned as the first clip (7F: shoulder-'
            f'torso contact in its physical audit); run another sample '
            f'first or pass override_first_clip after a fresh audit'
        )

    if method == 'Ours' and not request.override_gt_gate:
        scope = {'arms': sorted(request.arms), 'hands': sorted(request.hands)}
        gt_ok = any(
            h.get('sample') == sample and h.get('method') == 'GT'
            and h.get('pass')
            and h.get('scope') == scope
            and float(h.get('speed_scale', 0)) >= request.speed_scale
            for h in run_history
        )
        if not gt_ok:
            problems.append(
                f'GT-before-Ours (7D): no passing GT run of {sample} at '
                f'scope {scope} and speed >= {request.speed_scale}; run GT '
                f'first or pass override_gt_gate'
            )

    if problems:
        raise GateError('; '.join(problems))


def load_run_history(runs_dir: Path) -> list:
    """Scan run directories for tracking summaries (the 7D evidence)."""
    history = []
    if not runs_dir.exists():
        return history
    for summary in sorted(runs_dir.glob('*/tracking_summary.json')):
        try:
            data = json.loads(summary.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        history.append({
            'sample': data.get('sample'),
            'method': data.get('method'),
            'scope': data.get('scope'),
            'speed_scale': data.get('speed_scale'),
            'pass': data.get('pass'),
            'run_dir': str(summary.parent),
        })
    return history


# --------------------------------------------------------- Layer 3 monitor

@dataclass
class MonitorConfig:
    liveness_timeout_s: float = 1.0        # status topics are 10 Hz
    effort_saturation_s: float = 1.0       # spec: effort saturation > 1 s
    temp_warn_c: float = 50.0
    temp_trip_c: float = 60.0
    barrier_timeout_s: float = 30.0
    # False = sim profile (no wujihand driver): hand_diagnostics liveness
    # is not demanded; the diag-fed detectors are naturally silent.
    expect_hand_diagnostics: bool = True


class Layer3Monitor:
    """The six supervisor-only detectors. update() returns new fault
    strings (empty = healthy); warn() returns warning strings."""

    def __init__(self, config: MonitorConfig, scope: dict):
        self.cfg = config
        self.scope = scope
        self.mode_machine_ref: Optional[int] = None
        self._effort_over_since = {'left': None, 'right': None}
        self._warned = set()

    def _sources(self) -> list:
        sources = [('replay', 'replay publisher')]
        if self.scope.get('arms'):
            sources.append(('g1', 'arm node'))
        for side in self.scope.get('hands', []):
            sources.append((f'{side}_hand', f'{side} hand node'))
            if self.cfg.expect_hand_diagnostics:
                sources.append((f'{side}_hand_diag', f'{side} hand diagnostics'))
        return sources

    def record_mode_machine(self, snapshots: dict) -> None:
        g1 = (snapshots.get('g1') or {}).get('data') or {}
        self.mode_machine_ref = g1.get('mode_machine')

    def update(self, now: float, snapshots: dict) -> tuple:
        """Returns (faults, warnings), each a list of strings."""
        faults = []
        warnings = []

        # 1. Cross-device liveness.
        for key, label in self._sources():
            snap = snapshots.get(key) or {}
            age = snap.get('age')
            if age is None or age > self.cfg.liveness_timeout_s:
                faults.append(f'liveness: {label} silent '
                              f'(age {age if age is not None else "never"})')

        # 2. mode_machine change: the onboard controller changed state
        # under us -- cheap, unambiguous.
        if self.scope.get('arms') and self.mode_machine_ref is not None:
            g1 = (snapshots.get('g1') or {}).get('data') or {}
            current = g1.get('mode_machine')
            if current is not None and current != self.mode_machine_ref:
                faults.append(
                    f'mode_machine changed {self.mode_machine_ref} -> '
                    f'{current}: the onboard controller changed state'
                )

        for side in self.scope.get('hands', []):
            diag = (snapshots.get(f'{side}_hand_diag') or {}).get('data') or {}
            # 3. Hand joint offline + error codes.
            errors = [i for i, e in enumerate(diag.get('error_codes', [])) if e]
            if errors:
                faults.append(f'{side} hand error_codes at joints {errors}')
            offline = [i for i, en in enumerate(diag.get('enabled', [])) if not en]
            if offline:
                faults.append(f'{side} hand joints offline: {offline}')
            # 4. Temperature warn / trip (hand).
            temps = diag.get('joint_temperatures') or []
            if temps:
                worst = max(temps)
                if worst > self.cfg.temp_trip_c:
                    faults.append(f'{side} hand over-temperature {worst:.1f} C')
                elif worst > self.cfg.temp_warn_c:
                    key = f'{side}_temp'
                    if key not in self._warned:
                        self._warned.add(key)
                        warnings.append(f'{side} hand temperature warning '
                                        f'{worst:.1f} C')
            # 5. Effort saturation for over 1 s (drive current vs the
            # driver-published limits, both amps).
            limits = diag.get('effort_limits') or []
            eff = (snapshots.get(f'{side}_hand_effort') or {}).get('data') or []
            if limits and eff and len(limits) == len(eff):
                saturated = any(abs(e) >= lim for e, lim in zip(eff, limits))
                since = self._effort_over_since[side]
                if saturated:
                    if since is None:
                        self._effort_over_since[side] = now
                    elif now - since > self.cfg.effort_saturation_s:
                        faults.append(
                            f'{side} hand effort saturated for '
                            f'{now - since:.1f} s (> {self.cfg.effort_saturation_s} s)'
                        )
                else:
                    self._effort_over_since[side] = None

        # Arm temperature warn/trip from /g1/status.
        if self.scope.get('arms'):
            g1 = (snapshots.get('g1') or {}).get('data') or {}
            temp = g1.get('max_motor_temp_c')
            if temp is not None:
                if temp > self.cfg.temp_trip_c:
                    faults.append(f'arm motor over-temperature {temp:.1f} C')
                elif temp > self.cfg.temp_warn_c and 'arm_temp' not in self._warned:
                    self._warned.add('arm_temp')
                    warnings.append(f'arm motor temperature warning {temp:.1f} C')

        return faults, warnings


# --------------------------------------------------------- arm sequence

class ArmSeqState(Enum):
    PUBLISH_FIRST = 'publish_first'
    ENGAGE = 'engage'
    APPROACH = 'approach'
    BARRIER = 'barrier'
    DONE = 'done'
    FAILED = 'failed'


class ArmSequence:
    """The supervisor 'arm' flow, pinned order (plan amendment):
    load -> publish_first -> per-device engage -> approach -> frame-0
    barrier -> (start is a separate operator action).

    step() decides actions; the node executes them asynchronously and
    feeds fresh snapshots back. Actions are (target, service) pairs, e.g.
    ('replay', 'publish_first'), ('g1', 'engage'), ('left_hand', 'approach').
    Timeout anywhere -> FAILED with a reason (the caller faults: barrier
    timeout is a Layer 3 detector).
    """

    def __init__(self, scope: dict, now: float, timeout_s: float = 30.0):
        self.scope = scope
        self.state = ArmSeqState.PUBLISH_FIRST
        self.reason = ''
        self._deadline = now + timeout_s
        self._sent = set()

    def _devices(self) -> list:
        devices = []
        if self.scope.get('arms'):
            devices.append('g1')
        for side in self.scope.get('hands', []):
            devices.append(f'{side}_hand')
        return devices

    def step(self, now: float, snapshots: dict) -> list:
        if self.state in (ArmSeqState.DONE, ArmSeqState.FAILED):
            return []
        if now > self._deadline:
            stalled_stage = self.state.value
            self.state = ArmSeqState.FAILED
            self.reason = (f'alignment-barrier timeout in {stalled_stage} '
                           f'(devices did not all reach frame-0 hold)')
            return []

        actions = []
        replay = (snapshots.get('replay') or {}).get('data') or {}
        g1 = (snapshots.get('g1') or {}).get('data') or {}

        if self.state is ArmSeqState.PUBLISH_FIRST:
            if replay.get('state') == PacerState.FIRST_FRAME.value:
                self.state = ArmSeqState.ENGAGE
            elif 'replay/publish_first' not in self._sent:
                self._sent.add('replay/publish_first')
                actions.append(('replay', 'publish_first'))

        if self.state is ArmSeqState.ENGAGE:
            # Only the arm engages (hands have no weight).
            if not self.scope.get('arms'):
                self.state = ArmSeqState.APPROACH
            elif g1.get('engage_done'):
                self.state = ArmSeqState.APPROACH
            elif 'g1/engage' not in self._sent:
                self._sent.add('g1/engage')
                actions.append(('g1', 'engage'))

        if self.state is ArmSeqState.APPROACH:
            pending = False
            for dev in self._devices():
                data = (snapshots.get(dev) or {}).get('data') or {}
                if data.get('approach_done'):
                    continue
                pending = True
                if f'{dev}/approach' not in self._sent:
                    self._sent.add(f'{dev}/approach')
                    actions.append((dev, 'approach'))
            if not pending:
                self.state = ArmSeqState.BARRIER

        if self.state is ArmSeqState.BARRIER:
            # Every in-scope device reporting frame-0 hold, publisher still
            # repeating frame 0: the one condition start waits on.
            ok = replay.get('state') == PacerState.FIRST_FRAME.value
            for dev in self._devices():
                data = (snapshots.get(dev) or {}).get('data') or {}
                if not data.get('approach_done') or data.get('fault'):
                    ok = False
            if ok:
                self.state = ArmSeqState.DONE
        return actions


def start_actions(scope: dict) -> list:
    """Actions for the operator 'start': devices to track, then the pacer."""
    actions = []
    if scope.get('arms'):
        actions.append(('g1', 'track'))
    for side in scope.get('hands', []):
        actions.append((f'{side}_hand', 'track'))
    actions.append(('replay', 'start'))
    return actions


def fault_actions(scope: dict) -> list:
    """FAULT_HOLD fan-out: freeze the pacer and every in-scope device."""
    actions = [('replay', 'fault')]
    if scope.get('arms'):
        actions.append(('g1', 'fault'))
    for side in scope.get('hands', []):
        actions.append((f'{side}_hand', 'fault'))
    return actions
