"""Arm device state machine (spec_1 component 3, section 8). ROS-free.

    ready (hold measured, weight 0)
      -> engage   weight 0 -> 1 over >= 2 s, commanding the measured pose;
                  that pose is snapshotted as the release target
      -> approach measured -> frame 0 under approach limits
      -> track    follow the stamped stream through the safety chains
      -> end_hold hold the last target; confirm dq ~ 0 for >= 1 s
      -> approach re-entered with target = snapshot (park is an alias,
                  not a state)
      -> release  weight 1 -> 0 over >= 2 s while commanding the snapshot

Rules this class enforces:

  - Entering engage is gated: N consecutive fresh lowstate ticks with
    measured |dq| below threshold, so the snapshot (commanded truth for the
    whole run and the release target) is never taken from a stale frame or
    a settling arm.
  - Holds are CONSTANT snapshots, never live measured. Commanding measured
    at weight 1 gives zero position error, kd-only torque, and a gravity
    droop that chases itself down; the divergence monitor is structurally
    blind to it. (Plan amendment A1.)
  - Losing lowstate in any non-ready state resets the machine to ready:
    snapshot discarded, weight zeroed (unknown), fresh engage required. A
    node that comes back believing weight is 1 with a stale target would
    snap a drooped arm to that target.
  - Fault in any powered state: hold the last safe command at the current
    weight; never zero a command, never auto-release. Fault mid-engage
    freezes the weight at its current ramp value. De-escalation (park,
    release) is operator-only; clear_fault needs weight 0.
  - Every transition request validates its preconditions and returns
    immediately (accepted/rejected); progress happens in tick(). No
    request blocks on motion (single-threaded executors would deadlock).

The rclpy node adapts services onto the request_* methods and calls tick()
from its control loop. In sim (dry_run: no DDS), measured is taken to be
the command and lowstate is always fresh; divergence faults, engage-gate
rejections, and lowstate-loss resets are therefore exercised by the unit
tests, not by Stage 0 sim runs (plan amendment A6).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

import numpy as np

from g1_world_output.replay_safety import ReplaySafetyChain, rate_limit_step


class DeviceState(Enum):
    READY = 'ready'
    ENGAGE = 'engage'
    APPROACH = 'approach'
    TRACK = 'track'
    END_HOLD = 'end_hold'
    RELEASE = 'release'
    FAULT = 'fault'


POWERED_STATES = (DeviceState.ENGAGE, DeviceState.APPROACH, DeviceState.TRACK,
                  DeviceState.END_HOLD, DeviceState.RELEASE, DeviceState.FAULT)


@dataclass
class FsmConfig:
    control_dt: float = 1.0 / 250.0
    engage_ramp_s: float = 2.0          # spec: >= 2 s
    release_ramp_s: float = 2.0         # spec: >= 2 s
    engage_fresh_ticks: int = 50        # N consecutive fresh+still ticks
    engage_dq_max: float = 0.05         # rad/s, "measured |dq| below threshold"
    lowstate_staleness_s: float = 0.2
    approach_done_err: float = 0.05     # rad, spec: max error < 0.05
    approach_done_dq: float = 0.05      # rad/s, "measured dq ~ 0"
    end_hold_dq: float = 0.05           # rad/s
    end_hold_confirm_s: float = 1.0     # spec: >= 1 s
    sim: bool = False                   # dry_run: measured := cmd, always fresh


@dataclass
class TickInputs:
    now: float
    measured_q: Optional[np.ndarray]     # (dof,) or None
    measured_dq: Optional[np.ndarray]
    lowstate_age: Optional[float]        # None = never seen
    stream: Dict[str, Optional[np.ndarray]] = field(default_factory=dict)
    # side -> latest interpolated stream target (dof_side,) or None


@dataclass
class TickOutput:
    cmd: Optional[np.ndarray]            # (dof,) or None (ready: nothing to say)
    weight: float


class ArmDeviceFSM:
    def __init__(
        self,
        joint_names: list,
        chains: Dict[str, ReplaySafetyChain],
        deploy_velocity: np.ndarray,
        config: Optional[FsmConfig] = None,
    ):
        self.names = list(joint_names)
        self.dof = len(self.names)
        self.dof_side = self.dof // 2
        self.chains = chains
        self.deploy_velocity = np.asarray(deploy_velocity, dtype=float)
        self.cfg = config or FsmConfig()

        self.state = DeviceState.READY
        self.weight = 0.0
        self.cmd: Optional[np.ndarray] = None
        self.snapshot: Optional[np.ndarray] = None
        self.fault_info: Optional[dict] = None
        self.active_sides: list = []
        self.approach_target_kind: Optional[str] = None  # 'stream' | 'snapshot'
        self.events: list = []

        self._fresh_streak = 0
        self._ramp_start: Optional[float] = None
        self._ramp_from = 0.0
        self._settle_since: Optional[float] = None
        self._last_inputs: Optional[TickInputs] = None
        self._approach_done = False
        self._engage_done = False
        self._settled = False
        self._max_target_error = 0.0

    # ------------------------------------------------------------- helpers

    def _side_slice(self, side: str) -> slice:
        return slice(0, self.dof_side) if side == 'left' else \
            slice(self.dof_side, self.dof)

    def _event(self, text: str) -> None:
        self.events.append(text)

    def _refuse(self, msg: str):
        return False, msg

    def _fresh(self, inputs: TickInputs) -> bool:
        if self.cfg.sim:
            return True
        return (inputs.lowstate_age is not None
                and inputs.lowstate_age <= self.cfg.lowstate_staleness_s)

    def _stream_fresh(self, side: str, now: float) -> bool:
        return side in self.chains \
            and not self.chains[side].staleness.is_stale(now)

    # ------------------------------------------------- transition requests

    def request_engage(self):
        if self.state is not DeviceState.READY:
            return self._refuse(f'engage requires ready, is {self.state.value}')
        if self.fault_info is not None:
            return self._refuse('fault latched; clear_fault first')
        if self._last_inputs is None:
            return self._refuse('no tick yet; lowstate unknown')
        if self._fresh_streak < self.cfg.engage_fresh_ticks:
            return self._refuse(
                f'engage gate: need {self.cfg.engage_fresh_ticks} consecutive '
                f'fresh+still lowstate ticks, have {self._fresh_streak} '
                f'(a snapshot from a stale or settling arm would be wrong '
                f'for the whole run)'
            )
        measured = self._measured(self._last_inputs)
        self.snapshot = measured.copy()
        self.cmd = measured.copy()
        self.state = DeviceState.ENGAGE
        self._ramp_start = self._last_inputs.now
        self._ramp_from = self.weight
        self._engage_done = False
        self._event('engage: snapshot taken, weight ramp started')
        return True, 'engaging'

    def request_approach(self):
        """Approach toward the live stream target (frame 0)."""
        if self.fault_info is not None:
            return self._refuse('fault latched; clear_fault first')
        if not (self.state is DeviceState.ENGAGE and self._engage_done):
            return self._refuse(
                f'approach requires engage complete, is {self.state.value} '
                f'(weight {self.weight:.2f})'
            )
        now = self._last_inputs.now if self._last_inputs else 0.0
        active = [s for s in self.chains if self._stream_fresh(s, now)]
        if not active:
            return self._refuse(
                'no fresh in-scope target stream; publish_first must be '
                'running before approach'
            )
        self.active_sides = active
        self.approach_target_kind = 'stream'
        self.state = DeviceState.APPROACH
        self._approach_done = False
        self._event(f'approach: toward stream frame 0, sides {active}')
        return True, f'approaching (sides {active})'

    def request_track(self):
        if self.fault_info is not None:
            return self._refuse('fault latched; clear_fault first')
        if not (self.state is DeviceState.APPROACH
                and self.approach_target_kind == 'stream' and self._approach_done):
            return self._refuse(
                f'track requires approach(stream) complete, is '
                f'{self.state.value} (done={self._approach_done})'
            )
        self.state = DeviceState.TRACK
        self._event('track')
        return True, 'tracking'

    def request_end_hold(self):
        if self.state is not DeviceState.TRACK:
            return self._refuse(f'end_hold requires track, is {self.state.value}')
        self.state = DeviceState.END_HOLD
        self._settle_since = None
        self._settled = False
        self._event('end_hold')
        return True, 'holding last target'

    def request_park(self):
        """Alias: re-enter approach with target = snapshot (no park state).

        Allowed from end_hold, track, and fault (operator de-escalation)."""
        if self.state not in (DeviceState.END_HOLD, DeviceState.TRACK,
                              DeviceState.FAULT):
            return self._refuse(f'park requires end_hold/track/fault, is {self.state.value}')
        if self.snapshot is None:
            return self._refuse('no snapshot to park to (never engaged)')
        self.approach_target_kind = 'snapshot'
        self.state = DeviceState.APPROACH
        self._approach_done = False
        self._event('park: approach re-entered with target = snapshot')
        return True, 'parking to snapshot'

    def request_release(self):
        if not (self.state is DeviceState.APPROACH
                and self.approach_target_kind == 'snapshot' and self._approach_done):
            return self._refuse(
                f'release requires approach(snapshot) complete, is '
                f'{self.state.value} (done={self._approach_done})'
            )
        self.state = DeviceState.RELEASE
        self._ramp_start = self._last_inputs.now if self._last_inputs else 0.0
        self._ramp_from = self.weight
        self._event('release: weight ramp down at snapshot')
        return True, 'releasing'

    def fault(self, reason: str):
        """FAULT_HOLD: freeze command and weight exactly where they are."""
        self.fault_info = {'reason': reason, 'state': self.state.value,
                           'weight': self.weight}
        if self.state is not DeviceState.READY:
            self.state = DeviceState.FAULT
        self._event(f'FAULT: {reason} (weight frozen at {self.weight:.2f})')
        return True, 'fault latched'

    def request_clear_fault(self):
        if self.fault_info is None:
            return True, 'no fault latched'
        if self.weight != 0.0:
            return self._refuse(
                f'clear_fault requires weight 0 (park + release first); '
                f'weight is {self.weight:.2f}'
            )
        self.fault_info = None
        # The divergence monitors latch independently; leaving them latched
        # would re-fault on the first track tick after the documented
        # recovery (park -> release -> clear_fault -> rerun).
        for chain in self.chains.values():
            chain.divergence.reset()
        if self.state is DeviceState.FAULT:
            self.state = DeviceState.READY
            self.cmd = None
            self.snapshot = None
        self._event('fault cleared')
        return True, 'fault cleared'

    # ---------------------------------------------------------------- tick

    def _measured(self, inputs: TickInputs) -> np.ndarray:
        if self.cfg.sim:
            return (self.cmd if self.cmd is not None
                    else np.zeros(self.dof)).copy()
        return np.asarray(inputs.measured_q, dtype=float)

    def _measured_dq(self, inputs: TickInputs) -> np.ndarray:
        if self.cfg.sim:
            return np.zeros(self.dof)
        return np.asarray(inputs.measured_dq, dtype=float)

    def tick(self, inputs: TickInputs) -> TickOutput:
        self._last_inputs = inputs
        fresh = self._fresh(inputs)

        # Lowstate loss / e-stop reset (spec: forces the node back to ready
        # from any non-ready state; weight unknown -> zero; fresh engage
        # required). The fault latch survives the reset.
        if self.state is not DeviceState.READY and not fresh:
            self._event(
                f'lowstate loss (age {inputs.lowstate_age}); reset to ready: '
                'snapshot discarded, weight unknown'
            )
            self.state = DeviceState.READY
            self.weight = 0.0
            self.cmd = None
            self.snapshot = None
            self._fresh_streak = 0
            self._engage_done = False
            self._approach_done = False
            return self._output(inputs)

        # Engage-gate freshness streak (counted in ready only).
        if self.state is DeviceState.READY:
            if self.cfg.sim or (fresh and inputs.measured_dq is not None):
                dq = self._measured_dq(inputs)
                still = float(np.max(np.abs(dq))) <= self.cfg.engage_dq_max
                self._fresh_streak = self._fresh_streak + 1 if still else 0
            else:
                self._fresh_streak = 0
            return self._output(inputs)

        if self.state is DeviceState.FAULT:
            # Frozen: cmd and weight exactly as latched. Never zero.
            return self._output(inputs)

        measured = self._measured(inputs)
        measured_dq = self._measured_dq(inputs)

        if self.state is DeviceState.ENGAGE:
            elapsed = inputs.now - self._ramp_start
            if self.cfg.engage_ramp_s > 0:
                frac = min(1.0, elapsed / self.cfg.engage_ramp_s)
            else:
                frac = 1.0
            self.weight = self._ramp_from + frac * (1.0 - self._ramp_from)
            self._engage_done = self.weight >= 1.0
            self.cmd = self.snapshot.copy()

        elif self.state is DeviceState.APPROACH:
            target = self._approach_target_vector(inputs)
            self.cmd = rate_limit_step(self.cmd, target, self.deploy_velocity,
                                       self.cfg.control_dt)
            err = float(np.max(np.abs(measured - target)))
            self._max_target_error = err
            self._approach_done = (
                err < self.cfg.approach_done_err
                and float(np.max(np.abs(measured_dq))) < self.cfg.approach_done_dq
            )

        elif self.state is DeviceState.TRACK:
            new_cmd = self.cmd.copy()
            for side in self.active_sides:
                sl = self._side_slice(side)
                result = self.chains[side].process(
                    inputs.now,
                    inputs.stream.get(side),
                    self.cmd[sl],
                    None if self.cfg.sim else measured[sl],
                )
                new_cmd[sl] = result.cmd
                if result.divergence_fault:
                    mon = self.chains[side].divergence
                    self.fault(
                        f'{side} divergence: |measured - command| above '
                        f'{mon.threshold_rad} rad for {mon.m_consecutive} '
                        f'consecutive ticks (worst joint index '
                        f'{mon.worst_joint}, {mon.worst_error:.3f} rad)'
                    )
                    return self._output(inputs)
            # Out-of-scope sides keep their snapshot slice: constant by
            # construction (A1); nothing to recompute.
            self.cmd = new_cmd
            self._max_target_error = self._track_error(inputs, measured)

        elif self.state is DeviceState.END_HOLD:
            # cmd frozen at the last commanded value.
            still = float(np.max(np.abs(measured_dq))) < self.cfg.end_hold_dq
            if still:
                if self._settle_since is None:
                    self._settle_since = inputs.now
                self._settled = (inputs.now - self._settle_since
                                 >= self.cfg.end_hold_confirm_s)
            else:
                self._settle_since = None
                self._settled = False

        elif self.state is DeviceState.RELEASE:
            elapsed = inputs.now - self._ramp_start
            frac = elapsed / self.cfg.release_ramp_s if self.cfg.release_ramp_s > 0 else 1.0
            self.weight = max(0.0, self._ramp_from * (1.0 - frac))
            self.cmd = self.snapshot.copy()
            if self.weight <= 0.0:
                self._event('release complete; ready')
                self.state = DeviceState.READY
                self.weight = 0.0
                self.cmd = None
                self.snapshot = None
                self._fresh_streak = 0
                # Stale done-flags must not survive into the next run: the
                # supervisor's arm sequence trusts them and would skip
                # engage/approach, leaving track refused (found by the
                # sweep-test second-run traversal).
                self._engage_done = False
                self._approach_done = False

        return self._output(inputs)

    def _approach_target_vector(self, inputs: TickInputs) -> np.ndarray:
        if self.approach_target_kind == 'snapshot':
            return self.snapshot.copy()
        target = self.snapshot.copy() if self.snapshot is not None \
            else np.zeros(self.dof)
        for side in self.active_sides:
            raw = inputs.stream.get(side)
            if raw is not None:
                target[self._side_slice(side)] = raw
        return target

    def _track_error(self, inputs: TickInputs, measured: np.ndarray) -> float:
        errs = [0.0]
        for side in self.active_sides:
            raw = inputs.stream.get(side)
            if raw is not None:
                sl = self._side_slice(side)
                errs.append(float(np.max(np.abs(measured[sl] - raw))))
        return max(errs)

    def _output(self, inputs: TickInputs) -> TickOutput:
        # No per-tick status dict: the 10 Hz status publisher calls
        # status() itself; building one 250x/s would be pure GC churn.
        return TickOutput(
            cmd=None if self.cmd is None else self.cmd.copy(),
            weight=self.weight,
        )

    # -------------------------------------------------------------- status

    def status(self, inputs: Optional[TickInputs] = None) -> dict:
        return {
            'fsm_state': self.state.value,
            'weight': round(self.weight, 4),
            'engage_done': self._engage_done,
            'approach_done': self._approach_done,
            'approach_target': self.approach_target_kind,
            'settled': self._settled,
            'max_target_error_rad': round(self._max_target_error, 5),
            'fault': self.fault_info,
            'active_sides': list(self.active_sides),
            'snapshot_present': self.snapshot is not None,
            'fresh_streak': self._fresh_streak,
        }
