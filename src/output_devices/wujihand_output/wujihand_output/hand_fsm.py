"""Hand q20 device state machine (spec_1 component 4). ROS-free.

Four states, not the arm's seven: hold, approach, track, end_hold. Engage
exists on the arm only to manage the rt/arm_sdk weight; the hand has no
weight and no onboard controller to hand back to. Park is an alias that
re-enters approach with the configured neutral pose as the target (spec:
hands slew to a neutral pose at clip end under approach limits, section 8
"return smoothly to a safe pose"). Release is an acknowledgment only --
it succeeds on a parked (holding) hand so the supervisor can fan release
out to every in-scope device, and ramps nothing.

Layer-1 duties carried here (they must act with the supervisor dead):
  - position clamp + per-joint rate limit on every command (deploy rows of
    hand_limits.yaml; velocity capped at the 4.0 rad/s screening value, the
    named section-6 deviation -- never the URDF sim-model values)
  - target staleness -> hold last command
  - feedback watchdogs: joint_states staleness, hand_diagnostics staleness,
    any nonzero error_codes[i], joint offline (enabled false),
    over-temperature, sustained over-current (EffortGuard, amps) ->
    hold last command and latch a fault
  - holds are frozen commands, never live measured (plan amendment A1)

Transition requests validate and return immediately; progress happens in
tick() (plan amendment A3). In sim (no driver): measured := cmd and the
feedback watchdogs are off -- those paths are covered by unit tests, not
Stage 0 sim runs (plan amendment A6).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from wujihand_output.hand_safety import (
    NUM_JOINTS,
    EffortGuard,
    HandLimits,
    PositionClamp,
    StalenessTracker,
    rate_limit_step,
)


class HandState(Enum):
    HOLD = 'hold'
    APPROACH = 'approach'
    TRACK = 'track'
    END_HOLD = 'end_hold'
    FAULT = 'fault'


@dataclass
class HandFsmConfig:
    control_dt: float = 1.0 / 200.0     # spec: 200 Hz for replay (5 range 200-1k)
    target_staleness_s: float = 0.25
    state_staleness_s: float = 0.5      # joint_states nominally 1 kHz
    diagnostics_staleness_s: float = 2.0  # nominally 10 Hz; can stall silently
    approach_done_err: float = 0.05
    temperature_trip_c: float = 60.0
    effort_guard_ticks: int = 40        # ~0.2 s over-current at 200 Hz
    effort_guard_scale: float = 1.0
    require_feedback: bool = True       # False = sim (no driver)
    neutral_pose: Optional[np.ndarray] = None  # default zeros


@dataclass
class HandTickInputs:
    now: float
    measured_q: Optional[np.ndarray]        # (20,) from joint_states, or None
    measured_effort: Optional[np.ndarray]   # (20,) amps (filtered), or None
    state_age: Optional[float]              # joint_states age, None = never
    diagnostics: Optional[dict]             # latest hand_diagnostics fields
    diagnostics_age: Optional[float]
    stream: Optional[np.ndarray]            # latest interpolated target (20,)


@dataclass
class HandTickOutput:
    cmd: Optional[np.ndarray]               # (20,) to publish, or None


class HandDeviceFSM:
    def __init__(self, limits: HandLimits, config: Optional[HandFsmConfig] = None):
        self.limits = limits
        self.cfg = config or HandFsmConfig()
        self.state = HandState.HOLD
        self.cmd: Optional[np.ndarray] = None
        self.fault_info: Optional[dict] = None
        self.approach_target_kind: Optional[str] = None  # 'stream' | 'neutral'
        self.events: list = []

        self.position_clamp = PositionClamp(limits.pos_lower, limits.pos_upper)
        self.target_staleness = StalenessTracker(self.cfg.target_staleness_s)
        self.effort_guard = EffortGuard(self.cfg.effort_guard_ticks,
                                        self.cfg.effort_guard_scale)
        self._neutral = (np.zeros(NUM_JOINTS) if self.cfg.neutral_pose is None
                         else np.asarray(self.cfg.neutral_pose, dtype=float))
        clamped_neutral, hit = self.position_clamp.apply(self._neutral)
        if hit.any():
            raise ValueError('neutral_pose outside hand position limits')
        self._approach_done = False
        self._max_target_error = 0.0
        self._last_inputs: Optional[HandTickInputs] = None

    # ------------------------------------------------------------ requests

    def mark_target_input(self, t: float) -> None:
        self.target_staleness.mark(t)

    def request_approach(self):
        if self.fault_info is not None:
            return False, 'fault latched; clear_fault first'
        if self.state not in (HandState.HOLD, HandState.END_HOLD):
            return False, f'approach requires hold/end_hold, is {self.state.value}'
        if self._last_inputs is None:
            return False, 'no tick yet'
        if self.target_staleness.is_stale(self._last_inputs.now):
            return False, ('no fresh target stream; publish_first must be '
                           'running before approach')
        if self.cfg.require_feedback and self._last_inputs.measured_q is None:
            return False, 'no joint_states feedback yet'
        self.approach_target_kind = 'stream'
        self.state = HandState.APPROACH
        self._approach_done = False
        self.events.append('approach: toward stream target')
        return True, 'approaching'

    def request_track(self):
        if self.fault_info is not None:
            return False, 'fault latched; clear_fault first'
        if not (self.state is HandState.APPROACH
                and self.approach_target_kind == 'stream' and self._approach_done):
            return False, (f'track requires approach(stream) complete, is '
                           f'{self.state.value} (done={self._approach_done})')
        self.state = HandState.TRACK
        self.events.append('track')
        return True, 'tracking'

    def request_end_hold(self):
        if self.state is not HandState.TRACK:
            return False, f'end_hold requires track, is {self.state.value}'
        self.state = HandState.END_HOLD
        self.events.append('end_hold')
        return True, 'holding last target'

    def request_park(self):
        """Alias: approach re-entered with target = neutral pose."""
        if self.state not in (HandState.END_HOLD, HandState.TRACK, HandState.FAULT,
                              HandState.HOLD):
            return False, f'park refused from {self.state.value}'
        if self.cmd is None and self._last_inputs is not None \
                and self._last_inputs.measured_q is None and self.cfg.require_feedback:
            return False, 'nothing commanded and no feedback; cannot park'
        self.approach_target_kind = 'neutral'
        self.state = HandState.APPROACH
        self._approach_done = False
        self.events.append('park: approach re-entered with target = neutral')
        return True, 'parking to neutral'

    def request_release(self):
        """The hand has no weight: release only acknowledges a completed park.

        Mirrors the arm's release gate so the supervisor can fan release out
        to every in-scope device. A parked (holding) hand keeps holding its
        frozen command -- nothing ramps down and nothing is zeroed.
        """
        if self.state is HandState.HOLD:
            self.events.append('release: parked hand keeps holding '
                               '(no weight to ramp)')
            return True, 'released (holding)'
        if (self.state is HandState.APPROACH
                and self.approach_target_kind == 'neutral'
                and self._approach_done):
            # Park completing this very tick; equivalent to hold.
            self.state = HandState.HOLD
            self._approach_done = False  # never stale into the next run
            self.events.append('release: parked hand keeps holding '
                               '(no weight to ramp)')
            return True, 'released (holding)'
        return False, (f'release requires a parked hand (hold after park), '
                       f'is {self.state.value}')

    def fault(self, reason: str):
        self.fault_info = {'reason': reason, 'state': self.state.value}
        self.state = HandState.FAULT
        self.events.append(f'FAULT: {reason} (command frozen)')
        return True, 'fault latched'

    def request_clear_fault(self):
        if self.fault_info is None:
            return True, 'no fault latched'
        self.fault_info = None
        self.state = HandState.HOLD
        self._approach_done = False  # never stale into the next run
        self.effort_guard.reset()
        self.events.append('fault cleared')
        return True, 'fault cleared'

    # ---------------------------------------------------------------- tick

    def _measured(self, inputs: HandTickInputs) -> Optional[np.ndarray]:
        if not self.cfg.require_feedback:
            return None if self.cmd is None else self.cmd.copy()
        return inputs.measured_q

    def _watchdogs(self, inputs: HandTickInputs) -> None:
        if not self.cfg.require_feedback or self.fault_info is not None:
            return
        # age None = never seen: not yet online, refused by the approach
        # gate and caught by Layer-3 liveness during arm -- faulting here
        # would latch on the very first tick of every bring-up, before
        # any message could possibly arrive. Stale = went quiet AFTER
        # being alive.
        if inputs.state_age is not None \
                and inputs.state_age > self.cfg.state_staleness_s:
            self.fault(f'joint_states stale (age {inputs.state_age:.2f}s)')
            return
        if inputs.diagnostics_age is not None \
                and inputs.diagnostics_age > self.cfg.diagnostics_staleness_s:
            self.fault(f'hand_diagnostics stale (age {inputs.diagnostics_age:.2f}s); '
                       'the driver read thread can stall silently')
            return
        diag = inputs.diagnostics or {}
        errors = [i for i, e in enumerate(diag.get('error_codes', [])) if e]
        if errors:
            self.fault(f'nonzero error_codes at joints {errors}')
            return
        offline = [i for i, en in enumerate(diag.get('enabled', [])) if not en]
        if offline:
            self.fault(f'joints offline: {offline}')
            return
        temps = diag.get('joint_temperatures', [])
        if temps and max(temps) > self.cfg.temperature_trip_c:
            self.fault(f'over-temperature: {max(temps):.1f} C '
                       f'> {self.cfg.temperature_trip_c} C')
            return
        limits = diag.get('effort_limits')
        if limits is not None and not self.effort_guard.active:
            try:
                self.effort_guard.set_limits(limits)
            except ValueError:
                pass
        if inputs.measured_effort is not None \
                and self.effort_guard.update(inputs.measured_effort):
            self.fault(f'sustained over-current (worst joint '
                       f'{self.effort_guard.worst_joint})')

    def tick(self, inputs: HandTickInputs) -> HandTickOutput:
        self._last_inputs = inputs
        self._watchdogs(inputs)

        if self.state in (HandState.HOLD, HandState.FAULT, HandState.END_HOLD):
            # Frozen command (or silence if nothing was ever commanded).
            return self._output()

        measured = self._measured(inputs)

        if self.state is HandState.APPROACH:
            if self.approach_target_kind == 'neutral':
                target = self._neutral
            else:
                target = inputs.stream
            if target is None:
                return self._output()  # hold until the stream speaks again
            target, _ = self.position_clamp.apply(target)
            base = self.cmd if self.cmd is not None else measured
            if base is None:
                if self.cfg.require_feedback:
                    return self._output()   # nothing to ramp from yet
                base = self._neutral.copy()  # sim: no measured pose exists
            self.cmd = rate_limit_step(base, target, self.limits.deploy_velocity,
                                       self.cfg.control_dt)
            ref = measured if measured is not None else self.cmd
            self._max_target_error = float(np.max(np.abs(ref - target)))
            self._approach_done = self._max_target_error < self.cfg.approach_done_err
            if self._approach_done and self.approach_target_kind == 'neutral':
                # Park complete: a parked hand is a holding hand. Staying in
                # approach would refuse the next run's approach request
                # (which requires hold/end_hold) and stall its barrier.
                self.state = HandState.HOLD
                # Stale approach_done must not survive into the next run:
                # the supervisor's arm sequence trusts it and would skip
                # this hand's approach, leaving track refused.
                self._approach_done = False
                self.events.append('park complete; holding neutral')

        elif self.state is HandState.TRACK:
            stale = self.target_staleness.is_stale(inputs.now)
            if not stale and inputs.stream is not None and self.cmd is not None:
                target, _ = self.position_clamp.apply(inputs.stream)
                self.cmd = rate_limit_step(self.cmd, target,
                                           self.limits.deploy_velocity,
                                           self.cfg.control_dt)
                ref = measured if measured is not None else self.cmd
                self._max_target_error = float(np.max(np.abs(ref - target)))
            # stale -> self.cmd unchanged: hold last command, never zero.

        return self._output()

    def _output(self) -> HandTickOutput:
        # No per-tick status dict (the 10 Hz status publisher calls
        # status() itself).
        return HandTickOutput(
            cmd=None if self.cmd is None else self.cmd.copy(),
        )

    # -------------------------------------------------------------- status

    def status(self) -> dict:
        inp = self._last_inputs
        return {
            'fsm_state': self.state.value,
            'approach_done': self._approach_done,
            'approach_target': self.approach_target_kind,
            'max_target_error_rad': round(self._max_target_error, 5),
            'fault': self.fault_info,
            'target_age_s': (None if inp is None
                             else self.target_staleness.age(inp.now)),
            'state_age_s': None if inp is None else inp.state_age,
            'diagnostics_age_s': None if inp is None else inp.diagnostics_age,
            'effort_guard_active': self.effort_guard.active,
        }
