"""Replay pacer core: the publisher's state machine, ROS-free (spec_1
component 2).

The rclpy node (replay_publisher.py) is a thin adapter over this class:
services call load/publish_first/start/fault, the tick timer calls tick(),
and everything here is unit-testable in a bare venv.

States and the rules they carry:

    unloaded -> loaded          load(): artifact accepted through the gates
    loaded -> first_frame       publish_first(t0): repeat frame 0, stamps
                                advance (t0 + j*dt_play), frame index does not
    first_frame -> running      start(): frame index advances; the stamp
                                series continues with NO discontinuity (D10)
    running -> finished         clip end: hold the last frame forever
                                (TUITION section 8: never zero, never jump)
    any -> fault                fault(): freeze the current frame; keep
                                publishing it; only a fresh load leaves fault
    loaded/finished/fault -> loaded    load() again (supervisor gates when)

No pause and no mid-clip resume: section 9 forbids continuing after an
abnormal event, so a faulted run is parked, inspected, and rerun from the
start. The pacer publishes nothing before publish_first.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from replay.clip_artifact import (
    CANONICAL_ARM_JOINTS,
    ConditionedClip,
    VERDICT_PASS,
    hand_joint_names,
)

SIDES = ('left', 'right')


class PacerState(Enum):
    UNLOADED = 'unloaded'
    LOADED = 'loaded'
    FIRST_FRAME = 'first_frame'
    RUNNING = 'running'
    FINISHED = 'finished'
    FAULT = 'fault'


class LoadError(ValueError):
    """A load request failed a gate; the message says which."""


@dataclass
class LoadRequest:
    """Parsed load_request JSON: clip path, speed, per-side/device scope."""

    clip: str
    speed_scale: float = 1.0
    arms: tuple = ('left', 'right')
    hands: tuple = ('left', 'right')

    @classmethod
    def from_json(cls, text: str) -> 'LoadRequest':
        try:
            raw = json.loads(text)
        except json.JSONDecodeError as exc:
            raise LoadError(f"load_request is not valid JSON: {exc}") from exc
        if not isinstance(raw, dict) or 'clip' not in raw:
            raise LoadError("load_request must be an object with a 'clip' field")
        known = {'clip', 'speed_scale', 'arms', 'hands'}
        unknown = set(raw) - known
        if unknown:
            raise LoadError(f"load_request has unknown fields {sorted(unknown)}")
        arms = tuple(raw.get('arms', list(SIDES)))
        hands = tuple(raw.get('hands', list(SIDES)))
        for name, scope in (('arms', arms), ('hands', hands)):
            bad = [s for s in scope if s not in SIDES]
            if bad:
                raise LoadError(f"{name} scope has unknown sides {bad}")
        if not arms and not hands:
            raise LoadError("empty scope: nothing to publish")
        speed = float(raw.get('speed_scale', 1.0))
        if speed <= 0:
            raise LoadError(f"speed_scale must be > 0, got {speed}")
        return cls(clip=str(raw['clip']), speed_scale=speed, arms=arms, hands=hands)


@dataclass
class TickOutput:
    """What to publish this tick."""

    frame_index: int
    stamp_offset_s: float            # add to t0 for the header stamp
    arm_targets: dict                # side -> (names, q7 ndarray)
    hand_targets: dict               # side -> (names, q20 ndarray)
    clip_done: bool


class ReplayPacer:
    def __init__(self, force_sim: bool = False):
        self.force_sim = force_sim
        self.state = PacerState.UNLOADED
        self.clip: Optional[ConditionedClip] = None
        self.request: Optional[LoadRequest] = None
        self.dt_play: Optional[float] = None
        self._arm_names = {}     # side -> 7 names, cached at load
        self._hand_names = {}    # side -> 20 names, cached at load
        self._frame = 0          # frame index into the clip
        self._stamp_tick = 0     # advances every tick from publish_first on
        self._frozen_frame: Optional[int] = None

    # ------------------------------------------------------------- gates

    def load(self, clip: ConditionedClip, request: LoadRequest) -> None:
        if self.state in (PacerState.FIRST_FRAME, PacerState.RUNNING):
            raise LoadError(
                f"cannot load while {self.state.value}; fault or finish first"
            )
        problems = []
        if clip.verdict != VERDICT_PASS:
            problems.append(
                f"artifact verdict is '{clip.verdict}': "
                f"{clip.meta.get('verdict_reasons')}"
            )
        if request.speed_scale > clip.max_allowed_speed_scale:
            problems.append(
                f"speed_scale {request.speed_scale} exceeds the clip's "
                f"max_allowed_speed_scale {clip.max_allowed_speed_scale}"
            )
        if request.hands and not clip.hands_conditioned:
            problems.append(
                "hand scope requested but the artifact was conditioned with "
                "--no-hands (hands_conditioned=false)"
            )
        if problems and not self.force_sim:
            raise LoadError('; '.join(problems))

        self.clip = clip
        self.request = request
        self.dt_play = clip.dt_play(request.speed_scale)
        self._arm_names = {
            side: CANONICAL_ARM_JOINTS[slice(0, 7) if side == 'left'
                                       else slice(7, 14)]
            for side in request.arms
        }
        self._hand_names = {side: hand_joint_names(side)
                            for side in request.hands}
        self._frame = 0
        self._stamp_tick = 0
        self._frozen_frame = None
        self.state = PacerState.LOADED

    def publish_first(self) -> None:
        if self.state is not PacerState.LOADED:
            raise LoadError(f"publish_first requires state loaded, is {self.state.value}")
        self._frame = 0
        self._stamp_tick = 0
        self.state = PacerState.FIRST_FRAME

    def start(self) -> None:
        if self.state is not PacerState.FIRST_FRAME:
            raise LoadError(f"start requires state first_frame, is {self.state.value}")
        self.state = PacerState.RUNNING

    def fault(self) -> None:
        """Freeze. Keeps repeating the current frame; never resumes (§9)."""
        if self.state in (PacerState.FIRST_FRAME, PacerState.RUNNING,
                          PacerState.FINISHED):
            self._frozen_frame = self._current_frame()
        self.state = PacerState.FAULT

    # -------------------------------------------------------------- tick

    def _current_frame(self) -> int:
        if self.state is PacerState.FAULT and self._frozen_frame is not None:
            return self._frozen_frame
        if self.clip is None:
            return 0
        return min(self._frame, self.clip.num_frames - 1)

    def tick(self) -> Optional[TickOutput]:
        """One timer tick. None means publish nothing (pre-first states, or
        fault before any frame was ever published)."""
        if self.state in (PacerState.UNLOADED, PacerState.LOADED):
            return None
        if self.state is PacerState.FAULT and self._frozen_frame is None:
            return None
        clip = self.clip
        i = self._current_frame()
        stamp_offset = self._stamp_tick * self.dt_play
        self._stamp_tick += 1

        if self.state is PacerState.RUNNING:
            if self._frame >= clip.num_frames - 1:
                self.state = PacerState.FINISHED
            else:
                self._frame += 1

        arm_targets = {}
        for side in self.request.arms:
            sl = slice(0, 7) if side == 'left' else slice(7, 14)
            arm_targets[side] = (self._arm_names[side], clip.arm_q[i, sl])
        hand_targets = {}
        for side in self.request.hands:
            q20 = clip.left_hand_q20 if side == 'left' else clip.right_hand_q20
            hand_targets[side] = (self._hand_names[side], q20[i])

        return TickOutput(
            frame_index=i,
            stamp_offset_s=stamp_offset,
            arm_targets=arm_targets,
            hand_targets=hand_targets,
            clip_done=self.state is PacerState.FINISHED,
        )

    # ------------------------------------------------------------ status

    def status(self) -> dict:
        """/replay/status payload (docs/spec/spec_1_interfaces.md)."""
        meta = self.clip.meta if self.clip is not None else {}
        return {
            'state': self.state.value,
            'clip': str(self.clip.npz_path) if self.clip is not None else None,
            'sample': meta.get('sample'),
            'method': meta.get('method'),
            'speed_scale': self.request.speed_scale if self.request else None,
            'tick': self._current_frame() if self.clip is not None else 0,
            'total': self.clip.num_frames if self.clip is not None else 0,
            'clip_done': self.state is PacerState.FINISHED,
            'scope': {
                'arms': list(self.request.arms) if self.request else [],
                'hands': list(self.request.hands) if self.request else [],
            },
            'force_sim': self.force_sim,
        }
