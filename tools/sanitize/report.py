"""sanitize.json, and the operator-facing lines that go to stderr.

Two audiences. sanitize.json is the record: every option the run used, every
defect it measured, and every frame it could not fix, so a clip can be
re-judged without re-running anything. stderr is for the person watching the
run, and it says the same three things about each failure -- which frame,
what failed, and when in the clip it happens.

A failure is a frame the solve could not pull apart -- still touching,
or, under --fail-on clearance, still inside the clearance band. Those are
printed one line each. Two other kinds of contact are counted per pair
instead of printed per frame, because on a real bundle clip they run to
thousands of frame-pairs and a line each would bury everything else:

    near miss   inside the clearance at the end, but never actually touching
    unfixable   two segments of the same hand, which nothing this tool can
                change will move -- hand joints pass through untouched
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, TextIO

import numpy as np

from clip_audit import SIDES

from . import TOOL_ID
from .collision import Contact

# Constants. Each one says where its value comes from.

# 0 and 2 match prepare_clip.py; 3 is written-but-N-frames-uncleared.
EXIT_OK = 0
EXIT_ERROR = 1
EXIT_REFUSED = 2
EXIT_FAILURES = 3

# How many unfixable pairs the aggregate block lists, worst overlap first.
TOP_UNFIXABLE_PAIRS = 10


@dataclass
class Failure:
    """A frame the collision-constrained solve could not pull apart."""
    frame: int
    t_s: float
    klass: str
    a: str
    b: str
    gap_m: float
    touching: bool
    passes: int
    pos_err_m: float
    ori_err_rad: float

    @property
    def state(self) -> str:
        return ("TOUCHING" if self.touching
                else f"gap {1e3 * self.gap_m:+.2f} mm")

    def line(self, clearance_m: float, others: int = 0) -> str:
        also = f" (+{others} more pair(s) on this frame)" if others else ""
        return (f"FAIL  frame {self.frame:5d}  t={self.t_s:8.3f}s  {self.klass:<10s}  "
                f"{self.a} <-> {self.b}  {self.state} "
                f"(need {1e3 * clearance_m:.1f} mm)  "
                f"gave up after {self.passes} extra solve(s); "
                f"wrist residual {1e3 * self.pos_err_m:.1f} mm / "
                f"{math.degrees(self.ori_err_rad):.1f} deg{also}")

    def as_dict(self) -> dict:
        return {"frame": self.frame, "t_s": round(self.t_s, 4), "class": self.klass,
                "a": self.a, "b": self.b, "touching": self.touching,
                "gap_mm": None if self.touching else round(1e3 * self.gap_m, 3),
                "extra_solves": self.passes,
                "wrist_pos_residual_mm": round(1e3 * self.pos_err_m, 3),
                "wrist_ori_residual_deg": round(math.degrees(self.ori_err_rad), 3)}


@dataclass
class PairAggregate:
    """One pair, accumulated over every frame it was too close on.

    For the contacts counted rather than printed per frame: same-hand pairs,
    and pairs that stayed inside the clearance without interpenetrating.
    worst_gap_mm is the smallest gap seen, positive being clear air.
    """
    a: str
    b: str
    klass: str
    frames: int = 0
    touching_frames: int = 0
    worst_gap_mm: float = math.inf
    first_t_s: Optional[float] = None
    last_t_s: Optional[float] = None

    def observe(self, contact: Contact, t_s: float) -> None:
        self.frames += 1
        if contact.touching:
            self.touching_frames += 1
        else:
            self.worst_gap_mm = min(self.worst_gap_mm, 1e3 * contact.gap_m)
        self.first_t_s = t_s if self.first_t_s is None else self.first_t_s
        self.last_t_s = t_s

    @property
    def closest(self) -> str:
        if self.touching_frames:
            return f"touching on {self.touching_frames}"
        return f"closest {self.worst_gap_mm:+.2f} mm"

    def as_dict(self) -> dict:
        return {"a": self.a, "b": self.b, "class": self.klass, "frames": self.frames,
                "touching_frames": self.touching_frames,
                "closest_gap_mm": (None if not math.isfinite(self.worst_gap_mm)
                                   else round(self.worst_gap_mm, 3)),
                "first_t_s": round(self.first_t_s, 4) if self.first_t_s is not None else None,
                "last_t_s": round(self.last_t_s, 4) if self.last_t_s is not None else None}


class Report:
    """Everything one run measured."""

    def __init__(self, *, options: dict, source: dict, frames: int, rate_hz: float,
                 clearance_m: float, drift_warn_deg: float = math.inf,
                 drift_cap_deg: float = math.inf) -> None:
        self.drift_warn_deg = float(drift_warn_deg)
        self.drift_cap_deg = float(drift_cap_deg)
        self.options = dict(options)
        self.source = dict(source)
        self.frames = int(frames)
        self.rate_hz = float(rate_hz)
        self.clearance_m = float(clearance_m)

        self.wraps: Dict[str, Dict[str, int]] = {}
        self.drift: Dict[str, List[dict]] = {}
        self.flips: Dict[str, List[dict]] = {s: [] for s in SIDES}
        self.gated_frames: Dict[str, int] = {s: 0 for s in SIDES}
        self.hand_clamped: Dict[str, int] = {s: 0 for s in SIDES}

        self.collision: dict = {}
        self.failures: List[Failure] = []
        self.unfixable: Dict[tuple, PairAggregate] = {}
        self.near_miss: Dict[tuple, PairAggregate] = {}
        self.frames_with_contact = 0
        self.frames_constrained = 0

        self._pos_err = np.zeros(self.frames)
        self._ori_err = np.zeros(self.frames)
        self._elbow_err = np.zeros(self.frames)
        self._iterations: List[int] = []
        self._at_limit = np.zeros(0, dtype=int)
        self._at_step = np.zeros(0, dtype=int)
        self._max_step_rad = 0.0
        self._not_converged = 0

    # -- accumulation ------------------------------------------------------

    def observe_frame(self, frame: int, result, previous: Optional[np.ndarray]) -> None:
        self._pos_err[frame] = result.max_pos_err_m
        self._ori_err[frame] = result.max_ori_err_rad
        self._elbow_err[frame] = result.max_elbow_err_m
        self._iterations.append(int(sum(result.iterations.values())))
        if self._at_limit.size == 0:
            self._at_limit = np.zeros(result.at_limit.size, dtype=int)
            self._at_step = np.zeros(result.at_limit.size, dtype=int)
        self._at_limit += result.at_limit.astype(int)
        self._at_step += result.at_step_bound.astype(int)
        if not result.converged:
            self._not_converged += 1
        if previous is not None:
            self._max_step_rad = max(self._max_step_rad,
                                     float(np.abs(result.arm - previous).max()))
        if result.constrained:
            self.frames_constrained += 1

    @staticmethod
    def _accumulate(into: Dict[tuple, PairAggregate], contact: Contact, t_s: float) -> None:
        key = (contact.pair.name_a, contact.pair.name_b)
        entry = into.get(key)
        if entry is None:
            entry = PairAggregate(a=contact.pair.name_a, b=contact.pair.name_b,
                                  klass=contact.pair.klass)
            into[key] = entry
        entry.observe(contact, t_s)

    def observe_unfixable(self, contact: Contact, t_s: float) -> None:
        """A contact no arm-joint change can affect: same-hand segments."""
        self._accumulate(self.unfixable, contact, t_s)

    def observe_near_miss(self, contact: Contact, t_s: float) -> None:
        """Inside the clearance after the solve, but not interpenetrating."""
        self._accumulate(self.near_miss, contact, t_s)

    def observe_failure(self, failure: Failure) -> None:
        self.failures.append(failure)

    # -- output ------------------------------------------------------------

    @property
    def failed(self) -> bool:
        return bool(self.failures)

    @property
    def exit_code(self) -> int:
        return EXIT_FAILURES if self.failed else EXIT_OK

    def print_failures(self, stream: Optional[TextIO] = None,
                       verbose: bool = False) -> None:
        """One line per failing frame, then the aggregate of what is left.

        Grouped by frame: one clip measured 1053 pair-failures on 48 frames,
        so the worst pair names the frame and sanitize.json carries the rest.
        stream is resolved here, not bound to sys.stderr at import time.
        """
        stream = sys.stderr if stream is None else stream
        if verbose:
            for failure in self.failures:
                print(failure.line(self.clearance_m), file=stream)
        else:
            for frame in sorted({f.frame for f in self.failures}):
                group = [f for f in self.failures if f.frame == frame]
                # Touching first, then smallest gap: the worst thing names the frame.
                worst = sorted(group, key=lambda f: (not f.touching, f.gap_m))[0]
                print(worst.line(self.clearance_m, others=len(group) - 1), file=stream)
            if self.failures:
                print(f"      {len(self.failures)} contact(s) on "
                      f"{len({f.frame for f in self.failures})} frame(s) in total; "
                      f"every one is listed in the report", file=stream)

        self._print_aggregate(
            self.near_miss, "stayed inside the clearance without touching", stream)
        self._print_aggregate(
            self.unfixable, "this tool cannot act on (hand joints pass through unchanged)",
            stream)

        for side, entries in self.drift.items():
            for entry in entries:
                if abs(entry["measured_deg"]) >= self.drift_warn_deg:
                    fix = ("" if math.isfinite(self.drift_cap_deg) else
                           "; re-run with --max-drift-deg to remove the excess")
                    print(f"DRIFT {entry['joint']} ends {entry['measured_deg']:+.1f} deg from "
                          f"where it started (removed {entry['removed_deg']:+.1f}, "
                          f"residual {entry['residual_deg']:+.1f}){fix}", file=stream)

    def _print_aggregate(self, entries: Dict[tuple, PairAggregate], why: str,
                         stream: Optional[TextIO] = None) -> None:
        stream = sys.stderr if stream is None else stream
        if not entries:
            return
        # Worst first: touching by frame count, then near misses by closeness.
        worst = sorted(entries.values(),
                       key=lambda u: (-u.touching_frames, u.worst_gap_mm))
        total = sum(u.frames for u in worst)
        touched = sum(1 for u in worst if u.touching_frames)
        extra = f", {touched} of them touching" if touched else ""
        print(f"NOTE  {len(worst)} pair(s) on {total} frame-pair(s) {why}{extra}:", file=stream)
        for entry in worst[:TOP_UNFIXABLE_PAIRS]:
            print(f"      {entry.klass:<10s}  {entry.a} <-> {entry.b}  "
                  f"frames {entry.frames}  {entry.closest}  "
                  f"t={entry.first_t_s:.3f}s..{entry.last_t_s:.3f}s", file=stream)
        if len(worst) > TOP_UNFIXABLE_PAIRS:
            print(f"      ... and {len(worst) - TOP_UNFIXABLE_PAIRS} more, "
                  f"all listed in the report", file=stream)

    def print_summary(self, stream: Optional[TextIO] = None) -> None:
        stream = sys.stderr if stream is None else stream
        print(f"frames {self.frames} at {self.rate_hz:g} Hz  "
              f"wrist residual max {1e3 * float(self._pos_err.max() if self.frames else 0):.2f} mm"
              f" / {math.degrees(float(self._ori_err.max() if self.frames else 0)):.2f} deg  "
              f"max step {math.degrees(self._max_step_rad):.1f} deg", file=stream)
        flips = sum(len(v) for v in self.flips.values())
        print(f"branch flips {flips}  gated frames "
              f"{{{', '.join(f'{s}: {self.gated_frames[s]}' for s in SIDES)}}}  "
              f"frames with contact {self.frames_with_contact}  "
              f"solved with clearance rows {self.frames_constrained}  "
              f"failures {len(self.failures)} on {len({f.frame for f in self.failures})} frame(s)",
              file=stream)

    def as_dict(self) -> dict:
        return {
            "tool": TOOL_ID,
            "source": self.source,
            "options": self.options,
            "frames": self.frames,
            "rate_hz": self.rate_hz,
            "unwrap": {"wraps_removed": self.wraps, "wrist_drift": self.drift},
            "flips": {"detected": self.flips, "gated_frames": self.gated_frames},
            "ik": {
                "max_wrist_pos_residual_mm": round(1e3 * float(self._pos_err.max()), 4)
                if self.frames else 0.0,
                "max_wrist_ori_residual_deg": round(math.degrees(float(self._ori_err.max())), 4)
                if self.frames else 0.0,
                "max_elbow_residual_mm": round(1e3 * float(self._elbow_err.max()), 4)
                if self.frames else 0.0,
                "max_step_deg": round(math.degrees(self._max_step_rad), 4),
                "frames_not_converged": self._not_converged,
                "mean_iterations": round(float(np.mean(self._iterations)), 2)
                if self._iterations else 0.0,
                "frames_at_joint_limit": self._at_limit.tolist(),
                "frames_at_step_bound": self._at_step.tolist(),
            },
            "hand": {"clamped_values": self.hand_clamped},
            "collision": dict(self.collision, **{
                "frames_with_contact": self.frames_with_contact,
                "frames_constrained": self.frames_constrained,
                "failures": [f.as_dict() for f in self.failures],
                "near_miss": [u.as_dict() for u in
                              sorted(self.near_miss.values(), key=lambda u: u.worst_gap_mm)],
                "unfixable": [u.as_dict() for u in
                              sorted(self.unfixable.values(), key=lambda u: u.worst_gap_mm)],
            }),
            "verdict": {"failures": len(self.failures), "exit_code": self.exit_code},
        }
