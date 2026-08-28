"""Stage 0 assert: the arm command stream is piecewise-linear (spec_1).

Pure numpy. The container-side recorder (scripts/check_piecewise_linear.py)
captures /left,right_arm/joint_commands into an npz; this module judges it.

What the ZOH defect looked like: the control loop republished each 50 fps
stream sample unchanged for ~5 ticks at 250 Hz -- long runs of exactly
identical consecutive commands (plateaus) with a full stream step between
them. The D5 fix ramps every control tick, so during motion consecutive
commands differ by ~v * control_dt and plateaus only appear in deliberate
holds.

The analyzer therefore reports, over the MOVING segment of the recording
(deliberate holds are legitimate plateaus and are excluded):

    duplicate_fraction   consecutive-tick pairs with zero change anywhere
    max_tick_step        worst per-tick |dq| (jump detection)
    piecewise_linear     duplicate_fraction <= dup_threshold and
                         max_tick_step <= step_threshold

Tolerances derive from D5: with stream period T_s, control period T_c, and
per-joint stream step S, a healthy ramp advances S * T_c / T_s per tick, so
step_threshold = vel_limit * T_c with margin; a dropped frame at most
doubles one tick's step (still clipped by the safety chain downstream).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class StreamCheck:
    piecewise_linear: bool
    duplicate_fraction: float
    max_tick_step_rad: float
    moving_ticks: int
    total_ticks: int
    reasons: list


def analyze_command_stream(
    t: np.ndarray,
    q: np.ndarray,
    vel_limit_rad_s: float,
    dup_threshold: float = 0.10,
    step_margin: float = 2.5,
    move_eps_rad: float = 1e-6,
) -> StreamCheck:
    """t: (N,) receive times; q: (N, J) commanded positions.

    step_margin covers discretization plus one dropped stream frame (2x).
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if t.ndim != 1 or q.ndim != 2 or q.shape[0] != t.size or t.size < 10:
        raise ValueError(f'need (N,) and (N, J) with N >= 10, got {t.shape} {q.shape}')

    dq = np.abs(np.diff(q, axis=0))
    dt = np.diff(t)
    changed = dq.max(axis=1) > move_eps_rad

    # Moving segment: between the first and last change (holds at both ends
    # are deliberate: pre-start hold, end hold).
    idx = np.flatnonzero(changed)
    reasons = []
    if idx.size == 0:
        return StreamCheck(False, 1.0, 0.0, 0, int(t.size),
                           ['stream never moved; nothing to judge'])
    lo, hi = idx[0], idx[-1] + 1
    seg_changed = changed[lo:hi]
    seg_dq = dq[lo:hi]
    seg_dt = dt[lo:hi]

    duplicate_fraction = float(1.0 - seg_changed.mean())
    max_step = float(seg_dq.max())

    median_dt = float(np.median(seg_dt))
    step_threshold = vel_limit_rad_s * median_dt * step_margin

    if duplicate_fraction > dup_threshold:
        reasons.append(
            f'duplicate_fraction {duplicate_fraction:.2f} > {dup_threshold} '
            f'-- zero-order-hold plateaus inside the moving segment'
        )
    if max_step > step_threshold:
        reasons.append(
            f'max per-tick step {max_step:.4f} rad > {step_threshold:.4f} '
            f'({step_margin}x vel_limit*median_dt) -- stepping, not ramping'
        )

    return StreamCheck(
        piecewise_linear=not reasons,
        duplicate_fraction=duplicate_fraction,
        max_tick_step_rad=max_step,
        moving_ticks=int(hi - lo),
        total_ticks=int(t.size),
        reasons=reasons,
    )
