"""Offline conditioning core: audits, k selection, verdict (spec_1 comp. 1).

Pure numpy + the two limits loaders. No ROS, no retargeter import (the hand
retarget lives in hand_pipeline.py behind a lazy import) -- this module is
unit-testable in a bare numpy venv.

Grids. The bundle ships frames on the 50 fps target grid with its own baked
time_scale k_bundle (1 for all 30 shipped clips). Our integer redistribution
k_extra stretches the PLAY tick to dt_play = k_total / target_fps with
k_total = k_bundle * k_extra: waypoints untouched, exactly the bundle's own
retiming mechanism (section 7E compliant). Audits therefore exist on two
grids and every stat records which one it used:

    native grid  dt = k_bundle / fps    (what the bundle would play at)
    play grid    dt = k_total / fps     (what we will play at)

Velocities on the play grid are native / k_extra; accelerations divide by
k_extra squared.

Spikes are never smoothed and never enter the k computation: a frame whose
native-grid FD velocity exceeds the per-joint HARDWARE ceiling is a hard
violation of a sourced limit (branch-flip class; measured fact 3), and
section 7E is explicit that slowing down does not fix it. Spikes fail the
verdict; sustained overspeed (per-joint p99.5 vs the DEPLOY rows) is what k
fixes.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from replay.clip_artifact import CANONICAL_ARM_JOINTS

WAIST_JOINTS = ('waist_yaw', 'waist_roll', 'waist_pitch')

# Percentile defining "sustained" speed, from the bundle's own audit
# methodology (measured fact 3 uses p99.5 of |dq|).
SUSTAINED_PERCENTILE = 99.5

# A conditioned clip never advertises a speed_scale above recorded speed.
MAX_SPEED_SCALE = 1.0

# Waist columns are asserted zero (measured fact 1); anything above this is
# clip content this pipeline does not support (waist is uncommanded).
WAIST_ZERO_TOL_RAD = 1e-6

# Cap on recorded spike events so a pathological clip cannot bloat the JSON.
MAX_RECORDED_SPIKES = 100


def extract_arm_q(body_q: np.ndarray, actuator_names: Sequence[str]) -> np.ndarray:
    """Select the 14 arm columns from body_q by name, canonical order.

    Mapping is by joint name only, never by array index (TUITION 3.2).
    """
    body_q = np.asarray(body_q, dtype=np.float64)
    name_to_col = {n: i for i, n in enumerate(actuator_names)}
    missing = [n for n in CANONICAL_ARM_JOINTS if n not in name_to_col]
    if missing:
        raise ValueError(
            f"bundle actuator list lacks arm joints {missing}; cannot "
            "extract by name"
        )
    cols = [name_to_col[n] for n in CANONICAL_ARM_JOINTS]
    return body_q[:, cols]


def waist_motion(body_q: np.ndarray, actuator_names: Sequence[str]) -> dict:
    """Max |q| per waist joint. All three must be ~0 (measured fact 1)."""
    body_q = np.asarray(body_q, dtype=np.float64)
    name_to_col = {n: i for i, n in enumerate(actuator_names)}
    out = {}
    for name in WAIST_JOINTS:
        if name in name_to_col:
            out[name] = float(np.max(np.abs(body_q[:, name_to_col[name]])))
        else:
            out[name] = None  # 23-DoF-style bundle without that column
    return out


def _fd(q: np.ndarray, dt: float) -> np.ndarray:
    return np.diff(np.asarray(q, dtype=np.float64), axis=0) / float(dt)


def _per_joint(arr: np.ndarray, reducer) -> list:
    return [float(reducer(np.abs(arr[:, j]))) for j in range(arr.shape[1])]


def audit_tracks(
    q: np.ndarray,
    names: Sequence[str],
    pos_lower: np.ndarray,
    pos_upper: np.ndarray,
    vel_ceiling: Optional[np.ndarray],
    deploy_velocity: np.ndarray,
    deploy_acceleration: np.ndarray,
    native_dt: float,
    k_extra: int = 1,
) -> dict:
    """Audit one joint group (arms or one hand) on both grids.

    vel_ceiling of None disables the spike check (hands: the URDF velocity
    is a sim-model number, not a sourced ceiling -- the section-6 deviation
    caps hands with the deploy row only).
    Returns a JSON-ready dict; caller assembles verdict reasons from it.
    """
    q = np.asarray(q, dtype=np.float64)
    t, j = q.shape
    if t < 2:
        raise ValueError(f"need at least 2 frames, got {t}")

    finite = bool(np.isfinite(q).all())

    pos_low_viol = q < pos_lower[None, :]
    pos_high_viol = q > pos_upper[None, :]
    pos_violations = int(np.sum(pos_low_viol | pos_high_viol))

    v_native = _fd(q, native_dt)
    a_native = _fd(v_native, native_dt)

    spikes = []
    spike_count = 0
    if vel_ceiling is not None:
        over = np.abs(v_native) > vel_ceiling[None, :]
        idx_t, idx_j = np.nonzero(over)
        spike_count = int(idx_t.size)
        for ti, ji in list(zip(idx_t.tolist(), idx_j.tolist()))[:MAX_RECORDED_SPIKES]:
            spikes.append({
                'frame': ti,
                'joint': names[ji],
                'dq_rad': float(q[ti + 1, ji] - q[ti, ji]),
                'v_native_rad_s': float(v_native[ti, ji]),
                'ceiling_rad_s': float(vel_ceiling[ji]),
            })

    play_dt = native_dt * k_extra
    v_play = v_native / k_extra
    a_play = a_native / (k_extra ** 2)

    return {
        'finite': finite,
        'num_frames': t,
        'native_dt_s': float(native_dt),
        'play_dt_s': float(play_dt),
        'k_extra': int(k_extra),
        'joint_names': list(names),
        'position_violations': pos_violations,
        'position_min': _per_joint_signed(q, np.min),
        'position_max': _per_joint_signed(q, np.max),
        'native': {
            'peak_velocity': _per_joint(v_native, np.max),
            'sustained_velocity_p99_5': [
                float(np.percentile(np.abs(v_native[:, jj]), SUSTAINED_PERCENTILE))
                for jj in range(j)
            ],
            'peak_acceleration': _per_joint(a_native, np.max) if a_native.size else [0.0] * j,
        },
        'play': {
            'peak_velocity': _per_joint(v_play, np.max),
            'peak_acceleration': _per_joint(a_play, np.max) if a_play.size else [0.0] * j,
        },
        'deploy_velocity': [float(x) for x in deploy_velocity],
        'deploy_acceleration': [float(x) for x in deploy_acceleration],
        'velocity_ceiling': [float(x) for x in vel_ceiling] if vel_ceiling is not None else None,
        'spike_count': spike_count,
        'spikes': spikes,
    }


def _per_joint_signed(arr: np.ndarray, reducer) -> list:
    return [float(reducer(arr[:, j])) for j in range(arr.shape[1])]


def choose_k_extra(
    audits: Sequence[dict],
    k_max: int,
) -> tuple:
    """Integer redistribution factor from sustained speeds and accelerations.

    k fixes sustained overspeed only: per group, per joint,
    k_v = ceil(sustained_p99.5 / deploy_velocity) and
    k_a = ceil(sqrt(peak_native_accel_p-agnostic / deploy_acceleration)).
    Uses each audit's NATIVE stats. Returns (k_extra, capped: bool).
    """
    k = 1
    for audit in audits:
        sus = audit['native']['sustained_velocity_p99_5']
        dep_v = audit['deploy_velocity']
        acc = audit['native']['peak_acceleration']
        dep_a = audit['deploy_acceleration']
        for s, dv in zip(sus, dep_v):
            if s > dv:
                k = max(k, math.ceil(s / dv))
        for a, da in zip(acc, dep_a):
            if a > da:
                k = max(k, math.ceil(math.sqrt(a / da)))
    capped = k > k_max
    return (min(k, k_max), capped)


def allowed_speed_scale(audits: Sequence[dict]) -> float:
    """Largest speed_scale keeping PLAY-grid FD peaks inside the deploy rows.

    Velocity scales linearly with speed_scale, acceleration quadratically.
    Capped at MAX_SPEED_SCALE (never advertise faster-than-recorded).
    Consumers (load gate, Stage E ladder) read this field and never
    re-derive it.
    """
    scale = MAX_SPEED_SCALE
    for audit in audits:
        for pv, dv in zip(audit['play']['peak_velocity'], audit['deploy_velocity']):
            if pv > 0:
                scale = min(scale, dv / pv)
        for pa, da in zip(audit['play']['peak_acceleration'], audit['deploy_acceleration']):
            if pa > 0:
                scale = min(scale, math.sqrt(da / pa))
    return float(scale)


def collect_verdict_reasons(
    arm_audit: dict,
    hand_audits: Optional[dict],
    waist: dict,
    k_capped: bool,
    k_max: int,
    manifest_mismatches: Sequence[str] = (),
) -> list:
    """Assemble the fail reasons. Empty list means verdict pass."""
    reasons = []
    if not arm_audit['finite']:
        reasons.append('arm trajectory contains non-finite values')
    if arm_audit['position_violations']:
        reasons.append(
            f"arm position exceeds hardware ceilings in "
            f"{arm_audit['position_violations']} samples"
        )
    if arm_audit['spike_count']:
        worst = max(arm_audit['spikes'], key=lambda s: abs(s['v_native_rad_s']),
                    default=None)
        detail = (f" (worst {worst['v_native_rad_s']:.1f} rad/s on "
                  f"{worst['joint']} vs ceiling {worst['ceiling_rad_s']:.0f})"
                  if worst else '')
        reasons.append(
            f"{arm_audit['spike_count']} single-frame velocity spikes above "
            f"the sourced hardware ceiling{detail}: branch-flip class; "
            f"retiming does not fix these (section 7E) -- regenerate "
            f"upstream or exclude wrists"
        )
    if k_capped:
        reasons.append(
            f"sustained overspeed needs k above the configured maximum {k_max}"
        )
    for name, mag in waist.items():
        if mag is not None and mag > WAIST_ZERO_TOL_RAD:
            reasons.append(
                f"{name} moves (max |q| = {mag:.4g} rad); the waist is "
                f"uncommanded in this pipeline (slot policy) -- clip "
                f"content out of scope"
            )
    if hand_audits is not None:
        for side, audit in hand_audits.items():
            if not audit['finite']:
                reasons.append(f'{side} hand q20 contains non-finite values')
            if audit['position_violations']:
                reasons.append(
                    f"{side} hand q20 exceeds URDF position limits in "
                    f"{audit['position_violations']} samples"
                )
    for m in manifest_mismatches:
        reasons.append(f"input hash mismatch against MANIFEST.sha256: {m}")
    return reasons


# --------------------------------------------------------------- generators

def single_joint_ramp(
    num_joints: int,
    joint_index: int,
    amplitude: float,
    deploy_velocity: float,
    deploy_acceleration: float,
    fps: float,
    base_pose: Optional[np.ndarray] = None,
    headroom: float = 0.5,
) -> np.ndarray:
    """Stage B single-joint clip: one raised-cosine ramp 0 -> A -> 0.

    Everything except joint_index holds base_pose (zeros by default; the
    device approach phase owns getting there from measured). The profile
    duration is chosen so peak velocity and acceleration sit at `headroom`
    times the deploy caps: q(t) = A(1-cos(2 pi t/T))/2 has peak velocity
    A pi / T and peak acceleration 2 A pi^2 / T^2.
    """
    if not (0 < headroom <= 1):
        raise ValueError(f"headroom must be in (0, 1], got {headroom}")
    if amplitude == 0:
        raise ValueError("amplitude must be nonzero")
    a = abs(float(amplitude))
    t_vel = a * math.pi / (headroom * deploy_velocity)
    t_acc = math.sqrt(2 * a * math.pi ** 2 / (headroom * deploy_acceleration))
    duration = max(t_vel, t_acc, 2.0 / fps)
    num_frames = int(math.ceil(duration * fps)) + 1
    t = np.arange(num_frames) / fps
    profile = amplitude * (1.0 - np.cos(2.0 * math.pi * np.minimum(t / duration, 1.0))) / 2.0
    q = np.tile(
        np.zeros(num_joints) if base_pose is None else np.asarray(base_pose, dtype=float),
        (num_frames, 1),
    )
    q[:, joint_index] += profile
    return q
