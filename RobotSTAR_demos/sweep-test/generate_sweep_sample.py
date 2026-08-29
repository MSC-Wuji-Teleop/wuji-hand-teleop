#!/usr/bin/env python3
"""Generate the sweep-test sample in RobotSTAR bundle format.

Writes samples/90_sweep_joints/GT/ shaped exactly like a real sample's
method dir, so the sweep funnels through the SAME pipeline as every bundle
clip -- conditioning, gates, replay -- with no runner of its own (this
replaces the retired sweep_and_visualize.py flow):

    g1_reference/controller_reference_v7.npz   body_q [T, 29]
    g1_reference/target_meta.json              names/fps/timing metadata
    hand2_input/sweep_human_targets_v5.npz     21-point hand keypoints

One clip, two separated phases (arms only, stop, hands only):

  phase A  ARMS (hands hold a constant donor pose): the left arm's 7
           joints ramp together 0 -> A_j -> 0 as one small joint group
           (TUITION Stage B allows "one joint or one small joint group"),
           then the right arm's 7. Legs and waist stay exactly zero (the
           waist gate requires it).
  stop     a full-second neutral hold.
  phase B  HANDS (arms at zero): the THUMB keypoints blend toward a second
           donor frame and back -- left hand, hold, then right hand -- so
           the thumb visibly flexes through the PRODUCTION retargeter
           (bundle hands are keypoints by design; there is no joint-space
           hand track to author).

Why thumb-only, and why these fractions (all probed 2026-08-29 on the
sample-05 donor):
  - whole-hand blends press adjacent fingers into each other (the donor's
    sign pose holds fingers together; the collision audit flags 17
    finger-on-finger pairs up to 4.3 mm deep), while the thumb has lateral
    clearance -- thumb-only motion is contact-free;
  - the retargeter emits solver jumps whose FD stats do not shrink with
    slower input (full-frame blends: ~10-25 rad/s sustained, 700-1400
    rad/s^2 peak regardless of a 12-28 s cycle, vs the 4.0 rad/s /
    20 rad/s^2 deploy rows), so per-side blend targets/fractions are the
    largest that stay under the rows with ~2x margin: left thumb toward
    the max-displacement frame at 0.6 (accel 10.8 rad/s^2, 1.09 rad
    excursion), right thumb toward the median-displacement frame at 0.5
    (11.9 rad/s^2, 0.37 rad -- the max-displacement target crosses a
    solver boundary at any useful fraction).
Larger hand motion goes through the pipeline's existing single-joint path
instead (condition_clip --single-joint {left,right}_hand:<joint>).

Safety by construction, then verified independently:
  - arm amplitudes are capped (--arm-amplitude) and clipped 10% inside the
    URDF hardware position ranges; group ramp duration is sized so every
    joint's peak velocity/acceleration sit at or below `headroom` (0.5)
    times the deploy screening rows (g1_deploy_limits.yaml);
  - the verdict is still EARNED through condition_clip's audit (including
    the retargeted hands), and check_collisions.py (same folder) proves
    the conditioned artifact is kinematically self-collision-free.

Run on the HOST (RobotSTAR_demos is mounted read-only in the container),
then condition/replay in the teleop container -- see README.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

SWEEP_TEST_DIR = Path(__file__).resolve().parent
DEMOS_ROOT = SWEEP_TEST_DIR.parent

SAMPLE_NAME = '90_sweep_joints'
TARGET_FPS = 50.0
HEADROOM = 0.5          # ramp peaks at this fraction of the deploy rows
POSITION_MARGIN = 0.1   # keep 10% of the position range in reserve
HOLD_S = 0.5            # neutral hold between segments
STOP_S = 1.0            # the "stop" between the arm and hand phases
HAND_CYCLE_S = 6.0      # per-side hand blend cycle (0 -> frac -> 0)
# MediaPipe Hands 21-point ordering: indices 1-4 are the thumb chain.
THUMB_KEYPOINTS = [1, 2, 3, 4]
# Probed on the sample-05 donor (see module docstring): per-side blend
# target frame rule + fraction whose RETARGETED q20 stays under the hand
# deploy rows (4.0 rad/s, 20 rad/s^2) with ~2x margin.
HAND_BLEND = {'left': {'target': 'argmax', 'fraction': 0.6},
              'right': {'target': 'median', 'fraction': 0.5}}
GENERATOR_VERSION = 'sweep-test/4'


def group_ramp(num_cols: int, cols: list, amps: list, duration: float,
               fps: float) -> np.ndarray:
    """All `cols` ramp together 0 -> amp -> 0 over one shared duration."""
    num_frames = int(math.ceil(duration * fps)) + 1
    t = np.arange(num_frames) / fps
    profile = (1.0 - np.cos(2.0 * math.pi * np.minimum(t / duration, 1.0))) / 2.0
    q = np.zeros((num_frames, num_cols))
    for c, a in zip(cols, amps):
        q[:, c] = a * profile
    return q


def ramp_duration(amplitude: float, deploy_velocity: float,
                  deploy_acceleration: float, fps: float,
                  headroom: float) -> float:
    """Shortest raised-cosine duration keeping peak velocity A pi / T and
    peak acceleration 2 A pi^2 / T^2 at `headroom` times the deploy rows
    (same sizing law as replay/conditioning.py::single_joint_ramp)."""
    a = abs(float(amplitude))
    t_vel = a * math.pi / (headroom * deploy_velocity)
    t_acc = math.sqrt(2 * a * math.pi ** 2 / (headroom * deploy_acceleration))
    return max(t_vel, t_acc, 2.0 / fps)


def safe_amplitude(cap: float, lower: float, upper: float) -> float:
    """Signed amplitude toward the roomier side of [lower, upper] from
    neutral (q = 0), capped and kept POSITION_MARGIN inside the range."""
    room_up = max(0.0, float(upper)) * (1.0 - POSITION_MARGIN)
    room_down = max(0.0, -float(lower)) * (1.0 - POSITION_MARGIN)
    if room_up >= room_down:
        return min(cap, room_up)
    return -min(cap, room_down)


def load_arm_limits(path: Path) -> tuple:
    doc = yaml.safe_load(path.read_text())
    positions = {name: row['position']
                 for name, row in doc['hardware_ceilings'].items()}
    dep = doc['deploy']
    return positions, float(dep['velocity']), float(dep['acceleration'])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arm-amplitude', type=float, default=0.2,
                    help='per-joint arm ramp amplitude cap, rad (default 0.2)')
    ap.add_argument('--donor', type=Path, default=None,
                    help="donor sample method dir for the hand keypoints "
                         "(default: samples/05_*/GT; the blend fractions "
                         "were probed on this donor)")
    ap.add_argument('--arm-limits', type=Path, default=None,
                    help='override g1_deploy_limits.yaml path')
    args = ap.parse_args()

    donor = args.donor
    if donor is None:
        matches = sorted((DEMOS_ROOT / 'samples').glob('05_*/GT'))
        if not matches:
            raise SystemExit('no default donor sample 05_*/GT found; '
                             'pass --donor')
        donor = matches[0]
    donor_meta = json.loads(
        (donor / 'g1_reference' / 'target_meta.json').read_text())
    body_actuators = donor_meta['joint_actuator_order']['body_actuators']

    limits_path = args.arm_limits or (
        DEMOS_ROOT.parent / 'src' / 'output_devices' / 'g1_world_output'
        / 'config' / 'g1_deploy_limits.yaml')
    positions, dep_v, dep_a = load_arm_limits(limits_path)

    kp_files = sorted(donor.glob('hand2_input/*_human_targets_v5.npz'))
    if len(kp_files) != 1:
        raise SystemExit(f'expected one hand2_input npz in {donor}')
    donor_kp = np.load(kp_files[0])
    kp_base, kp_target = {}, {}
    for side in ('left', 'right'):
        kp = np.asarray(donor_kp[f'{side}_hand_keypoints21'], dtype=np.float64)
        disp = np.linalg.norm(kp - kp[0], axis=2).mean(axis=1)
        rule = HAND_BLEND[side]
        if rule['target'] == 'argmax':
            fb = int(np.argmax(disp))
        else:  # 'median'
            fb = int(np.argsort(disp)[len(disp) // 2])
        kp_base[side] = kp[0]
        # Thumb-only: everything but the thumb chain stays at the base
        # pose (see module docstring for why).
        target = kp[0].copy()
        target[THUMB_KEYPOINTS] = (
            kp[0][THUMB_KEYPOINTS]
            + rule['fraction'] * (kp[fb][THUMB_KEYPOINTS]
                                  - kp[0][THUMB_KEYPOINTS]))
        kp_target[side] = target

    ncols = len(body_actuators)
    col = {n: i for i, n in enumerate(body_actuators)}
    hold_frames = int(round(HOLD_S * TARGET_FPS))
    stop_frames = int(round(STOP_S * TARGET_FPS))
    hand_frames = int(round(HAND_CYCLE_S * TARGET_FPS))

    # ---- phase A: per-side 7-joint arm group ramps ----------------------
    blocks = [np.zeros((hold_frames, ncols))]
    segments = []
    amplitudes = {}
    for side in ('left', 'right'):
        names = [n for n in body_actuators if n.startswith(side)
                 and any(p in n for p in ('shoulder', 'elbow', 'wrist'))]
        amps, cols = [], []
        duration = 0.0
        for name in names:
            lo, hi = positions[name]
            amp = safe_amplitude(args.arm_amplitude, lo, hi)
            amplitudes[name] = round(float(amp), 4)
            amps.append(amp)
            cols.append(col[name])
            duration = max(duration, ramp_duration(amp, dep_v, dep_a,
                                                   TARGET_FPS, HEADROOM))
        start = sum(b.shape[0] for b in blocks)
        ramp = group_ramp(ncols, cols, amps, duration, TARGET_FPS)
        segments.append({'phase': 'arms', 'group': f'{side}_arm',
                         'joints': names,
                         'start_frame': start,
                         'end_frame': start + ramp.shape[0]})
        blocks.append(ramp)
        blocks.append(np.zeros((hold_frames, ncols)))

    # ---- the stop between phases ----------------------------------------
    blocks.append(np.zeros((stop_frames, ncols)))

    # ---- phase B: per-side hand keypoint blend cycles (arms zero) -------
    hand_windows = {}
    for side in ('left', 'right'):
        start = sum(b.shape[0] for b in blocks)
        hand_windows[side] = (start, start + hand_frames)
        segments.append({'phase': 'hands', 'group': f'{side}_hand',
                         'joints': f'thumb keypoint blend, '
                                   f'{HAND_BLEND[side]}',
                         'start_frame': start,
                         'end_frame': start + hand_frames})
        blocks.append(np.zeros((hand_frames, ncols)))
        blocks.append(np.zeros((hold_frames, ncols)))

    body_q = np.vstack(blocks)
    num_frames = body_q.shape[0]

    # Keypoints: constant base pose everywhere; inside each side's window,
    # a raised-cosine blend base -> target -> base.
    kp_out = {}
    for side in ('left', 'right'):
        arr = np.repeat(kp_base[side][None, :, :], num_frames, axis=0)
        a, b = hand_windows[side]
        t = np.arange(b - a) / TARGET_FPS
        s = (1.0 - np.cos(2.0 * math.pi * t / HAND_CYCLE_S)) / 2.0
        arr[a:b] = (kp_base[side][None]
                    + s[:, None, None]
                    * (kp_target[side] - kp_base[side])[None])
        kp_out[side] = arr

    # ---- write the sample ------------------------------------------------
    method_dir = SWEEP_TEST_DIR / 'samples' / SAMPLE_NAME / 'GT'
    (method_dir / 'g1_reference').mkdir(parents=True, exist_ok=True)
    (method_dir / 'hand2_input').mkdir(parents=True, exist_ok=True)

    npz_path = method_dir / 'g1_reference' / 'controller_reference_v7.npz'
    np.savez(npz_path, body_q=body_q,
             target_fps=np.float64(TARGET_FPS), time_scale=np.int64(1))

    dq = np.diff(body_q, axis=0) * TARGET_FPS
    meta = {
        'version': GENERATOR_VERSION,
        'generated': datetime.now(timezone.utc).isoformat(),
        'generator': 'RobotSTAR_demos/sweep-test/generate_sweep_sample.py',
        'donor_sample': str(donor.relative_to(DEMOS_ROOT)),
        'frames': int(num_frames),
        'target_fps': TARGET_FPS,
        'time_scale': 1,
        'joint_actuator_order': {'body_actuators': body_actuators},
        'max_arm_velocity_rad_s': float(np.abs(dq).max()),
        'safe_timing_at_requested_scale': True,
        'sweep': {
            'headroom': HEADROOM,
            'arm_amplitude_cap_rad': args.arm_amplitude,
            'position_margin': POSITION_MARGIN,
            'hold_s': HOLD_S,
            'stop_s': STOP_S,
            'hand_cycle_s': HAND_CYCLE_S,
            'hand_blend': HAND_BLEND,
            'thumb_keypoints': THUMB_KEYPOINTS,
            'arm_amplitudes_rad': amplitudes,
            'segments': segments,
        },
    }
    meta_path = method_dir / 'g1_reference' / 'target_meta.json'
    meta_path.write_text(json.dumps(meta, indent=1, sort_keys=True) + '\n')

    kp_path = method_dir / 'hand2_input' / 'sweep_human_targets_v5.npz'
    np.savez(kp_path, left_hand_keypoints21=kp_out['left'],
             right_hand_keypoints21=kp_out['right'])

    # Manifest so condition_clip's hash gate verifies these files (this
    # folder is its own bundle root).
    manifest_lines = []
    for p in (npz_path, meta_path, kp_path):
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        manifest_lines.append(f'{digest}  {p.relative_to(SWEEP_TEST_DIR)}')
    (SWEEP_TEST_DIR / 'MANIFEST.sha256').write_text(
        '\n'.join(manifest_lines) + '\n')

    dur = num_frames / TARGET_FPS
    print(f'wrote {method_dir}')
    print(f'  {num_frames} frames at {TARGET_FPS:.0f} fps = {dur:.1f} s')
    for seg in segments:
        t0 = seg['start_frame'] / TARGET_FPS
        t1 = seg['end_frame'] / TARGET_FPS
        print(f'  {t0:5.1f}-{t1:5.1f} s  {seg["phase"]:5} {seg["group"]}')
    print(f'  max arm |dq| = {meta["max_arm_velocity_rad_s"]:.3f} rad/s '
          f'(deploy row {dep_v}, headroom {HEADROOM})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
