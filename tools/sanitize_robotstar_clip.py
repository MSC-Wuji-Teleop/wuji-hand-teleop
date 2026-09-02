#!/usr/bin/env python3
"""Sanitize one RobotSTAR bundle trajectory for replay conditioning.

Reads a sample method dir (GT/ or Ours/), applies zero-phase low-pass
smoothing and a per-frame rate clamp to the 14 arm columns of body_q,
and writes a new method dir with the same layout that condition_clip
consumes (g1_reference/ npz + meta, hand2_input/ keypoints). Legs and
waist columns pass through untouched. Hand tracks in the npz pass
through untouched: condition_clip regenerates hands from the keypoints.

This fixes acceleration spikes and moderate frame-to-frame steps from
the bundle's unconstrained retargeting. It does NOT repair estimator
orientation flips (steps ~90 deg and above): smoothing one of those
sweeps the arm through the same wrong path more slowly. Clips with a
flip need the reference re-solved, not sanitized; the tool refuses them
unless --allow-flips is set.

Output is written outside the bundle tree so condition_clip records the
input hashes as out-of-bundle instead of failing them against
MANIFEST.sha256.

Usage:
    python3 tools/sanitize_robotstar_clip.py \
        --method-dir RobotSTAR_demos/samples/<sample>/Ours \
        --out-dir sanitized_clips [--cutoff-hz 6] [--max-step-deg 15] \
        [--trim-start N]
"""

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

ARM_PREFIXES = ('left_shoulder', 'left_elbow', 'left_wrist',
                'right_shoulder', 'right_elbow', 'right_wrist')
FLIP_STEP_RAD = np.radians(90.0)


def arm_columns(actuator_names):
    return [i for i, n in enumerate(actuator_names)
            if n.startswith(ARM_PREFIXES)]


def rate_clamp(q, max_step):
    """Forward then backward per-frame step clamp, per column."""
    out = q.copy()
    for i in range(1, len(out)):
        out[i] = out[i - 1] + np.clip(out[i] - out[i - 1], -max_step, max_step)
    for i in range(len(out) - 2, -1, -1):
        out[i] = out[i + 1] + np.clip(out[i] - out[i + 1], -max_step, max_step)
    return out


def sanitize(body_q, cols, fps, cutoff_hz, max_step):
    q = body_q.astype(np.float64).copy()
    b, a = butter(2, cutoff_hz / (fps / 2.0))
    q[:, cols] = filtfilt(b, a, q[:, cols], axis=0)
    q[:, cols] = rate_clamp(q[:, cols], max_step)
    return q


def stats(q, cols, fps):
    arm = q[:, cols]
    step = np.abs(np.diff(arm, axis=0))
    dq = np.gradient(arm, 1.0 / fps, axis=0)
    ddq = np.gradient(dq, 1.0 / fps, axis=0)
    return dict(max_step_deg=float(np.degrees(step.max())),
                peak_vel_rad_s=float(np.abs(dq).max()),
                peak_acc_rad_s2=float(np.abs(ddq).max()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--method-dir', required=True, type=Path)
    p.add_argument('--out-dir', required=True, type=Path)
    p.add_argument('--cutoff-hz', type=float, default=6.0)
    p.add_argument('--max-step-deg', type=float, default=15.0)
    p.add_argument('--trim-start', type=int, default=0,
                   help='drop this many leading frames before sanitizing')
    p.add_argument('--allow-flips', action='store_true',
                   help='sanitize even if a >=90 deg single-frame step is '
                        'present (the flip is smoothed, not repaired)')
    args = p.parse_args()

    ref_dir = args.method_dir / 'g1_reference'
    meta = json.loads((ref_dir / 'target_meta.json').read_text())
    names = meta['joint_actuator_order']['body_actuators']
    fps = float(meta['target_fps'])
    cols = arm_columns(names)
    if len(cols) != 14:
        sys.exit(f'expected 14 arm columns, found {len(cols)}')

    data = dict(np.load(ref_dir / 'controller_reference_v7.npz'))
    body_q = np.asarray(data['body_q'], dtype=np.float64)
    if args.trim_start:
        for k in ('body_q', 'body_dq', 'body_ddq', 'left_q', 'left_dq',
                  'left_ddq', 'right_q', 'right_dq', 'right_ddq'):
            data[k] = data[k][args.trim_start:]
        data['waypoint_indices'] = np.arange(len(data['body_q']))
        body_q = np.asarray(data['body_q'], dtype=np.float64)

    worst = np.abs(np.diff(body_q[:, cols], axis=0)).max()
    if worst >= FLIP_STEP_RAD and not args.allow_flips:
        sys.exit(f'refusing: {np.degrees(worst):.0f} deg single-frame step '
                 'is an orientation flip; re-solve the reference or pass '
                 '--allow-flips to smooth through it anyway')

    before = stats(body_q, cols, fps)
    q = sanitize(body_q, cols, fps, args.cutoff_hz,
                 np.radians(args.max_step_deg))
    after = stats(q, cols, fps)
    rmse = float(np.sqrt(np.mean((q[:, cols] - body_q[:, cols]) ** 2)))

    data['body_q'] = q.astype(np.float32)
    data['body_dq'] = np.gradient(q, 1.0 / fps, axis=0).astype(np.float32)
    data['body_ddq'] = np.gradient(
        np.asarray(data['body_dq'], dtype=np.float64), 1.0 / fps,
        axis=0).astype(np.float32)

    sample = args.method_dir.parent.name
    kind = args.method_dir.name
    out = args.out_dir / f'{sample}_{kind}'
    out_ref = out / 'g1_reference'
    out_ref.mkdir(parents=True, exist_ok=True)
    np.savez(out_ref / 'controller_reference_v7.npz', **data)
    shutil.copy2(ref_dir / 'target_meta.json', out_ref / 'target_meta.json')

    # motor_targets.csv is not read by condition_clip; rewrite the q
    # columns anyway so the artifact stays self-consistent.
    src_csv = ref_dir / 'motor_targets.csv'
    if src_csv.exists():
        rows = list(csv.reader(src_csv.open()))
        hdr, body = rows[0], rows[1 + args.trim_start:]
        for i, row in enumerate(body):
            row[:29] = [f'{v:.6f}' for v in q[i]]
        with (out_ref / 'motor_targets.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(body)

    out_kp = out / 'hand2_input'
    out_kp.mkdir(exist_ok=True)
    for f in (args.method_dir / 'hand2_input').glob('*_human_targets_v5.npz'):
        shutil.copy2(f, out_kp / f.name)

    report = dict(source=str(args.method_dir), frames=len(q),
                  trim_start=args.trim_start, cutoff_hz=args.cutoff_hz,
                  max_step_deg=args.max_step_deg, before=before, after=after,
                  arm_rmse_rad=rmse, tool='sanitize_robotstar_clip/1')
    (out / 'sanitize_report.json').write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
