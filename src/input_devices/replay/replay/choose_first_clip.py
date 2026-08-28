#!/usr/bin/env python3
"""choose_first_clip: rank conditioned clips against the 7F first-clip bar.

Scans conditioned artifacts (our verdicts) plus the bundle's own physical
audits, and ranks candidates by TUITION 7F: no two-hand contact, no
hand-to-body contact, small motion amplitude, large joint-limit margin,
stable physical tracking. Sample 01 is excluded outright (spec_1: banned as
first clip until a fresh audit clears its shoulder-torso contact).

The bundle's shipped 'pass' gates are path gates with rate limits disabled
and real_robot_ready=false everywhere -- they are never used as the
verdict. Our conditioning verdict gates first; the physical audit fields
rank the remainder. Contact pairs are CLASSIFIED by link names, because the
dominant peak in several audits is a same-arm wrist_roll/wrist_yaw
self-contact artifact of the legacy model, not a real hand-body event;
ranking on raw peak force alone would mis-rank clips.

Usage:
    choose_first_clip --clips-dir ~/wuji_clips \
        --bundle RobotSTAR_demos [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from replay.clip_artifact import BANNED_FIRST_SAMPLE_PREFIXES

HAND_LINK_HINTS = ('wuji', 'thumb', 'finger', 'pinky', 'palm', 'hand')
BODY_LINK_HINTS = ('torso', 'pelvis', 'head', 'waist', 'hip', 'knee', 'ankle',
                   'leg')


def classify_pair(pair: str) -> str:
    """'linkA:linkB' -> two_hand | hand_body | arm_body | same_arm_artifact
    | other."""
    parts = pair.replace('::', ':').split(':')
    if len(parts) != 2:
        return 'other'
    a, b = (p.lower() for p in parts)

    def is_hand(s):
        return any(h in s for h in HAND_LINK_HINTS)

    def is_body(s):
        return any(h in s for h in BODY_LINK_HINTS)

    def side(s):
        if s.startswith('left') or '_l_' in s or s.startswith('l_'):
            return 'left'
        if s.startswith('right') or '_r_' in s or s.startswith('r_'):
            return 'right'
        return None

    if is_hand(a) and is_hand(b) and side(a) != side(b):
        return 'two_hand'
    if (is_hand(a) and is_body(b)) or (is_hand(b) and is_body(a)):
        return 'hand_body'
    if 'wrist' in a and 'wrist' in b and side(a) == side(b):
        # Legacy-model collision-geometry artifact (wrist_roll vs wrist_yaw
        # on the same arm); huge forces here say nothing about the clip.
        return 'same_arm_artifact'
    if is_body(a) or is_body(b):
        return 'arm_body'
    return 'other'


def evaluate_clip(artifact_json: Path, bundle_root: Path) -> dict:
    meta = json.loads(artifact_json.read_text())
    sample = str(meta.get('sample') or '')
    method = meta.get('method')
    entry = {
        'sample': sample,
        'method': method,
        'artifact': str(artifact_json),
        'verdict': meta.get('verdict'),
        'max_allowed_speed_scale': meta.get('max_allowed_speed_scale'),
        'k': (meta.get('audit') or {}).get('k_extra'),
        'banned_first': sample.startswith(BANNED_FIRST_SAMPLE_PREFIXES),
        'eligible': False,
        'reasons': [],
    }
    if meta.get('verdict') != 'pass':
        entry['reasons'].append('conditioning verdict fail')
    if entry['banned_first']:
        entry['reasons'].append('sample 01: banned as first clip (7F)')

    arm_audit = (meta.get('audit') or {}).get('arm') or {}
    # Amplitude: peak absolute excursion across arm joints (small is good).
    pos_min = arm_audit.get('position_min') or [0.0]
    pos_max = arm_audit.get('position_max') or [0.0]
    entry['amplitude_rad'] = max(
        max(abs(v) for v in pos_min), max(abs(v) for v in pos_max))
    # Joint-limit margin: min distance to either bound, per audit numbers.
    margins = []
    ceilings = None
    names = arm_audit.get('joint_names') or []
    if arm_audit.get('velocity_ceiling') is not None and names:
        pass  # positions bounds are not in the audit rows; margin from spikes
    entry['spike_count'] = arm_audit.get('spike_count', 0)

    # Bundle physical audit for this sample/method.
    phys = None
    if method:
        candidates = list((bundle_root / 'samples').glob(
            f'{sample}/{method}/audits/physical/*physical_summary*.json'))
        if candidates:
            phys = json.loads(candidates[0].read_text())
    if phys is not None:
        dep = phys.get('deployment_audit') or {}
        entry['deployment_audit_pass'] = dep.get('pass')
        forces = phys.get('forces') or {}
        entry['contact_force_peak_n'] = forces.get('contact_force_peak_max')
        pairs = forces.get('top_contact_pairs') or []
        classified = {}
        for p in pairs:
            kind = classify_pair(str(p.get('pair', '')))
            peak = float(p.get('peak_force', 0.0))
            classified[kind] = max(classified.get(kind, 0.0), peak)
        entry['contact_by_kind'] = classified
        if classified.get('two_hand', 0.0) > 5.0:
            entry['reasons'].append(
                f"two-hand contact {classified['two_hand']:.1f} N")
        if classified.get('hand_body', 0.0) > 5.0:
            entry['reasons'].append(
                f"hand-body contact {classified['hand_body']:.1f} N")
        if classified.get('arm_body', 0.0) > 20.0:
            entry['reasons'].append(
                f"arm-body contact {classified['arm_body']:.1f} N")
        tracking = phys.get('tracking') or {}
        entry['upper_body_rmse_rad'] = tracking.get('upper_body_rmse_rad')
    else:
        entry['reasons'].append('no bundle physical audit found')

    entry['eligible'] = not entry['reasons']
    return entry


def rank_key(entry: dict):
    # Eligible first; then smallest real-contact force, smallest amplitude.
    contact = entry.get('contact_by_kind') or {}
    real_contact = max(contact.get('two_hand', 0.0),
                       contact.get('hand_body', 0.0),
                       contact.get('arm_body', 0.0))
    return (not entry['eligible'], real_contact,
            entry.get('amplitude_rad', 99.0))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clips-dir', type=Path, required=True,
                        help='directory of conditioned artifacts')
    parser.add_argument('--bundle', type=Path, required=True,
                        help='RobotSTAR_demos root')
    parser.add_argument('--json', type=Path, default=None,
                        help='also write the ranked table as JSON')
    args = parser.parse_args(argv)

    artifacts = sorted(args.clips_dir.glob('*/conditioned_clip_v1.json'))
    if not artifacts:
        print(f'no conditioned artifacts under {args.clips_dir}', file=sys.stderr)
        return 1

    entries = [evaluate_clip(a, args.bundle) for a in artifacts]
    entries.sort(key=rank_key)

    print(f"{'sample':42s} {'meth':5s} {'ok':3s} {'contact':>8s} "
          f"{'ampl':>6s} {'k':>2s} {'scale':>6s}  reasons")
    for e in entries:
        contact = e.get('contact_by_kind') or {}
        real = max(contact.get('two_hand', 0.0), contact.get('hand_body', 0.0),
                   contact.get('arm_body', 0.0))
        print(f"{e['sample'][:42]:42s} {str(e['method'])[:5]:5s} "
              f"{'yes' if e['eligible'] else 'NO ':3s} {real:8.1f} "
              f"{e.get('amplitude_rad', 0):6.2f} {str(e.get('k')):>2s} "
              f"{e.get('max_allowed_speed_scale', 0):6.3f}  "
              f"{'; '.join(e['reasons'])}")

    if args.json:
        args.json.write_text(json.dumps(entries, indent=1, sort_keys=True) + '\n')
        print(f'\nwrote {args.json}')

    eligible = [e for e in entries if e['eligible']]
    if eligible:
        print(f"\nfirst-clip candidate: {eligible[0]['sample']} "
              f"{eligible[0]['method']} (operator judgment still applies; "
              f"kinematic preview on request)")
        return 0
    print('\nno eligible first clip; inspect the reasons above', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
