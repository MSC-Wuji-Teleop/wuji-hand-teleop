#!/usr/bin/env python3
"""condition_clip: bundle sample -> audited conditioned clip artifact.

spec_1 component 1. Offline, deterministic (same inputs, same output
hashes), runs in the teleop container (the hand retarget needs
wuji_retargeting; everything else is numpy/scipy).

Bundle mode:
    condition_clip --method-dir RobotSTAR_demos/samples/<s>/GT \
        --out-dir ~/wuji_clips [--no-hands] [--k-max 8]

Single-joint mode (Stage B artifacts, same schema, same audit, same load
path -- the supervisor grows no second motion interface):
    condition_clip --single-joint arm:left_elbow --amplitude 0.2 \
        --out-dir ~/wuji_clips
    condition_clip --single-joint left_hand:thumb_cmc_flex --amplitude 0.3 \
        --out-dir ~/wuji_clips

Exit codes: 0 = artifact written with verdict pass, 2 = artifact written
with verdict fail, 1 = error (nothing usable written).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from replay.clip_artifact import (
    CANONICAL_ARM_JOINTS,
    NUM_ARM_JOINTS,
    NUM_HAND_JOINTS,
    SCHEMA_VERSION,
    VERDICT_FAIL,
    VERDICT_PASS,
    save_artifact,
)
from replay.conditioning import (
    allowed_speed_scale,
    audit_tracks,
    choose_k_extra,
    collect_verdict_reasons,
    extract_arm_q,
    single_joint_ramp,
    waist_motion,
)
from replay.hand_pipeline import (
    retarget_clip,
    retargeter_provenance,
    retime_to_grid,
    sha256_file,
)

TOOL_VERSION = 'condition_clip/1'
DEFAULT_K_MAX = 8


# ------------------------------------------------------------- resolution

def _resolve_pkg_config(package: str, filename: str) -> Path:
    """ament share dir first, then the source tree relative to this file.

    The source-tree fallback matters offline: conditioning may run where
    only the repo checkout exists (no sourced workspace).
    """
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
        try:
            p = Path(get_package_share_directory(package)) / 'config' / filename
            if p.exists():
                return p
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    src = Path(__file__).resolve().parents[3]  # .../src
    for sub in ('output_devices', 'input_devices', ''):
        p = src / sub / package / 'config' / filename
        if p.exists():
            return p
    raise FileNotFoundError(
        f"cannot resolve {package}/config/{filename} via ament or the "
        f"source tree under {src}"
    )


def _find_bundle_root(method_dir: Path) -> Optional[Path]:
    for parent in method_dir.resolve().parents:
        if (parent / 'MANIFEST.sha256').exists():
            return parent
    return None


def _load_manifest(bundle_root: Path) -> dict:
    entries = {}
    for line in (bundle_root / 'MANIFEST.sha256').read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        digest, _, rel = line.partition('  ')
        if digest and rel:
            entries[rel] = digest
    return entries


def _hash_inputs(paths: list, bundle_root: Optional[Path]) -> tuple:
    """Hash each input; verify against the manifest when the file is in it.

    Returns ({relpath_or_path: {sha256, manifest_match}}, [mismatch names]).
    manifest_match is True/False for bundle files, None for out-of-bundle
    files (e.g. retarget configs), which are recorded but not verifiable.
    """
    manifest = _load_manifest(bundle_root) if bundle_root else {}
    out, mismatches = {}, []
    for p in paths:
        p = Path(p).resolve()
        digest = sha256_file(p)
        rel = None
        if bundle_root is not None:
            try:
                rel = str(p.relative_to(bundle_root.resolve()))
            except ValueError:
                rel = None
        key = rel if rel is not None else str(p)
        match = None
        if rel is not None and rel in manifest:
            match = manifest[rel] == digest
            if not match:
                mismatches.append(rel)
        out[key] = {'sha256': digest, 'manifest_match': match}
    return out, mismatches


# ------------------------------------------------------------ bundle mode

def condition_bundle_sample(
    method_dir: Path,
    out_dir: Path,
    arm_limits_path: Path,
    hand_limits_path: Path,
    k_max: int,
    hands: bool,
    retarget_configs: Optional[dict] = None,
    retargeter_factory=None,
) -> tuple:
    """Returns (artifact base path, verdict). Raises on unusable input."""
    from g1_world_output.replay_safety import ArmLimits
    from wujihand_output.hand_safety import HandLimits

    meta_path = method_dir / 'g1_reference' / 'target_meta.json'
    npz_path = method_dir / 'g1_reference' / 'controller_reference_v7.npz'
    if not meta_path.exists():
        raise FileNotFoundError(
            f"{meta_path} not found; --method-dir must be a sample's GT/ or "
            "Ours/ directory"
        )
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    target_fps = float(meta['target_fps'])
    k_bundle = int(meta.get('time_scale', 1))
    actuator_names = meta['joint_actuator_order']['body_actuators']

    data = np.load(npz_path)
    body_q = np.asarray(data['body_q'], dtype=np.float64)
    if body_q.shape[1] != len(actuator_names):
        raise ValueError(
            f"body_q has {body_q.shape[1]} columns, meta names "
            f"{len(actuator_names)} -- npz/meta mismatch"
        )
    num_frames = body_q.shape[0]
    native_dt = k_bundle / target_fps

    arm_q = extract_arm_q(body_q, actuator_names)
    waist = waist_motion(body_q, actuator_names)

    arm_limits = ArmLimits.from_yaml(arm_limits_path, CANONICAL_ARM_JOINTS)
    hand_limits = HandLimits.from_yaml(hand_limits_path)

    input_paths = [meta_path, npz_path]
    retarget_meta = None
    hand_q = {}
    if hands:
        kp_candidates = sorted(method_dir.glob('hand2_input/*_human_targets_v5.npz'))
        if len(kp_candidates) != 1:
            raise FileNotFoundError(
                f"expected one hand2_input/*_human_targets_v5.npz in "
                f"{method_dir}, found {[p.name for p in kp_candidates]}"
            )
        kp_path = kp_candidates[0]
        input_paths.append(kp_path)
        kp_data = np.load(kp_path)

        configs = retarget_configs or {
            side: _resolve_pkg_config(
                'wujihand_output', f'retarget_keypoints_topic_{side}.yaml'
            )
            for side in ('left', 'right')
        }
        retarget_meta = {}
        for side in ('left', 'right'):
            kp = np.asarray(kp_data[f'{side}_hand_keypoints21'], dtype=np.float64)
            q_src = retarget_clip(kp, side, str(configs[side]),
                                  retargeter_factory=retargeter_factory)
            hand_q[side] = retime_to_grid(q_src, num_frames)
            retarget_meta[side] = retargeter_provenance(str(configs[side]))
    else:
        hand_q = {side: np.zeros((num_frames, NUM_HAND_JOINTS))
                  for side in ('left', 'right')}

    # Preliminary audits (k_extra=1) pick k; final audits record play stats.
    def _arm_audit(k_extra):
        return audit_tracks(
            arm_q, CANONICAL_ARM_JOINTS,
            arm_limits.pos_lower, arm_limits.pos_upper,
            arm_limits.vel_ceiling,
            arm_limits.deploy_velocity, arm_limits.deploy_acceleration,
            native_dt, k_extra,
        )

    def _hand_audit(side, k_extra):
        return audit_tracks(
            hand_q[side], hand_limits.side_names(side),
            hand_limits.pos_lower, hand_limits.pos_upper,
            None,  # no sourced velocity ceiling for the hand (section-6 deviation)
            hand_limits.deploy_velocity, hand_limits.deploy_acceleration,
            native_dt, k_extra,
        )

    prelim = [_arm_audit(1)]
    if hands:
        prelim += [_hand_audit(s, 1) for s in ('left', 'right')]
    k_extra, k_capped = choose_k_extra(prelim, k_max)

    arm_audit = _arm_audit(k_extra)
    hand_audits = {s: _hand_audit(s, k_extra) for s in ('left', 'right')} if hands else None

    audits_for_scale = [arm_audit] + (list(hand_audits.values()) if hand_audits else [])
    scale = allowed_speed_scale(audits_for_scale)

    input_sha, mismatches = _hash_inputs(input_paths, _find_bundle_root(method_dir))
    reasons = collect_verdict_reasons(
        arm_audit, hand_audits, waist, k_capped, k_max, mismatches,
    )
    verdict = VERDICT_PASS if not reasons else VERDICT_FAIL

    sample = method_dir.parent.name
    method = method_dir.name
    artifact_meta = {
        'schema_version': SCHEMA_VERSION,
        'sample': sample,
        'method': method,
        'source_dir': str(method_dir),
        'input_sha256': input_sha,
        'retargeter': retarget_meta,
        'limits': {
            'arm': {'path': str(arm_limits_path)},
            'hand': {'path': str(hand_limits_path)},
        },
        'audit': {
            'arm': arm_audit,
            'hands': hand_audits,
            'waist_max_abs_rad': waist,
            'k_bundle': k_bundle,
            'k_extra': k_extra,
        },
        'max_allowed_speed_scale': scale,
        'verdict': verdict,
        'verdict_reasons': reasons,
        'hands_conditioned': bool(hands),
        'first_frame': _frame_pose(arm_q, hand_q, 0),
        'last_frame': _frame_pose(arm_q, hand_q, -1),
        'tool_version': TOOL_VERSION,
    }

    base = Path(out_dir) / f'{sample}_{method}' / 'conditioned_clip_v1'
    save_artifact(base, arm_q, hand_q['left'], hand_q['right'],
                  target_fps, k_bundle * k_extra, artifact_meta)
    return base, verdict


def _frame_pose(arm_q, hand_q, idx) -> dict:
    return {
        'arm_q': [float(x) for x in arm_q[idx]],
        'left_hand_q20': [float(x) for x in hand_q['left'][idx]],
        'right_hand_q20': [float(x) for x in hand_q['right'][idx]],
    }


# ------------------------------------------------------- single-joint mode

def condition_single_joint(
    spec: str,
    amplitude: float,
    out_dir: Path,
    arm_limits_path: Path,
    hand_limits_path: Path,
    fps: float = 50.0,
    headroom: float = 0.5,
) -> tuple:
    """Stage B artifact: one slow raised-cosine ramp on one joint.

    spec is 'arm:<joint>' or '{left,right}_hand:<suffix-name>'. Audited by
    the same audit path as bundle clips, so the pass verdict is earned, not
    assumed.
    """
    from g1_world_output.replay_safety import ArmLimits
    from wujihand_output.hand_safety import HandLimits

    group, _, joint = spec.partition(':')
    if not joint:
        raise ValueError(f"--single-joint needs group:joint, got {spec!r}")
    arm_limits = ArmLimits.from_yaml(arm_limits_path, CANONICAL_ARM_JOINTS)
    hand_limits = HandLimits.from_yaml(hand_limits_path)

    if group == 'arm':
        if joint not in CANONICAL_ARM_JOINTS:
            raise ValueError(f"unknown arm joint {joint!r}; know {CANONICAL_ARM_JOINTS}")
        j = CANONICAL_ARM_JOINTS.index(joint)
        arm_q = single_joint_ramp(
            NUM_ARM_JOINTS, j, amplitude,
            float(arm_limits.deploy_velocity[j]),
            float(arm_limits.deploy_acceleration[j]),
            fps, headroom=headroom,
        )
        num_frames = arm_q.shape[0]
        hand_q = {s: np.zeros((num_frames, NUM_HAND_JOINTS)) for s in ('left', 'right')}
        scope_hint = {'arms': [joint.split('_')[0]], 'hands': []}
    elif group in ('left_hand', 'right_hand'):
        side = group.split('_')[0]
        if joint not in hand_limits.names:
            raise ValueError(f"unknown hand joint {joint!r}; know {hand_limits.names}")
        j = hand_limits.names.index(joint)
        ramp = single_joint_ramp(
            NUM_HAND_JOINTS, j, amplitude,
            float(hand_limits.deploy_velocity[j]),
            float(hand_limits.deploy_acceleration[j]),
            fps, headroom=headroom,
        )
        num_frames = ramp.shape[0]
        arm_q = np.zeros((num_frames, NUM_ARM_JOINTS))
        hand_q = {s: np.zeros((num_frames, NUM_HAND_JOINTS)) for s in ('left', 'right')}
        hand_q[side] = ramp
        scope_hint = {'arms': [], 'hands': [side]}
    else:
        raise ValueError(f"--single-joint group must be arm|left_hand|right_hand, got {group!r}")

    native_dt = 1.0 / fps
    arm_audit = audit_tracks(
        arm_q, CANONICAL_ARM_JOINTS, arm_limits.pos_lower, arm_limits.pos_upper,
        arm_limits.vel_ceiling, arm_limits.deploy_velocity,
        arm_limits.deploy_acceleration, native_dt, 1,
    )
    hand_audits = {
        s: audit_tracks(
            hand_q[s], hand_limits.side_names(s), hand_limits.pos_lower,
            hand_limits.pos_upper, None, hand_limits.deploy_velocity,
            hand_limits.deploy_acceleration, native_dt, 1,
        )
        for s in ('left', 'right')
    }
    waist = {name: 0.0 for name in ('waist_yaw', 'waist_roll', 'waist_pitch')}
    reasons = collect_verdict_reasons(arm_audit, hand_audits, waist, False, 1)
    verdict = VERDICT_PASS if not reasons else VERDICT_FAIL
    scale = allowed_speed_scale([arm_audit] + list(hand_audits.values()))

    name = f'single_joint_{group}_{joint}'
    artifact_meta = {
        'schema_version': SCHEMA_VERSION,
        'sample': 'single_joint',
        'method': None,
        'source_dir': None,
        'input_sha256': {},
        'retargeter': None,
        'limits': {
            'arm': {'path': str(arm_limits_path)},
            'hand': {'path': str(hand_limits_path)},
        },
        'audit': {
            'arm': arm_audit,
            'hands': hand_audits,
            'waist_max_abs_rad': waist,
            'k_bundle': 1,
            'k_extra': 1,
            'single_joint': {'spec': spec, 'amplitude': amplitude,
                             'headroom': headroom},
        },
        'max_allowed_speed_scale': scale,
        'verdict': verdict,
        'verdict_reasons': reasons,
        'hands_conditioned': True,
        'scope_hint': scope_hint,
        'first_frame': _frame_pose(arm_q, hand_q, 0),
        'last_frame': _frame_pose(arm_q, hand_q, -1),
        'tool_version': TOOL_VERSION,
    }
    base = Path(out_dir) / name / 'conditioned_clip_v1'
    save_artifact(base, arm_q, hand_q['left'], hand_q['right'], fps, 1,
                  artifact_meta)
    return base, verdict


# -------------------------------------------------------------------- CLI

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--method-dir', type=Path,
                        help="a sample's GT/ or Ours/ directory")
    parser.add_argument('--single-joint', metavar='GROUP:JOINT',
                        help="Stage B generator: arm:<joint> or "
                             "{left,right}_hand:<joint>")
    parser.add_argument('--amplitude', type=float, default=0.2,
                        help='single-joint ramp amplitude in rad (default 0.2)')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--arm-limits', type=Path, default=None,
                        help='override g1_deploy_limits.yaml path')
    parser.add_argument('--hand-limits', type=Path, default=None,
                        help='override hand_limits.yaml path')
    parser.add_argument('--k-max', type=int, default=DEFAULT_K_MAX,
                        help=f'max integer time redistribution (default {DEFAULT_K_MAX})')
    parser.add_argument('--no-hands', action='store_true',
                        help='arm-only conditioning (artifact marks '
                             'hands_conditioned=false; hand scope refused at load)')
    parser.add_argument('--retarget-config-left', type=Path, default=None)
    parser.add_argument('--retarget-config-right', type=Path, default=None)
    args = parser.parse_args(argv)

    if bool(args.method_dir) == bool(args.single_joint):
        parser.error('exactly one of --method-dir / --single-joint is required')

    arm_limits = args.arm_limits or _resolve_pkg_config(
        'g1_world_output', 'g1_deploy_limits.yaml')
    hand_limits = args.hand_limits or _resolve_pkg_config(
        'wujihand_output', 'hand_limits.yaml')

    try:
        if args.single_joint:
            base, verdict = condition_single_joint(
                args.single_joint, args.amplitude, args.out_dir,
                arm_limits, hand_limits,
            )
        else:
            retarget_configs = None
            if args.retarget_config_left and args.retarget_config_right:
                retarget_configs = {'left': args.retarget_config_left,
                                    'right': args.retarget_config_right}
            base, verdict = condition_bundle_sample(
                args.method_dir, args.out_dir, arm_limits, hand_limits,
                args.k_max, hands=not args.no_hands,
                retarget_configs=retarget_configs,
            )
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(f"condition_clip: error: {exc}", file=sys.stderr)
        return 1

    meta = json.loads(base.with_suffix('.json').read_text())
    print(f"artifact: {base}.npz")
    print(f"verdict:  {verdict}"
          + (f"  reasons: {meta['verdict_reasons']}" if verdict == VERDICT_FAIL else ''))
    print(f"k: {meta['audit']['k_bundle']} (bundle) x {meta['audit']['k_extra']} (ours)"
          f"  max_allowed_speed_scale: {meta['max_allowed_speed_scale']:.3f}")
    return 0 if verdict == VERDICT_PASS else 2


if __name__ == '__main__':
    sys.exit(main())
