#!/usr/bin/env python3
"""Kinematic self-collision audit for a conditioned clip artifact.

Loads the 29-DoF composed model (g1_wuji2_description/g1_29_wuji2_fixed.xml),
plays the artifact's arm_q + hand q20 through the model frame by frame with
pure forward kinematics (mj_forward, no dynamics), and reports every contact
pair that is not already present in the frame-0 baseline pose. This is the
"no collisions" gate for sweep-test samples, and works on any
conditioned_clip_v1.npz.

The check is kinematic on the COMMAND trajectory: it proves the commanded
path itself never self-intersects. It does not model tracking error --
that margin is what the conservative deploy speed rows are for.

Run in the teleop container (needs mujoco + numpy):

    python3 RobotSTAR_demos/sweep-test/check_collisions.py \
        ~/wuji_clips/90_sweep_joints_GT/conditioned_clip_v1.npz

Exit codes: 0 = no new contacts, 2 = collisions found, 1 = usage error.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# Hand actuator naming, kept in lockstep with
# src/output_devices/g1_world_output/scripts/_mujoco_common.py::HAND_CODES
# (wujihandros2 index order, finger1..5 x joint1..4) -- duplicated here so
# sweep-test stays a self-contained drop-in folder.
HAND_CODES = [
    "THJ0", "THJ1", "THJ2", "THJ3",
    "FFJ0", "FFJ1", "FFJ2", "FFJ3",
    "MFJ0", "MFJ1", "MFJ2", "MFJ3",
    "RFJ0", "RFJ1", "RFJ2", "RFJ3",
    "LFJ0", "LFJ1", "LFJ2", "LFJ3",
]

SWEEP_TEST_DIR = Path(__file__).resolve().parent


def default_mjcf() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = (Path(get_package_share_directory('g1_wuji2_description'))
             / 'g1_29_wuji2_fixed.xml')
        if p.exists():
            return p
    except Exception:
        pass
    return (SWEEP_TEST_DIR.parents[1] / 'src' / 'g1_wuji2_description'
            / 'g1_29_wuji2_fixed.xml')


def qpos_addr_via_actuator(model, mujoco, actuator_name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if aid < 0:
        raise SystemExit(f'actuator {actuator_name!r} not in the model')
    jid = model.actuator_trnid[aid, 0]
    return int(model.jnt_qposadr[jid])


def contact_pairs(model, data, mujoco) -> dict:
    """{(name1, name2): worst penetration depth (m, negative = overlap)}.

    Geoms in the composed MJCF are mostly unnamed, so contacts are labeled
    by the BODY each geom belongs to.
    """
    pairs = {}
    for i in range(data.ncon):
        con = data.contact[i]
        names = []
        for g in (con.geom1, con.geom2):
            b = model.geom_bodyid[g]
            name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
                    or f'body{b}')
            names.append(name)
        key = tuple(sorted(names))
        pairs[key] = min(pairs.get(key, 0.0), float(con.dist))
    return pairs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('artifact', type=Path,
                    help='conditioned_clip_v1.npz to audit')
    ap.add_argument('--mjcf', type=Path, default=None,
                    help='override g1_29_wuji2_fixed.xml path')
    args = ap.parse_args()

    import mujoco

    d = np.load(args.artifact)
    arm_q = d['arm_q']
    arm_names = [str(n) for n in d['arm_joint_names']]
    hand_q = {'left': d['left_hand_q20'], 'right': d['right_hand_q20']}
    frames = arm_q.shape[0]

    mjcf = args.mjcf or default_mjcf()
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'stand')
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    base_qpos = data.qpos.copy()

    arm_addr = [qpos_addr_via_actuator(model, mujoco, f'{n}_joint')
                for n in arm_names]
    hand_addr = {
        side: [qpos_addr_via_actuator(model, mujoco,
                                      f'{side}_wuji_{side[0]}_{code}')
               for code in HAND_CODES]
        for side in ('left', 'right')
    }

    def set_frame(f: int) -> None:
        data.qpos[:] = base_qpos
        for a, q in zip(arm_addr, arm_q[f]):
            data.qpos[a] = q
        for side in ('left', 'right'):
            for a, q in zip(hand_addr[side], hand_q[side][f]):
                data.qpos[a] = q
        mujoco.mj_forward(model, data)

    set_frame(0)
    baseline = set(contact_pairs(model, data, mujoco))
    print(f'{args.artifact.name}: {frames} frames, model {mjcf.name}')
    print(f'baseline (frame 0) contact pairs: {len(baseline)}')
    for p in sorted(baseline):
        print(f'  baseline: {p[0]} <-> {p[1]}')

    new_hits: dict = {}
    for f in range(frames):
        set_frame(f)
        for pair, dist in contact_pairs(model, data, mujoco).items():
            if pair in baseline:
                continue
            entry = new_hits.setdefault(pair, {'frames': [], 'depth': 0.0})
            entry['frames'].append(f)
            entry['depth'] = min(entry['depth'], dist)

    if not new_hits:
        print(f'OK: no new contact pairs across {frames} frames')
        return 0
    print(f'COLLISIONS: {len(new_hits)} new contact pair(s):')
    for pair, e in sorted(new_hits.items()):
        hits = e['frames']
        print(f'  {pair[0]} <-> {pair[1]}: {len(hits)} frames '
              f'(first {hits[0]}, last {hits[-1]}, '
              f'worst depth {e["depth"]*1000:.2f} mm)')
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
