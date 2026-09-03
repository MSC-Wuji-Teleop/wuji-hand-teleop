"""Pins for tools/clip_audit.py: helpers, index maps, clip loading, a hold-still audit."""

from __future__ import annotations

import json
import struct
import zlib

import mujoco
import numpy as np
import pytest

import clip_audit as ca


# -- pure helpers -----------------------------------------------------------

def test_passes_requires_both_thresholds():
    thr = ca.Thresholds(max_arm_torque_ratio=0.8, max_contact_force_n=80.0)
    assert ca.passes(0.8, 80.0, thr)
    assert ca.passes(0.0, 0.0, thr)
    assert not ca.passes(0.81, 0.0, thr)
    assert not ca.passes(0.0, 80.1, thr)


def test_speed_key_formats_floats():
    assert ca.speed_key(1) == "1.0"
    assert ca.speed_key(0.5) == "0.5"
    assert ca.speed_key(0.25) == "0.25"


def test_slew_toward_limits_each_element():
    cur = np.array([0.0, 0.0, 1.0])
    tgt = np.array([1.0, -0.05, 1.0])
    out = ca.slew_toward(cur, tgt, 0.1)
    assert np.allclose(out, [0.1, -0.05, 1.0])


def test_write_png_is_decodable(tmp_path):
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    rgb[1, 2] = (255, 0, 0)
    path = tmp_path / "f.png"
    ca.write_png(path, rgb)
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    length, tag = struct.unpack(">I4s", data[8:16])
    assert tag == b"IHDR" and length == 13
    w, h, depth, ctype = struct.unpack(">IIBB", data[16:26])
    assert (w, h, depth, ctype) == (6, 4, 8, 2)
    # IDAT follows IHDR (13 bytes + 4 crc); its payload inflates to h * (1 + 3 w).
    idat_len = struct.unpack(">I", data[33:37])[0]
    assert data[37:41] == b"IDAT"
    raw = zlib.decompress(data[41:41 + idat_len])
    assert len(raw) == 4 * (1 + 6 * 3)
    assert raw[1 + 3 * 6 + 1 + 2 * 3:1 + 3 * 6 + 1 + 2 * 3 + 3] == b"\xff\x00\x00"


# -- the rig ---------------------------------------------------------------

def test_name_id_raises_on_missing_name(rig):
    with pytest.raises(KeyError):
        ca.name_id(rig.model, mujoco.mjtObj.mjOBJ_JOINT, "left_shoulder_pitch")  # bundle name, no _joint
    assert ca.name_id(rig.model, mujoco.mjtObj.mjOBJ_JOINT, "left_shoulder_pitch_joint") >= 0


def test_arm_actuators_drive_the_named_joints_in_order(rig):
    m = rig.model
    names = ca.ARM_JOINT_NAMES["left"] + ca.ARM_JOINT_NAMES["right"]
    assert len(rig.arm_aid) == 14
    for aid, dof, qpos, name in zip(rig.arm_aid, rig.arm_dof, rig.arm_qpos, names):
        jid = m.actuator_trnid[aid, 0]
        assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid) == name + ca.MJCF_JOINT_SUFFIX
        assert m.jnt_dofadr[jid] == dof
        assert m.jnt_qposadr[jid] == qpos
    # Wrist tier: the last three joints of each side.
    assert rig.arm_is_wrist.tolist() == [False, False, False, False, True, True, True] * 2
    # Re-gained: kp on gain[0], -kp on bias[1], -kd on bias[2].
    for aid, wrist in zip(rig.arm_aid, rig.arm_is_wrist):
        kp, kd = (ca.WRIST_KP, ca.WRIST_KD) if wrist else (ca.ARM_KP, ca.ARM_KD)
        assert m.actuator_gainprm[aid, 0] == kp
        assert m.actuator_biasprm[aid, 1] == -kp
        assert m.actuator_biasprm[aid, 2] == -kd


def test_hand_actuators_drive_hand_joint_names_in_order(rig):
    m = rig.model
    assert len(rig.hand_aid) == 40
    for k, aid in enumerate(rig.hand_aid):
        side = "left" if k < 20 else "right"
        expect = ca.HAND_MJCF_PREFIX[side] + ca.HAND_JOINT_NAMES[side][k % 20]
        jid = m.actuator_trnid[aid, 0]
        assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid) == expect
    for side in ca.SIDES:
        assert rig.hand_jnt_range[side].shape == (20, 2)
        assert np.all(rig.hand_jnt_range[side][:, 0] < rig.hand_jnt_range[side][:, 1])


def test_hold_still_at_stand_passes_with_zero_contact(rig):
    n = 50
    arm_stand = rig.ctrl_stand[rig.arm_aid]
    hand_stand = rig.ctrl_stand[rig.hand_aid]
    arm_q = {"left": np.tile(arm_stand[:7], (n, 1)), "right": np.tile(arm_stand[7:], (n, 1))}
    hand = {"left": np.tile(hand_stand[:20], (n, 1)), "right": np.tile(hand_stand[20:], (n, 1))}
    res = rig.run(arm_q, hand, 50.0, 1.0)
    s = res.summary
    assert s["pass"] is True
    assert s["peak_contact_force_n"] == 0.0
    assert s["peak_contact_pair"] == []
    assert s["contact_frame_fraction"] == 0.0
    assert s["top_contact_pairs"] == []
    assert s["arm_saturation_fraction"] == 0.0
    assert 0.0 < s["peak_arm_torque_ratio"] < 0.5  # gravity only
    assert res.frame_torque_ratio.shape == (n,)
    assert res.frame_contact_force_n.shape == (n,)
    assert np.all(res.frame_contact_force_n == 0.0)


def test_audit_meta_has_the_spec_keys(rig):
    meta = rig.audit_meta([1.0, 0.5], ca.Thresholds(), note="why")
    assert list(meta) == ["model", "model_sha256", "mujoco_version", "timestep", "arm_gains",
                          "hand_command_slew_rad_s", "thresholds", "speeds", "note"]
    assert meta["model"] == "g1_29_wuji2_fixed.xml"
    assert meta["timestep"] == 0.002
    assert meta["speeds"] == [1.0, 0.5]
    assert meta["note"] == "why"


# -- load_clip_dir ------------------------------------------------------------

def _write_clip(tmp_path, arm_names=None, hand_names=None, frames=5, arm_shape=None, hand_shape=None,
                name="clip"):
    d = tmp_path / name
    d.mkdir()
    arm_names = arm_names or ca.ARM_JOINT_NAMES
    hand_names = hand_names or ca.HAND_JOINT_NAMES
    arm_shape = arm_shape or (frames, 7)
    hand_shape = hand_shape or (frames, 20)
    np.savez(d / "arm_q.npz", left=np.zeros(arm_shape), right=np.zeros(arm_shape))
    np.savez(d / "hand_q20.npz", left=np.zeros(hand_shape), right=np.zeros(hand_shape))
    (d / "clip.json").write_text(json.dumps({
        "frames": frames, "rate_hz": 50.0,
        "arm_joint_names": arm_names, "hand_joint_names": hand_names}))
    return d


def test_load_clip_dir_accepts_a_well_formed_clip(tmp_path):
    arm_q, hand_q20, meta = ca.load_clip_dir(_write_clip(tmp_path))
    assert arm_q["left"].shape == (5, 7) and hand_q20["right"].shape == (5, 20)
    assert meta["frames"] == 5


def test_load_clip_dir_refuses_wrong_arm_names(tmp_path):
    names = {s: list(reversed(ca.ARM_JOINT_NAMES[s])) for s in ca.SIDES}
    with pytest.raises(ValueError, match="arm_joint_names"):
        ca.load_clip_dir(_write_clip(tmp_path, arm_names=names))


def test_load_clip_dir_refuses_wrong_hand_names(tmp_path):
    names = {s: [n.upper() for n in ca.HAND_JOINT_NAMES[s]] for s in ca.SIDES}
    with pytest.raises(ValueError, match="hand_joint_names"):
        ca.load_clip_dir(_write_clip(tmp_path, hand_names=names))


def test_load_clip_dir_refuses_wrong_shapes(tmp_path):
    with pytest.raises(ValueError, match="bad shapes"):
        ca.load_clip_dir(_write_clip(tmp_path, arm_shape=(5, 6)))
    with pytest.raises(ValueError, match="bad shapes"):
        ca.load_clip_dir(_write_clip(tmp_path, hand_shape=(4, 20), name="clip2"))
