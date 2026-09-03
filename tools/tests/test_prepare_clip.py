"""Pins for tools/prepare_clip.py: bundle reading, sanitizer, keypoint mapping,
hand permutation, retarget stage, auto-trim selection, judge, filing, --all,
exit codes, and one real-retargeter check that skips without wuji_retargeting.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import clip_audit as ca
import prepare_clip as pc
from tests.conftest import (BODY_ACTUATORS, FAKE_HAND_RANGE, FAKE_LP_ALPHA, REPLAY_PKG_DIR,
                            FakeRetargeter, flat_hand_keypoints, make_bundle,
                            optimizer_joint_names, write_fake_urdf)

FPS = 50.0


# -- 0. bundle reading ---------------------------------------------------------

def test_read_bundle_selects_arm_columns_by_name(bundle_root):
    method_dir = make_bundle(bundle_root, spike_frame=None)
    traj = pc.read_bundle(method_dir)
    body_q = np.load(method_dir / "g1_reference" / "controller_reference_v7.npz")["body_q"].astype(np.float64)
    for side in ca.SIDES:
        assert traj.arm_q[side].shape == (120, 7)
        for j, name in enumerate(ca.ARM_JOINT_NAMES[side]):
            assert np.array_equal(traj.arm_q[side][:, j], body_q[:, BODY_ACTUATORS.index(name)])
    assert traj.sample == "synth" and traj.method == "Ours" and traj.name == "synth_Ours"
    assert traj.frames == 120 and traj.source_frames == 48
    assert traj.rate_hz == 50.0
    assert traj.keypoints["left"].shape == (48, 21, 3) and traj.keypoints["left"].dtype == np.float32


def test_read_bundle_rate_hz_divides_by_time_scale(bundle_root):
    traj = pc.read_bundle(make_bundle(bundle_root, time_scale=2))
    assert traj.rate_hz == 25.0


def test_read_bundle_records_manifest_sha_or_none(bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root)
    traj = pc.read_bundle(method_dir)
    expect = hashlib.sha256((bundle_root / "MANIFEST.sha256").read_bytes()).hexdigest()
    assert traj.manifest_sha256 == expect
    no_manifest = make_bundle(tmp_path / "other", write_manifest=False)
    assert pc.read_bundle(no_manifest).manifest_sha256 is None


def test_read_bundle_rejects_bad_shapes(bundle_root, tmp_path):
    bad_body = make_bundle(bundle_root, body_q_override=np.zeros((120, 28)))
    with pytest.raises(pc.PrepareError, match="body_q shape"):
        pc.read_bundle(bad_body)
    bad_kp = make_bundle(tmp_path / "kp", keypoints_shape_override=(48, 20, 3))
    with pytest.raises(pc.PrepareError, match="keypoints21 shape"):
        pc.read_bundle(bad_kp)


def test_read_bundle_rejects_missing_arm_name(bundle_root):
    method_dir = make_bundle(bundle_root)
    meta_path = method_dir / "g1_reference" / "target_meta.json"
    meta = json.loads(meta_path.read_text())
    names = meta["joint_actuator_order"]["body_actuators"]
    names[names.index("left_elbow")] = "left_elbow_joint"
    meta_path.write_text(json.dumps(meta))
    with pytest.raises(pc.PrepareError, match="left_elbow"):
        pc.read_bundle(method_dir)


def test_read_bundle_requires_the_method_dir_layout(tmp_path):
    with pytest.raises(pc.PrepareError, match="is missing"):
        pc.read_bundle(tmp_path)


# -- 1. sanitize -------------------------------------------------------------

def _arm14(method_dir):
    return pc.stack_sides(pc.read_bundle(method_dir).arm_q)


def test_sanitize_attenuates_the_spike_and_clamps_steps(bundle_root, tmp_path):
    arm14 = _arm14(make_bundle(bundle_root, spike_frame=40, spike_rad=0.3))
    clean = _arm14(make_bundle(tmp_path / "clean", spike_frame=None))
    out, stats = pc.sanitize_arms(arm14, FPS, 6.0, 15.0, allow_flips=False)
    out_clean, _ = pc.sanitize_arms(clean, FPS, 6.0, 15.0, allow_flips=False)
    assert out.shape == arm14.shape
    assert set(stats) == {"before", "after", "arm_rmse_rad", "flip_max_step_deg"}
    for block in ("before", "after"):
        assert set(stats[block]) == {"max_step_deg", "peak_vel_rad_s", "peak_acc_rad_s2"}
    assert stats["before"]["max_step_deg"] > 15.0
    assert stats["after"]["peak_acc_rad_s2"] < stats["before"]["peak_acc_rad_s2"]
    assert stats["after"]["max_step_deg"] <= 15.0 + 1e-9
    assert 0.0 < stats["arm_rmse_rad"] < 0.1
    assert stats["flip_max_step_deg"] == stats["before"]["max_step_deg"]
    # The spike is spread and attenuated: the output stays within half the
    # spike of the spike-free motion, and the smooth joints are untouched.
    assert np.abs(out - out_clean).max() < 0.15
    assert np.abs(out[:, 1:] - out_clean[:, 1:]).max() < 1e-9
    assert all(isinstance(v, float) for v in stats["after"].values())


def test_sanitize_refuses_a_flip_unless_allowed(bundle_root):
    arm14 = _arm14(make_bundle(bundle_root, flip_frame=60, flip_deg=100.0))
    with pytest.raises(pc.FlipRefused) as info:
        pc.sanitize_arms(arm14, FPS, 6.0, 15.0, allow_flips=False)
    assert 99.0 < info.value.max_step_deg < 102.0
    assert "allow-flips" in str(info.value)
    out, stats = pc.sanitize_arms(arm14, FPS, 6.0, 15.0, allow_flips=True)
    assert stats["flip_max_step_deg"] >= 90.0
    assert stats["after"]["max_step_deg"] <= 15.0 + 1e-9
    assert np.all(np.isfinite(out))


def test_flip_threshold_is_exactly_90_deg():
    q = np.zeros((20, 14))
    q[10:, 3] = np.radians(89.9)
    pc.sanitize_arms(q, FPS, 6.0, 15.0, allow_flips=False)
    q[10:, 3] = np.radians(90.0)
    with pytest.raises(pc.FlipRefused):
        pc.sanitize_arms(q, FPS, 6.0, 15.0, allow_flips=False)


def test_trim_frames_drops_exactly_the_requested_frames():
    arr = np.arange(10)[:, None] * np.ones((1, 14))
    out = pc.trim_frames(arr, 3, 2)
    assert out[:, 0].tolist() == [3, 4, 5, 6, 7]
    assert pc.trim_frames(arr, 0, 0).shape == (10, 14)
    with pytest.raises(pc.PrepareError):
        pc.trim_frames(arr, 5, 5)
    with pytest.raises(pc.PrepareError):
        pc.trim_frames(arr, -1, 0)


def test_trim_happens_before_the_flip_check(bundle_root):
    arm14 = _arm14(make_bundle(bundle_root, flip_frame=60, flip_deg=100.0, spike_frame=None))
    trimmed = pc.trim_frames(arm14, 61, 0)  # the step at 59->60 is gone
    out, stats = pc.sanitize_arms(trimmed, FPS, 6.0, 15.0, allow_flips=False)
    assert stats["flip_max_step_deg"] < 90.0
    assert out.shape[0] == 120 - 61


def test_step_clamp_forward_and_backward():
    q = np.zeros((6, 1))
    q[3:] = 1.0  # one big step
    out = pc.step_clamp(q, 0.25)
    assert np.all(np.abs(np.diff(out[:, 0])) <= 0.25 + 1e-12)


def test_butter_zero_phase_rejects_bad_cutoff_and_short_clips():
    with pytest.raises(pc.PrepareError, match="cutoff"):
        pc.butter_zero_phase(np.zeros((50, 14)), FPS, 25.0)
    with pytest.raises(pc.PrepareError, match="too few"):
        pc.butter_zero_phase(np.zeros((5, 14)), FPS, 6.0)


# -- 2. keypoint mapping and permutation ----------------------------------

def test_keypoint_frame_indices_exact_values():
    idx = pc.keypoint_frame_indices(100, 40)
    assert idx.tolist() == [round(i * 39 / 99) for i in range(100)]
    assert idx[0] == 0 and idx[-1] == 39
    assert pc.keypoint_frame_indices(1, 40).tolist() == [0]
    assert pc.keypoint_frame_indices(1, 1).tolist() == [0]
    assert pc.keypoint_frame_indices(48, 48).tolist() == list(range(48))
    # Upsampled hands: the last body frame still hits the last keypoint frame.
    up = pc.keypoint_frame_indices(30, 90)
    assert up[0] == 0 and up[-1] == 89 and np.all(np.diff(up) >= 0)


def test_urdf_movable_joints_skips_fixed_in_declaration_order(tmp_path):
    names = ["c", "a", "b"]
    path = write_fake_urdf(tmp_path / "f.urdf", names)
    assert pc.urdf_movable_joints(path) == names


def test_build_qpos_perm_restores_declaration_order():
    urdf = ["thumb", "index", "middle"]
    opt = ["index", "middle", "thumb"]
    perm = pc.build_qpos_perm(opt, urdf)
    q_opt = np.array([10.0, 20.0, 30.0])  # index, middle, thumb
    assert q_opt[perm].tolist() == [30.0, 10.0, 20.0]
    assert [opt[i] for i in perm] == urdf


def test_build_qpos_perm_raises_on_mismatch():
    with pytest.raises(pc.PrepareError, match="unmatched"):
        pc.build_qpos_perm(["a", "b"], ["a", "c"])
    with pytest.raises(pc.PrepareError):
        pc.build_qpos_perm(["a", "b", "c"], ["a", "b"])
    with pytest.raises(pc.PrepareError):
        pc.build_qpos_perm(["a", "a"], ["a", "a"])


def test_hardware_perm_with_a_scrambled_fake_retargeter(tmp_path):
    for side in ca.SIDES:
        urdf = write_fake_urdf(tmp_path / f"{side}.urdf", ca.HAND_JOINT_NAMES[side])
        fake = FakeRetargeter(side, urdf)
        perm = pc.hardware_perm(fake, side)
        names_opt = optimizer_joint_names(side)
        assert [names_opt[i] for i in perm] == ca.HAND_JOINT_NAMES[side]
        # The real optimizer order (index, middle, pinky, ring, thumb) gives this perm.
        assert perm.tolist() == [16, 17, 18, 19, 0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 8, 9, 10, 11]
        assert np.allclose(fake.retarget(np.zeros((21, 3)))[perm], fake.pattern_hw)


def test_hardware_perm_refuses_a_urdf_in_another_order(tmp_path):
    side = "left"
    wrong = list(reversed(ca.HAND_JOINT_NAMES[side]))
    urdf = write_fake_urdf(tmp_path / "wrong.urdf", wrong)
    fake = FakeRetargeter(side, urdf)
    with pytest.raises(pc.PrepareError, match="hardware order"):
        pc.hardware_perm(fake, side)


def test_hardware_perm_refuses_unalignable_names(tmp_path):
    side = "left"
    urdf = write_fake_urdf(tmp_path / "ok.urdf", ca.HAND_JOINT_NAMES[side])
    fake = FakeRetargeter(side, urdf)
    fake.optimizer.robot.dof_joint_names = [n + "_x" for n in optimizer_joint_names(side)]
    with pytest.raises(pc.PrepareError, match="cannot align"):
        pc.hardware_perm(fake, side)


def test_retargeter_urdf_path_resolves_relative_to_yaml_dir(tmp_path):
    fake = SimpleNamespace(config={"__yaml_dir": str(tmp_path / "cfg"), "optimizer": {"urdf_path": "../u/h.urdf"}})
    assert pc.retargeter_urdf_path(fake) == (tmp_path / "u" / "h.urdf").resolve()
    with pytest.raises(pc.PrepareError, match="urdf_path"):
        pc.retargeter_urdf_path(SimpleNamespace(config={"optimizer": {}}))


# -- 2. retarget stage --------------------------------------------------------

def test_retarget_hands_hardware_order_and_clipped_fraction(fake_retargeter_factory, bundle_root):
    traj = pc.read_bundle(make_bundle(bundle_root))
    pattern = np.linspace(-0.5, 1.0, 20)
    pattern[3] = 5.0    # above the range
    pattern[17] = -5.0  # below the range
    pattern[7] = FAKE_HAND_RANGE[7, 0] - 5e-8   # float32 noise past the bound: clamped, not counted
    pattern[11] = FAKE_HAND_RANGE[11, 1] + 5e-8
    fake_retargeter_factory.kwargs = {"pattern_hw": pattern}
    kp_idx = pc.keypoint_frame_indices(traj.frames, traj.source_frames)
    ranges = {s: FAKE_HAND_RANGE for s in ca.SIDES}
    hand_q20, block = pc.retarget_hands(traj.keypoints, kp_idx, pc.DEFAULT_RETARGET_CONFIG_DIR, ranges,
                                        fake_retargeter_factory)
    for side in ca.SIDES:
        assert hand_q20[side].shape == (120, 20)
        assert hand_q20[side].dtype == np.float64
        expect = np.clip(pattern, FAKE_HAND_RANGE[:, 0], FAKE_HAND_RANGE[:, 1])
        assert np.array_equal(hand_q20[side], np.repeat(expect[None, :], 120, axis=0))
        assert hand_q20[side][:, 7].min() >= FAKE_HAND_RANGE[7, 0]
        assert block["clipped_fraction"][side] == pytest.approx(2 / 20)
        assert block["config_sha256"][side] == ca.sha256_file(
            pc.DEFAULT_RETARGET_CONFIG_DIR / f"retarget_keypoints_topic_{side}.yaml")
    assert block["config"] == "retarget_keypoints_topic_{side}.yaml"
    assert [r.side for r in fake_retargeter_factory.made] == ["left", "right"]
    for r in fake_retargeter_factory.made:
        assert r.reset_calls == 1
        assert r.retarget_calls == 120
        assert r.seen_shapes == {(21, 3)}


def test_retarget_hands_uses_the_mapped_keypoint_frames(fake_retargeter_factory, bundle_root):
    traj = pc.read_bundle(make_bundle(bundle_root))
    fake_retargeter_factory.kwargs = {"kp_gain": 100.0, "pattern_hw": np.zeros(20)}
    kp_idx = pc.keypoint_frame_indices(traj.frames, traj.source_frames)
    ranges = {s: np.tile([[-1e9, 1e9]], (20, 1)) for s in ca.SIDES}
    hand_q20, _ = pc.retarget_hands(traj.keypoints, kp_idx, pc.DEFAULT_RETARGET_CONFIG_DIR, ranges,
                                    fake_retargeter_factory)
    means = np.array([traj.keypoints["left"][k].mean() for k in kp_idx], dtype=np.float64)
    assert np.allclose(hand_q20["left"][:, 0], 100.0 * means, atol=1e-5)


def test_retarget_hands_leaves_the_config_alpha_alone_by_default(fake_retargeter_factory, bundle_root):
    """No --hand-lp-alpha means the retargeter keeps whatever its config set."""
    traj = pc.read_bundle(make_bundle(bundle_root))
    kp_idx = pc.keypoint_frame_indices(traj.frames, traj.source_frames)
    ranges = {s: FAKE_HAND_RANGE for s in ca.SIDES}
    _, block = pc.retarget_hands(traj.keypoints, kp_idx, pc.DEFAULT_RETARGET_CONFIG_DIR, ranges,
                                 fake_retargeter_factory)
    assert [r.lp_filter.alpha for r in fake_retargeter_factory.made] == [FAKE_LP_ALPHA] * 2
    assert block["lp_alpha"] == {"left": FAKE_LP_ALPHA, "right": FAKE_LP_ALPHA}


def test_retarget_hands_applies_and_records_the_alpha_override(fake_retargeter_factory, bundle_root):
    """--hand-lp-alpha reaches every side's retargeter and lands in clip.json."""
    traj = pc.read_bundle(make_bundle(bundle_root))
    kp_idx = pc.keypoint_frame_indices(traj.frames, traj.source_frames)
    ranges = {s: FAKE_HAND_RANGE for s in ca.SIDES}
    _, block = pc.retarget_hands(traj.keypoints, kp_idx, pc.DEFAULT_RETARGET_CONFIG_DIR, ranges,
                                 fake_retargeter_factory, lp_alpha=0.6)
    assert [r.lp_filter.alpha for r in fake_retargeter_factory.made] == [0.6, 0.6]
    assert block["lp_alpha"] == {"left": 0.6, "right": 0.6}


def test_set_lp_alpha_refuses_a_retargeter_without_the_filter():
    """An override that cannot be applied raises instead of passing silently."""
    with pytest.raises(pc.PrepareError, match="no lp_filter"):
        pc.set_lp_alpha(SimpleNamespace(), 0.5, "left")
    # Reading with no override is fine: the value is simply unknown.
    assert pc.set_lp_alpha(SimpleNamespace(), None, "left") is None


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.5])
def test_options_refuse_an_alpha_outside_the_unit_interval(alpha):
    with pytest.raises(pc.PrepareError, match="hand-lp-alpha"):
        pc.Options(hand_lp_alpha=alpha).validate()


def test_options_accept_the_open_upper_bound():
    pc.Options(hand_lp_alpha=1.0).validate()


def test_retarget_hands_refuses_a_missing_config(fake_retargeter_factory, bundle_root, tmp_path):
    traj = pc.read_bundle(make_bundle(bundle_root))
    with pytest.raises(pc.PrepareError, match="retarget config"):
        pc.retarget_hands(traj.keypoints, np.zeros(3, dtype=int), tmp_path, {s: FAKE_HAND_RANGE for s in ca.SIDES},
                          fake_retargeter_factory)


# -- 5. auto-trim selection ------------------------------------------------

def _result(torque, force=None):
    torque = np.asarray(torque, dtype=float)
    force = np.zeros_like(torque) if force is None else np.asarray(force, dtype=float)
    return ca.AuditResult(summary={}, frame_torque_ratio=torque, frame_contact_force_n=force)


THR = ca.Thresholds(max_arm_torque_ratio=0.8, max_contact_force_n=80.0)


def test_longest_passing_window_basic():
    torque = [0.9, 0.1, 0.1, 0.9, 0.1, 0.1, 0.1, 0.9]
    assert pc.longest_passing_window(torque, np.zeros(8), THR) == (4, 7)


def test_longest_passing_window_needs_both_thresholds():
    torque = [0.1] * 8
    force = [0, 0, 0, 100, 0, 0, 0, 0]
    assert pc.longest_passing_window(torque, force, THR) == (4, 8)


def test_longest_passing_window_ties_take_the_earliest():
    torque = [0.1, 0.1, 0.9, 0.1, 0.1, 0.9]
    assert pc.longest_passing_window(torque, np.zeros(6), THR) == (0, 2)


def test_longest_passing_window_none_and_all():
    assert pc.longest_passing_window([0.9, 0.9], [0, 0], THR) == (0, 0)
    assert pc.longest_passing_window([0.1, 0.1, 0.1], [0, 0, 0], THR) == (0, 3)
    assert pc.longest_passing_window([0.8], [80.0], THR) == (0, 1)  # inclusive thresholds


def test_min_window_frames():
    assert pc.min_window_frames(3.0, 50.0) == 150
    assert pc.min_window_frames(0.1, 50.0) == 5
    assert pc.min_window_frames(0.001, 50.0) == 1


def test_choose_auto_trim_picks_the_fastest_speed_with_a_long_enough_window():
    results = {
        1.0: _result([0.9, 0.1, 0.1, 0.9, 0.9, 0.9]),           # window of 2
        0.5: _result([0.9, 0.1, 0.1, 0.1, 0.9, 0.1]),           # window of 3
        0.25: _result([0.1] * 6),                                # whole clip
    }
    assert pc.choose_auto_trim(results, THR, min_frames=3) == (0.5, 1, 4)
    assert pc.choose_auto_trim(results, THR, min_frames=2) == (1.0, 1, 3)
    assert pc.choose_auto_trim(results, THR, min_frames=6) == (0.25, 0, 6)
    assert pc.choose_auto_trim(results, THR, min_frames=7) is None


def test_choose_auto_trim_takes_the_longest_window_of_that_speed():
    results = {1.0: _result([0.1, 0.1, 0.9, 0.1, 0.1, 0.1, 0.1, 0.9, 0.1])}
    assert pc.choose_auto_trim(results, THR, min_frames=2) == (1.0, 3, 7)


def test_choose_auto_trim_none_found():
    results = {1.0: _result([0.9] * 4), 0.5: _result([0.9] * 4)}
    assert pc.choose_auto_trim(results, THR, min_frames=1) is None


# -- 4. judge and 6. json ----------------------------------------------------

def test_judge_safe_speeds_descending():
    per_speed = {0.25: {"pass": True}, 1.0: {"pass": False}, 0.5: {"pass": True}}
    assert pc.judge(per_speed) == ("safe", [0.5, 0.25])
    assert pc.judge({1.0: {"pass": False}}) == ("rejected", [])


def test_rejection_reason_names_the_failing_number():
    per_speed = {1.0: {"pass": False, "peak_arm_torque_ratio": 0.95, "peak_contact_force_n": 142.0,
                       "peak_contact_pair": ["a", "b"]},
                 0.5: {"pass": True, "peak_arm_torque_ratio": 0.1, "peak_contact_force_n": 0.0,
                       "peak_contact_pair": []}}
    reason = pc.rejection_reason(per_speed, THR)
    assert reason.startswith("1.0x:") and "0.95" in reason and "142.0 N" in reason and "a/b" in reason
    assert "0.5x" not in reason


def test_to_jsonable_strips_numpy_types():
    obj = {"a": np.float32(1.5), "b": np.int64(2), "c": np.bool_(True), "d": np.arange(2), "e": Path("/x"),
           "f": [np.float64(0.25)]}
    out = pc.to_jsonable(obj)
    assert out == {"a": 1.5, "b": 2, "c": True, "d": [0, 1], "e": "/x", "f": [0.25]}
    assert type(out["a"]) is float and type(out["b"]) is int and type(out["c"]) is bool
    json.dumps(out)


def test_options_validate_rejects_bad_values(tmp_path):
    pc.Options().validate()
    for bad in (dict(speeds=()), dict(speeds=(1.5,)), dict(speeds=(0.5, 0.5)), dict(cutoff_hz=0.0),
                dict(max_step_deg=-1.0), dict(trim_start=-1), dict(min_seconds=0.0),
                dict(max_contact_force_n=-1.0), dict(retarget_config_dir=tmp_path / "nope")):
        with pytest.raises(pc.PrepareError):
            pc.Options(**bad).validate()


def test_write_clip_dir_replaces_an_existing_directory(tmp_path):
    arm = {s: np.zeros((3, 7)) for s in ca.SIDES}
    hand = {s: np.zeros((3, 20)) for s in ca.SIDES}
    first = pc.write_clip_dir(tmp_path, "x", "safe", arm, hand, {"v": 1})
    assert first == tmp_path / "safe" / "x"
    (first / "stale.txt").write_text("old")
    second = pc.write_clip_dir(tmp_path, "x", "safe", arm, hand, {"v": 2})
    assert second == first
    assert not (second / "stale.txt").exists()
    assert json.loads((second / "clip.json").read_text()) == {"v": 2}
    assert not (tmp_path / "candidate" / "x").exists()
    assert list((tmp_path / "candidate").iterdir()) == []
    rejected = pc.write_clip_dir(tmp_path, "y", "rejected", arm, hand, {})
    assert rejected == tmp_path / "rejected" / "y"


# -- end to end --------------------------------------------------------------

SPEC_TOP_KEYS = ["tool", "source", "frames", "rate_hz", "arm_joint_names", "hand_joint_names",
                 "sanitize", "hand_retarget", "audit", "safe_speeds", "verdict"]
SPEC_SANITIZE_KEYS = ["cutoff_hz", "max_step_deg", "trim_start", "trim_end", "allow_flips", "auto_trim",
                      "min_seconds", "before", "after", "arm_rmse_rad", "flip_max_step_deg", "auto_trim_note"]
SPEC_AUDIT_KEYS = ["model", "model_sha256", "mujoco_version", "timestep", "arm_gains", "hand_command_slew_rad_s",
                   "thresholds", "speeds", "note", "per_speed"]
SPEC_PER_SPEED_KEYS = ["pass", "peak_arm_torque_ratio", "peak_contact_force_n", "peak_contact_pair",
                       "contact_frame_fraction", "arm_saturation_fraction", "hand_saturation_fraction",
                       "tracking_rmse_rad", "top_contact_pairs"]


def _check_clip_json(meta, speeds=(1.0, 0.5, 0.25)):
    assert list(meta) == SPEC_TOP_KEYS
    assert meta["tool"] == "prepare_clip/1"
    assert list(meta["source"]) == ["sample", "method", "bundle_manifest_sha256", "method_dir", "time_scale"]
    assert isinstance(meta["frames"], int) and isinstance(meta["rate_hz"], float)
    assert meta["arm_joint_names"] == ca.ARM_JOINT_NAMES
    assert meta["hand_joint_names"] == ca.HAND_JOINT_NAMES
    assert list(meta["sanitize"]) == SPEC_SANITIZE_KEYS
    assert list(meta["hand_retarget"]) == ["config", "config_sha256", "clipped_fraction", "lp_alpha"]
    assert set(meta["hand_retarget"]["config_sha256"]) == {"left", "right"}
    assert set(meta["hand_retarget"]["clipped_fraction"]) == {"left", "right"}
    assert list(meta["audit"]) == SPEC_AUDIT_KEYS
    assert meta["audit"]["arm_gains"] == {"kp": 140.0, "kd": 3.0, "kp_wrist": 50.0, "kd_wrist": 2.0}
    assert meta["audit"]["hand_command_slew_rad_s"] == 2.0
    assert list(meta["audit"]["per_speed"]) == [ca.speed_key(s) for s in speeds]
    for block in meta["audit"]["per_speed"].values():
        assert list(block) == SPEC_PER_SPEED_KEYS
        assert set(block["tracking_rmse_rad"]) == {"arms", "hands"}
    assert meta["verdict"] in ("safe", "rejected")
    assert (meta["verdict"] == "safe") == bool(meta["safe_speeds"])
    assert meta["safe_speeds"] == sorted(meta["safe_speeds"], reverse=True)
    passing = [float(k) for k, b in meta["audit"]["per_speed"].items() if b["pass"]]
    assert sorted(passing, reverse=True) == meta["safe_speeds"]


def _load_clip_module():
    if str(REPLAY_PKG_DIR) not in sys.path:
        sys.path.insert(0, str(REPLAY_PKG_DIR))
    from replay import clip
    return clip


def test_end_to_end_writes_a_clip_directory(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root)
    out = tmp_path / "clips"
    outcome = pc.prepare_one(method_dir, out, pc.Options(), rig, fake_retargeter_factory)
    assert outcome.name == "synth_Ours"
    clip_dir = outcome.clip_dir
    assert clip_dir.parent == out / outcome.verdict
    assert sorted(p.name for p in clip_dir.iterdir()) == ["arm_q.npz", "clip.json", "hand_q20.npz"]
    assert not (out / "candidate" / "synth_Ours").exists()

    meta = json.loads((clip_dir / "clip.json").read_text())
    _check_clip_json(meta)
    assert meta["frames"] == 120 and meta["rate_hz"] == 50.0
    assert meta["source"]["sample"] == "synth" and meta["source"]["method"] == "Ours"
    assert meta["source"]["bundle_manifest_sha256"] == hashlib.sha256(
        (bundle_root / "MANIFEST.sha256").read_bytes()).hexdigest()
    assert meta["source"]["time_scale"] == 1.0
    san = meta["sanitize"]
    assert (san["cutoff_hz"], san["max_step_deg"], san["trim_start"], san["trim_end"]) == (6.0, 15.0, 0, 0)
    assert san["allow_flips"] is False and san["auto_trim"] is False and san["min_seconds"] == 3.0
    assert san["after"]["max_step_deg"] <= 15.0 + 1e-9
    assert san["auto_trim_note"] == ""
    assert meta["audit"]["speeds"] == [1.0, 0.5, 0.25]
    assert meta["audit"]["thresholds"] == {"max_arm_torque_ratio": 0.8, "max_contact_force_n": 80.0}
    assert meta["audit"]["note"] == ""

    arm = np.load(clip_dir / "arm_q.npz")
    hand = np.load(clip_dir / "hand_q20.npz")
    for side in ca.SIDES:
        assert arm[side].shape == (120, 7) and arm[side].dtype == np.float64
        assert hand[side].shape == (120, 20) and hand[side].dtype == np.float64
        assert np.allclose(hand[side], fake_retargeter_factory.made[0].pattern_hw[None, :])
        assert meta["hand_retarget"]["clipped_fraction"][side] == 0.0

    # The synthetic motion stays clear of the body: the real audit passes it.
    assert outcome.verdict == "safe"
    assert outcome.peak_contact_force_n == 0.0
    assert outcome.reason == ""
    clip = _load_clip_module()
    loaded = clip.load_clip(clip_dir)
    assert loaded.frames == 120 and loaded.rate_hz == 50.0
    assert clip.default_speed(loaded) == max(meta["safe_speeds"])
    line = pc.format_result_line(outcome)
    assert line.startswith("synth_Ours: safe safe_speeds=[") and str(clip_dir) in line


def test_end_to_end_records_explicit_trim_and_note(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root)
    opts = pc.Options(speeds=(1.0,), trim_start=10, trim_end=5, note="why", max_contact_force_n=90.0)
    outcome = pc.prepare_one(method_dir, tmp_path / "clips", opts, rig, fake_retargeter_factory)
    meta = json.loads((outcome.clip_dir / "clip.json").read_text())
    _check_clip_json(meta, speeds=(1.0,))
    assert meta["frames"] == 105
    assert meta["sanitize"]["trim_start"] == 10 and meta["sanitize"]["trim_end"] == 5
    assert meta["audit"]["note"] == "why"
    assert meta["audit"]["thresholds"]["max_contact_force_n"] == 90.0
    assert np.load(outcome.clip_dir / "arm_q.npz")["left"].shape == (105, 7)
    # The trimmed clip retargets keypoint frames 10..114 of the untrimmed mapping.
    assert fake_retargeter_factory.made[0].retarget_calls == 105


def test_end_to_end_auto_trim_whole_clip_passes(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root)
    opts = pc.Options(speeds=(1.0,), auto_trim=True, min_seconds=1.0)
    outcome = pc.prepare_one(method_dir, tmp_path / "clips", opts, rig, fake_retargeter_factory)
    meta = json.loads((outcome.clip_dir / "clip.json").read_text())
    assert meta["frames"] == 120
    assert meta["sanitize"]["auto_trim"] is True and meta["sanitize"]["min_seconds"] == 1.0
    assert meta["sanitize"]["trim_start"] == 0 and meta["sanitize"]["trim_end"] == 0
    assert "whole clip passes" in meta["sanitize"]["auto_trim_note"]


def test_end_to_end_auto_trim_no_window_leaves_the_clip(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root)
    opts = pc.Options(speeds=(1.0,), auto_trim=True, min_seconds=10.0)  # longer than the clip
    outcome = pc.prepare_one(method_dir, tmp_path / "clips", opts, rig, fake_retargeter_factory)
    meta = json.loads((outcome.clip_dir / "clip.json").read_text())
    assert meta["frames"] == 120
    assert meta["sanitize"]["auto_trim"] is True
    assert "no speed has a passing window of at least 500 frames" in meta["sanitize"]["auto_trim_note"]
    assert meta["verdict"] == "safe"  # the first audit is kept


def test_end_to_end_auto_trim_trims_to_the_window(rig, fake_retargeter_factory, bundle_root, tmp_path):
    """A torque threshold inside the clip's own range forces a partial window."""
    method_dir = make_bundle(bundle_root)
    full = pc.prepare_one(method_dir, tmp_path / "full", pc.Options(speeds=(1.0,)), rig, fake_retargeter_factory)
    full_arm = np.load(full.clip_dir / "arm_q.npz")
    full_hand = np.load(full.clip_dir / "hand_q20.npz")
    # Re-run the audit to get the per-frame torque, then pick its median as the threshold.
    res = rig.run({s: full_arm[s] for s in ca.SIDES}, {s: full_hand[s] for s in ca.SIDES}, 50.0, 1.0)
    threshold = float(np.median(res.frame_torque_ratio))
    thr = ca.Thresholds(max_arm_torque_ratio=threshold, max_contact_force_n=80.0)
    start, stop = pc.longest_passing_window(res.frame_torque_ratio, res.frame_contact_force_n, thr)
    assert 0 < stop - start < 120

    opts = pc.Options(speeds=(1.0,), auto_trim=True, min_seconds=(stop - start) / 50.0,
                      max_arm_torque_ratio=threshold)
    outcome = pc.prepare_one(method_dir, tmp_path / "trimmed", opts, rig, fake_retargeter_factory)
    meta = json.loads((outcome.clip_dir / "clip.json").read_text())
    assert meta["frames"] == stop - start
    assert meta["sanitize"]["trim_start"] == start
    assert meta["sanitize"]["trim_end"] == 120 - stop
    assert f"kept frames [{start}, {stop}) of 120" in meta["sanitize"]["auto_trim_note"]
    arm = np.load(outcome.clip_dir / "arm_q.npz")
    hand = np.load(outcome.clip_dir / "hand_q20.npz")
    for side in ca.SIDES:
        assert np.array_equal(arm[side], full_arm[side][start:stop])
        assert np.array_equal(hand[side], full_hand[side][start:stop])
    # The recorded audit is the second one: it covers the trimmed frames only.
    assert meta["audit"]["per_speed"]["1.0"]["peak_arm_torque_ratio"] != pytest.approx(
        res.frame_torque_ratio.max())


def test_end_to_end_flip_is_refused_and_writes_nothing(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root, flip_frame=60)
    out = tmp_path / "clips"
    with pytest.raises(pc.FlipRefused):
        pc.prepare_one(method_dir, out, pc.Options(), rig, fake_retargeter_factory)
    assert not out.exists()
    assert fake_retargeter_factory.made == []


def test_end_to_end_allow_flips_records_it(rig, fake_retargeter_factory, bundle_root, tmp_path):
    method_dir = make_bundle(bundle_root, flip_frame=60)
    opts = pc.Options(speeds=(1.0,), allow_flips=True)
    outcome = pc.prepare_one(method_dir, tmp_path / "clips", opts, rig, fake_retargeter_factory)
    meta = json.loads((outcome.clip_dir / "clip.json").read_text())
    assert meta["sanitize"]["allow_flips"] is True
    assert meta["sanitize"]["flip_max_step_deg"] >= 90.0
    assert meta["sanitize"]["after"]["max_step_deg"] <= 15.0 + 1e-9


# -- --all -------------------------------------------------------------------

def _rows(summary_text):
    lines = [line for line in summary_text.splitlines() if line.startswith("| ")]
    header, rows = lines[0], lines[1:]
    assert header == ("| clip | verdict | safe speeds | at | peak contact (N) | pair "
                      "| peak torque ratio | reason |")
    return [[c.strip() for c in r.strip("|").split("|")] for r in rows]


def test_all_writes_summary_with_one_row_per_trajectory(rig, fake_retargeter_factory, bundle_root, tmp_path, capsys):
    make_bundle(bundle_root, sample="b_sample", method="GT")
    make_bundle(bundle_root, sample="a_sample", method="Ours")
    out = tmp_path / "clips"
    outcomes = pc.run_all(bundle_root / "samples", out, pc.Options(speeds=(1.0,)), rig, fake_retargeter_factory)
    assert [o.name for o in outcomes] == ["a_sample_Ours", "b_sample_GT"]
    rows = _rows((out / "summary.md").read_text())
    assert len(rows) == 2
    assert rows[0][0] == "a_sample_Ours" and rows[1][0] == "b_sample_GT"
    for row, o in zip(rows, outcomes):
        assert o.verdict in ("safe", "rejected")
        assert row[1] == o.verdict
        assert row[2] == pc.format_speeds(o.safe_speeds)
        assert row[3] == f"{o.reported_speed:g}x"
        assert row[4] == f"{o.peak_contact_force_n:.1f}"
        assert row[6] == f"{o.peak_arm_torque_ratio:.2f}"
        assert (out / o.verdict / o.name / "clip.json").is_file()
    printed = capsys.readouterr().out.splitlines()
    assert len(printed) == 2 and printed[0].startswith("a_sample_Ours: ")


def test_all_refused_flip_becomes_a_row_and_does_not_stop_the_run(rig, fake_retargeter_factory, bundle_root,
                                                                  tmp_path, capsys):
    make_bundle(bundle_root, sample="01", method="GT", flip_frame=60)
    make_bundle(bundle_root, sample="02", method="Ours")
    broken = make_bundle(bundle_root, sample="03", method="GT")
    (broken / "hand2_input" / "gt_human_targets_v5.npz").unlink()  # an unexpected per-trajectory error
    out = tmp_path / "clips"
    outcomes = pc.run_all(bundle_root / "samples", out, pc.Options(speeds=(1.0,)), rig, fake_retargeter_factory)
    assert [(o.name, o.verdict) for o in outcomes] == [("01_GT", "refused"), ("02_Ours", "safe"), ("03_GT", "error")]
    rows = _rows((out / "summary.md").read_text())
    assert len(rows) == 3
    assert rows[0][1] == "refused" and "allow-flips" in rows[0][7] and rows[0][3] == "-"
    assert rows[1][1] == "safe"
    assert rows[2][1] == "error" and "PrepareError" in rows[2][7]
    assert not (out / "safe" / "01_GT").exists() and not (out / "rejected" / "01_GT").exists()
    assert (out / "safe" / "02_Ours" / "clip.json").is_file()
    printed = capsys.readouterr().out.splitlines()
    assert printed[0].startswith("01_GT: refused (") and printed[2].startswith("03_GT: error (")


def test_find_trajectories_sorted_and_only_gt_ours(bundle_root):
    make_bundle(bundle_root, sample="z", method="Ours")
    make_bundle(bundle_root, sample="a", method="GT")
    (bundle_root / "samples" / "a" / "Other").mkdir()
    found = pc.find_trajectories(bundle_root / "samples")
    assert [f"{p.parent.name}/{p.name}" for p in found] == ["a/GT", "z/Ours"]


# -- main and exit codes -----------------------------------------------------

def test_main_refused_flip_exits_2(bundle_root, tmp_path, capsys):
    method_dir = make_bundle(bundle_root, flip_frame=60)
    code = pc.main(["--method-dir", str(method_dir), "--out", str(tmp_path / "clips")])
    assert code == 2
    assert capsys.readouterr().out.startswith("synth_Ours: refused (")
    assert not (tmp_path / "clips").exists()


def test_main_bad_arguments_exit_2(tmp_path):
    with pytest.raises(SystemExit) as info:
        pc.main(["--out", str(tmp_path)])  # neither --method-dir nor --all
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        pc.main(["--method-dir", "a", "--all", "b", "--out", str(tmp_path)])
    assert info.value.code == 2
    assert pc.main(["--method-dir", str(tmp_path), "--out", str(tmp_path), "--speeds", "2.0"]) == 2


def test_main_missing_bundle_exits_1(tmp_path):
    assert pc.main(["--method-dir", str(tmp_path / "nope"), "--out", str(tmp_path / "clips")]) == 1
    assert pc.main(["--all", str(tmp_path), "--out", str(tmp_path / "clips")]) == 1


def test_main_single_clip_exits_0(bundle_root, tmp_path, fake_retargeter_factory, capsys):
    method_dir = make_bundle(bundle_root)
    out = tmp_path / "clips"
    code = pc.main(["--method-dir", str(method_dir), "--out", str(out), "--speeds", "1.0", "--trim-start", "2"],
                   retargeter_factory=fake_retargeter_factory)
    assert code == 0
    line = capsys.readouterr().out.strip()
    assert line.startswith("synth_Ours: ")
    verdict = line.split()[1]
    meta = json.loads((out / verdict / "synth_Ours" / "clip.json").read_text())
    assert meta["frames"] == 118 and meta["sanitize"]["trim_start"] == 2


def test_main_all_exits_0_and_writes_summary(bundle_root, tmp_path, fake_retargeter_factory):
    make_bundle(bundle_root, sample="s1", method="GT")
    out = tmp_path / "clips"
    code = pc.main(["--all", str(bundle_root / "samples"), "--out", str(out), "--speeds", "1.0"],
                   retargeter_factory=fake_retargeter_factory)
    assert code == 0
    assert len(_rows((out / "summary.md").read_text())) == 1


# -- the real retargeter -----------------------------------------------------

def test_real_retargeter_flat_hand_is_finite_and_in_range(rig):
    pytest.importorskip("wuji_retargeting")
    for side in ca.SIDES:
        config = pc.DEFAULT_RETARGET_CONFIG_DIR / pc.RETARGET_CONFIG_PATTERN.format(side=side)
        retargeter = pc.default_retargeter_factory(config, side)
        perm = pc.hardware_perm(retargeter, side)
        assert perm.tolist() == [16, 17, 18, 19, 0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 8, 9, 10, 11]
        kp = np.repeat(flat_hand_keypoints(side)[None], 3, axis=0)
        q = pc.retarget_side(retargeter, perm, kp, np.array([0, 1, 2]))
        assert q.shape == (3, 20) and np.all(np.isfinite(q))
        lo, hi = rig.hand_jnt_range[side][:, 0], rig.hand_jnt_range[side][:, 1]
        assert np.all(q >= lo - 1e-3) and np.all(q <= hi + 1e-3)


# -- meta without source_frames, and which speed a row reports ----------------

def test_read_bundle_falls_back_to_the_keypoint_count_when_meta_omits_source_frames(bundle_root):
    # RobotSTAR_demos/sweep-test omits the key; its keypoints sit on the body grid.
    method_dir = make_bundle(bundle_root, frames=120, source_frames=120, omit_source_frames=True)
    traj = pc.read_bundle(method_dir)
    assert traj.source_frames == 120
    assert traj.keypoints["left"].shape == (120, pc.NUM_KEYPOINTS, 3)


def test_read_bundle_still_refuses_mismatched_keypoint_shapes_without_source_frames(bundle_root):
    method_dir = make_bundle(bundle_root, omit_source_frames=True)
    kp_path = next((method_dir / "hand2_input").glob("*_human_targets_v5.npz"))
    with np.load(kp_path) as data:
        arrays = {k: data[k] for k in data.files}
    arrays["right_hand_keypoints21"] = arrays["right_hand_keypoints21"][:-3]  # sides disagree
    np.savez(kp_path, **arrays)
    with pytest.raises(pc.PrepareError, match="shape"):
        pc.read_bundle(method_dir)


def test_a_safe_clip_reports_the_fastest_passing_speed_not_the_fastest_audited(rig, fake_retargeter_factory,
                                                                               bundle_root, tmp_path):
    # The 05_test_G42xKICVj9U_5-5-rgb_front_GT shape: safe at a slower speed only.
    # The row must carry that speed's numbers, or a "safe" verdict sits next to a
    # saturated joint.
    method_dir = make_bundle(bundle_root)
    out = tmp_path / "clips"
    per_speed = {1.0: {"pass": False, "peak_contact_force_n": 90.0, "peak_contact_pair": ["a", "b"],
                       "peak_arm_torque_ratio": 1.0},
                 0.5: {"pass": True, "peak_contact_force_n": 12.0, "peak_contact_pair": ["c", "d"],
                       "peak_arm_torque_ratio": 0.4}}
    outcome = pc.Outcome(name="x", verdict="safe", safe_speeds=[0.5],
                         peak_contact_force_n=per_speed[0.5]["peak_contact_force_n"],
                         peak_contact_pair=per_speed[0.5]["peak_contact_pair"],
                         peak_arm_torque_ratio=per_speed[0.5]["peak_arm_torque_ratio"],
                         reported_speed=0.5)
    row = pc.summary_row(outcome)
    assert "| 0.5x |" in row and "| 12.0 |" in row and "| 0.40 |" in row
    assert "90.0" not in row and "1.00" not in row

    # And end to end: a real safe clip's row reports one of its safe speeds.
    outcomes = pc.run_all(bundle_root / "samples", out, pc.Options(speeds=(1.0, 0.5)), rig,
                          fake_retargeter_factory)
    for o in outcomes:
        if o.verdict == "safe":
            assert o.reported_speed == max(o.safe_speeds)
