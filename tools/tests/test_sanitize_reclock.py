"""Pins for tools/sanitize/reclock.py: the wrist re-clock, its gate and its report.

The angle is a rotation of {side}_wrist_yaw_link about its own +x. The
closed form is the exact orientation answer for the three wrist joints; the
CLI tests show the IK reproduces that orientation and, unlike the closed
form, keeps the wrist link where the source put it.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from clip_audit import ARM_JOINT_NAMES, SIDES
from sanitize import reclock
from sanitize import report as report_mod
from sanitize.cli import main

FLAT_COLUMNS = ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"]

# The stand pose's arm values, one 14-vector (left block then right).
STAND = np.array([0.2, 0.2, 0.0, 1.28, 0.0, 0.0, 0.0,
                  0.2, -0.2, 0.0, 1.28, 0.0, 0.0, 0.0])


def wobble(frames=6, amplitude=0.05):
    """A few frames of gentle motion around the stand pose, wrists included."""
    t = np.linspace(0.0, 1.0, frames)
    arm = np.tile(STAND, (frames, 1))
    for j in range(arm.shape[1]):
        arm[:, j] += amplitude * np.sin(2 * math.pi * t + 0.4 * j)
    return arm


def write_flat(path, arm, rate_hz=50.0):
    frames = arm.shape[0]
    np.savez(path, arm_q=arm, arm_joint_names=np.array(FLAT_COLUMNS),
             left_hand_q20=np.zeros((frames, 20)), right_hand_q20=np.zeros((frames, 20)),
             target_fps=np.array(rate_hz))
    return path


def write_clip_dir(path, arm, meta_extra=None):
    path.mkdir()
    np.savez(path / "arm_q.npz", left=arm[:, :7], right=arm[:, 7:])
    np.savez(path / "hand_q20.npz",
             left=np.zeros((arm.shape[0], 20)), right=np.zeros((arm.shape[0], 20)))
    meta = {"tool": "prepare_clip/1", "frames": arm.shape[0], "rate_hz": 50.0}
    meta.update(meta_extra or {})
    (path / "clip.json").write_text(json.dumps(meta))
    return path


def wrist_pose(sm, arm14, side):
    """(rotation, translation) of {side}_wrist_yaw_link for one 14-vector."""
    pose = sm.frame_pose(sm.configuration(arm_all=arm14), sm.wrist_frame[side])
    return np.asarray(pose.rotation).copy(), np.asarray(pose.translation).copy()


def angle_between(ra, rb):
    cos = (np.trace(ra.T @ rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def load_arm_all(path):
    if path.is_dir():
        with np.load(path / "arm_q.npz") as handle:
            return np.concatenate([handle["left"], handle["right"]], axis=1)
    with np.load(path) as handle:
        columns = [str(n) for n in handle["arm_joint_names"]]
        return handle["arm_q"][:, [columns.index(n) for n in FLAT_COLUMNS]]


# -- the flag -----------------------------------------------------------------

def test_parse_clock_accepts_the_three_words_and_a_pair_of_angles():
    assert reclock.parse_clock("none") is None
    assert reclock.parse_clock("bundle") == {"left": 90.0, "right": -90.0}
    assert reclock.parse_clock(" 30,-30 ") == {"left": 30.0, "right": -30.0}
    for bad in ("sideways", "1,2,3", "nan,1", "90", ""):
        with pytest.raises(ValueError):
            reclock.parse_clock(bad)


def test_auto_follows_the_clip_json_provenance():
    legacy = {"source": {"detected_hand_model": "legacy_wuji"}}
    clock, reason = reclock.clock_for_clip("auto", legacy)
    assert clock == reclock.BUNDLE_WRIST_CLOCK_DEG
    assert "legacy_wuji" in reason

    clock, reason = reclock.clock_for_clip("auto", {})
    assert clock is None and "no source.detected_hand_model" in reason

    clock, reason = reclock.clock_for_clip("auto", {"source": {"detected_hand_model": "wuji_hand_2"}})
    assert clock is None and "wuji_hand_2" in reason

    # A source block without the key, or with null (a re-prepared sweep clip): no re-clock.
    for meta in ({"source": {"sample": "90_sweep_joints"}}, {"source": {"detected_hand_model": None}}):
        clock, reason = reclock.clock_for_clip("auto", meta)
        assert clock is None and "no source.detected_hand_model" in reason

    # An explicit word ignores the provenance either way.
    assert reclock.clock_for_clip("none", legacy)[0] is None
    assert reclock.clock_for_clip("bundle", {})[0] == reclock.BUNDLE_WRIST_CLOCK_DEG


def test_the_bundle_clock_is_ninety_degrees_mirrored():
    """The value the analysis settled on; a change here is a decision, not a tweak."""
    assert reclock.BUNDLE_WRIST_CLOCK_DEG == {"left": 90.0, "right": -90.0}
    assert reclock.BUNDLE_HAND_MODEL == "legacy_wuji"


# -- the closed form ----------------------------------------------------------

def test_closed_form_is_a_roll_offset_only_when_pitch_and_yaw_are_zero():
    q7 = np.array([0.1, 0.2, 0.3, 1.0, 0.3, 0.0, 0.0])
    out = reclock.closed_form_wrist(q7, 90.0)
    assert np.allclose(out[:4], q7[:4])
    assert out[4] == pytest.approx(0.3 + math.pi / 2, abs=1e-12)
    assert abs(out[5]) < 1e-12 and abs(out[6]) < 1e-12


def test_adding_the_angle_to_roll_alone_is_wrong_once_pitch_or_yaw_move():
    """Guard: the re-clock swaps the roles of pitch and yaw, so a roll offset is not it."""
    from scipy.spatial.transform import Rotation
    q7 = np.array([0.0, 0.0, 0.0, 1.0, 0.2, 0.5, -0.4])
    exact = reclock.closed_form_wrist(q7, 90.0)
    target = Rotation.from_euler("XYZ", exact[4:7]).as_matrix()
    roll_only = Rotation.from_euler("XYZ", [q7[4] + math.pi / 2, q7[5], q7[6]]).as_matrix()
    assert angle_between(roll_only, target) > 10.0


def test_closed_form_matches_forward_kinematics_in_orientation_not_position(sanitize_model):
    """FK(q') rotation is FK(q) rotation times R_x(deg); the link origin moves."""
    sm = sanitize_model
    rng = np.random.default_rng(7)
    moved = 0
    for _ in range(12):
        arm = STAND.copy()
        for i, side in enumerate(SIDES):
            cols = slice(i * 7, (i + 1) * 7)
            arm[cols] = STAND[cols] + rng.uniform(-0.6, 0.6, 7)
        arm = np.clip(arm, sm.arm_lower, sm.arm_upper)
        for i, side in enumerate(SIDES):
            deg = reclock.BUNDLE_WRIST_CLOCK_DEG[side]
            cols = slice(i * 7, (i + 1) * 7)
            re = arm.copy()
            re[cols] = reclock.closed_form_wrist(arm[cols], deg)
            rot_src, pos_src = wrist_pose(sm, arm, side)
            rot_new, pos_new = wrist_pose(sm, re, side)
            assert angle_between(rot_new, rot_src @ reclock.rotation_about_x(deg)) < 1e-4
            moved += np.linalg.norm(pos_new - pos_src) > 1e-3
    assert moved > 0  # the closed form is not a placement answer


def test_closed_form_seed_clamps_into_the_joint_range(sanitize_model):
    sm = sanitize_model
    arm = STAND.copy()
    arm[4] = math.radians(30.0)               # left roll 30 + 90 = 120, past +113
    seed = reclock.closed_form_seed(arm, reclock.BUNDLE_WRIST_CLOCK_DEG, sm.arm_lower, sm.arm_upper)
    assert seed[4] == pytest.approx(sm.arm_upper[4])
    assert np.all(seed >= sm.arm_lower - 1e-12) and np.all(seed <= sm.arm_upper + 1e-12)
    assert np.array_equal(reclock.closed_form_seed(arm, None, sm.arm_lower, sm.arm_upper), arm)

    # Past 180 the Euler extraction wraps (190 deg is -170 deg), and the clamp
    # then lands on the lower bound, which is the nearer reachable angle.
    arm[4] = math.radians(100.0)
    seed = reclock.closed_form_seed(arm, reclock.BUNDLE_WRIST_CLOCK_DEG, sm.arm_lower, sm.arm_upper)
    assert seed[4] == pytest.approx(sm.arm_lower[4])
    assert np.all(seed >= sm.arm_lower - 1e-12) and np.all(seed <= sm.arm_upper + 1e-12)


def test_reclock_placement_rotates_about_the_links_own_x_and_keeps_the_origin(sanitize_model):
    sm = sanitize_model
    pose = sm.frame_pose(sm.configuration(arm_all=STAND), sm.wrist_frame["left"])
    out = reclock.reclock_placement(sm.pin, pose, 90.0)
    assert np.allclose(out.translation, pose.translation)
    assert np.allclose(out.rotation, np.asarray(pose.rotation) @ reclock.rotation_about_x(90.0))
    # The link's own x axis is unchanged by a rotation about it.
    assert np.allclose(out.rotation[:, 0], np.asarray(pose.rotation)[:, 0])
    same = reclock.reclock_placement(sm.pin, pose, 0.0)
    assert np.allclose(same.rotation, pose.rotation) and np.allclose(same.translation, pose.translation)


# -- the CLI ------------------------------------------------------------------

def test_a_legacy_hand_clip_is_reclocked_and_the_ik_keeps_the_wrist_in_place(tmp_path, sanitize_model):
    sm = sanitize_model
    source = wobble()
    clip = write_clip_dir(tmp_path / "clip", source,
                          {"source": {"detected_hand_model": "legacy_wuji"}})
    out = tmp_path / "out"
    assert main([str(clip), "-o", str(out), "--no-collision"]) == report_mod.EXIT_OK

    data = json.loads((out / "sanitize.json").read_text())
    assert data["wrist_clock"] == {"applied": True, "deg": {"left": 90.0, "right": -90.0},
                                   "reason": "clip.json source.detected_hand_model is legacy_wuji"}
    assert data["options"]["wrist_clock"] == "auto"
    # The solve delivers about 0.15 mm and 4e-5 deg on this fixture; nothing binds.
    assert data["ik"]["max_wrist_ori_residual_deg"] < 0.01
    assert data["ik"]["max_wrist_pos_residual_mm"] < 1.0

    got = load_arm_all(out)
    for k in range(source.shape[0]):
        for side in SIDES:
            deg = reclock.BUNDLE_WRIST_CLOCK_DEG[side]
            rot_src, pos_src = wrist_pose(sm, source[k], side)
            rot_out, pos_out = wrist_pose(sm, got[k], side)
            # Orientation: the source's, turned about the forearm by the clock.
            assert angle_between(rot_out, rot_src @ reclock.rotation_about_x(deg)) < 0.01
            # Placement: held to the solve's floor, which the closed form alone would not do.
            assert np.linalg.norm(pos_out - pos_src) < 1e-3
    # And the joints really moved: this is not the input copied through.
    assert np.degrees(np.abs(got - source)).max() > 45.0


def test_a_clip_without_provenance_is_left_alone(tmp_path):
    source = wobble()
    src = write_flat(tmp_path / "in.npz", source)
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision"]) == report_mod.EXIT_OK
    data = json.loads((tmp_path / "out_sanitize.json").read_text())
    assert data["wrist_clock"]["applied"] is False
    assert data["wrist_clock"]["deg"] is None
    assert "no source.detected_hand_model" in data["wrist_clock"]["reason"]
    assert np.degrees(np.abs(load_arm_all(out) - source)).max() < 0.01


def test_none_overrides_the_provenance(tmp_path):
    source = wobble()
    clip = write_clip_dir(tmp_path / "clip", source,
                          {"source": {"detected_hand_model": "legacy_wuji"}})
    out = tmp_path / "out"
    assert main([str(clip), "-o", str(out), "--no-collision", "--wrist-clock", "none"]) \
        == report_mod.EXIT_OK
    data = json.loads((out / "sanitize.json").read_text())
    assert data["wrist_clock"] == {"applied": False, "deg": None, "reason": "--wrist-clock none"}
    assert np.degrees(np.abs(load_arm_all(out) - source)).max() < 0.01


def test_explicit_angles_apply_to_a_clip_without_provenance(tmp_path, sanitize_model):
    sm = sanitize_model
    source = wobble()
    src = write_flat(tmp_path / "in.npz", source)
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision", "--wrist-clock", "20,-20"]) \
        == report_mod.EXIT_OK
    data = json.loads((tmp_path / "out_sanitize.json").read_text())
    assert data["wrist_clock"]["deg"] == {"left": 20.0, "right": -20.0}
    got = load_arm_all(out)
    for side, deg in (("left", 20.0), ("right", -20.0)):
        rot_src, _ = wrist_pose(sm, source[0], side)
        rot_out, _ = wrist_pose(sm, got[0], side)
        assert angle_between(rot_out, rot_src @ reclock.rotation_about_x(deg)) < 0.5


def test_a_bad_wrist_clock_is_refused_before_the_model_loads(tmp_path, capsys, monkeypatch):
    from sanitize import cli as cli_mod

    def no_model(*args, **kwargs):
        raise AssertionError("the model was built before the flag was resolved")
    monkeypatch.setattr(cli_mod, "SanitizeModel", no_model)

    src = write_flat(tmp_path / "in.npz", wobble())
    assert main([str(src), "-o", str(tmp_path / "out.npz"), "--no-collision",
                 "--wrist-clock", "sideways"]) == report_mod.EXIT_REFUSED
    assert "--wrist-clock" in capsys.readouterr().err
    assert not (tmp_path / "out.npz").exists()


def test_a_negative_left_angle_needs_the_equals_form(tmp_path, sanitize_model):
    """argparse reads '-20,20' as a flag; '--wrist-clock=-20,20' is the documented spelling."""
    sm = sanitize_model
    source = wobble()
    src = write_flat(tmp_path / "in.npz", source)
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision", "--wrist-clock=-20,20"]) \
        == report_mod.EXIT_OK
    data = json.loads((tmp_path / "out_sanitize.json").read_text())
    assert data["wrist_clock"]["deg"] == {"left": -20.0, "right": 20.0}
    got = load_arm_all(out)
    for side, deg in (("left", -20.0), ("right", 20.0)):
        rot_src, _ = wrist_pose(sm, source[0], side)
        rot_out, _ = wrist_pose(sm, got[0], side)
        assert angle_between(rot_out, rot_src @ reclock.rotation_about_x(deg)) < 0.01


# -- what the sign means, so a flipped side fails on kinematics, not on a literal --

HAND_LINKS = ("wrist", "index_finger_proximal", "middle_finger_proximal",
              "ring_finger_proximal", "pinky_proximal", "thumb_proximal")


def hand_points(sm, arm14, side):
    """World positions of the hand's wrist body and five proximal bodies for one 14-vector."""
    q = sm.configuration(arm_all=arm14)
    sm.update(q)
    p = "l" if side == "left" else "r"
    return {name: np.asarray(sm.data.oMf[sm._frame_id(f"{side}_wuji_{p}_{name}")].translation).copy()
            for name in HAND_LINKS}


def palm_normal(pts, side):
    """Out of the palm: -cross(index - wrist, pinky - wrist), the bundle's formula, negated for the right."""
    n = -np.cross(pts["index_finger_proximal"] - pts["wrist"], pts["pinky_proximal"] - pts["wrist"])
    if side == "right":
        n = -n
    return n / np.linalg.norm(n)


def thumb_direction(pts):
    """Unit vector from the wrist body to the thumb's proximal body."""
    v = pts["thumb_proximal"] - pts["wrist"]
    return v / np.linalg.norm(v)


def test_the_bundle_clock_means_palms_down_and_thumbs_inward_at_arm_zero(sanitize_model):
    """At arm joint zero the G1 holds its arms straight forward. This rig's mount puts the
    palms facing each other, thumbs up. The bundle's mount, which the clock reproduces
    when applied to a zero pose, puts them palm down with the thumbs toward the midline
    (docs/issues/wrist-clock-2026-09-11.md, "What is wrong"). Flipping either side's sign
    turns that palm up instead."""
    sm = sanitize_model
    zero = np.zeros(14)
    for i, side in enumerate(SIDES):
        cols = slice(i * 7, (i + 1) * 7)
        ours = hand_points(sm, zero, side)
        assert abs(palm_normal(ours, side)[2]) < 0.2            # ours: palm faces sideways
        assert thumb_direction(ours)[2] > 0.5                     # thumb up

        re = zero.copy()
        re[cols] = reclock.closed_form_wrist(zero[cols], reclock.BUNDLE_WRIST_CLOCK_DEG[side])
        theirs = hand_points(sm, re, side)
        assert palm_normal(theirs, side)[2] < -0.9                # bundle clock: palm down
        thumb_y = thumb_direction(theirs)[1]
        assert (thumb_y < -0.5) if side == "left" else (thumb_y > 0.5)  # thumb toward the midline

        flipped = zero.copy()
        flipped[cols] = reclock.closed_form_wrist(zero[cols], -reclock.BUNDLE_WRIST_CLOCK_DEG[side])
        assert palm_normal(hand_points(sm, flipped, side), side)[2] > 0.9  # the wrong sign: palm up


BUNDLE_SAMPLE = (Path(__file__).resolve().parents[2] / "RobotSTAR_demos" / "samples"
                 / "05_test_G42xKICVj9U_5-5-rgb_front" / "GT")


def kabsch(P, Q):
    """Rotation R minimising sum |R p - q| over paired unit vectors."""
    U, _, Vt = np.linalg.svd(Q.T @ P)
    return U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def palm_basis_from(pts, side):
    """The bundle's palm basis: distal to the MCP centre, normal by its own formula, both hands alike."""
    w = pts["wrist"]
    mcps = np.stack([pts[f"{f}_proximal"] for f in ("index_finger", "middle_finger", "ring_finger", "pinky")])
    x = mcps.mean(0) - w
    x /= np.linalg.norm(x)
    n = -np.cross(pts["index_finger_proximal"] - w, pts["pinky_proximal"] - w)
    n -= x * np.dot(n, x)
    n /= np.linalg.norm(n)
    return np.stack([x, np.cross(n, x), n], 1)


@pytest.mark.skipif(not (BUNDLE_SAMPLE / "g1_reference" / "controller_reference_v7.npz").is_file(),
                    reason="RobotSTAR_demos bundle not on this machine")
def test_the_bundle_clock_brings_the_bundle_palms_onto_their_own_targets(sanitize_model):
    """On real bundle data: the bundle's joints on our model miss its human palm targets by
    about 90 deg; re-clocked in closed form they land within 30 deg (the bundle's own IK
    error); the opposite sign makes it worse than 120 deg. World alignment uses arm segment
    directions only, so the hand mount does not enter it."""
    sm = sanitize_model
    meta = json.loads((BUNDLE_SAMPLE / "g1_reference" / "target_meta.json").read_text())
    acts = meta["joint_actuator_order"]["body_actuators"]
    body_q = np.load(BUNDLE_SAMPLE / "g1_reference" / "controller_reference_v7.npz")["body_q"].astype(float)
    human = np.load(next((BUNDLE_SAMPLE / "hand2_input").glob("*_human_targets_v5.npz")))
    col = {n: i for i, n in enumerate(acts)}
    arm_cols = [col[n] for s in SIDES for n in ARM_JOINT_NAMES[s]]
    frames = [i for i in range(int(meta["ramp_in_frames"]) + 2, body_q.shape[0], 5)
              if round(0.4 * i) < human["left_wrist"].shape[0]]

    def errors(clock_sign):
        hum_dirs, rob_dirs, bases = [], [], {s: [] for s in SIDES}
        for i in frames:
            k = int(round(0.4 * i))
            arm = body_q[i, arm_cols]
            if clock_sign:
                for j, s in enumerate(SIDES):
                    c = slice(j * 7, (j + 1) * 7)
                    arm[c] = reclock.closed_form_wrist(arm[c], clock_sign * reclock.BUNDLE_WRIST_CLOCK_DEG[s])
            q = sm.configuration(arm_all=arm)
            sm.update(q)
            for s in SIDES:
                sh = np.asarray(sm.data.oMf[sm._frame_id(f"{s}_shoulder_roll_link")].translation)
                el = np.asarray(sm.data.oMf[sm.elbow_frame[s]].translation)
                pts = hand_points(sm, arm, s)
                rob_dirs += [el - sh, pts["wrist"] - el]
                hum_dirs += [human[f"{s}_elbow"][k] - human[f"{s}_shoulder"][k],
                             human[f"{s}_wrist"][k] - human[f"{s}_elbow"][k]]
                bases[s].append((palm_basis_from(pts, s), human[f"{s}_palm_basis_world"][k]))
        A = np.array(hum_dirs); A /= np.linalg.norm(A, axis=1, keepdims=True)
        B = np.array(rob_dirs); B /= np.linalg.norm(B, axis=1, keepdims=True)
        align = kabsch(A, B)
        return {s: float(np.median([angle_between(br, align @ bh) for br, bh in bases[s]])) for s in SIDES}

    before, after, wrong = errors(0), errors(+1), errors(-1)
    for s in SIDES:
        assert before[s] > 60.0, (s, before)
        assert after[s] < 30.0, (s, after)
        assert wrong[s] > 120.0, (s, wrong)
