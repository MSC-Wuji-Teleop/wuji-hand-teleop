#!/usr/bin/env python3
"""Tests for tools/make_home_clip.py: the rehome clip generator (docs/spec/spec1_1.md).

Everything here runs without MuJoCo. The generator imports clip_audit inside
its audit step for exactly this reason: the path arithmetic is what decides how
the robot moves, and it should be testable anywhere. The two tests that do
compare against clip_audit skip when it cannot be imported.

What these pin, in order of how much damage getting it wrong would do:

1. Frame 0 equals the start pose to the bit. That is what makes the first
   published frame a no-op against the pose the arms are already in, which is
   the entire reason this design needs no approach ramp.
2. Peak velocity never exceeds HOME_PEAK_VEL_RAD_S, measured off the frames at
   the rate they will be published, not off the formula that produced them.
3. The last frame is the home pose exactly.
4. The clip.json a run writes is one replay/clip.py will load.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import make_home_clip as mhc  # noqa: E402

DT = 1.0 / mhc.HOME_RATE_HZ
SIDES = mhc.SIDES


@pytest.fixture(scope="module")
def model() -> mhc.ModelTables:
    return mhc.load_model_tables()


def start_of(model: mhc.ModelTables, spec: str) -> dict:
    return mhc.parse_start_pose(spec, model)[0]


def measured_peaks(arm_q: dict) -> tuple[float, float]:
    """Peak velocity and acceleration from the frames, at the publish rate."""
    v = a = 0.0
    for side in SIDES:
        v = max(v, float(np.max(np.abs(np.diff(arm_q[side], axis=0)) / DT)))
        a = max(a, float(np.max(np.abs(np.diff(arm_q[side], n=2, axis=0)) / DT ** 2)))
    return v, a


# --- the model tables -------------------------------------------------------


def test_the_model_is_all_hinges_so_declaration_order_is_qpos_order(model):
    """What lets the stand keyframe be read here without a simulator."""
    assert len(model.joint_names) == model.stand_qpos.size


def test_stand_keyframe_arm_values_match_the_documented_pose(model):
    stand = model.stand_arm_pose()
    assert list(stand["left"]) == [0.2, 0.2, 0.0, 1.28, 0.0, 0.0, 0.0]
    assert list(stand["right"]) == [0.2, -0.2, 0.0, 1.28, 0.0, 0.0, 0.0]


def test_shoulder_roll_ranges_are_asymmetric_and_mirrored(model):
    """A symmetric clamp would be wrong: the ranges mirror between the sides."""
    lo_l, hi_l = model.arm_ranges("left")
    lo_r, hi_r = model.arm_ranges("right")
    assert (lo_l[1], hi_l[1]) == pytest.approx((-1.5882, 2.2515))
    assert (lo_r[1], hi_r[1]) == pytest.approx((-2.2515, 1.5882))


def test_hand_names_come_off_the_model_in_hardware_order(model):
    for side, prefix in (("left", "l_"), ("right", "r_")):
        names = model.hand_names(side)
        assert len(names) == mhc.HAND_JOINTS_PER_SIDE
        assert all(n.startswith(prefix) for n in names)
        assert names[0] == f"{prefix}thumb_cmc_flex"


def test_a_missing_model_is_refused(tmp_path):
    with pytest.raises(mhc.HomeClipError, match="model not found"):
        mhc.load_model_tables(tmp_path / "nope.xml")


# --- agreement with the audit ------------------------------------------------


def test_arm_joint_names_equal_the_audits():
    clip_audit = pytest.importorskip("clip_audit")
    assert {s: list(v) for s, v in clip_audit.ARM_JOINT_NAMES.items()} == mhc.ARM_JOINT_NAMES


def test_hand_joint_names_equal_the_audits(model):
    clip_audit = pytest.importorskip("clip_audit")
    for side in SIDES:
        assert model.hand_names(side) == clip_audit.HAND_JOINT_NAMES[side]


# --- duration and the ease ---------------------------------------------------


def test_duration_inverts_the_peak_velocity_of_the_ease():
    travel = 1.0
    duration = mhc.duration_for(travel)
    assert mhc.peak_velocity(travel, duration) == pytest.approx(mhc.HOME_PEAK_VEL_RAD_S)


def test_duration_is_clamped_at_both_ends():
    assert mhc.duration_for(0.0) == mhc.HOME_MIN_DURATION_S
    assert mhc.duration_for(1e6) == mhc.HOME_MAX_DURATION_S


def test_the_duration_ceiling_is_unreachable_from_a_pose_inside_the_ranges(model):
    """It bounds nonsense input, not any real rehome. Stated so a later change
    to HOME_PEAK_VEL_RAD_S that makes the cap bite shows up here."""
    largest = 0.0
    for side in SIDES:
        lo, hi = model.arm_ranges(side)
        home = np.array(mhc.HOME_POSE_RAD[side])
        largest = max(largest, float(np.max(np.maximum(np.abs(home - lo), np.abs(hi - home)))))
    assert largest == pytest.approx(3.0892, abs=1e-3)
    assert mhc.duration_for(largest) < mhc.HOME_MAX_DURATION_S


def test_frame_count_rounds_up_so_the_realised_motion_is_never_faster():
    """A clip of n frames takes (n - 1) / rate seconds. Rounding to nearest
    shortened the motion and put peak velocity over the limit; measured at
    0.2003 rad/s from the stand pose before this was ceil."""
    for travel in (0.05, 0.25, 0.382, 1.0, 2.0, 3.0892):
        nominal = mhc.duration_for(travel)
        realised = (mhc.frame_count(nominal) - 1) / mhc.HOME_RATE_HZ
        assert realised >= nominal - 1e-12


def test_the_ease_starts_and_ends_at_rest():
    s = mhc.ease(101)
    assert s[0] == 0.0 and s[-1] == pytest.approx(1.0)
    d = np.diff(s)
    assert d[0] < d[len(d) // 2] and d[-1] < d[len(d) // 2]
    assert np.all(d >= 0.0)


# --- the path ----------------------------------------------------------------


@pytest.mark.parametrize("spec", ["stand", "zeros", "0.4 " * 13 + "0.4"])
def test_frame_zero_is_the_start_pose_bit_for_bit(model, spec):
    start = start_of(model, spec)
    arm_q, _ = mhc.build_path(start, model)
    for side in SIDES:
        assert np.array_equal(arm_q[side][0], start[side])


@pytest.mark.parametrize("goal", ["zeros", "stand"])
def test_the_last_frame_is_the_home_pose_exactly(model, goal):
    arm_q, _ = mhc.build_path(start_of(model, "zeros" if goal == "stand" else "stand"),
                              model, goal=goal)
    expected = mhc.home_pose(goal, model)
    for side in SIDES:
        assert np.array_equal(arm_q[side][-1], expected[side])


def test_the_last_frame_is_home_pose_rad_by_default(model):
    arm_q, _ = mhc.build_path(start_of(model, "stand"), model)
    for side in SIDES:
        assert np.array_equal(arm_q[side][-1], np.array(mhc.HOME_POSE_RAD[side]))


@pytest.mark.parametrize("spec", ["stand", "0.9 " * 13 + "0.9", "-1.5 " * 13 + "-1.5"])
@pytest.mark.parametrize("goal", ["zeros", "stand"])
def test_peak_velocity_and_acceleration_stay_within_the_constants(model, spec, goal):
    arm_q, segments = mhc.build_path(start_of(model, spec), model, goal=goal)
    v, a = measured_peaks(arm_q)
    assert v <= mhc.HOME_PEAK_VEL_RAD_S
    assert a <= max(b["peak_acc_rad_s2"] for b in segments) * 1.01 + 1e-9


def test_a_straight_path_is_monotone_in_every_joint(model):
    arm_q, _ = mhc.build_path(start_of(model, "stand"), model)
    for side in SIDES:
        for column in range(mhc.ARM_JOINTS_PER_SIDE):
            steps = np.diff(arm_q[side], axis=0)[:, column]
            steps = steps[np.abs(steps) > 1e-15]
            assert steps.size == 0 or np.all(steps > 0) or np.all(steps < 0)


def test_the_motion_starts_and_stops_at_rest(model):
    """No commanded velocity step at either end of the clip."""
    arm_q, _ = mhc.build_path(start_of(model, "stand"), model)
    speeds = np.abs(np.diff(arm_q["left"], axis=0)) / DT
    assert speeds[0].max() < 0.05 * speeds.max()
    assert speeds[-1].max() < 0.05 * speeds.max()


def test_an_unknown_home_pose_is_refused(model):
    with pytest.raises(mhc.HomeClipError, match="unknown home pose"):
        mhc.build_path(start_of(model, "stand"), model, goal="folded")


# --- clamping ----------------------------------------------------------------


def test_a_start_outside_the_model_range_is_kept_at_frame_zero_and_reported(model):
    """Four wrist joints in the handoff note are commanded past their model
    limits, so this happens. Clamping frame 0 would put a step back in."""
    start = start_of(model, "zeros")
    start["left"][5] = 1.70                        # left wrist pitch, limit 1.61443
    arm_q, _ = mhc.build_path(start, model)
    report = mhc.clamp_to_ranges(arm_q, model)

    assert "left_wrist_pitch" in report["left"]
    assert report["left"]["left_wrist_pitch"]["start"] == pytest.approx(1.70)
    assert arm_q["left"][0, 5] == 1.70
    assert arm_q["left"][1, 5] <= model.arm_ranges("left")[1][5]
    assert "right" not in report


def test_an_in_range_start_reports_nothing(model):
    arm_q, _ = mhc.build_path(start_of(model, "stand"), model)
    assert mhc.clamp_to_ranges(arm_q, model) == {}


# --- start poses -------------------------------------------------------------


def test_json_start_pose_is_read_by_joint_name(model, tmp_path):
    document = {
        "captured_utc": "2026-09-03T12:00:00+00:00", "arms": "both",
        **{side: {name: 0.1 * i for i, name in enumerate(mhc.ARM_JOINT_NAMES[side])}
           for side in SIDES},
    }
    path = tmp_path / "measured.json"
    path.write_text(json.dumps(document))
    start, description = mhc.parse_start_pose(f"json:{path}", model)
    assert description.endswith("measured.json")
    assert list(start["left"]) == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_json_start_pose_missing_a_joint_is_refused(model, tmp_path):
    document = {side: {n: 0.0 for n in mhc.ARM_JOINT_NAMES[side][:-1]} for side in SIDES}
    path = tmp_path / "measured.json"
    path.write_text(json.dumps(document))
    with pytest.raises(mhc.HomeClipError, match="missing"):
        mhc.parse_start_pose(f"json:{path}", model)


def test_json_start_pose_missing_a_side_is_refused(model, tmp_path):
    path = tmp_path / "measured.json"
    path.write_text(json.dumps({"left": {n: 0.0 for n in mhc.ARM_JOINT_NAMES["left"]}}))
    with pytest.raises(mhc.HomeClipError, match="no 'right' entry"):
        mhc.parse_start_pose(f"json:{path}", model)


def test_clip_start_pose_reads_the_named_frame(model, tmp_path):
    clip_dir = tmp_path / "safe" / "x"
    clip_dir.mkdir(parents=True)
    arm = {"left": np.tile(np.arange(7, dtype=float), (4, 1)),
           "right": np.tile(-np.arange(7, dtype=float), (4, 1))}
    arm["left"][-1] += 10.0
    np.savez(clip_dir / mhc.ARM_FILE, **arm)

    first, _ = mhc.parse_start_pose(f"clip:{clip_dir}@first", model)
    last, _ = mhc.parse_start_pose(f"clip:{clip_dir}@last", model)
    assert list(first["left"]) == pytest.approx(list(range(7)))
    assert list(last["left"]) == pytest.approx([v + 10.0 for v in range(7)])
    # No @ means the last frame: homing follows a clip that has just played.
    assert list(mhc.parse_start_pose(f"clip:{clip_dir}", model)[0]["left"]) == pytest.approx(list(last["left"]))


@pytest.mark.parametrize("spec", [
    "nonsense", "1 2 3", "json:/definitely/not/here.json",
    "clip:/definitely/not/here@last", "stand@x", "",
])
def test_malformed_start_poses_are_refused(model, spec):
    with pytest.raises(mhc.HomeClipError):
        mhc.parse_start_pose(spec, model)


def test_fourteen_numbers_are_read_left_then_right(model):
    start, description = mhc.parse_start_pose(" ".join(str(i / 10) for i in range(14)), model)
    assert description == "explicit"
    assert list(start["left"]) == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert list(start["right"]) == pytest.approx([0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3])


# --- hand poses --------------------------------------------------------------


def test_the_open_hand_pose_is_the_one_the_driver_homes_to():
    for side in SIDES:
        assert np.array_equal(mhc.open_hand_pose()[side], np.zeros(mhc.HAND_JOINTS_PER_SIDE))


def test_the_curled_hand_pose_flexes_but_does_not_spread(model):
    curled = mhc.curled_hand_pose(model)
    for side in SIDES:
        names = model.hand_names(side)
        for i, name in enumerate(names):
            lo, hi = model.ranges[f"{mhc.HAND_MJCF_PREFIX[side]}{name}"]
            if name.endswith(mhc.ABDUCTION_SUFFIX):
                assert curled[side][i] == 0.0
            else:
                assert curled[side][i] == pytest.approx(mhc.CURLED_FLEX_FRACTION * hi)
                assert lo <= curled[side][i] <= hi


def test_the_curled_pips_land_near_the_curl_the_runbook_describes(model):
    """docs/replay.md puts the sweep clip's donor pose near 1.4 rad at the PIPs."""
    curled = mhc.curled_hand_pose(model)
    names = model.hand_names("left")
    pips = [curled["left"][i] for i, n in enumerate(names) if n.endswith("_pip")]
    assert pips and all(1.3 <= v <= 1.6 for v in pips)


def test_an_unknown_hand_pose_is_refused(model):
    with pytest.raises(mhc.HomeClipError, match="unknown hand pose"):
        mhc.hand_pose("fist", model)


# --- the written clip ---------------------------------------------------------


def fake_audit_block(passing: bool) -> tuple[dict, str, list]:
    summary = {"pass": passing, "peak_arm_torque_ratio": 0.3 if passing else 1.0,
               "peak_contact_force_n": 1.0, "peak_contact_pair": None}
    block = {"model": "g1_29_wuji2_fixed.xml", "speeds": [1.0],
             "per_speed": {"1.0": {"pass": passing, "worst_hand_pose": "curled",
                                   "per_hand_pose": {"open": summary, "curled": summary}}}}
    return block, (mhc.VERDICT_SAFE if passing else mhc.VERDICT_REJECTED), ([1.0] if passing else [])


@pytest.mark.parametrize("passing,expected_dir", [(True, "home"), (False, "rejected")])
def test_a_clip_is_filed_by_its_verdict(model, tmp_path, passing, expected_dir):
    start = start_of(model, "stand")
    arm_q, segments = mhc.build_path(start, model)
    outside = mhc.clamp_to_ranges(arm_q, model)
    frames = arm_q["left"].shape[0]
    hand = mhc.open_hand_pose()
    hand_q20 = {s: np.tile(hand[s], (frames, 1)) for s in SIDES}
    block, verdict, speeds = fake_audit_block(passing)
    meta = mhc.build_clip_json(model, start, "stand", segments, frames,
                               outside, block, verdict, speeds, "open")
    out = mhc.write_clip_dir(tmp_path, "home_test", verdict, arm_q, hand_q20, meta)

    assert out.parent.name == expected_dir
    assert (out / mhc.ARM_FILE).is_file() and (out / mhc.HAND_FILE).is_file()
    assert not (tmp_path / mhc.CANDIDATE_DIR / "home_test").exists()
    with np.load(out / mhc.ARM_FILE) as data:
        assert data["left"].shape == (frames, mhc.ARM_JOINTS_PER_SIDE)


def test_the_written_clip_json_is_one_the_publisher_will_load(model, tmp_path):
    """Round trip through replay/clip.py, so the two formats cannot drift apart."""
    # The replay package is installed in the container; from a bare checkout,
    # point at it in the source tree so this round trip still runs.
    replay_pkg = TOOLS_DIR.parent / "src" / "input_devices" / "replay"
    if replay_pkg.is_dir() and str(replay_pkg) not in sys.path:
        sys.path.insert(0, str(replay_pkg))
    load_clip = pytest.importorskip("replay.clip", reason="replay package not importable").load_clip

    start = start_of(model, "stand")
    arm_q, segments = mhc.build_path(start, model)
    outside = mhc.clamp_to_ranges(arm_q, model)
    frames = arm_q["left"].shape[0]
    hand = mhc.curled_hand_pose(model)
    hand_q20 = {s: np.tile(hand[s], (frames, 1)) for s in SIDES}
    block, verdict, speeds = fake_audit_block(True)
    meta = mhc.build_clip_json(model, start, "stand", segments, frames,
                               outside, block, verdict, speeds, "curled")
    out = mhc.write_clip_dir(tmp_path, "home_20260903T120000Z", verdict, arm_q, hand_q20, meta)

    clip = load_clip(out)
    assert clip.frames == frames
    assert clip.rate_hz == mhc.HOME_RATE_HZ
    assert clip.safe_speeds == (1.0,)
    assert clip.arm_names["left"] == tuple(mhc.ARM_JOINT_NAMES["left"])
    assert clip.hand_names["right"] == tuple(model.hand_names("right"))


def test_clip_json_records_that_the_hand_columns_are_never_published(model):
    start = start_of(model, "stand")
    arm_q, segments = mhc.build_path(start, model)
    block, verdict, speeds = fake_audit_block(True)
    meta = mhc.build_clip_json(model, start, "stand", segments,
                               arm_q["left"].shape[0], {}, block, verdict, speeds, "curled")
    assert meta["hands"]["published"] is False
    assert meta["hands"]["columns"] == "curled"
    assert meta["home"]["ease"] == "half_cosine"
    assert meta["home"]["limits"]["peak_vel_rad_s"] == mhc.HOME_PEAK_VEL_RAD_S
    assert meta["source"]["home_rad"]["left"] == list(mhc.HOME_POSE_RAD["left"])


def test_the_clip_name_is_a_utc_stamp():
    from datetime import datetime, timezone
    name = mhc.clip_name(datetime(2026, 9, 3, 12, 0, 0, tzinfo=timezone.utc))
    assert name == "home_20260903T120000Z"


# --- the home pose itself -----------------------------------------------------


def test_home_is_all_zeros(model):
    """The vendor's arm_sdk zero posture. Changing it is a decision, not a tweak:
    docs/spec/spec1_1.md records why, and the audit matrix is what may move it."""
    for side in SIDES:
        assert mhc.HOME_POSE_RAD[side] == (0.0,) * mhc.ARM_JOINTS_PER_SIDE


def test_home_sits_inside_every_joint_range(model):
    for side in SIDES:
        lo, hi = model.arm_ranges(side)
        home = np.array(mhc.HOME_POSE_RAD[side])
        assert np.all(home >= lo) and np.all(home <= hi)
