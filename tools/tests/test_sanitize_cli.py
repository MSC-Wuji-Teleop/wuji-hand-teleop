"""End-to-end pins for tools/sanitize/cli.py: exit codes, layouts, the report.

The kinematics-only path (--no-collision) is used wherever the test is not
about collision, because a collision-checked frame costs about a quarter of a
second and these clips only need to be a few frames long.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from clip_audit import ARM_JOINT_NAMES, SIDES
from sanitize import report as report_mod
from sanitize.cli import main

FLAT_COLUMNS = ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"]


@pytest.fixture(scope="module")
def stand_arm():
    """The stand pose's arm values, as one 14-vector."""
    out = []
    for side in SIDES:
        sign = 1.0 if side == "left" else -1.0
        out.extend([0.2, sign * 0.2, 0.0, 1.28, 0.0, 0.0, 0.0])
    return np.array(out)


def write_flat(path, arm, hand_value=0.0, rate_hz=50.0, extra=None):
    frames = arm.shape[0]
    payload = {"arm_q": arm, "arm_joint_names": np.array(FLAT_COLUMNS),
               "left_hand_q20": np.full((frames, 20), hand_value),
               "right_hand_q20": np.full((frames, 20), hand_value),
               "target_fps": np.array(rate_hz), "k": np.array(7)}
    payload.update(extra or {})
    np.savez(path, **payload)
    return path


def wobble(stand, frames=6, amplitude=0.05):
    """A few frames of gentle motion around the stand pose."""
    t = np.linspace(0.0, 1.0, frames)
    arm = np.tile(stand, (frames, 1))
    for j in range(arm.shape[1]):
        arm[:, j] += amplitude * np.sin(2 * math.pi * t + 0.4 * j)
    return arm


def read_report(path):
    return json.loads(path.read_text())


# -- arguments --------------------------------------------------------------

def test_without_out_or_check_it_refuses(tmp_path, stand_arm, capsys):
    src = write_flat(tmp_path / "in.npz", wobble(stand_arm))
    assert main([str(src)]) == report_mod.EXIT_REFUSED
    assert "--check" in capsys.readouterr().err


def test_a_missing_input_is_refused_not_crashed(tmp_path):
    assert main([str(tmp_path / "nope.npz"), "-o", str(tmp_path / "out.npz")]) \
        == report_mod.EXIT_REFUSED


def test_a_malformed_input_is_refused(tmp_path):
    np.savez(tmp_path / "bad.npz", something_else=np.zeros(3))
    assert main([str(tmp_path / "bad.npz"), "-o", str(tmp_path / "out.npz"),
                 "--no-collision"]) == report_mod.EXIT_REFUSED


# -- the flat npz path ------------------------------------------------------

def test_flat_npz_round_trip_is_clean_and_faithful(tmp_path, stand_arm):
    source = wobble(stand_arm)
    src = write_flat(tmp_path / "in.npz", source, extra={"note": np.array(["keep me"])})
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision"]) == report_mod.EXIT_OK

    with np.load(out, allow_pickle=False) as handle:
        # Passthrough keys survive, and the arm columns stay in file order.
        assert handle["note"][0] == "keep me"
        assert [str(n) for n in handle["arm_joint_names"]] == FLAT_COLUMNS
        assert handle["k"] == 7
        # A trajectory that already came from this model comes back as it went in.
        assert np.degrees(np.abs(handle["arm_q"] - source)).max() < 0.01

    data = read_report(tmp_path / "out_sanitize.json")
    assert data["tool"] == "sanitize/1"
    assert data["frames"] == source.shape[0]
    assert data["rate_hz"] == 50.0
    assert data["verdict"] == {"failures": 0, "exit_code": 0}
    assert data["ik"]["frames_not_converged"] == 0
    assert data["source"]["layout"] == "flat_npz"
    assert data["source"]["urdf_sha256"]
    assert data["options"]["collision"] is False


def test_check_mode_writes_nothing(tmp_path, stand_arm):
    src = write_flat(tmp_path / "in.npz", wobble(stand_arm))
    assert main([str(src), "--check", "--no-collision"]) == report_mod.EXIT_OK
    assert list(tmp_path.iterdir()) == [src]


# -- the clip directory path ------------------------------------------------

def test_clip_dir_round_trip_writes_the_report_into_clip_json(tmp_path, stand_arm):
    source = wobble(stand_arm)
    clip = tmp_path / "clip"
    clip.mkdir()
    np.savez(clip / "arm_q.npz", left=source[:, :7], right=source[:, 7:])
    np.savez(clip / "hand_q20.npz",
             left=np.zeros((source.shape[0], 20)), right=np.zeros((source.shape[0], 20)))
    (clip / "clip.json").write_text(json.dumps(
        {"tool": "prepare_clip/1", "frames": source.shape[0], "rate_hz": 50.0}))

    out = tmp_path / "out"
    assert main([str(clip), "-o", str(out), "--no-collision"]) == report_mod.EXIT_OK
    for name in ("arm_q.npz", "hand_q20.npz", "clip.json", "sanitize.json"):
        assert (out / name).is_file(), name

    meta = read_report(out / "clip.json")
    assert meta["tool"] == "prepare_clip/1"
    assert meta["sanitize"]["verdict"]["exit_code"] == 0
    with np.load(out / "arm_q.npz") as handle:
        assert handle["left"].shape == (source.shape[0], 7)
        assert np.degrees(np.abs(handle["left"] - source[:, :7])).max() < 0.01


# -- hands ------------------------------------------------------------------

def test_hand_angles_pass_through_and_out_of_range_ones_are_clamped(tmp_path, stand_arm):
    source = wobble(stand_arm)
    src = write_flat(tmp_path / "in.npz", source, hand_value=0.0)
    out = tmp_path / "out.npz"
    main([str(src), "-o", str(out), "--no-collision"])
    with np.load(out) as handle:
        assert np.allclose(handle["left_hand_q20"], 0.0)      # in range, untouched
    assert read_report(tmp_path / "out_sanitize.json")["hand"]["clamped_values"] == \
        {"left": 0, "right": 0}

    # 10 rad is past every Hand 2 joint limit, so every value must be pulled in.
    src = write_flat(tmp_path / "in2.npz", source, hand_value=10.0)
    out = tmp_path / "out2.npz"
    main([str(src), "-o", str(out), "--no-collision"])
    with np.load(out) as handle:
        assert handle["left_hand_q20"].max() < 10.0
    clamped = read_report(tmp_path / "out2_sanitize.json")["hand"]["clamped_values"]
    assert clamped["left"] == source.shape[0] * 20
    assert clamped["right"] == source.shape[0] * 20


# -- drift ------------------------------------------------------------------

def test_drift_is_measured_but_not_corrected_by_default(tmp_path, stand_arm, capsys):
    frames = 60
    arm = np.tile(stand_arm, (frames, 1))
    # 100 deg of end-to-start roll: warning scale, still inside the +-113 range.
    roll = FLAT_COLUMNS.index("left_wrist_roll")
    arm[:, roll] = np.linspace(0.0, math.radians(100.0), frames)
    src = write_flat(tmp_path / "in.npz", arm)
    out = tmp_path / "out.npz"
    # A one-frame window, so the measurement is the endpoints themselves.
    assert main([str(src), "-o", str(out), "--no-collision",
                 "--drift-window", "1"]) == report_mod.EXIT_OK

    entry = next(d for d in read_report(tmp_path / "out_sanitize.json")["unwrap"]
                 ["wrist_drift"]["left"] if d["joint"] == "left_wrist_roll")
    assert entry["measured_deg"] == pytest.approx(100.0, abs=2.0)
    assert entry["removed_deg"] == 0.0                     # measured, not removed
    assert "DRIFT" in capsys.readouterr().err              # but said out loud
    with np.load(out) as handle:
        assert np.degrees(np.abs(handle["arm_q"] - arm)).max() < 0.05


def test_max_drift_deg_removes_the_excess(tmp_path, stand_arm):
    frames = 60
    arm = np.tile(stand_arm, (frames, 1))
    roll = FLAT_COLUMNS.index("left_wrist_roll")
    arm[:, roll] = np.linspace(0.0, math.radians(120.0), frames)
    src = write_flat(tmp_path / "in.npz", arm)
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision",
                 "--max-drift-deg", "30", "--drift-window", "1"]) == report_mod.EXIT_OK
    entry = next(d for d in read_report(tmp_path / "out_sanitize.json")["unwrap"]
                 ["wrist_drift"]["left"] if d["joint"] == "left_wrist_roll")
    assert entry["removed_deg"] == pytest.approx(90.0, abs=3.0)
    assert entry["residual_deg"] == pytest.approx(30.0, abs=3.0)
    with np.load(out) as handle:
        got = handle["arm_q"][:, roll]
        assert math.degrees(got[-1] - got[0]) == pytest.approx(30.0, abs=3.0)


def test_a_source_past_a_joint_limit_is_clamped_and_counted(tmp_path, stand_arm):
    """An unreachable source angle is not silently followed."""
    frames = 20
    arm = np.tile(stand_arm, (frames, 1))
    # left_wrist_roll stops at 113 deg; ask for 130.
    roll = FLAT_COLUMNS.index("left_wrist_roll")
    arm[:, roll] = np.linspace(0.0, math.radians(130.0), frames)
    src = write_flat(tmp_path / "in.npz", arm)
    out = tmp_path / "out.npz"
    assert main([str(src), "-o", str(out), "--no-collision"]) == report_mod.EXIT_OK

    data = read_report(tmp_path / "out_sanitize.json")
    assert data["source"]["arm_values_outside_joint_limits"] > 0
    with np.load(out) as handle:
        got = handle["arm_q"][:, roll]
        assert math.degrees(got.max()) == pytest.approx(113.0, abs=1.0)
        # ... and the frames that were reachable are still tracked exactly.
        assert np.degrees(np.abs(got[:5] - arm[:5, roll])).max() < 0.05


# -- collision --------------------------------------------------------------

def test_a_clip_with_the_hands_folded_together_reports_contact(tmp_path):
    """The whole point of the tool, end to end, on a deliberately bad clip."""
    frames = 3
    arm = np.zeros((frames, 14))
    for i, side in enumerate(SIDES):
        sign = 1.0 if side == "left" else -1.0
        arm[:, i * 7:(i + 1) * 7] = np.array([-0.35, -sign * 0.35, 0.0, 1.05, 0.0, 0.0, 0.0])
    src = write_flat(tmp_path / "in.npz", arm)
    out = tmp_path / "out.npz"

    code = main([str(src), "-o", str(out)])
    assert code in (report_mod.EXIT_OK, report_mod.EXIT_FAILURES)
    data = read_report(tmp_path / "out_sanitize.json")
    assert data["options"]["collision"] is True
    assert data["collision"]["pairs_checked"] > 3000
    assert data["collision"]["frames_with_contact"] > 0
    # Folding the hands together gives cross-side contact: a failure or near miss.
    reported = (data["collision"]["failures"]
                + data["collision"]["near_miss"]
                + data["collision"]["unfixable"])
    assert any(entry["class"] == "cross_side" for entry in reported)
    # Whatever happened, the clip is still playable: written, right shape.
    with np.load(out) as handle:
        assert handle["arm_q"].shape == (frames, 14)


def test_a_failure_line_names_the_frame_the_time_and_the_pair():
    """The failure report is frame, timestamp, and what was wrong."""
    failure = report_mod.Failure(frame=128, t_s=2.56, klass="cross_side",
                                 a="left_wuji_l_thumb_distal", b="right_wuji_r_pinky_distal",
                                 gap_m=0.0, touching=True, passes=8,
                                 pos_err_m=0.0031, ori_err_rad=math.radians(1.2))
    line = failure.line(clearance_m=0.005)
    assert "frame   128" in line
    assert "t=   2.560s" in line
    assert "cross_side" in line
    assert "left_wuji_l_thumb_distal <-> right_wuji_r_pinky_distal" in line
    assert "TOUCHING" in line
    assert "8 extra solve(s)" in line
    as_dict = failure.as_dict()
    assert as_dict["touching"] is True and as_dict["gap_mm"] is None
    assert as_dict["frame"] == 128 and as_dict["t_s"] == 2.56
