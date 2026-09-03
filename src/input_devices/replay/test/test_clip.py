"""Pins the clip directory rules in replay/clip.py: what loads, what is refused, and the speed arithmetic."""

import json
from pathlib import Path

import numpy as np
import pytest

from replay.clip import (
    PLAYABLE_PARENT_DIR_NAMES,
    SIDE_CHOICES,
    ClipError,
    check_speed,
    default_speed,
    duration_s,
    frame_period,
    load_clip,
    parse_sides,
)
from .conftest import ARM_NAMES, CLIP_NAME, FRAMES, HAND_NAMES, RATE_HZ, clip_meta, synthetic_arrays, write_clip


def test_valid_clip_loads(clip_dir: Path):
    clip = load_clip(clip_dir)
    assert clip.name == CLIP_NAME
    assert clip.frames == FRAMES
    assert clip.rate_hz == RATE_HZ
    assert clip.verdict == "safe"
    assert clip.safe_speeds == (1.0, 0.5)
    for side in ("left", "right"):
        assert clip.arm_names[side] == ARM_NAMES[side]
        assert clip.hand_names[side] == HAND_NAMES[side]
        assert clip.arm_q[side].shape == (FRAMES, 7)
        assert clip.hand_q20[side].shape == (FRAMES, 20)
        assert clip.arm_q[side].dtype == np.float64
    arm, hand = synthetic_arrays(FRAMES)
    np.testing.assert_array_equal(clip.arm_q["right"], arm["right"])
    np.testing.assert_array_equal(clip.hand_q20["left"], hand["left"])
    assert clip.meta["tool"] == "prepare_clip/1"


def test_clip_accepts_a_relative_path_and_a_trailing_slash(clip_dir: Path, monkeypatch):
    monkeypatch.chdir(clip_dir.parent.parent)
    assert load_clip(Path("safe") / CLIP_NAME).name == CLIP_NAME
    assert load_clip(str(clip_dir) + "/").name == CLIP_NAME


def test_parent_not_a_playable_directory_is_refused(tmp_path: Path):
    """Only directories a tool files a clip into after an audit are playable.

    clips/rejected/ is where both prepare_clip.py and make_home_clip.py put a
    clip the audit turned down, so it has to stay out of this set.
    """
    for parent in ("candidate", "rejected", "clips", "homes", "Safe"):
        d = write_clip(tmp_path / parent / CLIP_NAME)
        with pytest.raises(ClipError, match="not under a directory named"):
            load_clip(d)


def test_playable_parents_are_safe_and_home(tmp_path: Path):
    assert PLAYABLE_PARENT_DIR_NAMES == ("safe", "home")


def test_a_home_clip_loads(tmp_path: Path):
    """tools/make_home_clip.py files an audited rehome clip under clips/home/."""
    d = write_clip(tmp_path / "home" / "home_20260903T120000Z")
    clip = load_clip(d)
    assert clip.name == "home_20260903T120000Z"
    assert clip.verdict == "safe"


def test_a_home_clip_the_audit_rejected_is_still_refused(tmp_path: Path):
    """A rejected home clip is filed under rejected/, and would be refused there.

    This pins the other half: even if one were placed under home/, the verdict
    check still stops it. The directory name is not the only guard.
    """
    d = write_clip(tmp_path / "home" / "home_20260903T120000Z", clip_meta(verdict="rejected"))
    with pytest.raises(ClipError, match="verdict is 'rejected'"):
        load_clip(d)


def test_verdict_rejected_is_refused(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(verdict="rejected"))
    with pytest.raises(ClipError, match="verdict is 'rejected'"):
        load_clip(d)


def test_empty_safe_speeds_is_refused(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=()))
    with pytest.raises(ClipError, match="safe_speeds is empty"):
        load_clip(d)


def test_safe_speed_above_one_is_refused(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=(1.5,)))
    with pytest.raises(ClipError, match="safe_speeds entry 1.5"):
        load_clip(d)


def test_wrong_frame_count_is_refused(tmp_path: Path):
    arm, _ = synthetic_arrays(FRAMES + 1)
    d = write_clip(tmp_path / "safe" / CLIP_NAME, arm_q=arm)
    with pytest.raises(ClipError, match=f"{FRAMES + 1} frames, clip.json says {FRAMES}"):
        load_clip(d)


def test_wrong_width_is_refused(tmp_path: Path):
    _, hand = synthetic_arrays(FRAMES)
    hand["left"] = hand["left"][:, :19]
    d = write_clip(tmp_path / "safe" / CLIP_NAME, hand_q20=hand)
    with pytest.raises(ClipError, match=r"expected \(T, 20\)"):
        load_clip(d)


def test_non_finite_value_is_refused(tmp_path: Path):
    _, hand = synthetic_arrays(FRAMES)
    hand["right"][3, 4] = np.nan
    d = write_clip(tmp_path / "safe" / CLIP_NAME, hand_q20=hand)
    with pytest.raises(ClipError, match="non-finite"):
        load_clip(d)


def test_missing_side_key_is_refused(tmp_path: Path):
    arm, _ = synthetic_arrays(FRAMES)
    del arm["right"]
    d = write_clip(tmp_path / "safe" / CLIP_NAME, arm_q=arm)
    with pytest.raises(ClipError, match="missing key 'right'"):
        load_clip(d)


def test_wrong_prefix_is_refused(tmp_path: Path):
    hand_names = dict(HAND_NAMES)
    hand_names["left"] = HAND_NAMES["right"]  # r_ names filed under left
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(hand_names=hand_names))
    with pytest.raises(ClipError, match="without the 'l_' prefix"):
        load_clip(d)
    arm_names = dict(ARM_NAMES)
    arm_names["right"] = ARM_NAMES["left"]
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(arm_names=arm_names))
    with pytest.raises(ClipError, match="without the 'right_' prefix"):
        load_clip(d)


def test_duplicate_names_are_refused(tmp_path: Path):
    arm_names = dict(ARM_NAMES)
    arm_names["left"] = ARM_NAMES["left"][:6] + (ARM_NAMES["left"][0],)
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(arm_names=arm_names))
    with pytest.raises(ClipError, match="duplicate names"):
        load_clip(d)


def test_wrong_name_count_is_refused(tmp_path: Path):
    hand_names = dict(HAND_NAMES)
    hand_names["right"] = HAND_NAMES["right"][:19]
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(hand_names=hand_names))
    with pytest.raises(ClipError, match="has 19 names, expected 20"):
        load_clip(d)


@pytest.mark.parametrize("fname", ["arm_q.npz", "hand_q20.npz", "clip.json"])
def test_missing_file_is_refused(clip_dir: Path, fname: str):
    (clip_dir / fname).unlink()
    with pytest.raises(ClipError, match=f"{fname} is missing"):
        load_clip(clip_dir)


def test_not_a_directory_is_refused(clip_dir: Path):
    with pytest.raises(ClipError, match="is not a directory"):
        load_clip(clip_dir / "clip.json")
    with pytest.raises(ClipError, match="is not a directory"):
        load_clip(clip_dir.parent / "nope")


def test_unparsable_json_is_refused(clip_dir: Path):
    (clip_dir / "clip.json").write_text("{not json")
    with pytest.raises(ClipError, match="cannot parse"):
        load_clip(clip_dir)
    (clip_dir / "clip.json").write_text(json.dumps([1, 2]))
    with pytest.raises(ClipError, match="top level must be an object"):
        load_clip(clip_dir)


def test_missing_key_names_the_key(clip_dir: Path):
    meta = json.loads((clip_dir / "clip.json").read_text())
    del meta["rate_hz"]
    (clip_dir / "clip.json").write_text(json.dumps(meta))
    with pytest.raises(ClipError, match="missing key 'rate_hz'"):
        load_clip(clip_dir)


def test_default_speed_is_the_fastest_safe_speed(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=(0.25, 0.5, 0.1)))
    assert default_speed(load_clip(d)) == 0.5


def test_check_speed_refusals(clip_dir: Path):
    clip = load_clip(clip_dir)  # safe at 1.0 and 0.5
    for bad in (0.0, -0.5, 1.5, float("nan"), float("inf")):
        with pytest.raises(ClipError):
            check_speed(clip, bad)
    with pytest.raises(ClipError, match="not a number|must be a number"):
        check_speed(clip, "fast")


def test_check_speed_refuses_a_speed_above_every_audited_one(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=(0.5, 0.25)))
    clip = load_clip(d)
    with pytest.raises(ClipError, match="not one of the clip's audited safe speeds"):
        check_speed(clip, 1.0)
    with pytest.raises(ClipError):
        check_speed(clip, 0.5001)


def test_check_speed_refuses_a_slower_unaudited_speed(tmp_path: Path):
    # The case that motivates the rule: 05_test_G42xKICVj9U_5-5-rgb_front_GT passes
    # the audit at 0.5 and fails it at 0.25, so "slower" is not "safer" and a speed
    # below the fastest safe one may be one the audit rejected.
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=(0.5,)))
    clip = load_clip(d)
    for unaudited in (0.25, 0.3, 0.1):
        with pytest.raises(ClipError, match="not one of the clip's audited safe speeds"):
            check_speed(clip, unaudited)


def test_check_speed_accepts_exactly_the_audited_speeds(tmp_path: Path):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(safe_speeds=(0.5, 0.25)))
    clip = load_clip(d)
    assert check_speed(clip, 0.5) == 0.5
    assert check_speed(clip, 0.5 + 1e-12) == pytest.approx(0.5)
    assert isinstance(check_speed(clip, 0.25), float)
    assert check_speed(clip, "0.25") == 0.25


def test_frame_period_and_duration(clip_dir: Path):
    clip = load_clip(clip_dir)  # 10 frames at 50 Hz
    assert frame_period(clip, 1.0) == pytest.approx(0.02)
    assert frame_period(clip, 0.5) == pytest.approx(0.04)
    assert frame_period(clip, 0.25) == pytest.approx(0.08)
    assert duration_s(clip, 1.0) == pytest.approx(0.2)
    assert duration_s(clip, 0.25) == pytest.approx(0.8)


def test_parse_sides():
    assert parse_sides("none") == ()
    assert parse_sides("left") == ("left",)
    assert parse_sides("right") == ("right",)
    assert parse_sides("both") == ("left", "right")
    assert set(SIDE_CHOICES) == {"none", "left", "right", "both"}
    with pytest.raises(ValueError, match="'all'"):
        parse_sides("all")
