"""Pins for tools/sanitize/clipio.py: both layouts, detected and echoed back."""

from __future__ import annotations

import json

import numpy as np
import pytest

from clip_audit import ARM_JOINT_NAMES, SIDES
from sanitize import clipio


def make_flat(path, frames=8, rate_hz=50.0, columns=None, extra=None):
    """A flat npz in the RoboSTAR handoff layout, with known column values.

    Column j of arm_q carries the value j + 0.001 * frame, so a reordering
    bug shows up as a swapped integer part.
    """
    columns = columns or (ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"])
    arm = np.zeros((frames, 14))
    for j in range(14):
        arm[:, j] = j + 0.001 * np.arange(frames)
    payload = {"arm_q": arm, "arm_joint_names": np.array(columns),
               "left_hand_q20": np.full((frames, 20), 0.1),
               "right_hand_q20": np.full((frames, 20), 0.2),
               "target_fps": np.array(rate_hz), "k": np.array(3)}
    payload.update(extra or {})
    np.savez(path, **payload)
    return path


def make_dir(path, frames=8, rate_hz=50.0, meta=True):
    path.mkdir(parents=True, exist_ok=True)
    np.savez(path / "arm_q.npz", left=np.full((frames, 7), 0.3), right=np.full((frames, 7), 0.4))
    np.savez(path / "hand_q20.npz",
             left=np.full((frames, 20), 0.1), right=np.full((frames, 20), 0.2))
    if meta:
        (path / "clip.json").write_text(json.dumps(
            {"tool": "prepare_clip/1", "frames": frames, "rate_hz": rate_hz}))
    return path


# -- detection --------------------------------------------------------------

def test_detect_layout_by_structure(tmp_path):
    flat = make_flat(tmp_path / "c.npz")
    d = make_dir(tmp_path / "clip")
    assert clipio.detect_layout(flat) == clipio.LAYOUT_FLAT
    assert clipio.detect_layout(d) == clipio.LAYOUT_DIR


def test_detect_layout_refuses_missing_and_incomplete(tmp_path):
    with pytest.raises(FileNotFoundError):
        clipio.detect_layout(tmp_path / "nope")
    (tmp_path / "empty").mkdir()
    with pytest.raises(ValueError, match="arm_q.npz"):
        clipio.detect_layout(tmp_path / "empty")


# -- flat npz ---------------------------------------------------------------

def test_flat_columns_are_reordered_into_model_order(tmp_path):
    # Right-arm-first columns must still land in the right per-side arrays.
    columns = ARM_JOINT_NAMES["right"] + ARM_JOINT_NAMES["left"]
    clip = clipio.read_clip(make_flat(tmp_path / "c.npz", columns=columns))
    assert clip.layout == clipio.LAYOUT_FLAT
    assert clip.frames == 8 and clip.rate_hz == 50.0
    # Columns 0..6 of the file are the right arm in this file.
    assert np.allclose(clip.arm["right"][:, 0], 0 + 0.001 * np.arange(8))
    assert np.allclose(clip.arm["left"][:, 0], 7 + 0.001 * np.arange(8))


def test_flat_round_trip_preserves_order_dtypes_and_passthrough(tmp_path):
    src = make_flat(tmp_path / "c.npz", extra={"provenance": np.array(["x"])})
    clip = clipio.read_clip(src)
    out = tmp_path / "out.npz"
    written = clipio.write_clip(clip, out, report={"tool": "sanitize/1"})
    assert out in written

    before = np.load(src, allow_pickle=False)
    after = np.load(out, allow_pickle=False)
    assert sorted(before.files) == sorted(after.files)
    for key in before.files:
        assert np.array_equal(before[key], after[key]), key
    report = json.loads((tmp_path / "out_sanitize.json").read_text())
    assert report["tool"] == "sanitize/1"


def test_flat_write_refuses_a_non_npz_path(tmp_path):
    clip = clipio.read_clip(make_flat(tmp_path / "c.npz"))
    with pytest.raises(ValueError, match="npz"):
        clipio.write_clip(clip, tmp_path / "out")


@pytest.mark.parametrize("drop", ["arm_q", "arm_joint_names", "left_hand_q20"])
def test_flat_refuses_missing_keys(tmp_path, drop):
    make_flat(tmp_path / "c.npz")
    with np.load(tmp_path / "c.npz") as handle:
        payload = {k: handle[k] for k in handle.files if k != drop}
    np.savez(tmp_path / "bad.npz", **payload)
    with pytest.raises(ValueError, match=drop):
        clipio.read_clip(tmp_path / "bad.npz")


def test_flat_refuses_an_unknown_joint_name(tmp_path):
    columns = list(ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"])
    columns[3] = "left_elbow_typo"
    make_flat(tmp_path / "c.npz", columns=columns)
    with pytest.raises(ValueError, match="left_elbow"):
        clipio.read_clip(tmp_path / "c.npz")


def test_flat_refuses_a_frame_count_mismatch(tmp_path):
    make_flat(tmp_path / "c.npz", frames=8)
    with np.load(tmp_path / "c.npz") as handle:
        payload = {k: handle[k] for k in handle.files}
    payload["left_hand_q20"] = np.zeros((7, 20))
    np.savez(tmp_path / "bad.npz", **payload)
    with pytest.raises(ValueError, match="frames"):
        clipio.read_clip(tmp_path / "bad.npz")


# -- clip directory ---------------------------------------------------------

def test_dir_round_trip_writes_three_files_and_a_sanitize_block(tmp_path):
    clip = clipio.read_clip(make_dir(tmp_path / "clip"))
    assert clip.layout == clipio.LAYOUT_DIR and clip.rate_hz == 50.0
    out = tmp_path / "out"
    clipio.write_clip(clip, out, report={"tool": "sanitize/1", "verdict": {"failures": 0}})
    for name in ("arm_q.npz", "hand_q20.npz", "clip.json", "sanitize.json"):
        assert (out / name).is_file(), name
    meta = json.loads((out / "clip.json").read_text())
    assert meta["tool"] == "prepare_clip/1"          # the source's own field survives
    assert meta["sanitize"]["verdict"]["failures"] == 0
    with np.load(out / "arm_q.npz") as handle:
        assert set(handle.files) == set(SIDES)
        assert handle["left"].shape == (8, 7)


def test_dir_without_clip_json_uses_the_default_rate(tmp_path):
    clip = clipio.read_clip(make_dir(tmp_path / "clip", meta=False))
    assert clip.rate_hz == clipio.DEFAULT_RATE_HZ
    out = tmp_path / "out"
    clipio.write_clip(clip, out, report={"tool": "sanitize/1"})
    assert not (out / "clip.json").exists()          # nothing to copy, none invented


def test_dir_refuses_a_frames_disagreement(tmp_path):
    path = make_dir(tmp_path / "clip", frames=8)
    (path / "clip.json").write_text(json.dumps({"frames": 9, "rate_hz": 50.0}))
    with pytest.raises(ValueError, match="frames"):
        clipio.read_clip(path)


# -- the Clip value type ----------------------------------------------------

def test_time_of_and_duration_use_the_rate(tmp_path):
    clip = clipio.read_clip(make_flat(tmp_path / "c.npz", frames=100, rate_hz=25.0))
    assert clip.duration_s == 4.0
    assert clip.time_of(50) == 2.0


def test_arm_all_is_left_then_right_and_round_trips(tmp_path):
    clip = clipio.read_clip(make_flat(tmp_path / "c.npz"))
    stacked = clip.arm_all()
    assert stacked.shape == (8, 14)
    assert np.allclose(stacked[:, :7], clip.arm["left"])
    assert np.allclose(stacked[:, 7:], clip.arm["right"])
    again = clip.with_arm_all(stacked)
    for side in SIDES:
        assert np.allclose(again.arm[side], clip.arm[side])
        assert np.allclose(again.hand[side], clip.hand[side])


def test_with_arm_all_refuses_the_wrong_shape(tmp_path):
    clip = clipio.read_clip(make_flat(tmp_path / "c.npz"))
    with pytest.raises(ValueError, match="14"):
        clip.with_arm_all(np.zeros((8, 7)))
