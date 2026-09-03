#!/usr/bin/env python3
"""Tests for capture_arm_pose: step 2 of the rehome program (docs/spec/spec1_1.md).

The thing this process must never do is command anything, so the first test
below asserts it creates no publisher. The rest pin what it writes, because
tools/make_home_clip.py turns that file into frame 0 of a clip the robot then
plays: a wrong name order or a stale reading becomes a step in the first
published frame.

ROS is the conftest fake, so nothing here needs a workspace.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from .conftest import ARM_NAMES, JointState

from replay.capture_arm_pose import (
    ARM_STATE_TOPIC,
    DEFAULT_SAMPLES,
    DEFAULT_TIMEOUT_S,
    EXIT_NOT_REPORTED,
    EXIT_OK,
    MAX_SAMPLE_SPREAD_RAD,
    STATE_QOS,
    CaptureArmPose,
    CapturedPose,
    main,
    parse_args,
    write_pose,
)
from replay.clip import ARM_JOINTS_PER_SIDE

# A reading is 7 numbers per side. Left slot j reads j / 100, right reads
# -(j / 100), so a mixed-up side or column order fails loudly.
def reading(side: str, offset: float = 0.0) -> list[float]:
    sign = 1.0 if side == "left" else -1.0
    return [sign * (j / 100.0) + offset for j in range(ARM_JOINTS_PER_SIDE)]


def state_msg(side: str, offset: float = 0.0) -> JointState:
    return JointState(name=list(ARM_NAMES[side]), position=reading(side, offset))


def feed(node: CaptureArmPose, side: str, count: int, offset: float = 0.0) -> None:
    sub = node.subscription(ARM_STATE_TOPIC.format(side=side))
    for _ in range(count):
        sub.deliver(state_msg(side, offset))


def timeout_timer(node: CaptureArmPose):
    """The only timer the node keeps: everything else resolves on a message."""
    assert len(node.timers) == 1
    return node.timers[0]


# --- the node ---------------------------------------------------------------


def test_the_node_creates_no_publisher():
    """It reads the robot. It must have no way to write to it."""
    node = CaptureArmPose(("left", "right"), DEFAULT_SAMPLES, DEFAULT_TIMEOUT_S)
    assert node.publishers == []


def test_subscribes_best_effort_to_the_selected_sides_only():
    node = CaptureArmPose(("left",), DEFAULT_SAMPLES, DEFAULT_TIMEOUT_S)
    topics = [s.topic for s in node.subscriptions]
    assert topics == ["/left_arm/joint_states"]
    # The G1 node publishes state BEST_EFFORT; a RELIABLE subscriber never
    # matches it and would wait out the timeout with no error.
    assert node.subscriptions[0].qos is STATE_QOS


def test_completes_once_every_side_has_enough_samples():
    node = CaptureArmPose(("left", "right"), 3, DEFAULT_TIMEOUT_S)
    feed(node, "left", 3)
    assert not node.done.done(), "one side is still short"
    feed(node, "right", 2)
    assert not node.done.done()
    feed(node, "right", 1)
    assert node.done.done() and node.done.result() is True


def test_times_out_with_the_missing_side_named():
    node = CaptureArmPose(("left", "right"), 3, DEFAULT_TIMEOUT_S)
    feed(node, "left", 3)
    timeout_timer(node).fire()
    assert node.done.result() is False
    assert node.captured.missing() == ("right",)


def test_a_message_with_the_wrong_joint_count_is_an_error_not_a_reading():
    node = CaptureArmPose(("left",), 2, DEFAULT_TIMEOUT_S)
    sub = node.subscription("/left_arm/joint_states")
    sub.deliver(JointState(name=list(ARM_NAMES["left"])[:6], position=reading("left")[:6]))
    assert node.done.result() is False
    assert "6 names" in node.captured.errors[0]


def test_joint_names_changing_mid_capture_is_an_error():
    node = CaptureArmPose(("left",), 3, DEFAULT_TIMEOUT_S)
    feed(node, "left", 1)
    renamed = list(ARM_NAMES["left"])
    renamed[0] = "left_shoulder_pitch_joint"      # the MJCF spelling, not the node's
    node.subscription("/left_arm/joint_states").deliver(
        JointState(name=renamed, position=reading("left"))
    )
    assert node.done.result() is False
    assert "names changed" in node.captured.errors[0]


# --- what it writes ---------------------------------------------------------


def test_the_pose_is_the_median_so_one_torn_sample_cannot_decide_it():
    captured = CapturedPose(("left",), 5)
    for _ in range(4):
        captured.add("left", list(ARM_NAMES["left"]), reading("left"))
    captured.add("left", list(ARM_NAMES["left"]), [99.0] * ARM_JOINTS_PER_SIDE)
    pose = captured.pose()["left"]
    assert [pose[n] for n in ARM_NAMES["left"]] == pytest.approx(reading("left"))


def test_spread_reports_the_largest_movement_across_the_samples():
    captured = CapturedPose(("left",), 2)
    captured.add("left", list(ARM_NAMES["left"]), reading("left"))
    captured.add("left", list(ARM_NAMES["left"]), reading("left", offset=0.03))
    assert captured.spread()["left"] == pytest.approx(0.03)


def test_written_json_is_keyed_by_the_g1_nodes_joint_names(tmp_path):
    captured = CapturedPose(("left", "right"), 1)
    for side in ("left", "right"):
        captured.add(side, list(ARM_NAMES[side]), reading(side))
    out = tmp_path / "measured.json"
    write_pose(out, captured, "both")

    document = json.loads(out.read_text())
    assert set(document) == {"captured_utc", "arms", "left", "right"}
    for side in ("left", "right"):
        assert list(document[side]) == list(ARM_NAMES[side])
        assert list(document[side].values()) == pytest.approx(reading(side))


def test_write_pose_creates_the_parent_directory(tmp_path):
    captured = CapturedPose(("left",), 1)
    captured.add("left", list(ARM_NAMES["left"]), reading("left"))
    out = tmp_path / "nested" / "deeper" / "measured.json"
    write_pose(out, captured, "left")
    assert out.is_file()


# --- arguments --------------------------------------------------------------


def test_defaults_match_the_replay_check_conventions():
    args = parse_args(["--out", "x.json"])
    assert (args.arms, args.timeout, args.samples) == ("both", DEFAULT_TIMEOUT_S, DEFAULT_SAMPLES)
    assert DEFAULT_TIMEOUT_S == 20.0


@pytest.mark.parametrize("argv", [
    ["--out", "x.json", "--arms", "none"],
    ["--out", "x.json", "--samples", "0"],
    ["--out", "x.json", "--timeout", "0"],
    ["--arms", "both"],
])
def test_bad_arguments_are_refused(argv):
    with pytest.raises(SystemExit) as exc:
        parse_args(argv)
    assert exc.value.code == 2


# --- main -------------------------------------------------------------------


def test_main_writes_the_file_and_exits_zero(fake_rclpy, tmp_path, monkeypatch):
    real_spin = fake_rclpy.spin_until_future_complete

    def spin_with_traffic(node, future, timeout_sec=None):
        for side in ("left", "right"):
            feed(node, side, DEFAULT_SAMPLES)
        return real_spin(node, future)

    monkeypatch.setattr(fake_rclpy, "spin_until_future_complete", spin_with_traffic)
    out = tmp_path / "measured.json"
    with pytest.raises(SystemExit) as exc:
        main(["--out", str(out)])
    assert exc.value.code == EXIT_OK == 0

    document = json.loads(out.read_text())
    assert document["arms"] == "both"
    assert document["left"]["left_elbow"] == pytest.approx(0.03)


def test_main_exits_one_and_writes_nothing_on_timeout(fake_rclpy, tmp_path, capsys):
    out = tmp_path / "measured.json"
    with pytest.raises(SystemExit) as exc:
        main(["--out", str(out), "--timeout", "3"])
    assert exc.value.code == EXIT_NOT_REPORTED == 1
    assert not out.exists()
    assert "nothing written" in capsys.readouterr().err


def test_main_refuses_a_pose_that_moved_while_it_was_being_read(
        fake_rclpy, tmp_path, capsys, monkeypatch):
    """Arms drifting during the capture would put a step in the clip's frame 0."""
    real_spin = fake_rclpy.spin_until_future_complete
    drift = MAX_SAMPLE_SPREAD_RAD * 3

    def spin_with_moving_arms(node, future, timeout_sec=None):
        for side in ("left", "right"):
            sub = node.subscription(ARM_STATE_TOPIC.format(side=side))
            for i in range(DEFAULT_SAMPLES):
                sub.deliver(state_msg(side, offset=i * drift))
        return real_spin(node, future)

    monkeypatch.setattr(fake_rclpy, "spin_until_future_complete", spin_with_moving_arms)
    out = tmp_path / "measured.json"
    with pytest.raises(SystemExit) as exc:
        main(["--out", str(out)])
    assert exc.value.code == EXIT_NOT_REPORTED
    assert not out.exists()
    assert "not holding still" in capsys.readouterr().err
