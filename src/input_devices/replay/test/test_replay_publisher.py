"""Pins replay_publisher against the recording Node: wait, ramp, interpolated play, hold, refusals."""

from pathlib import Path

import numpy as np
import pytest

from replay.clip import load_clip
from replay.motion import lerp_clip, quintic_blend
from replay.replay_publisher import (
    ARM_TOPIC,
    COMMAND_QOS,
    EXIT_REFUSED,
    HAND_TOPIC,
    PUBLISH_HZ,
    ReplayPublisher,
    main,
    parse_args,
)
from rclpy.qos import QoSReliabilityPolicy  # the stub installed by conftest

from .conftest import ARM_NAMES, CLIP_NAME, FRAMES, HAND_NAMES, RATE_HZ, Bool, JointState, clip_meta, write_clip


def publisher(clip, speed, arms, hands, **kwargs) -> ReplayPublisher:
    """Immediate play: no wait, no approach. The old one-tick-per-frame tests use this."""
    kwargs.setdefault("ready_timeout_s", 0.0)
    kwargs.setdefault("ramp_s", 0.0)
    return ReplayPublisher(clip, speed, arms, hands, **kwargs)


def test_parse_args_defaults():
    args = parse_args(["--clip", "clips/safe/x"])
    assert args.clip == "clips/safe/x"
    assert args.arms == "both"
    assert args.hands == "both"
    assert args.speed is None
    assert args.ready_timeout == 30.0
    assert args.ramp == 2.0


def test_parse_args_values():
    args = parse_args(
        ["--clip", "c", "--arms", "left", "--hands", "none", "--speed", "0.25", "--ready-timeout", "0", "--ramp", "0"]
    )
    assert (args.arms, args.hands, args.speed) == ("left", "none", 0.25)
    assert args.ready_timeout == 0.0
    assert args.ramp == 0.0


def test_parse_args_refuses_none_none():
    with pytest.raises(SystemExit) as exc:
        parse_args(["--clip", "c", "--arms", "none", "--hands", "none"])
    assert exc.value.code == 2


def test_parse_args_refuses_negative_timeout_or_ramp():
    with pytest.raises(SystemExit):
        parse_args(["--clip", "c", "--ready-timeout", "-1"])
    with pytest.raises(SystemExit):
        parse_args(["--clip", "c", "--ramp", "-0.1"])


def test_parse_args_refuses_bad_side_and_missing_clip():
    with pytest.raises(SystemExit):
        parse_args(["--clip", "c", "--arms", "all"])
    with pytest.raises(SystemExit):
        parse_args([])


def test_four_publishers_for_both_both(clip_dir: Path):
    node = publisher(load_clip(clip_dir), 1.0, ("left", "right"), ("left", "right"))
    assert [p.topic for p in node.publishers] == [
        "/left_arm/joint_targets",
        "/right_arm/joint_targets",
        "/left/wuji_hand/joint_command",
        "/right/wuji_hand/joint_command",
    ]
    assert ARM_TOPIC.format(side="left") == "/left_arm/joint_targets"
    assert HAND_TOPIC.format(side="left") == "/left/wuji_hand/joint_command"
    for pub in node.publishers:
        assert pub.qos is COMMAND_QOS
    assert COMMAND_QOS.reliability is QoSReliabilityPolicy.RELIABLE
    assert COMMAND_QOS.depth == 10
    assert node.name == "replay_publisher"


def test_two_publishers_for_arms_only(clip_dir: Path):
    node = publisher(load_clip(clip_dir), 1.0, ("left", "right"), ())
    assert [p.topic for p in node.publishers] == ["/left_arm/joint_targets", "/right_arm/joint_targets"]


def test_one_publisher_for_one_hand(clip_dir: Path):
    node = publisher(load_clip(clip_dir), 1.0, (), ("right",))
    assert [p.topic for p in node.publishers] == ["/right/wuji_hand/joint_command"]


def test_timer_period_is_one_over_publish_hz(clip_dir: Path):
    clip = load_clip(clip_dir)
    for speed in (1.0, 0.5, 0.25):
        node = publisher(clip, speed, ("left",), ())
        assert len(node.timers) == 1
        assert node.timers[0].period == pytest.approx(1.0 / PUBLISH_HZ)


def test_first_tick_is_frame_zero_then_lerp_then_hold(clip_dir: Path):
    clip = load_clip(clip_dir)
    speed = 0.5
    node = publisher(clip, speed, ("left", "right"), ("left", "right"))
    timer = node.timers[0]
    timer.fire()  # t=0: frame 0
    # Midway between frames 2 and 3: frame_f = elapsed * rate * speed = 2.5
    node.get_clock().advance(2.5 / (RATE_HZ * speed))
    timer.fire()
    # Past the last frame
    node.get_clock().advance(100.0)
    timer.fire()

    for side in ("left", "right"):
        arm_msgs = node.publisher(ARM_TOPIC.format(side=side)).published
        hand_msgs = node.publisher(HAND_TOPIC.format(side=side)).published
        assert len(arm_msgs) == 3
        assert arm_msgs[0].name == list(ARM_NAMES[side])
        assert arm_msgs[0].position == clip.arm_q[side][0].tolist()
        assert hand_msgs[0].position == clip.hand_q20[side][0].tolist()
        np.testing.assert_allclose(arm_msgs[1].position, lerp_clip(clip.arm_q[side], 2.5))
        np.testing.assert_allclose(hand_msgs[1].position, lerp_clip(clip.hand_q20[side], 2.5))
        assert arm_msgs[2].position == clip.arm_q[side][-1].tolist()
        assert hand_msgs[2].position == clip.hand_q20[side][-1].tolist()
        assert all(type(x) is float for x in arm_msgs[1].position)
        assert len(arm_msgs[0].position) == 7 and len(hand_msgs[0].position) == 20
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in node.publisher("/left_arm/joint_targets").published]
    assert stamps[0] == pytest.approx(0.0)
    assert stamps[1] == pytest.approx(2.5 / (RATE_HZ * speed))


def test_holds_last_frame_and_logs_once(clip_dir: Path):
    clip = load_clip(clip_dir)
    node = publisher(clip, 1.0, ("left",), ("left",))
    timer = node.timers[0]
    timer.fire()
    node.get_clock().advance((FRAMES - 1) / RATE_HZ)
    extra = 5
    for _ in range(extra):
        timer.fire()
    arm_msgs = node.publisher("/left_arm/joint_targets").published
    hand_msgs = node.publisher("/left/wuji_hand/joint_command").published
    assert len(arm_msgs) == 1 + extra
    last_arm = clip.arm_q["left"][-1].tolist()
    last_hand = clip.hand_q20["left"][-1].tolist()
    for msg in arm_msgs[1:]:
        assert msg.position == last_arm
    for msg in hand_msgs[1:]:
        assert msg.position == last_hand
    assert arm_msgs[0].position != last_arm
    hold_logs = [m for m in node.get_logger().of_level("info") if "last frame" in m and "reached" in m]
    assert len(hold_logs) == 1
    assert f"({FRAMES - 1})" in hold_logs[0]
    assert not timer.cancelled  # the timer keeps running: hold means keep publishing


def test_startup_log_names_the_clip(clip_dir: Path):
    node = publisher(load_clip(clip_dir), 0.5, ("left",), ())
    first = node.get_logger().of_level("info")[0]
    assert CLIP_NAME in first
    assert "speed 0.5" in first
    assert "arms left" in first and "hands none" in first
    assert "publish 100 Hz" in first
    assert "no wait" in first and "no approach" in first


def test_waits_for_consumers_before_publishing(clip_dir: Path):
    clip = load_clip(clip_dir)
    node = ReplayPublisher(clip, 1.0, ("left",), ("left",), ready_timeout_s=5.0, ramp_s=0.0)
    timer = node.timers[0]
    timer.fire()
    assert node.publisher("/left_arm/joint_targets").published == []
    assert node.publisher("/left/wuji_hand/joint_command").published == []

    node.subscription("/left_arm/joint_states").deliver(JointState(name=list(ARM_NAMES["left"]), position=[0.0] * 7))
    node.subscription("/joint_states").deliver(JointState(name=list(HAND_NAMES["left"]), position=[0.0] * 20))
    node.subscription("/left/wuji_hand/connected").deliver(Bool(True))
    timer.fire()
    assert node.publisher("/left_arm/joint_targets").published[0].position == clip.arm_q["left"][0].tolist()
    ready_logs = [m for m in node.get_logger().of_level("info") if "consumers ready" in m]
    assert len(ready_logs) == 1


def test_ready_timeout_cancels_without_publishing(clip_dir: Path):
    node = ReplayPublisher(load_clip(clip_dir), 1.0, ("left",), (), ready_timeout_s=1.0, ramp_s=0.0)
    timer = node.timers[0]
    node.get_clock().advance(1.0)
    timer.fire()
    assert node.ready_failed
    assert timer.cancelled
    assert node.publisher("/left_arm/joint_targets").published == []
    errors = node.get_logger().of_level("error")
    assert errors and "timeout" in errors[0]


def test_approach_ramp_is_quintic_from_measured_pose(clip_dir: Path):
    clip = load_clip(clip_dir)
    ramp_s = 2.0
    node = ReplayPublisher(clip, 1.0, ("left",), (), ready_timeout_s=0.0, ramp_s=ramp_s)
    measured = [0.5] * 7
    node.subscription("/left_arm/joint_states").deliver(JointState(name=list(ARM_NAMES["left"]), position=measured))
    timer = node.timers[0]
    timer.fire()  # t=0, u=0
    node.get_clock().advance(0.4)
    timer.fire()  # u=0.2
    node.get_clock().advance(1.6)
    timer.fire()  # u=1, join: frame 0

    q0 = np.array(measured, dtype=np.float64)
    q1 = clip.arm_q["left"][0]
    qd0 = np.zeros(7)
    qd1 = (clip.arm_q["left"][1] - clip.arm_q["left"][0]) * RATE_HZ * 1.0 * ramp_s
    msgs = node.publisher("/left_arm/joint_targets").published
    np.testing.assert_allclose(msgs[0].position, q0)
    np.testing.assert_allclose(msgs[1].position, quintic_blend(q0, qd0, q1, qd1, 0.2))
    np.testing.assert_allclose(msgs[2].position, q1)
    linear_mid = q0 + 0.2 * (q1 - q0)
    assert not np.allclose(msgs[1].position, linear_mid)
    assert any("approaching frame 0" in m for m in node.get_logger().of_level("info"))


def test_skips_ramp_when_no_measurement(clip_dir: Path):
    clip = load_clip(clip_dir)
    node = ReplayPublisher(clip, 1.0, ("left",), (), ready_timeout_s=0.0, ramp_s=2.0)
    node.timers[0].fire()
    assert node.publisher("/left_arm/joint_targets").published[0].position == clip.arm_q["left"][0].tolist()
    assert any("skipping approach ramp" in m for m in node.get_logger().of_level("warning"))


def test_main_exits_2_on_refused_clip(tmp_path: Path, fake_rclpy, capsys):
    bad = write_clip(tmp_path / "candidate" / CLIP_NAME)
    with pytest.raises(SystemExit) as exc:
        main(["--clip", str(bad)])
    assert exc.value.code == EXIT_REFUSED == 2
    assert "refused" in capsys.readouterr().err
    assert fake_rclpy.init_calls == 0  # refused before ROS is touched


def test_main_exits_2_on_refused_speed(clip_dir: Path, fake_rclpy):
    with pytest.raises(SystemExit) as exc:
        main(["--clip", str(clip_dir), "--speed", "2"])
    assert exc.value.code == 2
    with pytest.raises(SystemExit) as exc:
        main(["--clip", str(clip_dir), "--speed", "0"])
    assert exc.value.code == 2


def test_main_exits_2_on_rejected_verdict(tmp_path: Path, fake_rclpy):
    d = write_clip(tmp_path / "safe" / CLIP_NAME, clip_meta(verdict="rejected"))
    with pytest.raises(SystemExit) as exc:
        main(["--clip", str(d)])
    assert exc.value.code == 2


def test_main_spins_and_shuts_down_cleanly_on_interrupt(clip_dir: Path, fake_rclpy):
    fake_rclpy.spin_raises = KeyboardInterrupt
    assert main(["--clip", str(clip_dir), "--hands", "none", "--ros-args", "--log-level", "info"]) is None
    assert fake_rclpy.init_calls == 1
    assert fake_rclpy.shutdown_calls == 1
    node = fake_rclpy.spun[0]
    assert node.destroyed
    assert [p.topic for p in node.publishers] == ["/left_arm/joint_targets", "/right_arm/joint_targets"]
    assert node.timers[0].period == pytest.approx(1.0 / PUBLISH_HZ)


def test_main_treats_an_error_after_context_shutdown_as_the_shutdown(clip_dir: Path, fake_rclpy):
    # Humble: a signal between the executor's context check and its wait-set creation
    # raises RCLError rather than ExternalShutdownException; the context is already gone.
    fake_rclpy.spin_raises = RuntimeError
    fake_rclpy.spin_shuts_down = True
    assert main(["--clip", str(clip_dir)]) is None
    assert fake_rclpy.shutdown_calls == 0  # already shut down by the signal handler
    assert fake_rclpy.spun[0].destroyed


def test_main_propagates_a_real_error_while_the_context_is_alive(clip_dir: Path, fake_rclpy):
    fake_rclpy.spin_raises = RuntimeError
    fake_rclpy.spin_shuts_down = False
    with pytest.raises(RuntimeError):
        main(["--clip", str(clip_dir)])
    assert fake_rclpy.shutdown_calls == 1
    assert fake_rclpy.spun[0].destroyed
