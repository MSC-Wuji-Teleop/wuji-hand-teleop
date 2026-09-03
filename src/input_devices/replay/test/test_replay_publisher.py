"""Pins replay_publisher against the recording Node: publishers, per-tick messages, the hold, the refusals."""

from pathlib import Path

import pytest

from replay.clip import load_clip
from replay.replay_publisher import (
    ARM_TOPIC,
    COMMAND_QOS,
    EXIT_REFUSED,
    HAND_TOPIC,
    ReplayPublisher,
    main,
    parse_args,
)
from rclpy.qos import QoSReliabilityPolicy  # the stub installed by conftest

from .conftest import ARM_NAMES, CLIP_NAME, FRAMES, HAND_NAMES, RATE_HZ, clip_meta, write_clip


def test_parse_args_defaults():
    args = parse_args(["--clip", "clips/safe/x"])
    assert args.clip == "clips/safe/x"
    assert args.arms == "both"
    assert args.hands == "both"
    assert args.speed is None


def test_parse_args_values():
    args = parse_args(["--clip", "c", "--arms", "left", "--hands", "none", "--speed", "0.25"])
    assert (args.arms, args.hands, args.speed) == ("left", "none", 0.25)


def test_parse_args_refuses_none_none():
    with pytest.raises(SystemExit) as exc:
        parse_args(["--clip", "c", "--arms", "none", "--hands", "none"])
    assert exc.value.code == 2


def test_parse_args_refuses_bad_side_and_missing_clip():
    with pytest.raises(SystemExit):
        parse_args(["--clip", "c", "--arms", "all"])
    with pytest.raises(SystemExit):
        parse_args([])


def test_four_publishers_for_both_both(clip_dir: Path):
    node = ReplayPublisher(load_clip(clip_dir), 1.0, ("left", "right"), ("left", "right"))
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
    node = ReplayPublisher(load_clip(clip_dir), 1.0, ("left", "right"), ())
    assert [p.topic for p in node.publishers] == ["/left_arm/joint_targets", "/right_arm/joint_targets"]


def test_one_publisher_for_one_hand(clip_dir: Path):
    node = ReplayPublisher(load_clip(clip_dir), 1.0, (), ("right",))
    assert [p.topic for p in node.publishers] == ["/right/wuji_hand/joint_command"]


def test_timer_period_is_one_over_rate_times_speed(clip_dir: Path):
    clip = load_clip(clip_dir)
    for speed in (1.0, 0.5, 0.25):
        node = ReplayPublisher(clip, speed, ("left",), ())
        assert len(node.timers) == 1
        assert node.timers[0].period == pytest.approx(1.0 / (RATE_HZ * speed))


def test_each_tick_publishes_frame_i_with_names(clip_dir: Path):
    clip = load_clip(clip_dir)
    node = ReplayPublisher(clip, 0.5, ("left", "right"), ("left", "right"))
    timer = node.timers[0]
    for i in range(FRAMES):
        node.get_clock().advance(timer.period)
        timer.fire()
    for side in ("left", "right"):
        arm_msgs = node.publisher(ARM_TOPIC.format(side=side)).published
        hand_msgs = node.publisher(HAND_TOPIC.format(side=side)).published
        assert len(arm_msgs) == FRAMES
        assert len(hand_msgs) == FRAMES
        for i in range(FRAMES):
            assert arm_msgs[i].name == list(ARM_NAMES[side])
            assert arm_msgs[i].position == clip.arm_q[side][i].tolist()
            assert hand_msgs[i].name == list(HAND_NAMES[side])
            assert hand_msgs[i].position == clip.hand_q20[side][i].tolist()
            assert isinstance(arm_msgs[i].name, list) and isinstance(arm_msgs[i].position, list)
            assert all(type(x) is float for x in arm_msgs[i].position)
            assert all(type(x) is float for x in hand_msgs[i].position)
            assert len(arm_msgs[i].position) == 7 and len(hand_msgs[i].position) == 20
    # the stamp is the node clock at the tick
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in node.publisher("/left_arm/joint_targets").published]
    assert stamps == pytest.approx([(i + 1) * timer.period for i in range(FRAMES)])


def test_holds_last_frame_and_logs_once(clip_dir: Path):
    clip = load_clip(clip_dir)
    node = ReplayPublisher(clip, 1.0, ("left",), ("left",))
    timer = node.timers[0]
    extra = 5
    for _ in range(FRAMES + extra):
        timer.fire()
    arm_msgs = node.publisher("/left_arm/joint_targets").published
    hand_msgs = node.publisher("/left/wuji_hand/joint_command").published
    assert len(arm_msgs) == FRAMES + extra
    last_arm = clip.arm_q["left"][-1].tolist()
    last_hand = clip.hand_q20["left"][-1].tolist()
    for msg in arm_msgs[FRAMES - 1:]:
        assert msg.position == last_arm
    for msg in hand_msgs[FRAMES - 1:]:
        assert msg.position == last_hand
    assert arm_msgs[FRAMES - 2].position != last_arm
    hold_logs = [m for m in node.get_logger().of_level("info") if "last frame" in m and "reached" in m]
    assert len(hold_logs) == 1
    assert f"({FRAMES - 1})" in hold_logs[0]
    assert not timer.cancelled  # the timer keeps running: hold means keep publishing


def test_startup_log_names_the_clip(clip_dir: Path):
    node = ReplayPublisher(load_clip(clip_dir), 0.5, ("left",), ())
    first = node.get_logger().of_level("info")[0]
    assert CLIP_NAME in first
    assert "speed 0.5" in first
    assert "arms left" in first and "hands none" in first


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
    assert node.timers[0].period == pytest.approx(1.0 / (RATE_HZ * 1.0))  # default speed: fastest safe speed


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
