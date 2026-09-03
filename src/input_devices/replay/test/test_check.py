"""Pins the connection-check rules (replay/check.py) and the replay_check node against the recording Node."""

from itertools import product

import pytest

from replay.check import (
    ARM_STATE,
    DEFAULT_TIMEOUT_S,
    HAND_CONNECTED,
    HAND_STATE,
    ConnectionCheck,
    RateCounter,
    format_rate,
    format_row,
    hand_sides_in,
    required_sources,
)
from replay.clip import SIDE_CHOICES
from replay.replay_check import (
    EXIT_NOT_REPORTED,
    EXIT_OK,
    POLL_PERIOD_S,
    STATE_QOS,
    ReplayCheck,
    main,
    parse_args,
)
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy  # the stubs installed by conftest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .conftest import HAND_NAMES

L20 = list(HAND_NAMES["left"])
R20 = list(HAND_NAMES["right"])


# --- required sources --------------------------------------------------------


def _expected_keys(arms: str, hands: str) -> list[str]:
    sides = {"none": [], "left": ["left"], "right": ["right"], "both": ["left", "right"]}
    keys = [f"{ARM_STATE}:{s}" for s in sides[arms]]
    keys += [f"{HAND_STATE}:{s}" for s in sides[hands]]
    keys += [f"{HAND_CONNECTED}:{s}" for s in sides[hands]]
    return keys


@pytest.mark.parametrize("arms,hands", [(a, h) for a, h in product(SIDE_CHOICES, SIDE_CHOICES) if (a, h) != ("none", "none")])
def test_required_sources_per_selection(arms: str, hands: str):
    sources = required_sources(arms, hands)
    assert [s.key for s in sources] == _expected_keys(arms, hands)
    for s in sources:
        if s.kind == ARM_STATE:
            assert s.topic == f"/{s.side}_arm/joint_states"
        elif s.kind == HAND_STATE:
            assert s.topic == "/joint_states"
        else:
            assert s.topic == f"/{s.side}/wuji_hand/connected"


def test_required_sources_both_both_topics_in_table_order():
    assert [s.topic for s in required_sources("both", "both")] == [
        "/left_arm/joint_states",
        "/right_arm/joint_states",
        "/joint_states",
        "/joint_states",
        "/left/wuji_hand/connected",
        "/right/wuji_hand/connected",
    ]


def test_none_none_is_refused():
    with pytest.raises(ValueError, match="nothing to check"):
        required_sources("none", "none")
    with pytest.raises(ValueError, match="nothing to check"):
        ConnectionCheck("none", "none")


def test_bad_timeout_is_refused():
    with pytest.raises(ValueError, match="timeout"):
        ConnectionCheck("both", "both", timeout_s=0.0)


def test_timeout_default_is_twenty_seconds():
    assert DEFAULT_TIMEOUT_S == 20.0
    assert ConnectionCheck("left", "none").timeout_s == 20.0


# --- rate counter ------------------------------------------------------------


def test_rate_counter_needs_two_messages():
    rc = RateCounter()
    assert rc.count("a") == 0
    assert rc.rate("a") is None
    rc.record("a", 1.0)
    assert rc.count("a") == 1
    assert rc.rate("a") is None


def test_rate_counter_is_messages_minus_one_over_span():
    rc = RateCounter()
    for i in range(11):
        rc.record("a", 10.0 + i * 0.004)  # 11 messages over 40 ms: 250 Hz
    assert rc.count("a") == 11
    assert rc.rate("a") == pytest.approx(250.0)
    rc.record("b", 0.0)
    rc.record("b", 0.0)  # same instant: no span, no rate
    assert rc.rate("b") is None
    assert rc.rate("never") is None


def test_format_rate():
    assert format_rate(250.0) == "~250 Hz"
    assert format_rate(249.6) == "~250 Hz"
    assert format_rate(100.0) == "~100 Hz"
    assert format_rate(49.9) == "~50 Hz"
    assert format_rate(2.5) == "~2.5 Hz"
    assert format_rate(None) == "1 msg"


def test_format_row_columns():
    assert format_row("/left_arm/joint_states", "~250 Hz", "note") == "/left_arm/joint_states        ~250 Hz    note"
    assert format_row("/right/wuji_hand/connected", "true") == "/right/wuji_hand/connected    true"
    assert format_row("/joint_states", "~100 Hz") == "/joint_states                 ~100 Hz"


# --- hand sides in a /joint_states message -----------------------------------


def test_hand_sides_in_counts_a_side_only_with_all_twenty_names():
    assert hand_sides_in(L20) == ("left",)
    assert hand_sides_in(R20) == ("right",)
    assert hand_sides_in(L20 + R20) == ("left", "right")
    assert hand_sides_in(R20 + L20) == ("left", "right")
    assert hand_sides_in(L20[:19]) == ()
    assert hand_sides_in(L20[:19] + [L20[0]]) == ()  # 20 entries, 19 distinct
    assert hand_sides_in([]) == ()
    assert hand_sides_in(["left_shoulder_pitch", "left_elbow"]) == ()  # arm names are not l_ names
    assert hand_sides_in(L20 + ["waist_yaw"]) == ("left",)


# --- verdict -----------------------------------------------------------------


def _feed_arm(check: ConnectionCheck, side: str, t0: float, n: int = 11, dt: float = 0.004) -> None:
    for i in range(n):
        check.record_arm_state(side, t0 + i * dt)


def _feed_hand(check: ConnectionCheck, side: str, t0: float, n: int = 11, dt: float = 0.01) -> None:
    names = L20 if side == "left" else R20
    for i in range(n):
        check.record_joint_states(names, t0 + i * dt)
        check.record_hand_connected(side, True, t0 + i * dt)


def test_verdict_tracks_reported_and_missing():
    check = ConnectionCheck("both", "both", timeout_s=20.0, start_s=100.0)
    v = check.verdict(100.5)
    assert not v.complete and not v.timed_out
    assert v.reported == ()
    assert len(v.missing) == 6
    assert v.elapsed_s == pytest.approx(0.5)

    _feed_arm(check, "left", 101.0)
    v = check.verdict(101.5)
    assert [s.key for s in v.reported] == ["arm_state:left"]
    assert not v.complete

    _feed_arm(check, "right", 101.0)
    _feed_hand(check, "left", 101.0)
    _feed_hand(check, "right", 101.0)
    v = check.verdict(102.0)
    assert v.complete
    assert v.missing == ()
    assert len(v.reported) == 6
    assert not v.timed_out


def test_connected_false_alone_does_not_count_but_true_once_does():
    check = ConnectionCheck("none", "left", timeout_s=20.0, start_s=0.0)
    check.record_joint_states(L20, 1.0)
    check.record_joint_states(L20, 1.01)
    check.record_hand_connected("left", False, 1.0)
    v = check.verdict(2.0)
    assert [s.key for s in v.missing] == ["hand_connected:left"]
    assert v.lines[1] == "/left/wuji_hand/connected     false      never true in 2.0 s"
    check.record_hand_connected("left", True, 3.0)
    check.record_hand_connected("left", False, 4.0)  # the idle release drops it again: still counts
    assert check.verdict(5.0).complete


def test_joint_states_counts_only_for_selected_sides_with_all_names():
    check = ConnectionCheck("none", "both", timeout_s=20.0, start_s=0.0)
    assert check.record_joint_states(L20[:19], 1.0) == ()
    assert check.record_joint_states(L20, 1.0) == ("left",)
    assert check.record_joint_states(L20 + R20, 1.01) == ("left", "right")
    only_left = ConnectionCheck("none", "left")
    assert only_left.record_joint_states(R20, 1.0) == ()
    assert only_left.record_joint_states(L20 + R20, 1.0) == ("left",)


def test_timed_out_at_and_after_the_timeout():
    check = ConnectionCheck("left", "none", timeout_s=3.0, start_s=10.0)
    assert not check.verdict(12.9).timed_out
    assert check.verdict(13.0).timed_out
    assert check.verdict(20.0).timed_out
    # completing at the same instant as the timeout still counts as complete
    _feed_arm(check, "left", 12.0)
    v = check.verdict(13.0)
    assert v.complete and v.timed_out


# --- table -------------------------------------------------------------------


def test_table_both_both_all_reported_matches_the_runbook():
    check = ConnectionCheck("both", "both", timeout_s=20.0, start_s=0.0)
    _feed_arm(check, "left", 1.0)
    _feed_arm(check, "right", 1.0)
    _feed_hand(check, "left", 1.0)
    _feed_hand(check, "right", 1.0)
    v = check.verdict(2.0)
    assert v.lines == (
        "/left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose",
        "/right_arm/joint_states       ~250 Hz",
        "/joint_states                 ~100 Hz    both hands, 40 names (l_*, r_*)",
        "/left/wuji_hand/connected     true",
        "/right/wuji_hand/connected    true",
    )
    assert v.table == "\n".join(v.lines)


def test_table_joint_states_rate_is_the_slowest_side():
    check = ConnectionCheck("none", "both", timeout_s=20.0, start_s=0.0)
    _feed_hand(check, "left", 1.0, n=11, dt=0.01)  # 100 Hz
    _feed_hand(check, "right", 1.0, n=11, dt=0.02)  # 50 Hz
    assert check.verdict(2.0).lines[0] == "/joint_states                 ~50 Hz     both hands, 40 names (l_*, r_*)"


def test_table_left_hand_only():
    check = ConnectionCheck("none", "left", timeout_s=20.0, start_s=0.0)
    _feed_hand(check, "left", 1.0)
    assert check.verdict(2.0).lines == (
        "/joint_states                 ~100 Hz    left hand, 20 names (l_*)",
        "/left/wuji_hand/connected     true",
    )


def test_table_right_arm_and_right_hand():
    check = ConnectionCheck("right", "right", timeout_s=20.0, start_s=0.0)
    _feed_arm(check, "right", 1.0)
    _feed_hand(check, "right", 1.0)
    assert check.verdict(2.0).lines == (
        "/right_arm/joint_states       ~250 Hz    G1 node writing, arms holding measured pose",
        "/joint_states                 ~100 Hz    right hand, 20 names (r_*)",
        "/right/wuji_hand/connected    true",
    )


def test_table_marks_missing_rows_at_timeout():
    check = ConnectionCheck("both", "both", timeout_s=20.0, start_s=0.0)
    _feed_arm(check, "left", 1.0)
    _feed_hand(check, "left", 1.0)
    v = check.verdict(20.0)
    assert v.timed_out and not v.complete
    assert [s.key for s in v.missing] == ["arm_state:right", "hand_state:right", "hand_connected:right"]
    assert v.lines == (
        "/left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose",
        "/right_arm/joint_states       missing    no message in 20.0 s",
        "/joint_states                 missing    no r_* names in 20.0 s",
        "/left/wuji_hand/connected     true",
        "/right/wuji_hand/connected    missing    no message in 20.0 s",
    )


def test_table_nothing_reported():
    check = ConnectionCheck("left", "both", timeout_s=3.0, start_s=0.0)
    assert check.verdict(3.0).lines == (
        "/left_arm/joint_states        missing    no message in 3.0 s",
        "/joint_states                 missing    no message in 3.0 s",
        "/left/wuji_hand/connected     missing    no message in 3.0 s",
        "/right/wuji_hand/connected    missing    no message in 3.0 s",
    )


def test_table_single_message_has_no_rate_yet():
    check = ConnectionCheck("left", "none", timeout_s=20.0, start_s=0.0)
    check.record_arm_state("left", 1.0)
    v = check.verdict(1.2)
    assert v.complete
    assert v.lines == ("/left_arm/joint_states        1 msg      G1 node writing, arms holding measured pose",)


# --- node --------------------------------------------------------------------


def test_node_parse_args_defaults_and_refusals():
    args = parse_args([])
    assert (args.arms, args.hands, args.timeout) == ("both", "both", 20.0)
    args = parse_args(["--arms", "left", "--hands", "none", "--timeout", "3"])
    assert (args.arms, args.hands, args.timeout) == ("left", "none", 3.0)
    with pytest.raises(SystemExit) as exc:
        parse_args(["--arms", "none", "--hands", "none"])
    assert exc.value.code == 2
    with pytest.raises(SystemExit):
        parse_args(["--timeout", "0"])
    with pytest.raises(SystemExit):
        parse_args(["--arms", "all"])


def test_node_subscribes_best_effort_depth_10_to_the_selected_sources():
    node = ReplayCheck("both", "both", 20.0)
    assert node.name == "replay_check"
    assert [s.topic for s in node.subscriptions] == [
        "/left_arm/joint_states",
        "/right_arm/joint_states",
        "/joint_states",
        "/left/wuji_hand/connected",
        "/right/wuji_hand/connected",
    ]
    for sub in node.subscriptions:
        assert sub.qos is STATE_QOS
    assert STATE_QOS.reliability is QoSReliabilityPolicy.BEST_EFFORT
    assert STATE_QOS.history is QoSHistoryPolicy.KEEP_LAST
    assert STATE_QOS.depth == 10
    assert node.subscription("/joint_states").msg_type is JointState
    assert node.subscription("/left/wuji_hand/connected").msg_type is Bool
    assert node.publishers == []
    assert len(node.timers) == 1 and node.timers[0].period == POLL_PERIOD_S == 0.5


def test_node_arms_only_subscribes_no_hand_topics():
    node = ReplayCheck("left", "none", 20.0)
    assert [s.topic for s in node.subscriptions] == ["/left_arm/joint_states"]


def test_node_completes_when_every_source_reports():
    node = ReplayCheck("left", "left", 20.0)
    clock = node.get_clock()
    timer = node.timers[0]
    clock.advance(0.5)
    timer.fire()
    assert not node.done.done()
    for i in range(11):
        clock.advance(0.004)
        node.subscription("/left_arm/joint_states").deliver(JointState(name=["a"]))
    node.subscription("/joint_states").deliver(JointState(name=L20[:19]))  # 19 names: does not count
    node.subscription("/left/wuji_hand/connected").deliver(Bool(data=False))
    clock.advance(0.5)
    timer.fire()
    assert not node.done.done()
    for i in range(11):
        clock.advance(0.01)
        node.subscription("/joint_states").deliver(JointState(name=L20))
        node.subscription("/joint_states").deliver(JointState(name=R20))  # other side: ignored for --hands left
        node.subscription("/left/wuji_hand/connected").deliver(Bool(data=True))
    clock.advance(0.5)
    timer.fire()
    assert node.done.done()
    v = node.done.result()
    assert v.complete
    assert timer.cancelled
    assert v.lines[0].startswith("/left_arm/joint_states        ~250 Hz")
    assert v.lines[1] == "/joint_states                 ~100 Hz    left hand, 20 names (l_*)"
    assert v.lines[2] == "/left/wuji_hand/connected     true"
    assert any("all 3 sources reported" in m for m in node.get_logger().of_level("info"))


def test_node_times_out_and_logs_the_missing_sources():
    node = ReplayCheck("both", "none", 3.0)
    clock = node.get_clock()
    timer = node.timers[0]
    for _ in range(5):
        clock.advance(0.5)
        timer.fire()
        assert not node.done.done()
    clock.advance(0.5)
    timer.fire()
    assert node.done.done()
    v = node.done.result()
    assert v.timed_out and not v.complete
    assert v.lines == (
        "/left_arm/joint_states        missing    no message in 3.0 s",
        "/right_arm/joint_states       missing    no message in 3.0 s",
    )
    errors = node.get_logger().of_level("error")
    assert len(errors) == 1
    assert "2 of 2 sources missing" in errors[0]
    assert "/left_arm/joint_states" in errors[0] and "/right_arm/joint_states" in errors[0]


def test_main_exits_1_and_prints_table_on_timeout(fake_rclpy, capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--arms", "left", "--hands", "none", "--timeout", "3"])
    assert exc.value.code == EXIT_NOT_REPORTED == 1
    out = capsys.readouterr().out
    assert out.splitlines() == ["/left_arm/joint_states        missing    no message in 3.0 s"]
    assert fake_rclpy.shutdown_calls == 1
    assert fake_rclpy.spun[0].destroyed


def test_main_exits_0_when_the_source_reports(fake_rclpy, capsys, monkeypatch):
    # Have the stub spin deliver arm state before every poll.
    real_spin = fake_rclpy.spin_until_future_complete

    def spin_with_traffic(node, future, timeout_sec=None):
        sub = node.subscription("/left_arm/joint_states")
        for _ in range(10):
            node.get_clock().advance(0.004)
            sub.deliver(JointState(name=["a"]))
        return real_spin(node, future)

    monkeypatch.setattr(fake_rclpy, "spin_until_future_complete", spin_with_traffic)
    with pytest.raises(SystemExit) as exc:
        main(["--arms", "left", "--hands", "none"])
    assert exc.value.code == EXIT_OK == 0
    out = capsys.readouterr().out.splitlines()
    assert out == ["/left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose"]


def test_main_exits_2_for_none_none(fake_rclpy):
    with pytest.raises(SystemExit) as exc:
        main(["--arms", "none", "--hands", "none"])
    assert exc.value.code == 2
    assert fake_rclpy.init_calls == 0


def test_main_interrupt_prints_last_table_and_exits_1(fake_rclpy, capsys, monkeypatch):
    def spin_then_interrupt(node, future, timeout_sec=None):
        node.get_clock().advance(0.5)
        node.timers[0].fire()
        raise KeyboardInterrupt()

    monkeypatch.setattr(fake_rclpy, "spin_until_future_complete", spin_then_interrupt)
    with pytest.raises(SystemExit) as exc:
        main(["--arms", "right", "--hands", "none"])
    assert exc.value.code == 1
    captured = capsys.readouterr()
    assert captured.out.splitlines() == ["/right_arm/joint_states       missing    no message in 0.5 s"]
    assert "interrupted" in captured.err
    assert fake_rclpy.shutdown_calls == 1


def test_main_treats_an_error_after_context_shutdown_as_an_interrupt(fake_rclpy, capsys):
    fake_rclpy.spin_until_future_raises = RuntimeError
    fake_rclpy.spin_shuts_down = True
    with pytest.raises(SystemExit) as exc:
        main(["--arms", "left", "--hands", "none"])
    assert exc.value.code == 1
    captured = capsys.readouterr()
    assert "interrupted before the first poll" in captured.err
    assert captured.out == ""
    assert fake_rclpy.shutdown_calls == 0


def test_main_propagates_a_real_error_while_the_context_is_alive(fake_rclpy):
    fake_rclpy.spin_until_future_raises = RuntimeError
    with pytest.raises(RuntimeError):
        main(["--arms", "left", "--hands", "none"])
    assert fake_rclpy.shutdown_calls == 1
