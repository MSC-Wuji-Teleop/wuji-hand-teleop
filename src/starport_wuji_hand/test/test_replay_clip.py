"""Clip loading and resampling for hardware replay.

Two refusals matter here and are tested first: a clip whose channel is not what the operator
asked for, and a clip whose joint labels disagree with the hardware order. Both would produce a
plausible-looking replay that is wrong -- the worst failure mode on real actuators.
"""

from unittest.mock import MagicMock

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from starport_wuji_hand import replay_clip
from starport_wuji_hand.joint_map import NUM_JOINTS, joint_names, name_to_index
from starport_wuji_hand.replay_clip import ReplayClip, load_clip, resample, sample_hand_joints

RIGHT_NAMES = joint_names("right")
RIGHT_INDEX = name_to_index("right")

# The node tests drive the callbacks and read the trace buffers directly, as test_hand_node.py
# does: spinning a graph would make the recorded lengths a matter of timing.
# ruff: noqa: SLF001


def write_clip(tmp_path, channel="hand_joint_pos", names=None, frames=30, dt=1.0 / 30.0, side="right"):
    path = tmp_path / "clip.npz"
    if names is None:
        names = np.array(list(RIGHT_NAMES), dtype="<U32")
    np.savez(
        path,
        hand_joint_pos=np.linspace(0.0, 1.0, frames * NUM_JOINTS).reshape(frames, NUM_JOINTS),
        hand_joint_names=names,
        hand_side=np.asarray(side),
        dt=np.asarray(dt),
        traj_id=np.asarray("dexycb_0001"),
        dataset=np.asarray("dexycb-tracking:v7"),
        channel=np.asarray(channel),
    )
    return str(path)


def test_loads_a_matching_clip(tmp_path):
    clip = load_clip(write_clip(tmp_path), expect_channel="hand_joint_pos")
    assert clip.positions.shape == (30, NUM_JOINTS)
    assert clip.traj_id == "dexycb_0001"
    assert clip.dt == pytest.approx(1.0 / 30.0)
    assert clip.channel == "hand_joint_pos"


def test_refuses_a_channel_the_operator_did_not_ask_for(tmp_path):
    # A hand_joint_ctrl clip commands PAST the contact pose. On an empty bench that is extra
    # travel toward self-contact for no benefit, so it must not run by accident.
    path = write_clip(tmp_path, channel="hand_joint_ctrl")
    with pytest.raises(ValueError, match="channel"):
        load_clip(path, expect_channel="hand_joint_pos")


def test_accepts_a_ctrl_clip_when_that_is_what_was_requested(tmp_path):
    path = write_clip(tmp_path, channel="hand_joint_ctrl")
    clip = load_clip(path, expect_channel="hand_joint_ctrl")
    assert clip.channel == "hand_joint_ctrl"


def test_refuses_canonical_mjcf_names_that_are_not_the_hardware_order(tmp_path):
    # The reference corpus labels columns anatomically. The node's command topic speaks vendor
    # names, so a clip must be relabelled at export, not silently trusted here.
    anatomical = np.array(["r_thumb_cmc_flex"] + [f"r_other_{i}" for i in range(19)], dtype="<U32")
    with pytest.raises(ValueError, match="joint names"):
        load_clip(write_clip(tmp_path, names=anatomical), expect_channel="hand_joint_pos")


def test_refuses_a_permuted_joint_order(tmp_path):
    permuted = np.array([*RIGHT_NAMES[1:], RIGHT_NAMES[0]], dtype="<U32")
    with pytest.raises(ValueError, match="joint names"):
        load_clip(write_clip(tmp_path, names=permuted), expect_channel="hand_joint_pos")


@pytest.mark.parametrize("dt", [0.0, -1.0 / 30.0, float("nan"), float("inf")])
def test_refuses_a_clip_with_an_unusable_dt(tmp_path, dt):
    # dt is the grid the clip is resampled FROM: a zero collapses the whole trajectory into one
    # instant, and a non-finite one makes every interpolated frame meaningless.
    with pytest.raises(ValueError, match="dt must be finite and positive"):
        load_clip(write_clip(tmp_path, dt=dt), expect_channel="hand_joint_pos")


def test_refuses_a_clip_that_is_not_twenty_columns_wide(tmp_path):
    path = tmp_path / "narrow.npz"
    np.savez(
        path,
        hand_joint_pos=np.zeros((5, NUM_JOINTS - 1)),
        hand_joint_names=np.array(list(RIGHT_NAMES), dtype="<U32"),
        hand_side=np.asarray("right"),
        dt=np.asarray(1.0 / 30.0),
        traj_id=np.asarray("dexycb_0001"),
        dataset=np.asarray("dexycb-tracking:v7"),
        channel=np.asarray("hand_joint_pos"),
    )
    with pytest.raises(ValueError, match="hand_joint_pos must be"):
        load_clip(str(path), expect_channel="hand_joint_pos")


def test_resample_upward_preserves_endpoints():
    src = np.linspace(0.0, 1.0, 10 * NUM_JOINTS).reshape(10, NUM_JOINTS)
    out = resample(src, src_dt=1.0 / 30.0, dst_dt=1.0 / 100.0)
    np.testing.assert_allclose(out[0], src[0])
    np.testing.assert_allclose(out[-1], src[-1], atol=1e-9)


def test_resample_upward_lengthens_the_clip_by_the_rate_ratio():
    src = np.zeros((30, NUM_JOINTS))
    out = resample(src, src_dt=1.0 / 30.0, dst_dt=1.0 / 100.0)
    # 30 frames at 30 Hz is ~0.967 s; at 100 Hz that is ~97 frames.
    assert 90 <= out.shape[0] <= 105
    assert out.shape[1] == NUM_JOINTS


def test_resample_is_identity_when_rates_match():
    src = np.random.default_rng(0).normal(size=(12, NUM_JOINTS))
    out = resample(src, src_dt=0.01, dst_dt=0.01)
    np.testing.assert_allclose(out, src, atol=1e-12)


def test_resample_interpolates_rather_than_repeating_frames():
    src = np.zeros((2, NUM_JOINTS))
    src[1] = 1.0
    out = resample(src, src_dt=1.0, dst_dt=0.5)
    # A midpoint must be interpolated, not a held previous frame.
    assert 0.0 < out[1][0] < 1.0


def test_resample_rejects_a_single_frame_clip():
    with pytest.raises(ValueError, match="at least 2"):
        resample(np.zeros((1, NUM_JOINTS)), src_dt=0.03, dst_dt=0.01)


def test_resample_stops_just_short_when_the_grid_does_not_divide_the_clip():
    # 30 frames at 30 Hz is 0.9667 s, which is not a whole number of 10 ms steps.
    frames, src_dt, dst_dt = 30, 1.0 / 30.0, 0.01
    src = np.tile(np.linspace(0.0, 1.0, frames).reshape(frames, 1), (1, NUM_JOINTS))
    out = resample(src, src_dt=src_dt, dst_dt=dst_dt)
    # The grid never runs past the clip, so no frame is extrapolated or held ...
    assert (out.shape[0] - 1) * dst_dt <= (frames - 1) * src_dt
    # ... and it ends within one step's worth of travel of the clip's final pose, which is the
    # last thing the hand is asked for: the clip's true endpoint never enters the trace.
    per_step = (src[-1, 0] - src[-2, 0]) / src_dt * dst_dt
    assert 0.0 <= src[-1, 0] - out[-1, 0] <= per_step + 1e-12


@pytest.mark.parametrize(("src_dt", "dst_dt"), [(float("nan"), 0.01), (0.0, 0.01), (0.03, 0.0)])
def test_resample_refuses_an_unusable_grid(src_dt, dst_dt):
    # A NaN source step, a zero source step, and a zero destination step.
    with pytest.raises(ValueError, match="must be finite and positive"):
        resample(np.zeros((4, NUM_JOINTS)), src_dt=src_dt, dst_dt=dst_dt)


# ----------------------------- samples off the wire -----------------------------


def test_a_sample_is_placed_by_name_not_by_message_order():
    shuffled = list(reversed(RIGHT_NAMES))
    sample = sample_hand_joints(JointState(name=shuffled, position=[float(i) for i in range(NUM_JOINTS)]), RIGHT_INDEX)
    # Column j must hold the value sent for RIGHT_NAMES[j], not the j-th value on the wire.
    np.testing.assert_allclose(sample, np.arange(NUM_JOINTS - 1, -1, -1, dtype=np.float64))


def test_joint_states_from_another_publisher_are_not_a_hand_sample():
    # An arm's own message, then a hand message one joint short.
    assert sample_hand_joints(JointState(name=["shoulder_pan_joint"], position=[0.3]), RIGHT_INDEX) is None
    assert sample_hand_joints(JointState(name=list(RIGHT_NAMES[:19]), position=[0.1] * 19), RIGHT_INDEX) is None


def test_a_combined_publisher_message_yields_just_the_hand_columns():
    # An arm and the hand in one message, which is what a combined robot_state_publisher sends.
    names = ["shoulder_pan_joint", *RIGHT_NAMES, "wrist_3_joint"]
    positions = [9.0, *([0.5] * NUM_JOINTS), 9.0]
    sample = sample_hand_joints(JointState(name=names, position=positions), RIGHT_INDEX)
    np.testing.assert_allclose(sample, np.full(NUM_JOINTS, 0.5))


def test_a_malformed_message_is_not_a_hand_sample():
    assert sample_hand_joints(JointState(name=list(RIGHT_NAMES), position=[0.0] * 19), RIGHT_INDEX) is None


def test_a_non_finite_reading_is_not_a_hand_sample():
    positions = [0.0] * NUM_JOINTS
    positions[3] = float("nan")
    assert sample_hand_joints(JointState(name=list(RIGHT_NAMES), position=positions), RIGHT_INDEX) is None


# ----------------------------- the node -----------------------------


@pytest.fixture
def make_replay():
    """Build replay nodes with parameter overrides, cleaned up at the end of the test."""
    created = []

    def factory(**overrides):
        args = ["--ros-args"]
        for name, value in overrides.items():
            args += ["-p", f"{name}:={value}"]
        node = ReplayClip(cli_args=args)
        created.append(node)
        node._pub = MagicMock()
        # A subscriber is present unless the test says otherwise, so the wait is satisfied on
        # the first callback and every test that cares states the subscription state it means.
        node._pub.get_subscription_count.return_value = 1
        # Likewise the trace subscriptions count as matched. In a unit test nothing publishes on
        # them, and the gate would otherwise hold every frame forever waiting. What that gating
        # DOES is proven in test_first_frame_gate.py against the shared module.
        node.count_publishers = lambda _topic: 1
        # And the driver reports ready. Nothing publishes that flag in a unit test, and the gate
        # would otherwise hold every frame. What the readiness gating DOES is proven in
        # test_first_frame_gate.py against the shared module.
        node._driver_ready = True
        # The gate keeps its own reference to the publisher it watches, so the stand-in goes in
        # both places. That the two started out as one object is pinned separately below.
        node._gate._publisher = node._pub
        return node

    yield factory
    for node in created:
        try:
            node.destroy_node()
        except Exception:
            pass


def test_a_replay_without_a_clip_refuses_to_start(make_replay):
    with pytest.raises(ValueError, match="clip parameter is required"):
        make_replay(rate="30.0")


@pytest.mark.parametrize("rate", ["0.0", "-5.0", "nan", "inf"])
def test_a_replay_with_an_unusable_rate_refuses_to_start(tmp_path, make_replay, rate):
    with pytest.raises(ValueError, match="rate must be finite and positive"):
        make_replay(clip=write_clip(tmp_path), rate=rate)


def test_published_frames_carry_the_hardware_joint_names(tmp_path, make_replay):
    node = make_replay(clip=write_clip(tmp_path, frames=4), rate="30.0")
    node._publish_next()
    msg = node._pub.publish.call_args[0][0]
    assert list(msg.name) == list(RIGHT_NAMES)
    np.testing.assert_allclose(msg.position, node._frames[0])


# What the gate DOES is proven once, in test_first_frame_gate.py, against the shared module both
# bring-up tools now hold. What is left to pin here is the wiring: that this tool consults its gate
# before publishing, and that the gate it consults watches the topic this tool publishes on.


def test_the_replay_publishes_nothing_until_its_gate_opens(tmp_path, make_replay, monkeypatch):
    node = make_replay(clip=write_clip(tmp_path, frames=30), rate="100.0")
    monkeypatch.setattr(node._gate, "ready", lambda: False)
    for _ in range(3):
        node._publish_next()
    assert node._pub.publish.call_count == 0
    assert node._i == 0

    monkeypatch.setattr(node._gate, "ready", lambda: True)
    node._publish_next()
    # Publishing starts with the clip's first frame, not the fourth.
    assert node._pub.publish.call_count == 1
    np.testing.assert_allclose(node._pub.publish.call_args[0][0].position, node._frames[0])


def test_the_replay_gates_its_own_topic_with_the_wait_it_was_given(tmp_path):
    # Three wires, each of which fails silently -- test_wave_check.py's copy says how.
    node = ReplayClip(
        cli_args=[
            "--ros-args",
            "-p",
            f"clip:={write_clip(tmp_path)}",
            "-p",
            "command_topic:=/elsewhere/joint_command",
            "-p",
            "wait_for_subscriber_s:=0.25",
        ]
    )
    try:
        assert node._gate._publisher is node._pub
        assert node._gate._topic == "/elsewhere/joint_command"
        assert node._gate._wait_s == 0.25
    finally:
        node.destroy_node()


def test_an_interrupt_while_waiting_still_writes_the_trace(tmp_path, make_replay):
    # Nothing has gone out yet, and the trace is already writable.
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=30), rate="100.0", record=str(record))
    node._pub.get_subscription_count.return_value = 0

    node._publish_next()
    node.save_trace()
    with np.load(record) as trace:
        assert trace["raw"].shape == (0, NUM_JOINTS)


@pytest.mark.parametrize("wait", ["-0.5", "nan", "inf"])
def test_a_replay_with_an_unusable_wait_refuses_to_start(tmp_path, make_replay, wait):
    with pytest.raises(ValueError, match="wait_for_subscriber_s must be finite and non-negative"):
        make_replay(clip=write_clip(tmp_path), rate="30.0", wait_for_subscriber_s=wait)


def test_a_clip_shorter_than_one_destination_step_still_publishes_its_single_frame(tmp_path, make_replay, monkeypatch):
    # A 1 ms clip resamples to a single-frame grid. That frame is still what the hand is asked
    # for, and a grid that narrow must not make the subscriber gate misfire either.
    node = make_replay(clip=write_clip(tmp_path, frames=2, dt=0.001), rate="30.0")
    logger = MagicMock()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    assert node._frames.shape[0] == 1

    node._publish_next()
    assert node._pub.publish.call_count == 1
    with pytest.raises(SystemExit):
        node._publish_next()
    assert logger.warning.call_count == 0


def test_the_trace_records_all_three_channels_at_their_own_lengths(tmp_path, make_replay):
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=4), rate="30.0", record=str(record))

    # Unequal counts on purpose: 4 published, 2 post-guard, 3 measured.
    for _ in range(2):
        node._on_commanded(JointState(name=list(RIGHT_NAMES), position=[0.1] * NUM_JOINTS))
    for _ in range(2):
        node._on_measured(JointState(name=list(RIGHT_NAMES), position=[0.2] * NUM_JOINTS))
    # A gripper-style message on the shared topic, then a hand sample whose columns arrive
    # backwards: the callback must drop the first and reorder the second.
    node._on_measured(JointState(name=["finger_joint"], position=[0.3]))
    backwards = list(reversed(RIGHT_NAMES))
    node._on_measured(JointState(name=backwards, position=[float(i) for i in range(NUM_JOINTS)]))
    with pytest.raises(SystemExit):
        for _ in range(5):
            node._publish_next()

    with np.load(record) as trace:
        assert trace["raw"].shape == (4, NUM_JOINTS)
        assert trace["commanded"].shape == (2, NUM_JOINTS)
        assert trace["measured"].shape == (3, NUM_JOINTS)
        # Values, not only shapes: every sample must reach the trace through the by-name selection.
        np.testing.assert_allclose(trace["raw"][0], node._frames[0])
        np.testing.assert_allclose(trace["commanded"][1], np.full(NUM_JOINTS, 0.1))
        np.testing.assert_allclose(trace["measured"][1], np.full(NUM_JOINTS, 0.2))
        np.testing.assert_allclose(trace["measured"][2], np.arange(NUM_JOINTS - 1, -1, -1, dtype=np.float64))
        assert list(trace["joint_names"]) == list(RIGHT_NAMES)
        assert str(trace["traj_id"]) == "dexycb_0001"
        assert str(trace["dataset"]) == "dexycb-tracking:v7"
        assert str(trace["channel"]) == "hand_joint_pos"
        assert float(trace["command_rate"]) == pytest.approx(30.0)


def test_the_trace_stamps_every_recorded_sample(tmp_path, make_replay):
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=3), rate="30.0", record=str(record))
    stamped = JointState(name=list(RIGHT_NAMES), position=[0.2] * NUM_JOINTS)
    stamped.header.stamp.sec = 7
    stamped.header.stamp.nanosec = 500_000_000
    node._on_measured(stamped)
    with pytest.raises(SystemExit):
        for _ in range(4):
            node._publish_next()

    with np.load(record) as trace:
        assert trace["measured_t"] == pytest.approx([7.5])
        assert trace["raw_t"].shape == (trace["raw"].shape[0],)
        assert trace["commanded_t"].shape == (trace["commanded"].shape[0],)
        assert trace["raw_t"][0] > 0.0
        assert np.all(np.diff(trace["raw_t"]) >= 0.0)


def test_refused_messages_are_counted_per_channel_for_the_operator(tmp_path, make_replay):
    # "0 measured" reads the same whether the driver was absent or publishing something unusable,
    # and those have different fixes. So do the two channels: an unusable post-guard message is
    # the driver, an unusable measured one is whatever publishes state.
    node = make_replay(clip=write_clip(tmp_path), rate="30.0")
    node._on_measured(JointState(name=list(RIGHT_NAMES[:19]), position=[0.1] * 19))
    node._on_measured(JointState(name=list(RIGHT_NAMES), position=[float("nan")] * NUM_JOINTS))
    node._on_commanded(JointState(name=list(RIGHT_NAMES[:19]), position=[0.1] * 19))
    assert (node._measured.rejected, node._commanded.rejected) == (2, 1)


def test_traffic_carrying_none_of_the_hand_is_not_counted_as_a_refusal(tmp_path, make_replay, monkeypatch):
    # Neither counted nor warned about; _record says why foreign traffic gets that treatment. Both
    # halves are asserted, since a silent count and a warned zero would each leave it half done.
    node = make_replay(clip=write_clip(tmp_path), rate="30.0")
    logger = MagicMock()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    for _ in range(3):
        node._on_measured(JointState(name=["shoulder_pan_joint", "finger_joint"], position=[0.3, 0.1]))

    assert node._measured.rejected == 0
    assert node._measured.samples == []
    assert logger.warning.call_args_list == []


def test_a_refusal_on_one_channel_does_not_silence_the_other(tmp_path, make_replay, monkeypatch):
    # Both channels refuse, twice each, and each is reported exactly once and by its own name.
    node = make_replay(clip=write_clip(tmp_path), rate="30.0")
    logger = MagicMock()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    partial = JointState(name=list(RIGHT_NAMES[:19]), position=[0.1] * 19)

    node._on_commanded(partial)
    node._on_measured(partial)
    node._on_commanded(partial)
    node._on_measured(partial)
    warned = [call[0][0] for call in logger.warning.call_args_list]
    assert len(warned) == 2, warned
    assert [w for w in warned if "post-guard" in w] and [w for w in warned if "measured" in w]


def test_the_closing_line_reports_each_channel_separately(tmp_path, make_replay, monkeypatch):
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=3), rate="30.0", record=str(record))
    logger = MagicMock()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    partial = JointState(name=list(RIGHT_NAMES[:19]), position=[0.1] * 19)

    node._on_commanded(partial)
    node._on_measured(partial)
    node._on_measured(partial)
    node.save_trace()
    written = logger.info.call_args[0][0]
    assert "post-guard (1 ignored)" in written
    assert "measured (2 ignored)" in written


def test_the_wired_topics_follow_their_parameters(tmp_path, make_replay):
    node = make_replay(
        clip=write_clip(tmp_path),
        rate="30.0",
        command_topic="/left_hand/joint_command",
        commanded_topic="/left_hand/commanded_joint_states",
        measured_topic="/cell/joint_states",
    )
    # A declared-but-ignored topic parameter is a trap in a namespaced run.
    assert {"/left_hand/commanded_joint_states", "/cell/joint_states"} <= {s.topic_name for s in node.subscriptions}
    assert "/left_hand/joint_command" in {p.topic_name for p in node.publishers}


def test_an_interrupted_spin_still_writes_the_trace(tmp_path, monkeypatch, make_replay):
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=30), rate="30.0", record=str(record))

    def spin(target):
        target._publish_next()
        target._publish_next()
        raise KeyboardInterrupt

    # init and shutdown are stubbed out because the test session owns the rclpy context.
    monkeypatch.setattr(replay_clip.rclpy, "init", lambda *a, **k: None)
    monkeypatch.setattr(replay_clip.rclpy, "shutdown", lambda *a, **k: None)
    monkeypatch.setattr(replay_clip.rclpy, "spin", spin)
    monkeypatch.setattr(replay_clip, "ReplayClip", lambda: node)
    replay_clip.main()

    with np.load(record) as trace:
        assert trace["raw"].shape == (2, NUM_JOINTS)


def test_a_channel_that_recorded_nothing_stays_twenty_columns_wide(tmp_path, make_replay):
    # Nothing was ever published on either subscribed topic.
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=3), rate="30.0", record=str(record))
    with pytest.raises(SystemExit):
        for _ in range(4):
            node._publish_next()
    with np.load(record) as trace:
        assert trace["commanded"].shape == (0, NUM_JOINTS)
        assert trace["measured"].shape == (0, NUM_JOINTS)


def test_an_interrupted_run_still_writes_what_it_measured(tmp_path, make_replay, monkeypatch):
    record = tmp_path / "trace.npz"
    node = make_replay(clip=write_clip(tmp_path, frames=30), rate="30.0", record=str(record))
    # What the gate does is proven in test_first_frame_gate.py; this is about save_trace, and with
    # a record path the gate would otherwise hold every frame waiting for a real publisher.
    monkeypatch.setattr(node._gate, "ready", lambda: True)
    node._publish_next()
    node._publish_next()
    node._on_measured(JointState(name=list(RIGHT_NAMES), position=[0.2] * NUM_JOINTS))
    node.save_trace()
    with np.load(record) as trace:
        assert trace["raw"].shape == (2, NUM_JOINTS)
        assert trace["measured"].shape == (1, NUM_JOINTS)


def test_a_run_with_no_record_path_completes_without_writing_a_trace(tmp_path, make_replay):
    node = make_replay(clip=write_clip(tmp_path, frames=3), rate="30.0")
    with pytest.raises(SystemExit):
        for _ in range(4):
            node._publish_next()
    assert [p.name for p in tmp_path.glob("*.npz")] == ["clip.npz"]
