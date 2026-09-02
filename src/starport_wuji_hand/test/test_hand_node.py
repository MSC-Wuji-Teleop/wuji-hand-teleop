"""In-process tests for the Wuji hand driver node, with no hardware.

wujihandpy is imported lazily inside _connect(), so constructing the node never touches a device,
and conftest.py makes that import fail outright so no test here can reach a real one.
A fake hand and realtime controller are injected to drive the command, publish and lifecycle
paths; callbacks are invoked directly (no graph spinning) and publishers are mocked so published
messages can be asserted.
"""

import json
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import yaml
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import JointState
from starport_wuji_hand import hand_node
from starport_wuji_hand.hand_node import _MAX_TICK_FACTOR, WujiHandNode
from starport_wuji_hand.joint_map import NUM_JOINTS, index_to_nid, joint_names
from starport_wuji_hand.limits_io import load_friction

RIGHT_NAMES = joint_names("right")

# The tests intentionally poke the node's private connection state and callbacks.
# ruff: noqa: SLF001

LIMITS_YAML = str(Path(__file__).resolve().parents[1] / "config" / "joint_limits_hand2_beta1_right.yaml")
FRICTION_JSON = str(Path(__file__).resolve().parents[1] / "config" / "measured_friction_WH2KA01260810003.json")


class FakeStreamHandle:
    """Stands in for the node's stream holder: newest frame plus its age."""

    def __init__(self, frame=None, age=0.0):
        self._frame = frame
        self._age = age

    def set(self, positions=None, efforts=None, **kw):
        self._frame = fake_frame(positions, efforts, **kw)

    def get(self):
        return self._frame

    def age(self):
        # Fresh by default; a test that wants a dead link passes an age past the node's timeout.
        return self._age

    def close(self):
        pass


class FakePublisher:
    """Records the setpoint frames the driver ships.

    The USB SDK took a (5, 4) array through a controller object; this one takes a flat list of
    twenty JointCommand. Recording the poses rather than the calls keeps the assertions about what
    reached the hand instead of about the shape of the call that carried it.
    """

    def __init__(self, side_effect=None):
        self.sent = []
        self.velocities = []
        self.efforts = []
        self.side_effect = side_effect

    def send(self, commands):
        if self.side_effect is not None:
            raise self.side_effect
        self.sent.append(np.array([c.pos for c in commands], dtype=float))
        self.velocities.append(np.array([c.vel for c in commands], dtype=float))
        self.efforts.append(np.array([c.eff for c in commands], dtype=float))

    def reset(self):
        self.sent.clear()
        self.velocities.clear()
        self.efforts.clear()


def fake_ctrl(side_effect=None):
    """The publisher double, under the name the tests have always used for "what gets written"."""
    return FakePublisher(side_effect)


def matching_hand(*_args, **_kwargs):
    """A hand whose reported state matches the committed table.

    Kept as a name because many tests read better for it, but it no longer has to reconcile limit
    registers: the device exposes none, and handedness is checked directly instead.
    """
    return fake_hand()


def fake_frame(positions=None, efforts=None, ext_state=2, error_code=0, position_limit=False):
    """One joint_states / joint_diagnostics frame, in the device's own bus-node numbering.

    Entries carry ``nid``, not a joint index -- the gap between the two is the wrong-finger hazard
    this package guards, so the doubles reproduce it rather than handing the driver a convenient
    dense list it would never see from a real hand.
    """
    positions = np.zeros(NUM_JOINTS) if positions is None else np.asarray(positions, dtype=float)
    efforts = np.zeros(NUM_JOINTS) if efforts is None else np.asarray(efforts, dtype=float)
    joints = [
        SimpleNamespace(
            nid=index_to_nid(i),
            position=float(positions[i]),
            velocity=0.0,
            effort=float(efforts[i]),
            error_code=error_code,
            status_word=SimpleNamespace(ext_state=ext_state, position_limit=position_limit),
        )
        for i in range(NUM_JOINTS)
    ]
    return SimpleNamespace(
        header=SimpleNamespace(seq=1, timestamp_us=0, frame_id=""), num_joints=NUM_JOINTS, joints=joints
    )


class FakeStream:
    """A stand-in stream that hands its frame to the subscriber immediately."""

    def __init__(self, frame):
        self.frame = frame
        self.closed = False

    def subscribe_with_callback(self, callback):
        callback(self.frame)
        stream = self

        class _Sub:
            def close(self_inner):
                stream.closed = True

        return _Sub()


def fake_hand(positions=None, efforts=None, handedness="right", online=NUM_JOINTS, ext_state=2, error_code=0):
    """A stand-in wuji_sdk WujiHand2."""
    hand = MagicMock()
    hand.handedness.return_value = handedness
    hand.serial_number = "WHFAKE0000000001"
    hand.online_joints_count.return_value = SimpleNamespace(get=lambda: online)
    hand.joint_states.return_value = FakeStream(fake_frame(positions, efforts))
    hand.joint_diagnostics.return_value = FakeStream(
        fake_frame(positions, efforts, ext_state=ext_state, error_code=error_code)
    )
    hand.joint_command.return_value.publish.return_value = FakePublisher()
    return hand


def set_hand_pose(hand, positions):
    """Point the fake hand's state stream at a different pose."""
    hand.joint_states.return_value = FakeStream(fake_frame(positions))
    return hand


def fake_sdk(monkeypatch, hand, found=True):
    """Install a stand-in wuji_sdk module so _connect can be driven with no device.

    conftest's interlock makes the real import fail -- which matters more than it used to, because
    the hand is on the network rather than on a cable, so an unguarded connect would find a real
    one. This replaces the module for one test.
    """
    device = SimpleNamespace(sn="WHFAKE0000000001", device_type="WujiHand2", address="192.0.2.1:7447")
    manager = MagicMock()
    manager.scan.return_value = [device] if found else []
    manager.connect.return_value = hand
    module = SimpleNamespace(
        SdkManager=SimpleNamespace(instance=lambda: manager),
        DeviceType=SimpleNamespace(WujiHand2="WujiHand2"),
        JointCommand=lambda pos, vel, eff: SimpleNamespace(pos=pos, vel=vel, eff=eff),
    )
    monkeypatch.setitem(sys.modules, "wuji_sdk", module)
    return module, hand.joint_command.return_value.publish.return_value


@pytest.fixture
def node():
    n = None
    try:
        n = WujiHandNode(cli_args=["--ros-args", "-p", f"limits_file:={LIMITS_YAML}", "-p", "home_on_start:=false"])
        n._pub_joint = MagicMock()
        n._pub_commanded = MagicMock()
        n._pub_connected = MagicMock()
        n._pub_diag = MagicMock()
        yield n
    finally:
        if n is not None:
            try:
                n.destroy_node()
            except Exception:
                pass


@pytest.fixture
def make_node():
    """Build nodes with extra parameter overrides, cleaned up at the end of the test."""
    created = []

    def factory(**overrides):
        args = ["--ros-args", "-p", f"limits_file:={LIMITS_YAML}", "-p", "home_on_start:=false"]
        for name, value in overrides.items():
            args += ["-p", f"{name}:={value}"]
        n = WujiHandNode(cli_args=args)
        created.append(n)
        n._pub_joint = MagicMock()
        n._pub_commanded = MagicMock()
        n._pub_connected = MagicMock()
        n._pub_diag = MagicMock()
        return n

    yield factory
    for n in created:
        try:
            n.destroy_node()
        except Exception:
            pass


def test_construction_never_touches_hardware(node):
    assert node._hand is None
    assert node._pub is None


def test_limits_load_from_the_committed_yaml(node):
    assert node._limits.raw_lower.shape == (NUM_JOINTS,)
    # r_thumb_cmc_flex is the thumb cmc_flex: -1.187 .. 1.291 in the MJCF.
    assert node._limits.raw_lower[0] == pytest.approx(-1.187)
    assert node._limits.raw_upper[0] == pytest.approx(1.291)


def test_guard_chain_is_seeded_and_holds_before_any_command(node):
    assert node._chain.last_safe.shape == (NUM_JOINTS,)
    assert np.isfinite(node._chain.last_safe).all()


def test_publish_state_no_ops_while_disconnected(node):
    node._publish_state()
    node._pub_joint.publish.assert_not_called()
    # Connection health is still reported, so a down link is visible rather than silent.
    node._pub_connected.publish.assert_called_once()
    assert node._pub_connected.publish.call_args[0][0].data is False


def test_publish_state_emits_twenty_named_positions_when_connected(node):
    connected(node)
    node._state.set(np.arange(20, dtype=np.float64))
    node._publish_state()
    msg = node._pub_joint.publish.call_args[0][0]
    assert isinstance(msg, JointState)
    assert list(msg.name) == list(RIGHT_NAMES)
    assert msg.position == pytest.approx(list(range(20)))


def test_velocity_is_derived_from_the_position_stream(node, monkeypatch):
    """The hand measures position and current only, and the PACT policy observes twenty velocities.

    Zero on the first sample -- there is nothing to difference yet -- then a filtered derivative.
    The filter means the first moving sample is a FRACTION of the true rate, not the rate itself,
    so this asserts direction and bound rather than an exact value: it is a one-pole low pass, and
    pinning its output to a constant would be pinning the cutoff.
    """
    connected(node)
    clock = [100.0]
    monkeypatch.setattr(node, "_now_seconds", lambda: clock[0])

    node._state.set(np.zeros(NUM_JOINTS))
    node._publish_state()
    assert node._pub_joint.publish.call_args[0][0].velocity == pytest.approx([0.0] * NUM_JOINTS)

    clock[0] += 0.01  # one publish period at the default 100 Hz
    node._state.set(np.full(NUM_JOINTS, 0.02))  # 0.02 rad in 10 ms = 2.0 rad/s
    node._publish_state()
    velocity = np.asarray(node._pub_joint.publish.call_args[0][0].velocity)
    assert velocity.shape == (NUM_JOINTS,)
    assert np.all(velocity > 0.0), "moving in +q must read positive"
    assert np.all(velocity < 2.0), "a one-pole filter cannot reach the true rate in one sample"


def test_a_long_gap_is_not_reported_as_a_velocity(node, monkeypatch):
    """After a stall the position difference spans time the hand was not watched.

    Dividing it by that time reports an average that never happened -- and on a hand that moved
    while nobody was looking, it reports it as if it were happening now.
    """
    connected(node)
    clock = [100.0]
    monkeypatch.setattr(node, "_now_seconds", lambda: clock[0])
    node._state.set(np.zeros(NUM_JOINTS))
    node._publish_state()

    clock[0] += 5.0  # far beyond `_velocity_max_gap`
    node._state.set(np.full(NUM_JOINTS, 1.0))
    node._publish_state()

    assert node._pub_joint.publish.call_args[0][0].velocity == pytest.approx([0.0] * NUM_JOINTS)


def test_a_reconnect_does_not_difference_across_the_disconnection(node, monkeypatch):
    connected(node)
    clock = [100.0]
    monkeypatch.setattr(node, "_now_seconds", lambda: clock[0])
    node._state.set(np.zeros(NUM_JOINTS))
    node._publish_state()

    node._disconnect()
    connected(node)
    clock[0] += 0.01
    node._state.set(np.full(NUM_JOINTS, 0.5))
    node._publish_state()

    assert node._pub_joint.publish.call_args[0][0].velocity == pytest.approx(
        [0.0] * NUM_JOINTS
    ), "the sample after a reconnect is a FIRST sample"


def test_publish_state_drops_the_connection_on_a_read_failure(node):
    connected(node)
    node._state = FakeStreamHandle(None)
    node._publish_state()
    assert node._hand is None
    # Two connected publishes: the optimistic one, then False after the drop.
    assert node._pub_connected.publish.call_args[0][0].data is False


def test_publish_state_drops_the_connection_on_a_non_finite_read(node):
    # A NaN pose on /joint_states would propagate through robot_state_publisher into every TF
    # consumer, so an unusable read is a dead link, not a state to forward.
    connected(node)
    node._state.set(np.full(NUM_JOINTS, np.nan))
    node._publish_state()
    node._pub_joint.publish.assert_not_called()
    assert node._hand is None


def test_disconnect_disables_the_hand(node):
    hand = fake_hand()
    node._hand = hand
    node._disconnect()
    hand.disable.assert_called_once()
    assert node._hand is None


def test_disconnect_survives_a_failing_disable(node):
    hand = fake_hand()
    hand.write_joint_enabled.side_effect = RuntimeError("already gone")
    node._hand = hand
    node._disconnect()  # must not raise: shutdown has to complete
    assert node._hand is None


def test_a_missing_limits_file_refuses_to_start():
    with pytest.raises(ValueError, match="limits_file"):
        WujiHandNode(cli_args=["--ros-args", "-p", "home_on_start:=false"])


@pytest.mark.parametrize(
    ("name", "value"),
    [
        # A zero rate divides by zero when the timers are built.
        ("command_rate", "0.0"),
        ("publish_rate", "0.0"),
        ("diagnostics_rate", "0.0"),
        # A non-positive travel budget drives the wrong way or locks every joint.
        ("max_joint_velocity", "-1.0"),
        # A non-positive or NaN timeout makes the watchdog's staleness test permanently false.
        ("command_timeout", "0.0"),
        ("command_timeout", ".nan"),
        # At zero, a reconnect is attempted on every tick -- ~100 USB opens a second, each one
        # leaking a Hand -- and the homing sweep collapses to the single step it exists to avoid.
        ("reconnect_interval", "0.0"),
        ("home_duration_s", "0.0"),
    ],
)
def test_a_parameter_that_would_disable_a_guard_refuses_to_start(make_node, name, value):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: value})


def tick_one_nominal_period(node, monkeypatch) -> None:
    """Run one tick whose measured gap is exactly the nominal period.

    The travel budget tracks the measured gap and has no floor, so a callback invoked directly in a
    test would otherwise see a microsecond-long gap and grant a microsecond of travel. Freezing the
    clock is what makes the granted budget an exact number rather than a scheduling artefact.
    """
    now = time.monotonic()
    monkeypatch.setattr(hand_node.time, "monotonic", lambda: now)
    node._last_tick = now - 1.0 / node._command_rate
    node._tick()


def correction_array(default: float, changes: dict[int, float]) -> np.ndarray:
    """A ``(20,)`` array, ``default`` everywhere except the named indices."""
    values = np.full(NUM_JOINTS, default)
    for index, value in changes.items():
        values[index] = value
    return values


def array_arg(default: float, changes: dict[int, float]) -> str:
    """The same, as the literal a launch argument or -p would carry."""
    return "[" + ",".join(str(value) for value in correction_array(default, changes).tolist()) + "]"


@pytest.mark.parametrize(
    ("name", "value"),
    [
        # Why only the two signs: _require_signs says.
        ("joint_sign", array_arg(1.0, {0: 0.0})),
        ("joint_sign", array_arg(1.0, {5: 2.0})),
        ("joint_sign", array_arg(1.0, {19: -0.5})),
        # A correction that does not cover all twenty joints silently corrects the wrong ones.
        ("joint_sign", "[1.0,1.0]"),
        ("joint_offset", "[0.0,0.0]"),
        # A non-finite offset would make every mapped command NaN, before any guard sees it.
        ("joint_offset", "[.nan,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
    ],
)
def test_an_unusable_sign_or_zero_correction_refuses_to_start(make_node, name, value):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: value})


def test_a_negative_joint_sign_inverts_exactly_one_joints_commanded_value(make_node, monkeypatch):
    # A logical command well inside every joint's range, with a budget wide enough that the slew
    # limiter is not what is being measured here.
    node = make_node(max_joint_velocity="1000.0", joint_sign=array_arg(1.0, {6: -1.0}))
    connected(node)
    node._on_command(JointState(position=[0.3] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    expected = np.full(NUM_JOINTS, 0.3)
    expected[6] = -0.3
    written = node._pub.sent[-1]
    np.testing.assert_allclose(written, expected)
    # The chain holds what was written, because the chain runs in the hand's frame.
    np.testing.assert_allclose(node._chain.last_safe, expected)


def test_a_joint_offset_shifts_exactly_one_joints_commanded_value(make_node, monkeypatch):
    node = make_node(max_joint_velocity="1000.0", joint_offset=array_arg(0.0, {3: 0.15}))
    connected(node)
    node._on_command(JointState(position=[0.3] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    expected = np.full(NUM_JOINTS, 0.3)
    expected[3] = 0.45
    written = node._pub.sent[-1]
    np.testing.assert_allclose(written, expected)


def test_a_flipped_joint_is_clamped_in_the_hands_own_frame_and_reported(make_node, monkeypatch):
    # r_index_finger_pip is the hand's most asymmetric joint (-1.047 .. 2.094), so a flip is where
    # the frames are furthest apart and a write outside the envelope would be largest. Clamping in
    # the hand's frame is what makes the written value the value the clamp bounded, so the assertion
    # below is on the MIRRORED soft bound; a clamp applied before the flip would bound a pre-image
    # instead and let the write land outside the hand's own declared envelope, with clamped False --
    # the silent firmware truncation this guard chain exists to expose.
    node = make_node(max_joint_velocity="1000.0", joint_sign=array_arg(1.0, {6: -1.0}))
    connected(node)
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    written = node._pub.sent[-1]
    # The mirrored envelope's soft lower end: -(2.094) + the 0.02 margin.
    assert written[6] == pytest.approx(-2.074)
    assert written[6] >= -2.094, "the write left the hand's declared envelope"
    node._publish_diagnostics()
    assert RIGHT_NAMES[6] in published_values(node)["clamped"]


def test_the_soft_limits_are_expressed_in_the_hands_own_frame(make_node):
    # A flipped joint's range reflects, ends swapped and re-ordered; an offset joint's shifts. Both
    # are compared pre-margin, which is the envelope cross_check holds the hardware to.
    node = make_node(joint_sign=array_arg(1.0, {6: -1.0}), joint_offset=array_arg(0.0, {3: 0.15}))
    assert node._limits.raw_lower[6] == pytest.approx(-2.094)
    assert node._limits.raw_upper[6] == pytest.approx(1.047)
    assert node._limits.raw_lower[3] == pytest.approx(-1.047 + 0.15)
    assert node._limits.raw_upper[3] == pytest.approx(1.57 + 0.15)
    # Every other joint is untouched by either correction.
    assert node._limits.raw_lower[0] == pytest.approx(-1.187)
    assert node._limits.raw_upper[0] == pytest.approx(1.291)


def test_a_named_command_holds_an_unlisted_joint_where_it_physically_is(make_node, monkeypatch):
    # resolve_command fills unlisted joints from the pose it is given, so that pose has to be in
    # the same frame as the values arriving beside it -- the LOGICAL one. Handed the chain's state
    # unmapped, a flipped joint's held value would be mapped a second time and the joint would jump
    # by twice its position while nobody commanded it. Only a NAMED command reads that hold base,
    # which is why a bare-vector test cannot see this.
    node = make_node(max_joint_velocity="1000.0", joint_sign=array_arg(1.0, {6: -1.0}))
    connected(node)
    node._on_command(JointState(position=[0.3] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    node._on_command(JointState(name=[RIGHT_NAMES[0]], position=[0.5]))
    tick_one_nominal_period(node, monkeypatch)

    expected = np.full(NUM_JOINTS, 0.3)
    expected[0] = 0.5
    expected[6] = -0.3  # untouched by the named command, and still where the hand is
    written = node._pub.sent[-1]
    np.testing.assert_allclose(written, expected)


def test_the_ghost_stays_in_the_logical_frame(make_node, monkeypatch):
    # The raw goal ghost carries what a publisher sent and the URDF joint names are logical, so a
    # post-guard ghost in the hand's frame would silently break the three-way RViz comparison.
    node = make_node(max_joint_velocity="1000.0", joint_sign=array_arg(1.0, {6: -1.0}))
    connected(node)
    node._on_command(JointState(position=[0.3] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    ghost = node._pub_commanded.publish.call_args[0][0]
    np.testing.assert_allclose(ghost.position, np.full(NUM_JOINTS, 0.3))
    assert node._pub.sent[-1][6] == pytest.approx(-0.3)


def test_the_measured_state_is_published_in_the_logical_frame(make_node):
    # /joint_states carries URDF joint names and is what robot_state_publisher draws the solid hand
    # from, so it has to be in the same frame as the ghosts. In the hand's frame a flipped joint
    # would render mirrored against them -- the two channels differing by 2q, not by nothing.
    node = make_node(joint_sign=array_arg(1.0, {6: -1.0}), joint_offset=array_arg(0.0, {3: 0.1}))
    connected(node)
    node._state.set(np.full(NUM_JOINTS, 0.3))
    node._publish_state()

    expected = np.full(NUM_JOINTS, 0.3)
    expected[6] = -0.3  # the hand reads +0.3 on a joint wired the other way round
    expected[3] = 0.2  # and +0.3 on a joint whose zero sits 0.1 away
    np.testing.assert_allclose(node._pub_joint.publish.call_args[0][0].position, expected)


def test_both_published_channels_are_in_the_same_frame(make_node, monkeypatch):
    # The property the three-way RViz comparison actually rests on: what the driver was asked for
    # and what it measures are published in one frame, and the hand's own numbers appear in neither.
    node = make_node(max_joint_velocity="1000.0", joint_sign=array_arg(1.0, {6: -1.0}))
    connected(node)
    node._on_command(JointState(position=[0.3] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)
    written = node._pub.sent[-1]
    assert written[6] == pytest.approx(-0.3)  # the hand is driven the other way round ...

    node._state.set(written)  # ... and reports it so
    node._publish_state()
    ghost = node._pub_commanded.publish.call_args[0][0]
    measured = node._pub_joint.publish.call_args[0][0]
    np.testing.assert_allclose(ghost.position, np.full(NUM_JOINTS, 0.3))
    np.testing.assert_allclose(measured.position, np.full(NUM_JOINTS, 0.3))


def test_an_unknown_joint_in_the_limits_table_is_still_named(make_node, tmp_path):
    # The transform passes a name the hand does not have straight through, so Limits.from_mapping
    # still reports it by name. Indexing it instead would turn a typo in a limits file into a bare
    # KeyError naming nothing.
    table = yaml.safe_load(Path(LIMITS_YAML).read_text())
    table["joints"]["right_finger9_joint1"] = {"lower": -1.0, "upper": 1.0}
    path = tmp_path / "limits.yaml"
    path.write_text(yaml.safe_dump(table))
    with pytest.raises(ValueError, match="unknown joint"):
        make_node(limits_file=str(path), joint_sign=array_arg(1.0, {6: -1.0}))


def test_a_correction_does_not_make_the_first_write_after_connect_a_step(make_node, monkeypatch):
    # The chain is seeded from the measured pose and the chain is in the hand's frame, so the first
    # write reproduces the pose the hand is already in -- no mapping involved, under any correction.
    flip, zero = {6: -1.0}, {3: 0.1}
    node = make_node(joint_sign=array_arg(1.0, flip), joint_offset=array_arg(0.0, zero))
    # The fake hand is wired the way the correction says it is, so the cross-check agrees.
    hand = matching_hand(sign=correction_array(1.0, flip), offset=correction_array(0.0, zero))
    set_hand_pose(hand, np.full(NUM_JOINTS, 0.3))
    _sdk, ctrl = fake_sdk(monkeypatch, hand)

    assert node._connect() is True
    node._tick()  # no command has ever arrived, so this is the held seed going out
    written = ctrl.sent[-1]
    np.testing.assert_allclose(written, np.full(NUM_JOINTS, 0.3))


def test_the_homing_sweep_runs_in_the_hands_frame_to_the_corrected_home(make_node, monkeypatch):
    # Homing writes never pass the guard chain, so they have to be in the hand's frame themselves:
    # from the measured pose to wherever a logical zero lands, which is where the next tick holds.
    zero = {3: 0.1}
    node = make_node(joint_offset=array_arg(0.0, zero))
    _sdk, ctrl = fake_sdk(monkeypatch, fake_hand(positions=np.zeros(NUM_JOINTS)))
    node._home_on_start = True
    node._home_duration = 0.02

    assert node._connect() is True
    assert ctrl.sent, "homing wrote nothing"
    # The sweep ends at the corrected home: logical zero mapped into the hand's frame.
    np.testing.assert_allclose(ctrl.sent[-1], correction_array(0.0, zero), atol=1e-9)


def test_a_negative_limit_margin_refuses_to_start(make_node):
    # A negative margin WIDENS the soft limits past the declared envelope, which is the exact
    # inverse of what the parameter is for.
    with pytest.raises(ValueError, match="limit_margin"):
        make_node(limit_margin="-0.05")


# ----------------------------- command path -----------------------------


def connected(node, positions=None, efforts=None):
    """Put a node into the state _connect would leave it in, with no device involved."""
    hand = fake_hand()
    node._hand = hand
    node._pub = fake_ctrl()
    node._state = FakeStreamHandle(fake_frame(positions, efforts))
    node._diag = FakeStreamHandle(fake_frame(positions, efforts))
    node._command_type = lambda pos, vel, eff: SimpleNamespace(pos=pos, vel=vel, eff=eff)
    node._energized = True
    return hand


def published_values(node):
    """The key/value pairs of the most recently published DiagnosticArray."""
    arr = node._pub_diag.publish.call_args[0][0]
    return {kv.key: kv.value for st in arr.status for kv in st.values}


def test_bare_twenty_array_command_is_accepted(node):
    connected(node)
    msg = JointState(position=[0.1] * NUM_JOINTS)
    node._on_command(msg)
    assert node._pending is not None
    np.testing.assert_allclose(node._pending, np.full(NUM_JOINTS, 0.1))


def test_named_command_resolves_by_name_not_position(node):
    connected(node)
    msg = JointState(name=["r_index_finger_pip"], position=[0.4])
    node._on_command(msg)
    assert node._pending[6] == pytest.approx(0.4)


def test_unknown_joint_name_is_refused_and_leaves_nothing_pending(node):
    connected(node)
    node._on_command(JointState(name=["bogus_joint"], position=[0.4]))
    assert node._pending is None


def test_wrong_length_bare_command_is_refused(node):
    connected(node)
    node._on_command(JointState(position=[0.1] * 19))
    assert node._pending is None


def test_tick_writes_the_post_guard_target_to_the_controller(node):
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._tick()
    assert bool(node._pub.sent)
    written = node._pub.sent[-1]
    assert written.shape == (NUM_JOINTS,)
    assert np.isfinite(written).all()


def test_publish_pose_ships_the_velocity_it_is_given(node):
    connected(node)
    node._pub.reset()
    node._publish_pose(np.zeros(NUM_JOINTS), np.full(NUM_JOINTS, 0.25))
    np.testing.assert_allclose(node._pub.velocities[-1], np.full(NUM_JOINTS, 0.25))


def test_publish_pose_defaults_to_a_zero_velocity(node):
    # Only correct for a caller that is not moving; a moving one must supply its own.
    connected(node)
    node._pub.reset()
    node._publish_pose(np.zeros(NUM_JOINTS))
    np.testing.assert_allclose(node._pub.velocities[-1], np.zeros(NUM_JOINTS))


def test_a_moving_setpoint_carries_its_own_velocity(node):
    # Without this the damping term reads the motion as error and spends kd*qd opposing it.
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._last_tick = time.monotonic() - 0.01  # one nominal tick, so the filter actually advances
    node._tick()
    assert np.all(node._pub.velocities[-1] > 0.0)


def test_release_clears_the_setpoint_velocity(node):
    # A limp hand is not moving, and the stale value would land on the first setpoint after it.
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._last_tick = time.monotonic() - 0.01
    node._tick()
    assert np.any(node._setpoint_velocity != 0.0)
    node._release()
    np.testing.assert_allclose(node._setpoint_velocity, np.zeros(NUM_JOINTS))


def test_no_friction_table_means_no_feedforward(node):
    # The default: an unmeasured hand gets nothing, because compensating friction it does not
    # have is worse than compensating none.
    np.testing.assert_allclose(node._friction_feedforward(), np.zeros(NUM_JOINTS))


def test_friction_feedforward_opposes_the_direction_of_travel(node):
    node._friction = np.full(NUM_JOINTS, 0.1)
    node._friction_scale = 1.0
    node._friction_deadzone = 0.02
    node._setpoint_velocity = np.full(NUM_JOINTS, 0.5)  # well past the deadzone
    np.testing.assert_allclose(node._friction_feedforward(), np.full(NUM_JOINTS, 0.1))
    node._setpoint_velocity = np.full(NUM_JOINTS, -0.5)
    np.testing.assert_allclose(node._friction_feedforward(), np.full(NUM_JOINTS, -0.1))


def test_friction_feedforward_ramps_through_the_deadzone(node):
    # A hard sign would put the full current on a standstill and step twice the friction across
    # every zero crossing.
    node._friction = np.full(NUM_JOINTS, 0.1)
    node._friction_scale = 1.0
    node._friction_deadzone = 0.02
    node._setpoint_velocity = np.zeros(NUM_JOINTS)
    np.testing.assert_allclose(node._friction_feedforward(), np.zeros(NUM_JOINTS))
    node._setpoint_velocity = np.full(NUM_JOINTS, 0.01)  # half the deadzone
    np.testing.assert_allclose(node._friction_feedforward(), np.full(NUM_JOINTS, 0.05))


def test_publish_pose_ships_the_effort_it_is_given(node):
    connected(node)
    node._pub.reset()
    node._publish_pose(np.zeros(NUM_JOINTS), None, np.full(NUM_JOINTS, 0.07))
    np.testing.assert_allclose(node._pub.efforts[-1], np.full(NUM_JOINTS, 0.07))


def test_publish_pose_defaults_to_no_feedforward(node):
    connected(node)
    node._pub.reset()
    node._publish_pose(np.zeros(NUM_JOINTS))
    np.testing.assert_allclose(node._pub.efforts[-1], np.zeros(NUM_JOINTS))


def _friction_file(table):
    fh = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump({"friction_a": table}, fh)
    fh.close()
    return fh.name


def test_friction_table_must_name_every_joint():
    with pytest.raises(ValueError, match="no friction for"):
        load_friction(_friction_file({n: 0.1 for n in RIGHT_NAMES[:-1]}), RIGHT_NAMES)


def test_friction_table_rejects_a_joint_this_hand_does_not_have():
    table = {n: 0.1 for n in RIGHT_NAMES} | {"l_pinky_dip": 0.1}
    with pytest.raises(ValueError, match="does not have"):
        load_friction(_friction_file(table), RIGHT_NAMES)


def test_friction_table_rejects_a_negative_value():
    table = {n: 0.1 for n in RIGHT_NAMES} | {RIGHT_NAMES[0]: -0.1}
    with pytest.raises(ValueError, match="non-negative"):
        load_friction(_friction_file(table), RIGHT_NAMES)


def test_the_committed_friction_table_loads_for_its_hand():
    values = load_friction(FRICTION_JSON, RIGHT_NAMES)
    assert len(values) == NUM_JOINTS
    assert all(v >= 0.0 for v in values)


def test_tick_publishes_the_post_guard_target_for_the_rviz_ghost(node):
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._tick()
    msg = node._pub_commanded.publish.call_args[0][0]
    assert list(msg.name) == list(RIGHT_NAMES)
    assert len(msg.position) == NUM_JOINTS


def test_tick_clears_pending_so_one_command_is_written_once(node):
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._tick()
    assert node._pending is None


def test_tick_holds_the_last_safe_target_when_no_command_arrives(node):
    connected(node)
    node._on_command(JointState(position=[0.2] * NUM_JOINTS))
    node._tick()
    first = node._pub.sent[-1].copy()
    node._tick()  # nothing new pending
    second = node._pub.sent[-1]
    np.testing.assert_allclose(second, first)


def test_out_of_range_command_is_clamped_before_reaching_the_controller(node):
    connected(node)
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._tick()
    written = node._pub.sent[-1]
    assert np.all(written <= node._limits.upper + 1e-9)


def test_nan_command_never_reaches_the_controller(node):
    connected(node)
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    node._tick()
    node._pub.reset()
    bad = [0.1] * NUM_JOINTS
    bad[3] = float("nan")
    node._on_command(JointState(position=bad))
    node._tick()
    written = node._pub.sent[-1]
    assert np.isfinite(written).all()


def test_a_stalled_tick_does_not_buy_a_larger_jump(node):
    # A long gap between ticks -- a homing sweep, a blocked USB write, a suspended process -- must
    # not hand the slew limit a proportionally larger travel budget on the next tick.
    connected(node)
    before = node._chain.last_safe
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._last_tick = time.monotonic() - 10.0
    node._tick()
    written = node._pub.sent[-1]
    budget = node._max_velocity * _MAX_TICK_FACTOR / node._command_rate
    assert np.all(np.abs(written - before) <= budget + 1e-9)


def test_two_ticks_in_one_clock_instant_take_the_chains_glitch_fallback(make_node, monkeypatch):
    # The budget tracks the MEASURED gap, with a cap and no floor, so a zero-length gap -- two
    # ticks landing on one clock reading -- reaches the guard chain's clock-glitch fallback. That
    # branch is only reachable from here while no floor exists, and the travel it grants is the
    # chain's own nominal tick rather than this node's period: 2.0 rad/s * 0.01 s, not * 0.02 s.
    node = make_node(command_rate="50.0")
    connected(node)
    monkeypatch.setattr(hand_node.time, "monotonic", lambda: 1234.5)
    node._last_tick = 1234.5
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._tick()
    written = node._pub.sent[-1]
    np.testing.assert_allclose(written, np.full(NUM_JOINTS, 2.0 * 0.01))


def test_tick_no_ops_while_disconnected(node):
    # Belt and braces over conftest's import interlock: this is also the ordinary steady state
    # after a failed connect, when the backoff has not yet elapsed.
    node._next_attempt = time.monotonic() + 60.0
    node._pending = np.zeros(NUM_JOINTS)
    node._tick()  # must not raise with _ctrl None
    assert node._pub_commanded.publish.called is False


def test_diagnostics_report_clamp_activity(node):
    connected(node)
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._tick()
    node._publish_diagnostics()
    # The VALUE, not the key: the key is published on every tick whether anything clamped or not.
    reported = published_values(node)
    assert RIGHT_NAMES[0] in reported["clamped"]
    assert reported["clamped"].count(",") == NUM_JOINTS - 1


def test_diagnostics_keep_a_dropped_link_at_error_level(node):
    # The last guard report outlives the connection, so clamp activity from before a drop must not
    # downgrade a dead link to a mere warning.
    connected(node)
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._tick()
    node._disconnect()
    node._publish_diagnostics()
    status = node._pub_diag.publish.call_args[0][0].status[0]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "not connected"


def test_diagnostics_report_staleness_separately_from_the_rejection_reason(make_node):
    # A rejected command never refreshes the watchdog, so a stream of malformed messages must not
    # hide a dead publisher behind a reason string that only names the rejection.
    node = make_node(command_timeout="0.02")
    connected(node)
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    node._tick()  # one accepted command starts the watchdog clock
    bad = [0.1] * NUM_JOINTS
    bad[3] = float("nan")
    for _ in range(3):
        time.sleep(0.01)
        node._on_command(JointState(position=bad))
        node._tick()
    node._publish_diagnostics()
    reported = published_values(node)
    assert reported["stale"] == "True"
    assert "not finite" in reported["last_reason"]


def test_write_failure_drops_the_connection(node):
    connected(node)
    node._pub.side_effect = RuntimeError("usb gone")
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    node._tick()
    assert node._hand is None


# ----------------------------- the connect path -----------------------------


def test_connect_seeds_the_guard_chain_from_the_measured_pose(node, monkeypatch):
    # What a zero seed would do instead, and why the slew limit cannot catch it: hand_node._connect.
    hand = matching_hand()
    set_hand_pose(hand, np.full(NUM_JOINTS, 0.3))
    _sdk, ctrl = fake_sdk(monkeypatch, hand)

    assert node._connect() is True
    np.testing.assert_allclose(node._chain.last_safe, np.full(NUM_JOINTS, 0.3))

    node._tick()  # no command has ever arrived
    written = ctrl.sent[-1]
    np.testing.assert_allclose(written, np.full(NUM_JOINTS, 0.3))


def test_homing_refuses_a_non_finite_measured_pose(node):
    # The homing writes are the only ones that never pass the guard chain, so a garbage read would
    # otherwise become a sweep of NaN setpoints straight into the hand.
    connected(node)
    node._state = FakeStreamHandle(fake_frame(np.full(NUM_JOINTS, np.nan)))
    with pytest.raises(RuntimeError):
        node._home()
    assert not node._pub.sent


def test_an_unusable_pose_during_homing_is_retried_rather_than_latched_as_a_config_fault(node, monkeypatch):
    # An unusable pose is a bad reading, not a bad configuration: latching would put "refusing to
    # run" in front of a bench engineer over a transient.
    fake_sdk(monkeypatch, fake_hand(positions=np.full(NUM_JOINTS, np.nan)))
    node._home_on_start = True

    assert node._connect() is False
    assert node._fatal_reason is None


def test_an_unusable_hand_side_is_refused_before_a_node_exists(make_node):
    # Refused at construction, so `ros2 run ... -p hand_side:=sideways` cannot reach a device at
    # all. It used to be caught only at connect, which left the launch file's `choices` as the
    # only thing standing between a typo and an opened hand.
    with pytest.raises(ValueError, match="hand_side must be one of"):
        make_node(hand_side="sideways")


# ----------------------------- diagnostics -----------------------------


def test_diagnostics_report_hardware_health(node):
    # Health comes off the diagnostics stream now. No temperature is reported: the device exposes
    # none per joint, and a field the hand does not measure is worse than an absent one.
    connected(node)
    node._diag = FakeStreamHandle(fake_frame(error_code=7))
    node._publish_diagnostics()
    reported = published_values(node)
    assert "r_thumb_cmc_flex=7" in reported["error_codes"]
    assert "link_age_s" in reported
    assert "max_temperature_c" not in reported


def test_latched_clamp_activity_survives_the_holds_that_follow_it(node):
    # At 100 Hz most ticks are holds reporting no activity, so a clamp has to be latched or the
    # 10 Hz diagnostics sample almost never lands on the tick that saw it.
    connected(node)
    node._on_command(JointState(position=[99.0] * NUM_JOINTS))
    node._tick()
    for _ in range(5):
        node._tick()  # holds, each reporting nothing clamped

    node._publish_diagnostics()
    first = published_values(node)
    assert RIGHT_NAMES[0] in first["clamped"]
    assert first["last_rejection"] == ""  # a clamp is not a rejection

    node._publish_diagnostics()
    assert published_values(node)["clamped"] == ""  # cleared once reported


def test_disconnect_drops_a_pending_target(node):
    # A target from before an outage must not be written by the first tick after the link returns.
    connected(node)
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    assert node._pending is not None
    node._disconnect()
    assert node._pending is None


def test_a_hand_reporting_the_other_side_is_refused_and_latched(node, monkeypatch):
    # The device states its own handedness, so this is a direct check rather than the inference
    # the USB driver had to make from limit registers -- which could not see a mirrored correction
    # at all. Latched, because retrying cannot turn a left hand into a right one.
    fake_sdk(monkeypatch, fake_hand(handedness="left"))

    assert node._connect() is False
    assert node._fatal_reason is not None
    assert "left" in node._fatal_reason


def test_a_hand_with_joints_offline_is_refused(node, monkeypatch):
    # Every sweep and every held pose assumes all twenty joints are there to hold.
    fake_sdk(monkeypatch, fake_hand(online=19))

    assert node._connect() is False
    assert node._fatal_reason is not None
    assert "19" in node._fatal_reason


def test_a_stream_that_stops_delivering_drops_the_connection(node):
    # The detector the USB driver could not have: there, steady state came from a cache with no
    # timeout, so a link that died without raising left /joint_states frozen and connected true.
    connected(node)
    node._publish_state()
    assert node._pub_connected.publish.call_args[0][0].data is True

    node._state = FakeStreamHandle(fake_frame(), age=node._link_timeout + 1.0)
    node._publish_state()
    assert node._pub_connected.publish.call_args[0][0].data is False
    assert node._hand is None, "a dead link must release the hand, not keep reporting it"


def test_a_long_idle_releases_the_motors(node, monkeypatch):
    # The watchdog holding the last safe target is right for a momentary gap. Left alone it clamps
    # the hand in whatever pose a rollout finished on, energized, until someone kills the process.
    hand = connected(node)
    node._idle_release = 1.0
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)
    assert node._energized is True
    node._pub.reset()

    later = time.monotonic() + 30.0
    monkeypatch.setattr(hand_node.time, "monotonic", lambda: later)
    node._tick()

    hand.disable.assert_called_once()
    assert node._energized is False
    assert not node._pub.sent, "a released hand must not still be receiving setpoints"


def test_a_command_after_a_release_re_enables_and_reseeds(node, monkeypatch):
    # Re-seeding matters more here than at connect: the hand has been limp, so it is wherever
    # gravity left it, and the pre-release target is stale by definition.
    hand = connected(node)
    node._idle_release = 1.0
    node._last_command_at = time.monotonic()
    later = time.monotonic() + 30.0
    monkeypatch.setattr(hand_node.time, "monotonic", lambda: later)
    node._tick()
    assert node._energized is False

    # The hand settled somewhere new while it was limp.
    node._state = FakeStreamHandle(fake_frame(np.full(NUM_JOINTS, 0.42)))
    node._on_command(JointState(position=[0.42] * NUM_JOINTS))
    node._tick()

    assert node._energized is True
    assert hand.enable.call_count >= 1
    np.testing.assert_allclose(node._chain.last_safe, np.full(NUM_JOINTS, 0.42), atol=1e-9)


def test_idle_release_can_be_switched_off(node, monkeypatch):
    # Zero keeps the old behaviour, for anyone who wants the pose held indefinitely.
    hand = connected(node)
    node._idle_release = 0.0
    node._on_command(JointState(position=[0.1] * NUM_JOINTS))
    tick_one_nominal_period(node, monkeypatch)

    later = time.monotonic() + 300.0
    monkeypatch.setattr(hand_node.time, "monotonic", lambda: later)
    node._tick()

    hand.disable.assert_not_called()
    assert node._energized is True
