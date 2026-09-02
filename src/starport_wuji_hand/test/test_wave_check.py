"""The curl sequence is pure data, so the thing that first moves real fingers is testable.

This is the check that finds sign flips and zero offsets, so its shape matters: exactly one
finger moves at a time, and every finger returns to zero before the next starts.
"""

from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest
from starport_wuji_hand.hand_node import WujiHandNode
from starport_wuji_hand.joint_map import NUM_JOINTS
from starport_wuji_hand.wave_check import DEFAULT_AMPLITUDE, DEFAULT_STEPS, WaveCheck, curl_sequence

LIMITS_YAML = str(Path(__file__).resolve().parents[1] / "config" / "joint_limits_hand2_beta1_right.yaml")

# The wiring tests drive the timer callback and read the node's own counters, as
# test_replay_clip.py does: spinning a graph would make discovery a matter of timing.
# ruff: noqa: SLF001


def test_sequence_moves_exactly_one_finger_at_a_time():
    for label, target in curl_sequence(amplitude=0.5, steps=5):
        assert target.shape == (NUM_JOINTS,)
        moving = np.flatnonzero(np.abs(target) > 1e-9)
        if moving.size == 0:
            continue
        fingers = {int(i) // 4 for i in moving}
        assert len(fingers) == 1, (label, sorted(fingers))


def test_every_finger_is_exercised():
    fingers = set()
    for _, target in curl_sequence(amplitude=0.5, steps=5):
        for i in np.flatnonzero(np.abs(target) > 1e-9):
            fingers.add(int(i) // 4)
    assert fingers == {0, 1, 2, 3, 4}


def test_sequence_starts_and_ends_at_zero():
    frames = curl_sequence(amplitude=0.5, steps=5)
    np.testing.assert_allclose(frames[0][1], np.zeros(NUM_JOINTS))
    np.testing.assert_allclose(frames[-1][1], np.zeros(NUM_JOINTS))


def test_each_finger_returns_to_zero_before_the_next_moves():
    seen_fingers = []
    for _, target in curl_sequence(amplitude=0.5, steps=5):
        moving = np.flatnonzero(np.abs(target) > 1e-9)
        current = {int(i) // 4 for i in moving}
        if current:
            f = current.pop()
            # A finger may not reappear after a later finger has already moved.
            assert f not in seen_fingers[:-1] or (seen_fingers and seen_fingers[-1] == f)
            if not seen_fingers or seen_fingers[-1] != f:
                seen_fingers.append(f)
    assert seen_fingers == [0, 1, 2, 3, 4]


def test_amplitude_is_respected_and_never_exceeded():
    amplitude = 0.4
    for _, target in curl_sequence(amplitude=amplitude, steps=7):
        assert np.max(np.abs(target)) <= amplitude + 1e-9


def test_labels_name_the_finger_being_moved():
    labels = {label for label, _ in curl_sequence(amplitude=0.5, steps=3)}
    assert any("finger1" in label for label in labels)
    assert any("finger5" in label for label in labels)


# ----------------------------- refused inputs -----------------------------
# A wave that publishes only zero poses and then reports success is the worst failure this tool
# has: it is indistinguishable from a hand that does not respond, and it sends the operator to
# check wiring. Every input that would produce one is refused instead.


@pytest.mark.parametrize("steps", [0, -1, -5])
def test_curl_sequence_refuses_non_positive_steps(steps):
    # steps=0 divides by zero, but a NEGATIVE steps is the quiet one: range(steps + 1) is empty,
    # so the wave would silently collapse to its two home frames.
    with pytest.raises(ValueError, match="steps must be positive"):
        curl_sequence(amplitude=0.5, steps=steps)


@pytest.mark.parametrize("amplitude", [0.0, float("nan"), float("inf"), float("-inf")])
def test_curl_sequence_refuses_an_unusable_amplitude(amplitude):
    with pytest.raises(ValueError, match="amplitude must be finite and non-zero"):
        curl_sequence(amplitude=amplitude, steps=5)


@pytest.mark.parametrize("frame_rate", ["0.0", "-5.0", "nan", "inf"])
def test_wave_check_refuses_an_unusable_frame_rate(frame_rate):
    with pytest.raises(ValueError, match="frame_rate must be finite and positive"):
        WaveCheck(cli_args=["--ros-args", "-p", f"frame_rate:={frame_rate}"])


# ----------------------------- guard coupling -----------------------------


def test_the_default_wave_step_is_half_a_nominal_ticks_budget():
    """Pin the default wave's step against ONE NOMINAL TICK of the driver's travel budget.

    Both numbers are defaults in two different files, and nothing else compares them.

    Half a nominal tick is a ratio, not a guarantee. The driver's budget scales with its MEASURED
    tick gap and has no floor, so this buys headroom for gaps down to half nominal and no further:
    below that the limiter engages on the default wave, and that is scheduler jitter rather than
    evidence about ``max_joint_velocity``. wave_check.DEFAULT_STEPS says why no ratio fixes that.
    """
    node = None
    try:
        node = WujiHandNode(cli_args=["--ros-args", "-p", f"limits_file:={LIMITS_YAML}"])
        max_velocity = float(node.get_parameter("max_joint_velocity").value)
        command_rate = float(node.get_parameter("command_rate").value)
    finally:
        if node is not None:
            node.destroy_node()

    # Commands arrive slower than the driver ticks, so a whole frame's step is spent against a
    # single tick's budget rather than being spread over the interval.
    nominal_budget = max_velocity / command_rate
    frames = curl_sequence(amplitude=DEFAULT_AMPLITUDE, steps=DEFAULT_STEPS)
    step = max(float(np.max(np.abs(frames[i + 1][1] - frames[i][1]))) for i in range(len(frames) - 1))
    assert step <= 0.5 * nominal_budget + 1e-9, f"step {step} exceeds half of the {nominal_budget} nominal budget"


# ----------------------------- the subscriber gate -----------------------------
# What the gate DOES is proven once, in test_first_frame_gate.py, against the shared module both
# bring-up tools now hold. What is left to pin here is the wiring: that this tool consults its
# gate before publishing, and that the gate it consults watches the topic this tool publishes on.


@pytest.fixture
def make_wave():
    """Build wave nodes with parameter overrides, cleaned up at the end of the test."""
    created = []

    def factory(**overrides):
        args = ["--ros-args"]
        for name, value in overrides.items():
            args += ["-p", f"{name}:={value}"]
        node = WaveCheck(cli_args=args)
        created.append(node)
        node._pub = MagicMock()
        node._pub.get_subscription_count.return_value = 1
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


@pytest.mark.parametrize("wait", ["-0.5", "nan", "inf"])
def test_wave_check_refuses_an_unusable_wait(wait):
    # The refusal is the gate module's, and this is what pins that the wave asks for it.
    with pytest.raises(ValueError, match="wait_for_subscriber_s must be finite and non-negative"):
        WaveCheck(cli_args=["--ros-args", "-p", f"wait_for_subscriber_s:={wait}"])


def test_the_wave_publishes_nothing_until_its_gate_opens(make_wave, monkeypatch):
    node = make_wave(steps="3")
    monkeypatch.setattr(node._gate, "ready", lambda: False)
    for _ in range(3):
        node._publish_next()
    assert node._pub.publish.call_count == 0
    assert node._i == 0

    monkeypatch.setattr(node._gate, "ready", lambda: True)
    node._publish_next()
    # The wave starts at its own first frame, not three frames into the sequence.
    assert node._pub.publish.call_count == 1
    np.testing.assert_allclose(node._pub.publish.call_args[0][0].position, np.zeros(NUM_JOINTS))


def test_the_wave_gates_its_own_topic_with_the_wait_it_was_given():
    # Three wires, each of which fails silently. A gate built on a default topic rather than the
    # parameter watches a topic nobody is commanded from and reports another tool's match; one built
    # on the module default rather than the validated value checks the operator's number and then
    # ignores it, holding the first frame for a length nobody asked for.
    node = WaveCheck(
        cli_args=["--ros-args", "-p", "command_topic:=/elsewhere/joint_command", "-p", "wait_for_subscriber_s:=0.25"]
    )
    try:
        assert node._gate._publisher is node._pub
        assert node._gate._topic == "/elsewhere/joint_command"
        assert node._gate._wait_s == 0.25
    finally:
        node.destroy_node()
