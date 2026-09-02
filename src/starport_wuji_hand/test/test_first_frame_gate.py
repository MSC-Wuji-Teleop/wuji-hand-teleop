"""The shared first-frame gate: the decision, then the holder around it.

``gate_decision`` needs no ROS graph, so every combination it can be asked about is cheap to
reach. The holder needs a node, a publisher and a clock, all of which are stood in for here --
spinning a real graph would make discovery a matter of timing, and what has to be pinned is that
the latch and the two log lines behave, not that DDS matches.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from starport_wuji_hand.first_frame_gate import (
    DEFAULT_WAIT_FOR_SUBSCRIBER_S,
    WAIT_PARAM,
    FirstFrameGate,
    GateDecision,
    declare_and_validate,
    gate_decision,
)


class FakeClock:
    """A clock the test moves by hand, so waiting out a timeout costs no real time."""

    def __init__(self, ns: int = 0) -> None:
        self.ns = ns

    def advance(self, seconds: float) -> None:
        self.ns += int(seconds * 1e9)

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(nanoseconds=self.ns)


@pytest.fixture
def gate():
    """A gate over a fake node, its clock and its logger, with no subscribers and a 2 s wait."""
    clock = FakeClock()
    logger = MagicMock()
    publisher = MagicMock()
    publisher.get_subscription_count.return_value = 0
    node = SimpleNamespace(get_clock=lambda: clock, get_logger=lambda: logger)
    return SimpleNamespace(
        gate=FirstFrameGate(node, publisher, "/wuji_hand/joint_command", 2.0),
        clock=clock,
        logger=logger,
        publisher=publisher,
    )


# ----------------------------- the decision -----------------------------


@pytest.mark.parametrize("count", [1, 2, 50])
def test_a_matched_subscriber_opens_the_gate_at_any_elapsed_time(count):
    assert gate_decision(count, elapsed_s=0.0, wait_s=2.0) is GateDecision.MATCHED
    assert gate_decision(count, elapsed_s=99.0, wait_s=2.0) is GateDecision.MATCHED


def test_a_match_beats_a_zero_wait_so_the_open_is_reported_as_a_match():
    # Zero disables the wait, but a subscriber that is already there is still the reason the frame
    # goes out -- and the difference is an info line rather than a warning about an empty topic.
    assert gate_decision(1, elapsed_s=0.0, wait_s=0.0) is GateDecision.MATCHED


def test_no_subscriber_inside_the_wait_holds_the_frame():
    assert gate_decision(0, elapsed_s=0.0, wait_s=2.0) is GateDecision.WAIT
    assert gate_decision(0, elapsed_s=1.999, wait_s=2.0) is GateDecision.WAIT


def test_no_subscriber_at_or_past_the_wait_expires():
    # At exactly the wait, not only past it: the boundary is where a run that has been held for its
    # whole budget must be allowed to proceed.
    assert gate_decision(0, elapsed_s=2.0, wait_s=2.0) is GateDecision.EXPIRED
    assert gate_decision(0, elapsed_s=2.001, wait_s=2.0) is GateDecision.EXPIRED


def test_a_zero_wait_expires_immediately_rather_than_holding_a_frame_forever():
    assert gate_decision(0, elapsed_s=0.0, wait_s=0.0) is GateDecision.EXPIRED


# ----------------------------- the holder -----------------------------


def test_the_gate_holds_the_frame_and_says_nothing_while_it_waits(gate):
    for _ in range(5):
        assert gate.gate.ready() is False
    assert gate.logger.info.call_count == 0
    assert gate.logger.warning.call_count == 0


def test_a_match_opens_the_gate_and_reports_it_once(gate):
    gate.publisher.get_subscription_count.return_value = 1
    assert gate.gate.ready() is True
    for _ in range(5):
        assert gate.gate.ready() is True
    assert gate.logger.info.call_count == 1
    assert gate.logger.warning.call_count == 0
    assert "matched /wuji_hand/joint_command" in gate.logger.info.call_args[0][0]


def test_an_expired_wait_opens_the_gate_and_warns_once(gate):
    gate.clock.advance(1.9)
    assert gate.gate.ready() is False
    gate.clock.advance(0.2)
    for _ in range(5):
        assert gate.gate.ready() is True
    assert gate.logger.warning.call_count == 1
    assert "nothing is subscribed" in gate.logger.warning.call_args[0][0]


def test_the_latch_holds_when_the_subscriber_goes_away_mid_run(gate):
    # A run already going does not stop because a recorder was closed, and the match is not
    # re-reported when the subscriber comes back.
    gate.publisher.get_subscription_count.return_value = 1
    assert gate.gate.ready() is True
    gate.publisher.get_subscription_count.return_value = 0
    assert gate.gate.ready() is True
    assert gate.logger.info.call_count == 1


def test_the_elapsed_time_is_measured_from_construction(gate):
    # Not from the first poll: the wait covers discovery, which begins when the publisher exists.
    gate.clock.advance(2.5)
    assert gate.gate.ready() is True
    assert gate.logger.warning.call_count == 1


# ----------------------------- the parameter -----------------------------


class FakeNode:
    """Just enough node to declare and read one parameter."""

    def __init__(self, value=None) -> None:
        self.declared = {}
        self._override = value

    def declare_parameter(self, name, default):
        self.declared[name] = default

    def get_parameter(self, name):
        value = self.declared[name] if self._override is None else self._override
        return SimpleNamespace(value=value)


def test_the_wait_parameter_is_declared_with_the_shared_default():
    node = FakeNode()
    assert declare_and_validate(node) == DEFAULT_WAIT_FOR_SUBSCRIBER_S
    assert node.declared == {WAIT_PARAM: DEFAULT_WAIT_FOR_SUBSCRIBER_S}


@pytest.mark.parametrize("value", [0.0, 0.5, 30.0])
def test_a_usable_wait_is_returned_as_given(value):
    assert declare_and_validate(FakeNode(value)) == value


@pytest.mark.parametrize("value", [-0.5, float("nan"), float("inf"), float("-inf")])
def test_an_unusable_wait_is_refused_rather_than_coerced(value):
    # There is deliberately no wait-forever setting: an infinite wait would hold a bench run's
    # first frame indefinitely with nothing to say why.
    with pytest.raises(ValueError, match=f"{WAIT_PARAM} must be finite and non-negative"):
        declare_and_validate(FakeNode(value))
