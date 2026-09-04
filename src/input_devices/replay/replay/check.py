"""Rules for the replay connection check. Pure Python, no ROS.

``replay_check`` (replay/replay_check.py) is what ``scripts/replay.sh --check``
runs in place of the publisher: the device nodes are started and, before any
clip is played, the operator wants to see that every one of them reports
state. The node subscribes and forwards arrivals here; this module holds what
must report and how the result is printed, so both are tested without ROS.

Sources (docs/spec/spec1.md "Topics"; the table in docs/replay.md section 2):

    arm side   /{side}_arm/joint_states      G1 node state, 250 Hz on the rig
    hand side  /joint_states                 hand driver state, one 20-name
                                             message per driver at 100 Hz;
                                             counts for a side only when 20
                                             distinct names with that side's
                                             prefix (l_ / r_) are present
               /{side}/wuji_hand/connected   std_msgs/Bool; must have been true
                                             at least once. The driver reports
                                             false again once its idle release
                                             (5 s without commands, always the
                                             case during a check) drops the
                                             motors, so the current value is not
                                             the test.

A check is complete when every required source has reported. It times out
after ``timeout_s`` (default DEFAULT_TIMEOUT_S) with the sources that have not
reported marked in the table. A rate is messages-1 over the span between the
first and the last arrival, so a source that produced one message has no rate
yet ("1 msg").

Table, as in docs/replay.md (topic padded to 30, status to 11, then a note):

    /left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose
    /right_arm/joint_states       ~250 Hz
    /joint_states                 ~100 Hz    both hands, 40 names (l_*, r_*)
    /left/wuji_hand/connected     true
    /right/wuji_hand/connected    true

The /joint_states row stands for every selected hand side at once (one topic,
one row). Its rate is the slowest selected side's, since each driver publishes
its own message; its note names the sides it covers. A row that has not
reported shows ``missing`` (or ``false`` for a flag that was never true) and
says in the note what did not arrive in the time waited.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from replay.clip import HAND_JOINTS_PER_SIDE, HAND_NAME_PREFIX, SIDES, parse_sides

# Two-hand UDP scan is ~16 s on the rig, then the driver homes for 3 s
# inside the connect callback and publishes no ROS state until that sweep
# ends. 30 s matches replay_publisher's --ready-timeout so a check that
# passes is the same wait a clip run uses.
DEFAULT_TIMEOUT_S = 30.0

# Topic patterns. The arm pattern is what g1_world_output publishes its
# measured state on; the hand patterns follow starport hand.launch.py, which
# names each driver node wuji_hand in the /{side} namespace and publishes
# measured joints on the global /joint_states.
ARM_STATE_TOPIC = "/{side}_arm/joint_states"
HAND_STATE_TOPIC = "/joint_states"
HAND_CONNECTED_TOPIC = "/{side}/wuji_hand/connected"

# Source kinds. One Source per (kind, side); the /joint_states row merges the
# HAND_STATE sources of every selected side.
ARM_STATE = "arm_state"
HAND_STATE = "hand_state"
HAND_CONNECTED = "hand_connected"

# Column widths from the docs/replay.md table: the longest topic
# (/right/wuji_hand/connected, 26 characters) plus four spaces, and the
# widest status ("~250 Hz", 7 characters) plus four spaces.
TOPIC_COLUMN_WIDTH = 30
STATUS_COLUMN_WIDTH = 11

# Status words for a source that has not reported.
MISSING = "missing"
NEVER_TRUE = "false"

# Notes, from the docs/replay.md table. The arm note sits on the first arm row
# only; the hand-state note says which sides the /joint_states row covers.
ARM_NOTE = "G1 node writing, arms holding measured pose"
HAND_STATE_NOTE = {
    ("left", "right"): "both hands, 40 names (l_*, r_*)",
    ("left",): "left hand, 20 names (l_*)",
    ("right",): "right hand, 20 names (r_*)",
}

# Rates at or above this are printed as whole hertz; below it one decimal
# keeps a slow source from reading as "~0 Hz".
WHOLE_HZ_ABOVE = 10.0


def source_key(kind: str, side: str) -> str:
    """The RateCounter key for one (kind, side)."""
    return f"{kind}:{side}"


@dataclass(frozen=True)
class Source:
    """One thing that must report: a topic, for one side."""

    kind: str
    side: str
    topic: str

    @property
    def key(self) -> str:
        return source_key(self.kind, self.side)


@dataclass(frozen=True)
class Verdict:
    """What the check knows at one instant."""

    elapsed_s: float
    timed_out: bool
    reported: tuple[Source, ...]
    missing: tuple[Source, ...]
    lines: tuple[str, ...]

    @property
    def complete(self) -> bool:
        return not self.missing

    @property
    def table(self) -> str:
        return "\n".join(self.lines)


def required_sources(arms: str, hands: str) -> tuple[Source, ...]:
    """The sources an --arms / --hands selection must hear from, in table order.

    Raises ValueError when both are ``none``: there is nothing to check.
    """
    arm_sides = parse_sides(arms)
    hand_sides = parse_sides(hands)
    if not arm_sides and not hand_sides:
        raise ValueError("nothing to check: --arms none and --hands none together")
    sources = [Source(ARM_STATE, side, ARM_STATE_TOPIC.format(side=side)) for side in arm_sides]
    sources += [Source(HAND_STATE, side, HAND_STATE_TOPIC) for side in hand_sides]
    sources += [Source(HAND_CONNECTED, side, HAND_CONNECTED_TOPIC.format(side=side)) for side in hand_sides]
    return tuple(sources)


def hand_sides_in(names: Iterable[str]) -> tuple[str, ...]:
    """The sides whose 20 hand names a /joint_states message carries.

    A side counts when at least HAND_JOINTS_PER_SIDE distinct names start with
    its prefix. The driver publishes exactly 20 per side; a combined publisher
    carrying both hands counts for both.
    """
    names = list(names)
    out = []
    for side in SIDES:
        prefix = HAND_NAME_PREFIX[side]
        if len({n for n in names if n.startswith(prefix)}) >= HAND_JOINTS_PER_SIDE:
            out.append(side)
    return tuple(out)


class RateCounter:
    """Arrival count and first/last arrival time per source key."""

    def __init__(self) -> None:
        self._count: dict[str, int] = {}
        self._first: dict[str, float] = {}
        self._last: dict[str, float] = {}

    def record(self, key: str, t: float) -> None:
        t = float(t)
        if key not in self._count:
            self._count[key] = 0
            self._first[key] = t
        self._count[key] += 1
        self._last[key] = t

    def count(self, key: str) -> int:
        return self._count.get(key, 0)

    def rate(self, key: str) -> Optional[float]:
        """Messages-1 over the observed span, in Hz. None with fewer than two messages."""
        n = self.count(key)
        if n < 2:
            return None
        span = self._last[key] - self._first[key]
        if span <= 0.0:
            return None
        return (n - 1) / span


def format_rate(rate: Optional[float]) -> str:
    """"~250 Hz" style; "1 msg" for a source with one arrival and no rate yet."""
    if rate is None:
        return "1 msg"
    if rate >= WHOLE_HZ_ABOVE:
        return f"~{rate:.0f} Hz"
    return f"~{rate:.1f} Hz"


def format_row(topic: str, status: str, note: str = "") -> str:
    """One table line: topic and status left-justified to their columns, note, no trailing blanks."""
    return f"{topic:<{TOPIC_COLUMN_WIDTH}}{status:<{STATUS_COLUMN_WIDTH}}{note}".rstrip()


class ConnectionCheck:
    """State of one check: what arrived, from which source, when."""

    def __init__(self, arms: str, hands: str, timeout_s: float = DEFAULT_TIMEOUT_S, start_s: float = 0.0):
        if not (timeout_s > 0.0):
            raise ValueError(f"timeout must be > 0 s, got {timeout_s}")
        self.arm_sides = parse_sides(arms)
        self.hand_sides = parse_sides(hands)
        self.sources = required_sources(arms, hands)
        self.timeout_s = float(timeout_s)
        self.start_s = float(start_s)
        self._rates = RateCounter()
        self._connected_seen: set[str] = set()
        self._connected_true: set[str] = set()
        self._joint_states_messages = 0

    # --- recording -------------------------------------------------------

    def record_arm_state(self, side: str, t: float) -> None:
        self._rates.record(source_key(ARM_STATE, side), t)

    def record_joint_states(self, names: Iterable[str], t: float) -> tuple[str, ...]:
        """Count a /joint_states message for every selected side whose 20 names it carries."""
        self._joint_states_messages += 1
        sides = tuple(s for s in hand_sides_in(names) if s in self.hand_sides)
        for side in sides:
            self._rates.record(source_key(HAND_STATE, side), t)
        return sides

    def record_hand_connected(self, side: str, value: bool, t: float) -> None:
        self._connected_seen.add(side)
        if value:
            self._connected_true.add(side)
        self._rates.record(source_key(HAND_CONNECTED, side), t)

    # --- reading ---------------------------------------------------------

    def rate(self, source: Source) -> Optional[float]:
        return self._rates.rate(source.key)

    def reported(self, source: Source) -> bool:
        if source.kind == HAND_CONNECTED:
            return source.side in self._connected_true
        return self._rates.count(source.key) > 0

    def verdict(self, now: float) -> Verdict:
        """Which sources have reported at time ``now``, and the table that says so."""
        elapsed = float(now) - self.start_s
        reported = tuple(s for s in self.sources if self.reported(s))
        missing = tuple(s for s in self.sources if not self.reported(s))
        lines = self._arm_lines(elapsed) + self._hand_state_lines(elapsed) + self._connected_lines(elapsed)
        return Verdict(
            elapsed_s=elapsed,
            timed_out=elapsed >= self.timeout_s,
            reported=reported,
            missing=missing,
            lines=tuple(lines),
        )

    # --- table -----------------------------------------------------------

    def _arm_lines(self, elapsed: float) -> list[str]:
        lines = []
        for i, source in enumerate(s for s in self.sources if s.kind == ARM_STATE):
            if self.reported(source):
                lines.append(format_row(source.topic, format_rate(self.rate(source)), ARM_NOTE if i == 0 else ""))
            else:
                lines.append(format_row(source.topic, MISSING, f"no message in {elapsed:.1f} s"))
        return lines

    def _hand_state_lines(self, elapsed: float) -> list[str]:
        sources = [s for s in self.sources if s.kind == HAND_STATE]
        if not sources:
            return []
        unreported = [s for s in sources if not self.reported(s)]
        if not unreported:
            rates = [self.rate(s) for s in sources]
            rate = None if any(r is None for r in rates) else min(rates)
            return [format_row(HAND_STATE_TOPIC, format_rate(rate), HAND_STATE_NOTE[self.hand_sides])]
        if self._joint_states_messages == 0:
            note = f"no message in {elapsed:.1f} s"
        else:
            prefixes = ", ".join(HAND_NAME_PREFIX[s.side] + "*" for s in unreported)
            note = f"no {prefixes} names in {elapsed:.1f} s"
        return [format_row(HAND_STATE_TOPIC, MISSING, note)]

    def _connected_lines(self, elapsed: float) -> list[str]:
        lines = []
        for source in (s for s in self.sources if s.kind == HAND_CONNECTED):
            if self.reported(source):
                lines.append(format_row(source.topic, "true"))
            elif source.side in self._connected_seen:
                lines.append(format_row(source.topic, NEVER_TRUE, f"never true in {elapsed:.1f} s"))
            else:
                lines.append(format_row(source.topic, MISSING, f"no message in {elapsed:.1f} s"))
        return lines
