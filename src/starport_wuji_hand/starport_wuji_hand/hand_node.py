"""Wuji hand2 (beta1) driver node -- 20-DoF joint-position control over wuji-sdk.

Generic, hardware-only driver in the same spirit as starport_robotiq_gripper: it takes joint
targets in RADIANS and publishes measured state. It carries no semantics -- no grasp poses, no
end-effector routing, no normalization. That belongs in starport_manager, which sits in front of
this node, not in this package.

WHAT IT DOES OWN is safety, because it is the one path every publisher traverses:
  01 finite check   02 position clamp   03 slew-rate limit   04 command watchdog
See safety.py. Below those, set once at connect: a hardware per-joint effort ceiling and the
MIT impedance gains.

TWO FRAMES. Every topic speaks the driver's LOGICAL frame -- the one the joint names, the URDF and
the reference clips are in -- and everything between the command boundary and the SDK is in the
HAND's frame: the limits table, all four guards, the setpoints, the chain's seed and the homing
sweep. `joint_sign` / `joint_offset` are the map, and it is applied at the SDK boundary and nowhere
else, which is what makes the clamp bound the value actually written rather than a pre-image of it.
Four crossings, all of them named in _to_hardware and _to_logical. With the default correction the
two frames are identical.

RATE ARCHITECTURE. The hand is an Ethernet device that pushes joint_states at ~1 kHz and accepts
setpoint frames the SDK ships without waiting for a response, so nothing here blocks the executor.
Command RECEIPT is decoupled from setpoint WRITING: the subscriber stores the latest target and a
fixed-rate timer runs the guard chain and publishes, which is what gives the slew limit and the
watchdog a well-defined dt.

Control is MIT impedance -- the hand holds a commanded position against kp/kd gains rather than
servoing to it -- so a stepped target is a torque impulse. Every motion path here ramps.

Setpoints carry their own VELOCITY, because the damping term is kd*(commanded - measured): a zero
commanded velocity asks a moving joint to hold still while the position term asks it to move, and
the joint pays kd*qd of current for the contradiction. Measured on the bench, supplying it halved
mean tracking error and made the result nearly independent of kd.

Interfaces (generic ROS messages only):
  sub  ~/joint_command            sensor_msgs/JointState     targets, radians; named or bare 20
  pub  /joint_states              sensor_msgs/JointState     measured positions (rad) + effort,
                                                             and a DERIVED velocity (see below)
  pub  ~/commanded_joint_states   sensor_msgs/JointState     POST-guard target (RViz ghost)
  pub  ~/connected                std_msgs/Bool              live, enabled link
  pub  ~/diagnostics              diagnostic_msgs/DiagnosticArray  health + guard activity

CONTROL MODES. The hand is position-controlled only. `effort` in feedback is an actuation value
in current space, filtered before output -- relative drive intensity, not measured current, and
useful for load monitoring and collision detection. There is no velocity or true force mode.
Documented here intentionally; not implemented.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .joint_map import HAND_SIDES, NUM_JOINTS, index_of, joint_names, nid_to_index, resolve_command
from .limits_io import load_friction, load_limits_mapping
from .safety import ConfigurationFault, GuardChain, GuardReport, Limits

# Ceiling on the dt handed to the guard chain, as a multiple of the nominal tick period. The chain
# sizes the per-tick travel budget as max_velocity * dt, so an unclamped measured gap turns any
# stall -- the homing sweep, a GC pause, a blocked setpoint write, a suspended process -- into a
# proportionally larger jump on the very next tick. That jump would still land inside the soft
# limits, so the worst case is a fast move within the legal envelope rather than an out-of-range
# one, but on a real hand it is a lurch nobody asked for. Measuring the gap is what keeps the slew
# limit honest when the timer runs slow; capping it is what keeps a stall from buying travel.
#
# There is deliberately no FLOOR on the measured gap. rcl timers keep a fixed schedule and skip
# missed periods, so a late tick is systematically followed by a short one: on an instrumented
# 100 Hz timer with two 25 ms stalls, 49 of 100 ticks measured a sub-nominal gap. Granting each of
# those a whole nominal period of travel would authorise several times the configured rad/s
# instantaneously; scaling with the measured gap is what ties the budget to elapsed time. A
# publisher slower than command_rate therefore spends each frame's step against one tick's budget
# -- see wave_check.DEFAULT_STEPS, which is sized for exactly that.
_MAX_TICK_FACTOR = 3.0

# ext_state in a joint's status word: 0=Init 1=Ready 2=Enabled 3=Stopped.
_ENABLED = 2


def _sdk_value(obj: Any, name: str) -> Any:
    """Read ``name`` off an SDK object, whatever shape it takes.

    wuji_sdk mixes plain properties, zero-argument methods and resources returning a future. A
    call site that guesses wrong gets a bound-method repr rather than an error, so the guess is
    made once, here.
    """
    value = getattr(obj, name)
    if callable(value):
        value = value()
    if hasattr(value, "get") and not isinstance(value, (str, int, float, bool, list, tuple, dict)):
        value = value.get()
    return value


class _Stream:
    """Newest frame from an SDK stream, with the age that makes staleness measurable.

    The subscription handle must outlive the reader: dropping it tears the stream down silently,
    which presents as "no frames" rather than as an error.
    """

    def __init__(self, stream) -> None:
        self._frame = None
        self._stamp = None
        self._sub = stream.subscribe_with_callback(self._on_frame)

    def _on_frame(self, frame) -> None:
        self._frame = frame
        self._stamp = time.monotonic()

    def get(self):
        return self._frame

    def age(self) -> float | None:
        return None if self._stamp is None else time.monotonic() - self._stamp

    def close(self) -> None:
        try:
            self._sub.close()
        except Exception:
            pass


def _require_positive(name: str, value: Any) -> float:
    """Return ``value`` as a float, refusing anything non-finite or non-positive."""
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be finite and positive, got {number}")
    return number


def _require_non_negative(name: str, value: Any) -> float:
    """Return ``value`` as a float, refusing anything non-finite or negative."""
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative, got {number}")
    return number


def _require_joint_array(name: str, value: Any) -> np.ndarray:
    """Return ``value`` as a finite ``(20,)`` float array, refusing any other length."""
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.shape != (NUM_JOINTS,):
        raise ValueError(f"{name} must carry exactly {NUM_JOINTS} values, got {array.size}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must be finite")
    return array


def _require_signs(name: str, value: Any, names: Sequence[str]) -> np.ndarray:
    """Return a ``(20,)`` array of +1/-1, refusing any other value.

    The two signs and nothing between them, because this parameter states a FRAME relationship --
    which way round a joint is wired -- and not a gear ratio. A reflection maps the declared
    envelope onto another valid envelope (``_mapped_limits`` just re-orders the ends), keeps
    ``max_joint_velocity`` meaning the same rad/s on both sides of the map, and inverts exactly for
    the ghost. An arbitrary scale would do none of those: it would rescale the envelope and the
    reference trajectory with it, silently.
    """
    array = _require_joint_array(name, value)
    bad = np.flatnonzero((array != 1.0) & (array != -1.0))
    if bad.size:
        raise ValueError(f"{name} entries must be +1 or -1, and {[names[i] for i in bad]} are not")
    return array


def _mapped_limits(
    raw: Mapping[str, tuple[float, float]], sign: np.ndarray, offset: np.ndarray
) -> dict[str, tuple[float, float]]:
    """The declared envelope expressed in the hand's own frame.

    Each bound goes through ``sign * bound + offset``, which reflects and shifts the range, so a
    flipped joint's two ends swap over and have to be re-ordered.

    Mapping the TABLE rather than each command is what puts the guards in the frame the hardware is
    driven in -- see this module's frame note -- and it is also what gives ``cross_check`` a view of
    the correction: it compares the hand's reported bounds against the envelope the guards will
    really hold it to, so a correction that MOVES the envelope off what the hand reports is a
    startup refusal naming the joints. One that maps the envelope onto itself is invisible there --
    the README's sign and zero note says which corrections those are.

    A name the hand does not have passes through untouched, leaving ``Limits.from_mapping`` to
    report it as unknown rather than this raising a bare ``KeyError`` first.
    """
    mapped: dict[str, tuple[float, float]] = {}
    for joint, (lower, upper) in raw.items():
        try:
            index = index_of(joint)
        except KeyError:
            mapped[joint] = (lower, upper)
            continue
        ends = (sign[index] * lower + offset[index], sign[index] * upper + offset[index])
        mapped[joint] = (float(min(ends)), float(max(ends)))
    return mapped


class WujiHandNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("wuji_hand", **kwargs)

        self.declare_parameter("serial_number", "")
        self.declare_parameter("hand_side", "right")
        # Amps; the device default is 1.5. Measured down from 1.0: at 1.0 A a clip replay drives
        # this hand into an audible limit cycle partway through the motion, which then sustains
        # even after the setpoint stops moving; at 0.6 A the same replay is silent. Established by
        # alternating ONLY this value four times in one session with kp fixed -- kp 10 at 0.6 A is
        # silent and kp 9 at 1.0 A is not, so the ceiling owns it and the gains do not. The cycle
        # needs exciting before it sustains: reaching the same pose gently is quiet at either
        # value. Free-air tracking is unchanged (mean 0.0086 rad vs 0.0085 at 1.0 A). A task that
        # must GRIP rather than track may need more, which is what the launch argument is for.
        self.declare_parameter("effort_limit_a", 0.6)
        self.declare_parameter("setpoint_velocity_filter_hz", 10.0)
        self.declare_parameter("friction_file", "")
        self.declare_parameter("friction_scale", 1.0)  # swept on the bench; see the README
        self.declare_parameter("friction_velocity_deadzone", 0.02)  # rad/s
        self.declare_parameter("command_rate", 100.0)
        self.declare_parameter("publish_rate", 100.0)
        # Cutoff for the derived joint velocity. A STARTING POINT, not a measurement: it has to
        # pass the hand's real motion at a 60 Hz consumer while suppressing the quantization
        # step that differencing turns into a spike, and neither of those has been characterised
        # on this hand. Raise it if the velocity lags; lower it if it is hashy at rest.
        self.declare_parameter("measured_velocity_filter_hz", 20.0)
        self.declare_parameter("diagnostics_rate", 10.0)
        self.declare_parameter("max_joint_velocity", 2.0)  # rad/s
        self.declare_parameter("command_timeout", 0.25)  # s
        self.declare_parameter("limits_file", "")
        self.declare_parameter("limit_margin", 0.02)  # rad
        # The bench sign/zero correction, per joint -- see _to_hardware.
        self.declare_parameter("joint_sign", [1.0] * NUM_JOINTS)
        self.declare_parameter("joint_offset", [0.0] * NUM_JOINTS)  # rad
        self.declare_parameter("home_on_start", True)
        self.declare_parameter("home_duration_s", 3.0)
        self.declare_parameter("reconnect_interval", 2.0)
        self.declare_parameter("max_connect_attempts", 10)
        # MIT impedance gains. The vendor documents no values at all for these; the defaults here
        # were measured on a real hand -- see scripts/calibrate_joint_limits.py for the matrix and
        # what each direction cost. They are provisional.
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 0.2)
        # How long the ~1 kHz state stream may go quiet before the link counts as dead.
        self.declare_parameter("link_timeout", 0.5)  # measured, not nominal -- see _link_is_live
        # How long the command stream may be quiet before the motors are released. The watchdog
        # holds the last safe pose at command_timeout, which is right for a momentary gap; holding
        # it indefinitely after a rollout has ended just leaves the hand clamped in whatever pose
        # the clip finished on. Zero keeps the old behaviour of holding forever.
        self.declare_parameter("idle_release_s", 5.0)

        self._serial = self.get_parameter("serial_number").value
        self._side = self.get_parameter("hand_side").value
        # Validated here, not at connect: everything this driver publishes, commands and clamps is
        # one hand's, and `ros2 run` reaches the node without passing a launch file's `choices`.
        if self._side not in HAND_SIDES:
            raise ValueError(f"hand_side must be one of {list(HAND_SIDES)}, got {self._side!r}")
        # Resolved once, so no later code re-derives which hand it is talking about.
        self._joint_names = joint_names(self._side)
        self._home_on_start = bool(self.get_parameter("home_on_start").value)

        # The guard chain validates what it is handed, but the numbers below are operator input and
        # several of them DISABLE a guard rather than failing loudly: a non-positive command_timeout
        # makes the watchdog's staleness test permanently false, a negative limit_margin widens the
        # soft limits past the declared envelope -- the exact inverse of the parameter's purpose --
        # a zero rate divides by zero while building the timers, a zero reconnect_interval scans the
        # network on every tick, and a zero home_duration_s reduces the homing sweep to a single
        # step -- which IS the snap it exists to avoid. Refuse to start instead: a driver running
        # with a silently disabled watchdog looks healthy and is not.
        self._effort_limit = _require_positive("effort_limit_a", self.get_parameter("effort_limit_a").value)
        self._velocity_filter = _require_positive(
            "setpoint_velocity_filter_hz", self.get_parameter("setpoint_velocity_filter_hz").value
        )
        self._friction_scale = float(self.get_parameter("friction_scale").value)
        self._friction_deadzone = _require_positive(
            "friction_velocity_deadzone", self.get_parameter("friction_velocity_deadzone").value
        )
        self._command_rate = _require_positive("command_rate", self.get_parameter("command_rate").value)
        publish_rate = _require_positive("publish_rate", self.get_parameter("publish_rate").value)
        self._velocity_filter_hz = _require_positive(
            "measured_velocity_filter_hz", self.get_parameter("measured_velocity_filter_hz").value
        )
        #: The derived velocity and what it is derived from: (positions, stamp) of the previous
        #: published sample, and the filtered result. None until a second sample exists.
        self._velocity_prev: tuple[np.ndarray, float] | None = None
        self._velocity: np.ndarray = np.zeros(NUM_JOINTS)
        #: A gap longer than this many publish periods is not a velocity, it is a resumption.
        self._velocity_max_gap = 5.0 / publish_rate
        diagnostics_rate = _require_positive("diagnostics_rate", self.get_parameter("diagnostics_rate").value)
        self._max_velocity = _require_positive("max_joint_velocity", self.get_parameter("max_joint_velocity").value)
        self._command_timeout = _require_positive("command_timeout", self.get_parameter("command_timeout").value)
        margin = _require_non_negative("limit_margin", self.get_parameter("limit_margin").value)
        self._joint_sign = _require_signs("joint_sign", self.get_parameter("joint_sign").value, self._joint_names)
        self._joint_offset = _require_joint_array("joint_offset", self.get_parameter("joint_offset").value)
        self._home_duration = _require_positive("home_duration_s", self.get_parameter("home_duration_s").value)
        self._reconnect_interval = _require_positive(
            "reconnect_interval", self.get_parameter("reconnect_interval").value
        )
        # Bounded so that bringing up both hands with only one plugged in reports the absent one
        # and stops, instead of logging the same failure forever with ~/connected false. Zero
        # restores an unbounded wait, for a hand that is expected to appear late.
        self._kp = _require_positive("kp", self.get_parameter("kp").value)
        self._kd = _require_non_negative("kd", self.get_parameter("kd").value)
        self._link_timeout = _require_positive("link_timeout", self.get_parameter("link_timeout").value)
        self._idle_release = _require_non_negative("idle_release_s", self.get_parameter("idle_release_s").value)
        self._max_connect_attempts = int(
            _require_non_negative("max_connect_attempts", self.get_parameter("max_connect_attempts").value)
        )

        limits_file = self.get_parameter("limits_file").value
        if not limits_file:
            raise ValueError(
                "limits_file is required: the guard chain will not run without per-joint limits. "
                "hand.launch.py passes the packaged config/joint_limits_hand2_beta1.yaml."
            )
        # In the hand's own frame, like everything else from the command boundary inward.
        self._limits = Limits.from_mapping(
            _mapped_limits(load_limits_mapping(limits_file), self._joint_sign, self._joint_offset),
            margin=margin,
            names=self._joint_names,
        )
        # Said out loud once every parameter has been accepted, so a node that is about to refuse
        # to start does not first announce a correction it will never apply. A correction changes
        # what a command MEANS, and every trace or ghost read through one has to account for it.
        corrected = np.flatnonzero((self._joint_sign < 0.0) | (self._joint_offset != 0.0))
        if corrected.size:
            self.get_logger().warning(
                f"a sign/zero correction is configured for {[self._joint_names[i] for i in corrected]}; "
                "see the README's sign and zero note for what it does and does not change"
            )
        # Zero until a link exists: there is no hand to read a pose from yet. _connect reseeds it
        # from the measured pose before any setpoint is written -- see there for why that matters.
        self._chain = self._new_chain(np.zeros(NUM_JOINTS))

        # joint_states is GLOBAL so one combined robot_state_publisher can animate the hand
        # alongside anything else on the cell; the rest are node-local.
        self._pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_commanded = self.create_publisher(JointState, "~/commanded_joint_states", 10)
        # READY, not merely linked: false while the motors are released after an idle, because a
        # command sent then is dropped -- re-acquiring takes ~0.7 s, and an open-loop client that
        # keeps streaming through it loses that much of its trajectory silently.
        self._pub_connected = self.create_publisher(Bool, "~/connected", 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, "~/diagnostics", 10)
        self.create_subscription(JointState, "~/joint_command", self._on_command, 10)

        self._hand: Any = None
        self._state: Any = None
        self._diag: Any = None
        self._pub: Any = None
        self._command_type: Any = None
        self._energized = False
        self._pending: np.ndarray | None = None
        self._last_report: GuardReport | None = None
        self._last_command_at: float | None = None
        self._next_attempt = 0.0
        self._connect_attempts = 0
        self._last_tick = time.monotonic()
        # Set from a ConfigurationFault in _connect and never cleared; see that handler.
        self._fatal_reason: str | None = None
        # Guard activity is latched between diagnostics publishes: at 100 Hz most ticks are holds
        # that report no activity at all, so sampling the latest report at 10 Hz would miss almost
        # every clamp. Cleared once published.
        self._sticky_clamped = np.zeros(NUM_JOINTS, dtype=bool)
        self._sticky_rate_limited = np.zeros(NUM_JOINTS, dtype=bool)
        self._last_rejection = ""
        # Smoothed, because the chain's per-tick delta is only as smooth as the publisher: a source
        # slower than the tick rate moves the setpoint on some ticks and not others, and the raw
        # ratio would be a train of spikes several times the true velocity with zeros between them.
        # kd turns those straight into torque impulses.
        self._setpoint_velocity = np.zeros(NUM_JOINTS)
        # Zero unless a measured table is supplied: compensating friction the hand does not have
        # is worse than not compensating at all, and no default is right for both hands.
        friction_file = str(self.get_parameter("friction_file").value)
        self._friction = (
            np.asarray(load_friction(friction_file, self._joint_names), dtype=np.float64)
            if friction_file
            else np.zeros(NUM_JOINTS)
        )
        if friction_file:
            self.get_logger().info(
                f"friction feedforward from {friction_file}: "
                f"{self._friction.min():.3f}-{self._friction.max():.3f} A, scale {self._friction_scale}"
            )

        self.create_timer(1.0 / self._command_rate, self._tick)
        self.create_timer(1.0 / publish_rate, self._publish_state)
        self.create_timer(1.0 / diagnostics_rate, self._publish_diagnostics)
        # Said once, because the message cannot say it: `JointState.velocity` on this topic is
        # DERIVED from the position stream. The hand has no velocity sense, and a consumer reading
        # that field has no way to tell a differenced number from a measured one.
        self.get_logger().info(
            f"joint velocity is DERIVED by differencing positions, low-passed at "
            f"{self._velocity_filter_hz:.1f} Hz -- this hand measures position and current only"
        )

    def _new_chain(self, initial: np.ndarray) -> GuardChain:
        """A guard chain holding ``initial``, built from the already-validated parameters.

        The seed matters: the chain holds it until a command arrives and rate-limits travel FROM
        it, so it has to be where the hand actually is. Built here rather than inline three times,
        and from the stored numbers rather than by re-reading the parameters, which are only
        validated at construction.
        """
        return GuardChain(
            limits=self._limits,
            max_velocity=np.full(NUM_JOINTS, self._max_velocity),
            timeout_s=self._command_timeout,
            initial=initial,
        )

    # --------------------------------------------------------------- sign and zero correction
    def _to_hardware(self, target: np.ndarray) -> np.ndarray:
        """A commanded target in the hand's frame: ``sign * target + offset``.

        The finger-curl check's escape hatch, and the whole of it. That check exists to FIND a
        flipped direction or a non-neutral zero, and with nothing able to absorb a finding the
        remaining choices at the bench would be editing code or abandoning the run.

        Two callers, both of them a LOGICAL quantity crossing into the hand's frame: an arriving
        command, and the homing sweep's target. Why the frames are split that way is this module's
        frame note; ``_require_signs`` owns why the sign is +-1; the README owns what a correction
        costs an operator reading a trace.
        """
        return self._joint_sign * target + self._joint_offset

    def _to_logical(self, value: np.ndarray) -> np.ndarray:
        """The inverse of ``_to_hardware``, exact because every sign is +-1.

        Three callers, each needing a logical quantity from a hardware one: the two published
        topics, which carry URDF joint names and have to agree with each other for the RViz
        comparison to mean anything, and the hold base a partial command resolves against --
        ``resolve_command`` fills unlisted joints from it, so it has to be in the same frame as the
        values arriving beside them.
        """
        return (value - self._joint_offset) * self._joint_sign

    # --------------------------------------------------------------- connection
    @staticmethod
    def _usable_position(reported: Any) -> np.ndarray:
        """One position read as a finite ``(20,)`` vector, exactly as reported.

        Not clipped: this is the measurement, and a real pose may legitimately sit just outside the
        MARGIN-shrunken soft limits. Callers that need a target rather than a measurement clip it
        themselves. A non-finite read is a link fault, not a pose -- it must never become a guard
        chain seed, a homing start point, or a published joint state.
        """
        measured = np.asarray(reported, dtype=np.float64).reshape(-1)
        if measured.shape != (NUM_JOINTS,) or not np.isfinite(measured).all():
            raise RuntimeError(f"unusable joint position read from the hand: {measured!r}")
        return measured

    def _measured_position(self) -> np.ndarray:
        """The measured pose from the newest streamed frame.

        The hand pushes joint_states at ~1 kHz over its own link, so there is no bus round-trip to
        make here and no timeout to fail: every reader takes the last frame the stream delivered.
        That collapses the checked-read / cached-read split this driver used to need, and with it
        the whole question of which callers could afford a blocking read.

        Frame AGE is what replaces it, and it is a better liveness signal than either. A cache can
        sit stale forever with nothing to notice; a stream that stops delivering is measurable, and
        ``_link_is_live`` is what measures it.
        """
        frame = self._state.get() if self._state is not None else None
        if frame is None or not frame.joints:
            raise RuntimeError("no joint_states frame has arrived from the hand")
        pose = np.full(NUM_JOINTS, np.nan)
        for entry in frame.joints:
            pose[nid_to_index(entry.nid)] = float(entry.position)
        return self._usable_position(pose)

    def _measured_effort(self) -> np.ndarray:
        """Per-joint current from the newest streamed frame, in amps."""
        frame = self._state.get() if self._state is not None else None
        effort = np.full(NUM_JOINTS, np.nan)
        if frame is not None:
            for entry in frame.joints:
                effort[nid_to_index(entry.nid)] = float(entry.effort)
        return effort

    def _link_is_live(self) -> bool:
        """Whether the state stream is still delivering.

        The hand streams at ~1 kHz, so a gap of ``link_timeout`` seconds is unambiguous rather than
        a judgement call. This is the detector the USB driver could not have: there, steady state
        came from a cache with no timeout, so a link that died without raising left /joint_states
        frozen and ``connected`` true indefinitely.

        THE DEFAULT IS SIZED FROM MEASUREMENT, not from the nominal 1 ms period. The stream stalls
        in bursts on a non-realtime host -- the SDK reported queues up to 46 frames behind during
        clip replay -- and at 0.1 s the driver tore the link down mid-replay twice in one run, each
        time reconnecting into a recording that then captured nothing. At 0.5 s: zero link-down
        events across ten replays. The cost of the wider window is bounded and small: the guard
        chain keeps slewing from its own last safe target, so a stale link means commanding from
        the last setpoint for another 0.4 s, not a jump. The cost of the narrower one was a
        silently empty dataset.
        """
        age = self._state.age() if self._state is not None else None
        return age is not None and age <= self._link_timeout

    def _verify_handedness(self, hand: Any) -> None:
        """Refuse a hand that is not the side this driver was configured for.

        The device reports its own handedness, so this is a direct check. The USB driver had to
        infer the same thing by comparing limit registers against the declared envelope, which
        could not see a mirrored correction at all -- a reflection about a joint's range midpoint
        maps the envelope onto itself. Asking the hand removes that blind spot rather than
        documenting it.
        """
        reported = str(_sdk_value(hand, "handedness"))
        if reported != self._side:
            raise ConfigurationFault(
                f"connected hand reports handedness {reported!r} but this driver is configured for "
                f"{self._side!r}; every joint name, limit and clip here is the {self._side} hand"
            )

    def _connect(self) -> bool:
        """Find the hand on the network and bring it up, rate-limited so a late hand is survivable."""
        if self._hand is not None:
            return True
        if self._fatal_reason is not None:
            return False
        now = time.monotonic()
        if now < self._next_attempt:
            return False
        self._next_attempt = now + self._reconnect_interval

        try:
            import wuji_sdk  # noqa: PLC0415  -- lazy, so this module imports with no SDK and no hand
        except ImportError as exc:
            self.get_logger().error(f"wuji_sdk unavailable: {exc}")
            return False

        try:
            manager = wuji_sdk.SdkManager.instance()
            found = [d for d in manager.scan() if d.device_type == wuji_sdk.DeviceType.WujiHand2]
            if self._serial:
                found = [d for d in found if str(d.sn) == self._serial]
            if not found:
                which = f"a hand with serial {self._serial}" if self._serial else "any Wuji Hand 2"
                raise ConnectionError(
                    f"no {which} on the network; the hand uses a static IP, so check the host NIC " "is on its subnet"
                )
            # Owned from the moment it exists: everything below can fail, and _disconnect is the
            # only code that disables the hand, so the handle must be reachable from the failure
            # path before anything is energized.
            self._hand = manager.connect(sn=found[0].sn, device_name=self.get_name())
            # Before any current flows: the hand states its own side, and driving the wrong one
            # would mislabel every topic and clamp against the wrong envelope.
            self._verify_handedness(self._hand)

            online = int(_sdk_value(self._hand, "online_joints_count"))
            if online != NUM_JOINTS:
                raise ConfigurationFault(
                    f"hand reports {online} of {NUM_JOINTS} joints online; a pose cannot be held "
                    "for joints that are not there"
                )

            # State first, so the seed below reads a real pose rather than waiting on one.
            self._state = _Stream(self._hand.joint_states())
            self._diag = _Stream(self._hand.joint_diagnostics())
            self._await_frame()

            self._hand.effort_limit().set(self._effort_limit)
            self._hand.mit_params().set((self._kp, self._kd))
            self._hand.enable()
            if not self._await_enabled():
                raise ConnectionError("not every motor reached Enabled within the timeout")
            self._pub = self._hand.joint_command().publish()
            # Captured once rather than imported per tick: this runs at the command rate.
            self._command_type = wuji_sdk.JointCommand

            # Seed the chain where the hand ACTUALLY is, before any setpoint is written. The chain
            # holds its seed until a command arrives, so seeding it at zero would make the very
            # first tick write the zero pose -- a full-range step on a hand resting flexed. The
            # slew limit cannot catch that: it bounds change in the commanded value, and the
            # commanded value would start at zero, already wrong.
            self._chain = self._new_chain(np.clip(self._measured_position(), self._limits.lower, self._limits.upper))
            self.get_logger().info(
                f"connected ({_sdk_value(self._hand, 'serial_number')}, {self._side}); "
                f"effort_limit={self._effort_limit} A, kp={self._kp}, kd={self._kd}"
            )
            self._energized = True
            # The idle clock starts at connect, not at the first command: a driver brought up and
            # never commanded should not sit holding its seed pose indefinitely either.
            self._last_command_at = time.monotonic()
            self._connect_attempts = 0
            if self._home_on_start:
                self._home()
            # Homing burns seconds inside this callback; do not bill them to the first tick.
            self._last_tick = time.monotonic()
            return True
        except ConfigurationFault as exc:
            # Latched: retrying cannot fix a hand that is the wrong side or missing joints.
            self._fatal_reason = str(exc)
            self.get_logger().error(f"refusing to run: {exc}")
            self._disconnect()
            return False
        except Exception as exc:
            self._connect_attempts += 1
            if 0 < self._max_connect_attempts <= self._connect_attempts:
                self._fatal_reason = f"no {self._side} hand after {self._connect_attempts} attempts: {exc}"
                self.get_logger().error(f"giving up: {self._fatal_reason}")
                self._disconnect()
                return False
            budget = self._max_connect_attempts or "unlimited"
            self.get_logger().error(
                f"connect failed ({self._connect_attempts}/{budget}), "
                f"retrying in {self._reconnect_interval:.0f}s: {exc}"
            )
            self._disconnect()
            return False

    def _await_frame(self, timeout: float = 5.0) -> None:
        """Block until the state stream delivers, so nothing downstream reads an empty cache."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._state.get() if self._state is not None else None
            if frame is not None and frame.joints:
                return
            time.sleep(0.005)
        raise ConnectionError("connected, but no joint_states frame arrived")

    def _await_enabled(self, timeout: float = 5.0) -> bool:
        """Wait for every motor to report Enabled.

        enable() is an action, not a write that has landed by the time it returns; publishing
        setpoints before the motors are ready races the enable.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._diag.get() if self._diag is not None else None
            if frame is not None and frame.joints:
                if all(e.status_word.ext_state == _ENABLED for e in frame.joints):
                    return True
            time.sleep(0.02)
        return False

    def _release(self) -> None:
        """Drop motor current but keep the link, after a long idle.

        The hand goes limp rather than clamping the last pose forever. The link, the streams and
        the publisher all stay up, so the next command re-acquires without a reconnect.
        """
        if not self._energized or self._hand is None:
            return
        try:
            self._hand.disable()
        except Exception as exc:
            self.get_logger().warning(f"idle release failed: {exc}")
            return
        self._energized = False
        # A limp hand is not moving; carrying the old velocity into the re-acquire would put a
        # stale kd*qd on the first setpoint after it.
        self._setpoint_velocity[:] = 0.0
        self.get_logger().info(f"no command for {self._idle_release:.1f}s; motors released")

    def _reacquire(self) -> bool:
        """Re-enable after an idle release, re-seeding from where the hand actually ended up.

        Seeding matters more here than at connect: the hand has been limp, so it is wherever
        gravity left it, and the pre-release target is stale by definition.
        """
        assert self._hand is not None
        try:
            self._hand.enable()
            if not self._await_enabled():
                self.get_logger().error("re-enable timed out")
                return False
            self._chain = self._new_chain(np.clip(self._measured_position(), self._limits.lower, self._limits.upper))
        except Exception as exc:
            self.get_logger().error(f"re-enable failed: {exc}")
            return False
        self._energized = True
        self._last_tick = time.monotonic()
        self.get_logger().info("command arrived; motors re-enabled")
        return True

    def _friction_feedforward(self) -> np.ndarray:
        """Coulomb current opposing the direction the setpoint is travelling.

        Ramped through a velocity deadzone rather than switched on its sign. A hard sign would
        apply the full compensating current at a standstill and flip it across zero crossings --
        a torque step twice the friction, in the place a trajectory spends most of its reversals --
        and it would make a held pose creep.
        """
        if not self._friction.any():
            return np.zeros(NUM_JOINTS)
        ramp = np.clip(self._setpoint_velocity / self._friction_deadzone, -1.0, 1.0)
        return self._friction_scale * self._friction * ramp

    def _publish_pose(
        self,
        pose: np.ndarray,
        velocity: np.ndarray | None = None,
        effort: np.ndarray | None = None,
    ) -> None:
        """Ship one 20-joint setpoint frame.

        Positional: entry i is joint i, the same dense order ``joint_names`` defines and the state
        stream arrives in. A fire-and-forget deposit -- the SDK serialises and sends with no
        response wait, so this does not block the executor thread the timers share.

        ``velocity`` defaults to zero only for callers that genuinely are not moving; a moving
        caller must supply its own, or the damping term will fight it. ``effort`` is a feedforward
        CURRENT in amps, which the device adds to the impedance torque.
        """
        assert self._pub is not None and self._command_type is not None
        qd = np.zeros(NUM_JOINTS) if velocity is None else velocity
        ff = np.zeros(NUM_JOINTS) if effort is None else effort
        self._pub.send([self._command_type(float(q), float(v), float(a)) for q, v, a in zip(pose, qd, ff, strict=True)])

    def _disconnect(self) -> None:
        """DISABLE the hand and release everything, in an order that is safe half-built.

        Disabling in the teardown path is what makes a crashed node fail safe rather than leaving
        a powered hand holding a pose. Every step is individually guarded: shutdown must complete.
        The publisher is dropped BEFORE the disable so nothing is still shipping setpoints into a
        hand that is being switched off.
        """
        self._pub = None
        # The next sample is a first sample: differencing across a disconnection would report the
        # whole gap's travel as one tick's velocity.
        self._velocity_prev = None
        self._velocity = np.zeros(NUM_JOINTS)
        self._command_type = None
        self._energized = False
        self._setpoint_velocity[:] = 0.0
        if self._hand is not None:
            try:
                self._hand.disable()
            except Exception:
                pass
        for stream in (self._state, self._diag):
            if stream is not None:
                stream.close()
        self._state = None
        self._diag = None
        if self._hand is not None:
            try:
                self._hand.disconnect()
            except Exception:
                pass
            self._hand = None
        # Drop any target that arrived while the link was up. On reconnect the first tick would
        # otherwise write a command from before the outage, and the watchdog that would have
        # refused to trust it by age does not police a target already handed to the chain.
        self._pending = None

    def _home(self) -> None:
        """Ease from the current pose to the home pose before accepting commands, so nothing snaps.

        These are the only writes that never pass through the guard chain, so they carry its two
        position guarantees themselves: the start pose is finite (_measured_position raises
        otherwise) and clipped into the soft limits, and so is the pose being swept to -- which
        makes every interpolated frame between them legal as well. Both ends are in the hand's own
        frame, so the sweep needs no mapping and cannot leave the envelope; the home pose is where
        a logical zero lands, which is where the first tick after this will hold.
        """
        assert self._pub is not None
        start = np.clip(self._measured_position(), self._limits.lower, self._limits.upper)
        end = np.clip(self._to_hardware(np.zeros(NUM_JOINTS)), self._limits.lower, self._limits.upper)
        steps = max(1, int(self._home_duration * 50.0))  # 50 Hz smoothing
        dt = self._home_duration / steps
        for i in range(steps):
            t = (i + 1) / steps
            smooth = t * t * (3.0 - 2.0 * t)  # ease in-out
            # The ease's own derivative in real time. The sweep knows exactly how fast it is
            # going, so it says so rather than leaving kd to oppose it.
            rate = 6.0 * t * (1.0 - t) / self._home_duration
            self._publish_pose(start + smooth * (end - start), (end - start) * rate)
            time.sleep(dt)
        # Reseeded where the sweep ended, which is where the hand now is.
        self._chain = self._new_chain(end)

    # --------------------------------------------------------------- command path
    def _on_command(self, msg: JointState) -> None:
        """Store the latest target, in the hand's frame. Writing happens on the tick, not here.

        The only place an arriving command is mapped: resolve in the frame the publisher speaks,
        then cross into the hand's frame once -- see this module's frame note.

        Refusals log and store NOTHING: a rejected command must not become a stale pending target
        that a later tick would honor.
        """
        names = list(msg.name) if len(msg.name) else None
        try:
            target = resolve_command(names, list(msg.position), self._to_logical(self._chain.last_safe))
        except ValueError as exc:
            self.get_logger().warning(f"command refused: {exc}")
            return
        self._pending = self._to_hardware(target)
        self._last_command_at = time.monotonic()

    def _tick(self) -> None:
        """Run the guard chain at a fixed rate and deposit the result as the SDK setpoint."""
        if not self._connect():
            return
        assert self._pub is not None  # _connect reports success only with a live publisher
        now = time.monotonic()

        # Release the motors after a long idle, and take them back when a command returns. The
        # watchdog holding the last safe pose is right for a momentary gap; after a rollout ends
        # it just clamps the hand in the clip's final pose indefinitely.
        idle = now - (self._last_command_at if self._last_command_at is not None else now)
        if self._pending is not None and not self._energized:
            if not self._reacquire():
                return
            now = time.monotonic()
        elif self._pending is None and self._energized and 0.0 < self._idle_release <= idle:
            self._release()
        if not self._energized:
            # Released: nothing is written, so the hand stays limp until a command arrives.
            self._last_tick = now
            return

        # Measured, then capped -- see _MAX_TICK_FACTOR.
        nominal = 1.0 / self._command_rate
        dt = min(now - self._last_tick, _MAX_TICK_FACTOR * nominal)
        self._last_tick = now

        had_target = self._pending is not None
        safe, report = self._chain.apply(self._pending, dt=dt, now=now)
        self._pending = None
        self._last_report = report
        # Latched, not sampled: a clamp on one tick has to survive until diagnostics next publish.
        self._sticky_clamped |= report.clamped
        self._sticky_rate_limited |= report.rate_limited
        # A hold is not a rejection -- only a target that was offered and refused is, and that
        # reason must not be overwritten by the "no command" of the holds that follow it.
        if had_target and not report.accepted:
            self._last_rejection = report.reason

        # One-pole toward the chain's reported velocity. dt is the measured tick, so the filter
        # keeps its time constant when the timer runs slow.
        alpha = 1.0 - math.exp(-2.0 * math.pi * self._velocity_filter * dt)
        self._setpoint_velocity += alpha * (report.velocity - self._setpoint_velocity)

        try:
            # Already in the hand's frame, and already clamped there.
            self._publish_pose(safe, self._setpoint_velocity, self._friction_feedforward())
        except Exception as exc:
            self.get_logger().warning(f"setpoint write failed: {exc}")
            self._disconnect()
            return

        ghost = JointState()
        ghost.header.stamp = self.get_clock().now().to_msg()
        ghost.name = list(self._joint_names)
        # Mapped back: the raw goal ghost carries what a publisher sent and the URDF names are
        # logical, so a post-guard ghost in the hand's frame would break the RViz comparison.
        ghost.position = self._to_logical(safe).tolist()
        self._pub_commanded.publish(ghost)

    # --------------------------------------------------------------- diagnostics
    def _publish_diagnostics(self) -> None:
        """Report link health, guard activity and hardware health.

        Guard activity lives here rather than in the ghost topic on purpose: the ghost shows the
        post-guard target, so the RViz gap reads as pure tracking error, and clamping is visible
        separately instead of being confused for lag.

        Three signals that are three different faults, so three different keys. ``stale`` comes
        from the watchdog, because a rejected command never refreshes it and a stream of malformed
        messages would otherwise hide a dead publisher behind a reason string. ``clamped`` /
        ``rate_limited`` are the LATCHED union since the previous publish, because at 100 Hz most
        ticks are holds reporting nothing and a 10 Hz sample of the newest report would miss almost
        every clamp. ``last_rejection`` is kept apart from ``last_reason`` for the same reason:
        ``last_reason`` is whatever the last tick did, which on a healthy link is a hold.
        """
        status = DiagnosticStatus()
        status.name = "wuji_hand"
        status.hardware_id = str(self._serial or self._side)
        connected = self._hand is not None
        clamped = [self._joint_names[i] for i in np.flatnonzero(self._sticky_clamped)]
        limited = [self._joint_names[i] for i in np.flatnonzero(self._sticky_rate_limited)]

        if self._fatal_reason is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = "refusing to run"
        elif not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "not connected"
        elif clamped or limited:
            status.level = DiagnosticStatus.WARN
            status.message = "guards active"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"

        values = [
            KeyValue(key="connected", value=str(connected)),
            KeyValue(key="stale", value=str(self._chain.stale(time.monotonic()))),
            KeyValue(key="clamped", value=",".join(clamped)),
            KeyValue(key="rate_limited", value=",".join(limited)),
            KeyValue(key="last_reason", value="" if self._last_report is None else self._last_report.reason),
            KeyValue(key="last_rejection", value=self._last_rejection),
        ]
        if self._fatal_reason is not None:
            values.append(KeyValue(key="fatal", value=self._fatal_reason))
        if connected:
            # Health comes off the diagnostics stream, so this callback spends no bus time. No
            # temperature is published: the device exposes none per joint, and reporting a field
            # the hand does not measure is worse than omitting it.
            frame = self._diag.get() if self._diag is not None else None
            if frame is None or not frame.joints:
                values.append(KeyValue(key="health", value="no diagnostics frame yet"))
            else:
                faults = {}
                for entry in frame.joints:
                    code = int(getattr(entry, "error_code", 0) or 0)
                    if code:
                        faults[self._joint_names[nid_to_index(entry.nid)]] = code
                values += [
                    KeyValue(key="error_codes", value=",".join(f"{n}={c}" for n, c in sorted(faults.items()))),
                    KeyValue(key="link_age_s", value=f"{self._state.age() or float('nan'):.3f}"),
                ]

        status.values = values
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._pub_diag.publish(arr)
        # Cleared only once published, so nothing latched is ever dropped unreported.
        self._sticky_clamped[:] = False
        self._sticky_rate_limited[:] = False
        self._last_rejection = ""

    # --------------------------------------------------------------- state
    def _derived_velocity(self, logical: np.ndarray) -> np.ndarray:
        """Joint velocity by differencing this hand's own position stream, low-pass filtered.

        DERIVED, NOT MEASURED. The hand reports positions and currents; it has no velocity sense at
        all, and this field is the only place a consumer can get one. It is published in
        `JointState.velocity` -- the field consumers read as measured -- because that is where they
        look, and the alternative is every consumer differencing it themselves, worse, from a
        subsampled topic. The log says so once at startup; there is nothing in the message that can.

        WHY IT IS WORTH DOING AT ALL: the PACT policy observes twenty velocity columns and they are
        not decorative. On the training fixture their mean magnitude is 2.97 against ~0.5 for the
        position block, and zeroing them moves the policy's action by ~24% of its own magnitude. A
        differenced velocity at roughly the right scale is much closer to what the policy trained on
        than a block of zeros.

        DIFFERENCED IN THE LOGICAL FRAME, the same one `position` is published in, so a sign-flipped
        joint's velocity flips with its position instead of disagreeing with it. The offsets cancel
        in the subtraction; the signs do not, which is exactly why this differences the mapped value
        rather than the raw one.

        A LONG GAP IS NOT A VELOCITY. After a stall, a reconnect or a dropped stream, the position
        difference spans a time the hand was not being watched, and dividing it by that time reports
        an average that never happened. Those samples restart the estimate at zero instead.
        """
        now = self._now_seconds()
        previous, self._velocity_prev = self._velocity_prev, (logical.copy(), now)
        if previous is None:
            return self._velocity
        last, last_stamp = previous
        dt = now - last_stamp
        if dt <= 0.0 or dt > self._velocity_max_gap:
            self._velocity = np.zeros(NUM_JOINTS)
            return self._velocity
        # One-pole low pass, its coefficient built from the ACTUAL dt rather than the nominal
        # period: the publish timer is not a hard real-time clock, and a fixed coefficient would
        # make the cutoff drift with whatever jitter the loop happens to have.
        tau = 1.0 / (2.0 * math.pi * self._velocity_filter_hz)
        alpha = dt / (tau + dt)
        self._velocity = self._velocity + alpha * ((logical - last) / dt - self._velocity)
        return self._velocity

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_state(self) -> None:
        connected = self._hand is not None
        # READY, not merely linked. A command sent while the motors are released is dropped: the
        # re-acquire takes ~0.7 s, and an open-loop client streaming through it loses that much of
        # its trajectory with nothing to notice. Clients gate on this.
        self._pub_connected.publish(Bool(data=connected and self._energized))
        if not connected:
            return
        assert self._hand is not None
        # A stream that has stopped delivering is a dead link, and unlike the polled cache this
        # replaced, that is directly observable rather than indistinguishable from a still hand.
        if not self._link_is_live():
            self.get_logger().warning(f"no joint_states for {self._link_timeout:.2f}s; treating the link as down")
            self._disconnect()
            self._pub_connected.publish(Bool(data=False))
            return
        try:
            positions = self._measured_position()
            efforts = self._measured_effort()
        except Exception as exc:
            self.get_logger().warning(f"state read failed: {exc}")
            self._disconnect()
            self._pub_connected.publish(Bool(data=False))
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        # Mapped back for the same reason the ghost is: these names are the URDF's, and this topic
        # is what robot_state_publisher draws the hand from. Left in the hand's frame, a flipped
        # joint would render mirrored against its own ghosts -- the three-way comparison breaking at
        # the moment an operator has just found a flip and needs it most.
        logical = self._to_logical(positions)
        msg.position = logical.tolist()
        msg.velocity = self._derived_velocity(logical).tolist()
        msg.effort = efforts.tolist()
        self._pub_joint.publish(msg)

    def destroy_node(self) -> None:
        self._disconnect()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    # Constructed inside the try: a parameter the node refuses to start on must still reach
    # rclpy.shutdown, or the process exits with the context up and the error buried under it.
    node = None
    try:
        node = WujiHandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
