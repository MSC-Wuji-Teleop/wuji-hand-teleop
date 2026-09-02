"""Replay an exported clip at the driver's command topic, recording the tracking trace.

Open-loop: no feedback, no policy. The point is not that the hand succeeds at anything -- it is
to measure the two gaps that exist (raw -> post-guard is our guard activity; post-guard -> actual
is the plant and the SDK filter), which is why this records all three channels rather than only
what it published. They are stored exactly as recorded: the three run at their own rates and are
subscribed at their own moments, so equal lengths would mean samples had been invented.

Each sample is stored with the stamp of the message it came from, because the nominal rates cannot
align the three channels: the start offset alone is up to a publish period plus discovery latency,
a timer that misses a deadline never makes the time back, and the lag being measured against a 3 Hz
low-pass is itself only tens of milliseconds. A gap computed from stamps is a measurement; one
computed from `command_rate` is an assumption.

The published JointState always carries `name`, in the hardware order the clip was verified
against, so the driver's order check does real work instead of trusting column position.

Nothing eases into the first frame: it is wherever the reference starts, and the driver's slew
limit is what ramps into it. That ramp is guard activity the trace records, which is the
measurement -- not something to paper over with an approach ramp of our own.

    ros2 run starport_wuji_hand replay_clip --ros-args \\
        -p clip:=/tmp/clip.npz -p record:=/tmp/replay_trace.npz
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .first_frame_gate import FirstFrameGate, declare_and_validate
from .joint_map import HAND_SIDES, NUM_JOINTS, joint_names, name_to_index

if TYPE_CHECKING:
    # Type-only: builtin_interfaces is not one of this package's declared dependencies.
    from builtin_interfaces.msg import Time

DEFAULT_CHANNEL = "hand_joint_pos"
# One driver node per hand, each in its own namespace, so a tool's topics follow the side.
COMMAND_TOPIC_TEMPLATE = "/{side}/wuji_hand/joint_command"
COMMANDED_TOPIC_TEMPLATE = "/{side}/wuji_hand/commanded_joint_states"
# The driver reports READY here -- linked AND energized. It publishes false while the motors are
# released after an idle, and a command sent then is dropped rather than queued: re-acquiring takes
# around 0.7 s, which an open-loop client would otherwise stream straight through and lose.
READY_TOPIC_TEMPLATE = "/{side}/wuji_hand/connected"
DEFAULT_RATE = 100.0  # Hz -- the driver's command_rate default, so no setpoint is dropped or doubled
# Measured state is published globally by the driver so one robot_state_publisher can animate the
# whole cell.
DEFAULT_MEASURED_TOPIC = "/joint_states"


@dataclass(frozen=True)
class Clip:
    positions: np.ndarray
    names: tuple[str, ...]
    dt: float
    traj_id: str
    dataset: str
    channel: str


def load_clip(path: str, expect_channel: str, side: str = "right") -> Clip:
    """Load an exported clip, refusing anything the operator did not ask for.

    All three refusals guard against a replay that LOOKS right and is not: the wrong channel
    commands past a contact pose that an empty bench never provides, mislabelled columns send
    every finger's targets to the wrong joint, and the other hand's clip is a mirror image that
    stays inside every limit while curling the wrong way.
    """
    expected = joint_names(side)
    with np.load(path, allow_pickle=False) as data:
        channel = str(data["channel"])
        if channel != expect_channel:
            raise ValueError(
                f"clip channel is {channel!r} but {expect_channel!r} was requested; "
                "hand_joint_ctrl commands past the contact pose and is only correct with an "
                "object in the hand"
            )
        # Checked before the labels: the clip states its own side, and a mirror-image clip
        # would otherwise be reported as twenty unrecognised joint names.
        clip_side = str(data["hand_side"]) if "hand_side" in data else None
        if clip_side is None:
            raise ValueError(
                f"{path} carries no hand_side; re-export it -- a clip that does not say which "
                "hand it is for cannot be checked against the hand this driver is connected to"
            )
        if clip_side != side:
            raise ValueError(
                f"clip is labelled for the {clip_side} hand but this replay drives the {side} "
                f"hand; the two are mirror images, so its targets are inside every limit and "
                f"curl the wrong way"
            )
        names = tuple(str(n) for n in np.asarray(data["hand_joint_names"]).tolist())
        if names != expected:
            unexpected = sorted(set(names) - set(expected))
            raise ValueError(
                "clip joint names do not match the hardware order "
                f"(first mismatch at index {_first_mismatch(names, expected)}; "
                f"unexpected: {unexpected[:3]}). "
                "Relabel at export; do not trust column position."
            )
        positions = np.asarray(data["hand_joint_pos"], dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1] != NUM_JOINTS:
            raise ValueError(f"hand_joint_pos must be (T, {NUM_JOINTS}), got {positions.shape}")
        dt = float(data["dt"])
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"clip dt must be finite and positive, got {dt}")
        return Clip(
            positions=positions,
            names=names,
            dt=dt,
            traj_id=str(data["traj_id"]),
            dataset=str(data["dataset"]),
            channel=channel,
        )


def _first_mismatch(names: tuple[str, ...], expected: tuple[str, ...]) -> int:
    # strict=False: a clip may carry a different NUMBER of labels than the hand has joints, which
    # is itself part of the mismatch being reported.
    for i, (a, b) in enumerate(zip(names, expected, strict=False)):
        if a != b:
            return i
    return min(len(names), NUM_JOINTS)


def resample(positions: np.ndarray, src_dt: float, dst_dt: float) -> np.ndarray:
    """Linearly resample a (T, 20) clip from src_dt onto a dst_dt grid.

    Reference clips run 30-60 Hz and the driver takes setpoints at 100 Hz, so this interpolates
    UP. Interpolating (rather than holding the previous frame) matters: a held frame injects a
    staircase the SDK's low-pass then smooths, which would read as plant lag.
    """
    if positions.shape[0] < 2:
        raise ValueError("need at least 2 frames to resample")
    # Left to numpy, a non-finite or non-positive step yields an empty or nonsensical grid instead
    # of an error, and the nonsense would be published as a trajectory.
    if not math.isfinite(src_dt) or src_dt <= 0.0 or not math.isfinite(dst_dt) or dst_dt <= 0.0:
        raise ValueError(f"src_dt and dst_dt must be finite and positive, got {src_dt} and {dst_dt}")
    duration = (positions.shape[0] - 1) * src_dt
    src_t = np.arange(positions.shape[0]) * src_dt
    # The epsilon keeps the final source frame in the grid when duration is an exact multiple of
    # dst_dt; beyond it np.interp holds the endpoint, so a fractional tail cannot overshoot.
    dst_t = np.arange(0.0, duration + 1e-12, dst_dt)
    return np.stack([np.interp(dst_t, src_t, positions[:, j]) for j in range(positions.shape[1])], axis=1)


def sample_hand_joints(msg: JointState, index_of: Mapping[str, int]) -> np.ndarray | None:
    """The hand's twenty joints pulled out of a JointState BY NAME, or None if it carries none.

    Selecting by name is what makes the trace trustworthy. ``/joint_states`` is shared, so an arm
    publishing its own joints there is not a hand sample, and a hand sample whose columns arrive
    in another order must still land in hardware order. Appending raw position arrays instead
    would leave the trace ragged -- and a ragged trace cannot be saved at all, which loses the
    whole run's measurement at the moment it ends.

    A partial or non-finite row is refused for the same reason: either would quietly corrupt any
    gap computed from the trace.
    """
    if len(msg.name) != len(msg.position):
        return None
    sample = np.full(NUM_JOINTS, np.nan)
    # Lengths are equal by the check above, so strict=True can never trip here.
    for name, value in zip(msg.name, msg.position, strict=True):
        index = index_of.get(name)
        if index is not None:
            sample[index] = value
    if not np.isfinite(sample).all():
        return None
    return sample


def _stamp_seconds(stamp: Time) -> float:
    """A message stamp as float seconds on the ROS clock, which every node on the cell shares."""
    return stamp.sec + stamp.nanosec * 1e-9


def _split(recorded: list[tuple[float, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    """A recorded channel as an (N,) stamp array and an (N, 20) sample array.

    Stamps and samples are held together as pairs until here so the two can never drift out of
    step. Both stay two-dimensional in shape when nothing arrived: a channel is usually empty
    because the driver was not running, which is exactly when the trace gets read, so it still has
    to be indexable per joint.
    """
    if not recorded:
        return np.zeros(0), np.zeros((0, NUM_JOINTS))
    return np.asarray([t for t, _ in recorded]), np.asarray([sample for _, sample in recorded])


@dataclass
class _Channel:
    """One subscribed channel's samples, its refusal count and its own warn-once state.

    Per channel rather than per node because rclpy's ``once=True`` is keyed by CALL SITE, not by
    message: one shared warn site would let the first refused sample on either channel silence
    the other for the rest of the run, and one shared counter cannot say which channel refused.

    ``rejected`` counts only messages that carried some of the hand and could not be used -- see
    ``_record`` for what is deliberately left out of it.
    """

    label: str
    samples: list[tuple[float, np.ndarray]] = field(default_factory=list)
    rejected: int = 0
    warned: bool = False


class ReplayClip(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("replay_clip", **kwargs)
        self.declare_parameter("clip", "")
        self.declare_parameter("channel", DEFAULT_CHANNEL)
        self.declare_parameter("command_topic", "")
        self.declare_parameter("commanded_topic", "")
        self.declare_parameter("measured_topic", DEFAULT_MEASURED_TOPIC)
        self.declare_parameter("rate", DEFAULT_RATE)
        self.declare_parameter("record", "")
        self.declare_parameter("hand_side", "right")

        side = str(self.get_parameter("hand_side").value)
        if side not in HAND_SIDES:
            raise ValueError(f"hand_side must be one of {list(HAND_SIDES)}, got {side!r}")
        self._side = side
        self._joint_names = joint_names(side)
        # Built once: every sample read off a shared /joint_states is filtered through it, so the
        # other hand's joints are foreign traffic here rather than columns to fill.
        self._index_of = name_to_index(side)
        # Declared empty above so the side -- read just now -- can supply the default.
        for name, template in (
            ("command_topic", COMMAND_TOPIC_TEMPLATE),
            ("commanded_topic", COMMANDED_TOPIC_TEMPLATE),
        ):
            if not str(self.get_parameter(name).value):
                self.set_parameters([rclpy.parameter.Parameter(name, value=template.format(side=side))])

        clip_path = str(self.get_parameter("clip").value)
        if not clip_path:
            raise ValueError("clip parameter is required: the path of an exported replay NPZ")
        # Checked before the clip is read: 1/rate is the destination grid and the timer period, so
        # a zero or non-finite rate builds a nonsense trajectory and a timer that never fires.
        self._rate = float(self.get_parameter("rate").value)
        if not math.isfinite(self._rate) or self._rate <= 0.0:
            raise ValueError(f"rate must be finite and positive, got {self._rate}")
        wait_s = declare_and_validate(self)

        self._clip = load_clip(clip_path, expect_channel=str(self.get_parameter("channel").value), side=side)
        self._frames = resample(self._clip.positions, self._clip.dt, 1.0 / self._rate)
        self._record_path = str(self.get_parameter("record").value)
        self._topic = str(self.get_parameter("command_topic").value)

        self.get_logger().info(
            f"replaying {self._clip.traj_id} ({self._clip.dataset}, channel={self._clip.channel}): "
            f"{self._clip.positions.shape[0]} frames @ {1.0 / self._clip.dt:.1f} Hz "
            f"-> {self._frames.shape[0]} @ {self._rate:.0f} Hz"
        )

        self._pub = self.create_publisher(JointState, self._topic, 10)
        # All three channels are recorded: what we asked for, what the guards passed, what happened.
        self._trace_raw: list[tuple[float, np.ndarray]] = []
        self._commanded = _Channel("post-guard")
        self._measured = _Channel("measured")
        self._sub_commanded = self.create_subscription(
            JointState, str(self.get_parameter("commanded_topic").value), self._on_commanded, 50
        )
        self._sub_measured = self.create_subscription(
            JointState, str(self.get_parameter("measured_topic").value), self._on_measured, 50
        )
        self._driver_ready = False
        self._sub_ready = self.create_subscription(Bool, READY_TOPIC_TEMPLATE.format(side=side), self._on_ready, 10)
        # After the subscriptions, because the gate holds the first frame until they are live when
        # a trace was asked for. Discovery starts when they are created, so this is also the
        # earliest the clock should start.
        self._gate = FirstFrameGate(
            self,
            self._pub,
            self._topic,
            wait_s,
            require_matched=(
                (self._sub_commanded.topic_name, self._sub_measured.topic_name) if self._record_path else ()
            ),
            require_ready=lambda: self._driver_ready,
        )

        self._i = 0
        self.create_timer(1.0 / self._rate, self._publish_next)

    def _on_commanded(self, msg: JointState) -> None:
        self._record(self._commanded, msg)

    def _on_measured(self, msg: JointState) -> None:
        self._record(self._measured, msg)

    def _record(self, channel: _Channel, msg: JointState) -> None:
        sample = sample_hand_joints(msg, self._index_of)
        if sample is None:
            # A message carrying SOME of the hand is a refusal: the publisher we are measuring sent
            # something unusable, so it is counted and worth interrupting the operator for. One
            # carrying NONE of it is just another publisher on a shared topic -- /joint_states
            # carries the gripper and both arms -- so it is neither counted nor warned about.
            # Counting those would close an ordinary replay on thousands of "ignored" and bury the
            # one number that means a sample of ours went missing. An UNNAMED message lands on the
            # foreign side of this test, a bare vector naming nothing the map knows, so an unusable
            # bare-vector sample is invisible rather than counted -- theoretical only, since both
            # publishers being measured always fill `name`.
            if not any(name in self._index_of for name in msg.name):
                return
            channel.rejected += 1
            if not channel.warned:
                channel.warned = True
                self.get_logger().warning(
                    f"ignoring a partial or non-finite {channel.label} sample; the channel will be short"
                )
            return
        channel.samples.append((_stamp_seconds(msg.header.stamp), sample))

    def _publish_next(self) -> None:
        if not self._gate.ready():
            return
        if self._i >= self._frames.shape[0]:
            self.save_trace()
            raise SystemExit(0)
        target = self._frames[self._i]
        self._i += 1
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self._joint_names)
        msg.position = target.tolist()
        self._pub.publish(msg)
        self._trace_raw.append((_stamp_seconds(stamp), target.copy()))

    def _on_ready(self, msg: Bool) -> None:
        self._driver_ready = bool(msg.data)

    def save_trace(self) -> None:
        """Write everything recorded so far, however far the replay got.

        Public and safe to call mid-run: a run stopped early still measured what it measured, and
        an interrupted bench session is often the one worth looking at.
        """
        if not self._record_path:
            self.get_logger().info("replay complete; no record path given, so the trace is discarded")
            return
        raw_t, raw = _split(self._trace_raw)
        commanded_t, commanded = _split(self._commanded.samples)
        measured_t, measured = _split(self._measured.samples)
        np.savez(
            self._record_path,
            raw=raw,
            raw_t=raw_t,
            commanded=commanded,
            commanded_t=commanded_t,
            measured=measured,
            measured_t=measured_t,
            joint_names=np.asarray(list(self._joint_names)),
            traj_id=np.asarray(self._clip.traj_id),
            dataset=np.asarray(self._clip.dataset),
            channel=np.asarray(self._clip.channel),
            command_rate=np.asarray(self._rate),
        )
        self.get_logger().info(
            f"trace written to {self._record_path}: {len(self._trace_raw)} published, "
            f"{len(self._commanded.samples)} post-guard ({self._commanded.rejected} ignored), "
            f"{len(self._measured.samples)} measured ({self._measured.rejected} ignored)"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    # Constructed inside the try: a clip the node refuses to replay must still reach
    # rclpy.shutdown, or the process exits with the context up and the error buried under it.
    node = None
    try:
        node = ReplayClip()
        rclpy.spin(node)
    except SystemExit:
        pass  # the replay ran out of frames and saved its own trace
    except KeyboardInterrupt:
        if node is not None:
            node.save_trace()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
