"""Test scaffolding for the replay package: ROS stubs and a synthetic clip writer.

The tests need numpy and pytest only. Before any node module is imported,
``rclpy``, ``rclpy.node``, ``rclpy.qos``, ``rclpy.utilities``, ``rclpy.task``,
``rclpy.executors``, ``sensor_msgs.msg`` and ``std_msgs.msg`` are replaced in
``sys.modules`` by the small fakes below. The fake Node records every
``create_publisher`` / ``create_subscription`` / ``create_timer`` call and
carries a clock the test advances by hand, so a node's callbacks run in the
test process with no executor.

The stubs are installed unconditionally, also inside the container where the
real rclpy exists: the tests are written against the recording Node, and a
symlink-installed ``replay`` resolves to the same files either way.
"""

from __future__ import annotations

import enum
import json
import sys
import types
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))


# --- fake ROS -------------------------------------------------------------------


class FakeTimeMsg:
    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class FakeTime:
    def __init__(self, nanoseconds: int):
        self.nanoseconds = int(nanoseconds)

    def to_msg(self) -> FakeTimeMsg:
        return FakeTimeMsg(self.nanoseconds // 1_000_000_000, self.nanoseconds % 1_000_000_000)


class FakeClock:
    """Seconds since an arbitrary zero; the test moves it."""

    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> FakeTime:
        return FakeTime(round(self.t * 1e9))

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def _log(self, level: str, msg: str) -> None:
        self.messages.append((level, str(msg)))

    def debug(self, msg):
        self._log("debug", msg)

    def info(self, msg):
        self._log("info", msg)

    def warning(self, msg):
        self._log("warning", msg)

    warn = warning

    def error(self, msg):
        self._log("error", msg)

    def of_level(self, level: str) -> list[str]:
        return [m for lvl, m in self.messages if lvl == level]


class FakePublisher:
    def __init__(self, msg_type, topic: str, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeSubscription:
    def __init__(self, msg_type, topic: str, callback, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos

    def deliver(self, msg) -> None:
        self.callback(msg)


class FakeTimer:
    def __init__(self, period: float, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True

    def fire(self) -> None:
        self.callback()


class FakeNode:
    """Records what a node asks for. Stands in for rclpy.node.Node."""

    def __init__(self, name: str, **kwargs):
        self.name = name
        self.publishers: list[FakePublisher] = []
        self.subscriptions: list[FakeSubscription] = []
        self.timers: list[FakeTimer] = []
        self.destroyed = False
        self._clock = FakeClock()
        self._logger = FakeLogger()

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(msg_type, topic, qos)
        self.publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = FakeSubscription(msg_type, topic, callback, qos)
        self.subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_clock(self) -> FakeClock:
        return self._clock

    def get_logger(self) -> FakeLogger:
        return self._logger

    def destroy_node(self) -> None:
        self.destroyed = True

    # test helpers
    def subscription(self, topic: str) -> FakeSubscription:
        matches = [s for s in self.subscriptions if s.topic == topic]
        assert len(matches) == 1, f"{len(matches)} subscriptions on {topic!r}"
        return matches[0]

    def publisher(self, topic: str) -> FakePublisher:
        matches = [p for p in self.publishers if p.topic == topic]
        assert len(matches) == 1, f"{len(matches)} publishers on {topic!r}"
        return matches[0]


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._result = None

    def done(self) -> bool:
        return self._done

    def set_result(self, result) -> None:
        self._result = result
        self._done = True

    def result(self):
        return self._result


class ExternalShutdownException(Exception):
    pass


class QoSReliabilityPolicy(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class QoSHistoryPolicy(enum.Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSDurabilityPolicy(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class QoSProfile:
    def __init__(self, **kwargs):
        self.reliability = kwargs.get("reliability", QoSReliabilityPolicy.RELIABLE)
        self.history = kwargs.get("history", QoSHistoryPolicy.KEEP_LAST)
        self.depth = kwargs.get("depth", 10)
        self.durability = kwargs.get("durability", QoSDurabilityPolicy.VOLATILE)


def remove_ros_args(args=None):
    args = list(sys.argv if args is None else args)
    if "--ros-args" in args:
        i = args.index("--ros-args")
        rest = args[i + 1:]
        tail = rest[rest.index("--") + 1:] if "--" in rest else []
        return args[:i] + tail
    return args


class FakeRclpy(types.ModuleType):
    """The rclpy module surface the nodes use. ``spin`` raises the interrupt it is told to."""

    def __init__(self) -> None:
        super().__init__("rclpy")
        self.initialized = False
        self.init_calls = 0
        self.shutdown_calls = 0
        self.spin_raises: type[BaseException] = KeyboardInterrupt
        self.spin_until_future_raises: Optional[type[BaseException]] = None
        self.spin_shuts_down = False  # when True, the raise above also invalidates the context first
        self.spun: list = []
        self.max_poll_iterations = 10_000

    def init(self, args=None, **kwargs) -> None:
        self.initialized = True
        self.init_calls += 1

    def ok(self) -> bool:
        return self.initialized

    def shutdown(self, **kwargs) -> None:
        self.initialized = False
        self.shutdown_calls += 1

    def _raise(self, exc_type: type[BaseException]) -> None:
        if self.spin_shuts_down:
            self.initialized = False
        raise exc_type()

    def spin(self, node) -> None:
        self.spun.append(node)
        self._raise(self.spin_raises)

    def spin_until_future_complete(self, node, future, timeout_sec=None) -> None:
        """Drive the node's timers on its fake clock until the future completes."""
        self.spun.append(node)
        if self.spin_until_future_raises is not None:
            self._raise(self.spin_until_future_raises)
        for _ in range(self.max_poll_iterations):
            if future.done():
                return
            active = [t for t in node.timers if not t.cancelled]
            assert active, "no active timer; the future can never complete"
            node.get_clock().advance(min(t.period for t in active))
            for timer in active:
                timer.fire()
        raise AssertionError("future never completed")


class FakeHeader:
    def __init__(self) -> None:
        self.stamp = FakeTimeMsg()
        self.frame_id = ""


class JointState:
    def __init__(self, name=None, position=None):
        self.header = FakeHeader()
        self.name = list(name) if name is not None else []
        self.position = list(position) if position is not None else []
        self.velocity: list = []
        self.effort: list = []


class Bool:
    def __init__(self, data: bool = False):
        self.data = data


def _install_ros_stubs() -> FakeRclpy:
    rclpy = FakeRclpy()
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = QoSProfile
    qos_mod.QoSReliabilityPolicy = QoSReliabilityPolicy
    qos_mod.QoSHistoryPolicy = QoSHistoryPolicy
    qos_mod.QoSDurabilityPolicy = QoSDurabilityPolicy
    util_mod = types.ModuleType("rclpy.utilities")
    util_mod.remove_ros_args = remove_ros_args
    task_mod = types.ModuleType("rclpy.task")
    task_mod.Future = FakeFuture
    exec_mod = types.ModuleType("rclpy.executors")
    exec_mod.ExternalShutdownException = ExternalShutdownException
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    rclpy.utilities = util_mod
    rclpy.task = task_mod
    rclpy.executors = exec_mod

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = JointState
    sensor_msgs.msg = sensor_msgs_msg
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
    std_msgs.msg = std_msgs_msg

    for name, mod in (
        ("rclpy", rclpy),
        ("rclpy.node", node_mod),
        ("rclpy.qos", qos_mod),
        ("rclpy.utilities", util_mod),
        ("rclpy.task", task_mod),
        ("rclpy.executors", exec_mod),
        ("sensor_msgs", sensor_msgs),
        ("sensor_msgs.msg", sensor_msgs_msg),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msgs_msg),
    ):
        sys.modules[name] = mod
    # A node module imported before this conftest ran would hold the real rclpy.
    for name in ("replay.replay_publisher", "replay.replay_check"):
        sys.modules.pop(name, None)
    return rclpy


FAKE_RCLPY = _install_ros_stubs()


@pytest.fixture
def fake_rclpy() -> FakeRclpy:
    FAKE_RCLPY.initialized = False
    FAKE_RCLPY.init_calls = 0
    FAKE_RCLPY.shutdown_calls = 0
    FAKE_RCLPY.spin_raises = KeyboardInterrupt
    FAKE_RCLPY.spin_until_future_raises = None
    FAKE_RCLPY.spin_shuts_down = False
    FAKE_RCLPY.spun = []
    return FAKE_RCLPY


# --- synthetic clip -------------------------------------------------------------

# Joint names as the brief fixes them: arm names are the G1 node's
# (G1_29_ARM_JOINT_NAMES, no _joint suffix), hand names are the driver's
# hardware order with the l_/r_ prefix (tools/clip_audit.py HAND_JOINT_NAMES).
ARM_NAMES = {
    side: (
        f"{side}_shoulder_pitch",
        f"{side}_shoulder_roll",
        f"{side}_shoulder_yaw",
        f"{side}_elbow",
        f"{side}_wrist_roll",
        f"{side}_wrist_pitch",
        f"{side}_wrist_yaw",
    )
    for side in ("left", "right")
}
HAND_NAMES = {
    side: (
        f"{p}_thumb_cmc_flex", f"{p}_thumb_cmc_abd", f"{p}_thumb_mcp", f"{p}_thumb_ip",
        f"{p}_index_finger_mcp_flex", f"{p}_index_finger_mcp_abd", f"{p}_index_finger_pip", f"{p}_index_finger_dip",
        f"{p}_middle_finger_mcp_flex", f"{p}_middle_finger_mcp_abd", f"{p}_middle_finger_pip", f"{p}_middle_finger_dip",
        f"{p}_ring_finger_mcp_flex", f"{p}_ring_finger_mcp_abd", f"{p}_ring_finger_pip", f"{p}_ring_finger_dip",
        f"{p}_pinky_mcp_flex", f"{p}_pinky_mcp_abd", f"{p}_pinky_pip", f"{p}_pinky_dip",
    )
    for side, p in (("left", "l"), ("right", "r"))
}

CLIP_NAME = "11_val_a5yNwUSiYpA_9-3-rgb_front_Ours"
FRAMES = 10
RATE_HZ = 50.0
SAFE_SPEEDS = (1.0, 0.5)


def synthetic_arrays(frames: int) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Deterministic (T, 7) and (T, 20) arrays per side; frame i, column j holds i + j / 100 (right side negated)."""
    arm, hand = {}, {}
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        arm[side] = sign * (np.arange(frames)[:, None] + np.arange(7)[None, :] / 100.0)
        hand[side] = sign * (np.arange(frames)[:, None] + np.arange(20)[None, :] / 100.0)
    return arm, hand


def clip_meta(frames: int = FRAMES, rate_hz: float = RATE_HZ, safe_speeds=SAFE_SPEEDS, verdict: str = "safe",
              arm_names=None, hand_names=None) -> dict:
    """A minimal clip.json body; the audit block is present but empty, the loader does not read it."""
    arm_names = ARM_NAMES if arm_names is None else arm_names
    hand_names = HAND_NAMES if hand_names is None else hand_names
    return {
        "tool": "prepare_clip/1",
        "source": {"sample": "11_val_a5yNwUSiYpA_9-3-rgb_front", "method": "Ours", "bundle_manifest_sha256": None},
        "frames": frames,
        "rate_hz": rate_hz,
        "arm_joint_names": {s: list(arm_names[s]) for s in ("left", "right")},
        "hand_joint_names": {s: list(hand_names[s]) for s in ("left", "right")},
        "audit": {},
        "safe_speeds": list(safe_speeds),
        "verdict": verdict,
    }


def write_clip(clip_dir: Path, meta: dict | None = None, arm_q=None, hand_q20=None) -> Path:
    """Write a clip directory. Arrays default to synthetic ones with meta['frames'] rows."""
    meta = clip_meta() if meta is None else meta
    default_arm, default_hand = synthetic_arrays(int(meta["frames"]))
    arm_q = default_arm if arm_q is None else arm_q
    hand_q20 = default_hand if hand_q20 is None else hand_q20
    clip_dir.mkdir(parents=True, exist_ok=True)
    np.savez(clip_dir / "arm_q.npz", **arm_q)
    np.savez(clip_dir / "hand_q20.npz", **hand_q20)
    (clip_dir / "clip.json").write_text(json.dumps(meta, indent=1))
    return clip_dir


@pytest.fixture
def clip_dir(tmp_path: Path) -> Path:
    """A valid clip directory at tmp_path/safe/<name>: 10 frames, 50 Hz, safe at 1.0 and 0.5."""
    return write_clip(tmp_path / "safe" / CLIP_NAME)
