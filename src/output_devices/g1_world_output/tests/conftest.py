#!/usr/bin/env python3
"""Stubs for the Unitree SDK, so G1ArmController can be constructed in a test.

Every other test in this package deliberately fakes nothing: it covers only
the ROS-free and DDS-free modules. G1ArmController cannot be covered that way,
because what it does at construction time -- which pose it commands first, and
when it raises the arm_sdk weight -- is exactly what reaches the robot before
anything else in the stack has run. So this file stands in for the four
unitree_sdk2py modules robot_arm.py imports, with the smallest objects that
satisfy it: a publisher that records what was written, a subscriber that
returns a fixed LowState, a no-op CRC, and a LowCmd with a motor_cmd array.

Nothing here models DDS behaviour. It models the shape of the message, so a
test can read the bytes robot_arm.py would have put on the wire.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

# Package root (the directory holding g1_world_output/), matching the other
# tests in this directory.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

# Unitree's unified motor array length, and the weight slot index. Duplicated
# from robot_arm.py rather than imported, because these stubs have to exist
# before robot_arm.py can be imported at all.
NUM_MOTORS = 35
WEIGHT_SLOT = 29

# Distinctive measured joint angles, one per motor slot: slot i reads
# i / 100 rad. Every arm slot is then non-zero and unique, so a test can tell
# "commanded the measured pose" apart from "commanded zeros" and from
# "commanded the wrong slot" without ambiguity.
MEASURED_Q_PER_SLOT = 0.01

# What the robot reports as its mode_machine. The rig's value
# (docs/spec/hardware_spec.md); nothing in robot_arm.py branches on it, it is
# only copied into the command.
MODE_MACHINE = 5


class StubMotorCmd:
    """One motor_cmd slot: the fields robot_arm.py writes."""

    def __init__(self) -> None:
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0


class StubLowCmd:
    def __init__(self) -> None:
        self.motor_cmd = [StubMotorCmd() for _ in range(NUM_MOTORS)]
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0


class StubPublisher:
    """Records a snapshot of every Write, in order, on self.writes.

    robot_arm.py mutates and re-writes one LowCmd object, so storing the
    object itself would give every entry the same (latest) values. Each Write
    therefore copies out the fields a test asserts on.
    """

    def __init__(self, topic, msg_type) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.writes: list[dict] = []

    def Init(self) -> None:
        pass

    def Write(self, msg) -> None:
        self.writes.append({
            "q": [m.q for m in msg.motor_cmd],
            "kp": [m.kp for m in msg.motor_cmd],
            "kd": [m.kd for m in msg.motor_cmd],
            "weight": msg.motor_cmd[WEIGHT_SLOT].q,
        })


class StubSubscriber:
    """Returns a LowState whose slot i reads i * MEASURED_Q_PER_SLOT rad."""

    def __init__(self, topic, msg_type) -> None:
        self.topic = topic
        self.msg_type = msg_type

    def Init(self, *args) -> None:
        pass

    def Read(self):
        return SimpleNamespace(
            motor_state=[
                SimpleNamespace(q=i * MEASURED_Q_PER_SLOT, dq=0.0)
                for i in range(NUM_MOTORS)
            ],
            mode_machine=MODE_MACHINE,
        )


class StubCRC:
    def Crc(self, msg) -> int:
        return 0


def _install_sdk_stubs() -> dict:
    """Put the four unitree_sdk2py modules robot_arm.py imports into sys.modules."""
    created: dict[str, ModuleType | None] = {}
    factory_calls: list[tuple] = []

    def module(name: str) -> ModuleType:
        created[name] = sys.modules.get(name)
        mod = ModuleType(name)
        sys.modules[name] = mod
        return mod

    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg",
                 "unitree_sdk2py.utils"):
        module(name)

    channel = module("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a: factory_calls.append(a)
    channel.ChannelPublisher = StubPublisher
    channel.ChannelSubscriber = StubSubscriber

    default = module("unitree_sdk2py.idl.default")
    default.unitree_hg_msg_dds__LowCmd_ = StubLowCmd

    dds = module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowCmd_ = StubLowCmd
    dds.LowState_ = object

    crc = module("unitree_sdk2py.utils.crc")
    crc.CRC = StubCRC

    return {"created": created, "factory_calls": factory_calls}


@pytest.fixture(scope="module")
def robot_arm():
    """The robot_arm module, imported against the stubs above."""
    state = _install_sdk_stubs()
    previous = sys.modules.pop("g1_world_output.robot_arm", None)
    import g1_world_output.robot_arm as module_under_test

    module_under_test._stub_factory_calls = state["factory_calls"]
    yield module_under_test

    sys.modules.pop("g1_world_output.robot_arm", None)
    if previous is not None:
        sys.modules["g1_world_output.robot_arm"] = previous
    for name, original in state["created"].items():
        if original is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = original


@pytest.fixture
def make_controller(robot_arm):
    """Build a G1ArmController and tear it down without the shutdown ramp.

    shutdown() spends just over a second ramping the arm_sdk weight, which is
    the right thing on the robot and dead weight in a test. Teardown stops the
    write thread and releases the writer flock directly, so the next test can
    take the lock.
    """
    import fcntl

    built = []

    def build(**kwargs):
        params = {"motion_mode": True, "simulation_mode": False, "arm_type": "G1_29"}
        params.update(kwargs)
        controller = robot_arm.G1ArmController(**params)
        built.append(controller)
        return controller

    yield build

    for controller in built:
        controller._running = False
        controller.publish_thread.join(timeout=1.0)
        try:
            fcntl.flock(controller._lock_file, fcntl.LOCK_UN)
            controller._lock_file.close()
        except Exception:
            pass


def measured_arm_q(robot_arm, arm_type: str = "G1_29") -> np.ndarray:
    """The arm slots of the stub LowState, in the controller's arm order."""
    return np.array([i * MEASURED_Q_PER_SLOT
                     for i in robot_arm.ARM_INDICES_BY_TYPE[arm_type]])
