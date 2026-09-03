#!/usr/bin/env python3
"""What G1ArmController puts on the wire before anything else has run.

The first DDS command out of a fresh controller reaches the robot before the
node has finished constructing, before rclpy.spin, and therefore before any
control loop or publisher exists. It raises the arm_sdk weight to 1.0 at the
same time, so whatever pose it names is a pose the robot will move to. These
tests pin that pose to the measured one.

The regression: q_target was initialised to zeros and the write thread was
started with it, so the first commands walked the arms toward all-zeros at
arm_velocity_limit for as long as the rest of startup took. Unitree's arm_sdk
example avoids the same thing the same way, by making its first commanded pose
the measured one (example/g1/high_level/g1_arm7_sdk_dds_example.py, stage 1 at
ratio 0).

Not covered here, because a stub cannot know it: what the onboard controller
does with the command. These tests read the message, not the robot.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from conftest import MODE_MACHINE, WEIGHT_SLOT, measured_arm_q

# How long to wait for the write thread's first Write. The loop period is
# control_dt (4 ms) and the thread is already running when the constructor
# returns, so this is two orders of margin, not a timing assumption.
FIRST_WRITE_TIMEOUT_S = 2.0

# Float comparison tolerance. The path from the stub LowState to the command is
# a copy and one no-op arithmetic step in clip_arm_q_target, so anything above
# representation noise would be a real difference.
TOL = 1e-12


def wait_for_writes(controller, count: int = 1) -> list:
    """Block until the write thread has published ``count`` messages."""
    deadline = time.time() + FIRST_WRITE_TIMEOUT_S
    while time.time() < deadline:
        writes = controller.lowcmd_publisher.writes
        if len(writes) >= count:
            return writes
        time.sleep(0.002)
    raise AssertionError(
        f"no {count} write(s) within {FIRST_WRITE_TIMEOUT_S} s "
        f"(got {len(controller.lowcmd_publisher.writes)})"
    )


def test_q_target_is_the_measured_pose_after_construction(robot_arm, make_controller):
    controller = make_controller()
    np.testing.assert_allclose(
        controller.q_target, measured_arm_q(robot_arm), atol=TOL
    )


def test_first_command_names_the_measured_pose_not_zeros(robot_arm, make_controller):
    controller = make_controller()
    first = wait_for_writes(controller)[0]

    measured = measured_arm_q(robot_arm)
    commanded = np.array([first["q"][i] for i in robot_arm.ARM_INDICES_BY_TYPE["G1_29"]])
    np.testing.assert_allclose(commanded, measured, atol=TOL)
    # The regression would have produced a pose strictly between measured and
    # zeros, so state the negative too: no arm slot is at zero.
    assert not np.any(np.abs(commanded) < TOL)


def test_first_command_raises_the_arm_sdk_weight(robot_arm, make_controller):
    controller = make_controller()
    first = wait_for_writes(controller)[0]

    assert first["weight"] == pytest.approx(1.0)
    assert controller.lowcmd_publisher.topic == robot_arm.kTopicLowCommand_Motion


def test_the_arms_do_not_drift_while_nothing_commands_them(robot_arm, make_controller):
    """Many ticks with no new target leave the command where it started.

    The stub LowState never changes, so measured never moves; the point is that
    q_target does not either. Before the seed this test would have shown the
    command marching toward zeros one clip step per tick.
    """
    controller = make_controller()
    writes = wait_for_writes(controller, count=20)

    measured = measured_arm_q(robot_arm)
    arm_indices = robot_arm.ARM_INDICES_BY_TYPE["G1_29"]
    for write in writes[:20]:
        commanded = np.array([write["q"][i] for i in arm_indices])
        np.testing.assert_allclose(commanded, measured, atol=TOL)


def test_non_arm_slots_hold_their_startup_measured_value(robot_arm, make_controller):
    """Legs and waist are commanded at the pose they were in at startup.

    Recorded rather than endorsed: this is what the vendored reference code
    does, and it is unchanged by the seed. It is here so a future change to the
    slot policy shows up as a failing test rather than silently.
    """
    controller = make_controller()
    first = wait_for_writes(controller)[0]

    arm_indices = set(robot_arm.ARM_INDICES_BY_TYPE["G1_29"])
    for slot in robot_arm.G1_23_JointIndex:
        if slot.value in arm_indices or slot.value == WEIGHT_SLOT:
            continue
        assert first["q"][slot.value] == pytest.approx(
            controller.all_motor_q[slot.value], abs=TOL
        )


def test_wrist_slots_get_the_wrist_gain_tier(robot_arm, make_controller):
    """kp 50 / kd 2 on the six wrist pitch/yaw slots, 140 / 3 on the rest.

    The audit in tools/clip_audit.py re-gains its model to these numbers, so a
    change here silently invalidates every clip verdict.
    """
    controller = make_controller()
    first = wait_for_writes(controller)[0]

    wrist = set(robot_arm.WRIST_MOTORS_BY_TYPE["G1_29"])
    for slot in robot_arm.ARM_INDICES_BY_TYPE["G1_29"]:
        if slot in wrist:
            assert first["kp"][slot] == pytest.approx(controller.kp_wrist)
            assert first["kd"][slot] == pytest.approx(controller.kd_wrist)
        else:
            assert first["kp"][slot] == pytest.approx(controller.kp_low)
            assert first["kd"][slot] == pytest.approx(controller.kd_low)


def test_mode_machine_is_copied_from_the_robot(robot_arm, make_controller):
    controller = make_controller()
    assert controller.msg.mode_machine == MODE_MACHINE


def test_network_interface_reaches_the_dds_participant(robot_arm, make_controller):
    """The NIC pin is the only thing binding the robot link on a multi-NIC host."""
    robot_arm._stub_factory_calls.clear()
    make_controller(network_interface="enxTEST")
    assert robot_arm._stub_factory_calls == [(0, "enxTEST")]


def test_simulation_mode_moves_the_dds_domain(robot_arm, make_controller):
    robot_arm._stub_factory_calls.clear()
    make_controller(simulation_mode=True)
    assert robot_arm._stub_factory_calls == [(1,)]


def test_a_second_controller_is_refused_the_writer_lock(robot_arm, make_controller):
    """Only one process may write rt/arm_sdk. Also what stops a rehome from
    running while a replay holds the arms."""
    make_controller()
    with pytest.raises(RuntimeError, match="lowcmd writer lock"):
        make_controller()
