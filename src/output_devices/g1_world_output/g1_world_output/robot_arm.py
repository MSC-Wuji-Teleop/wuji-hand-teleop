"""
Unitree G1_23 dual-arm DDS controller.

G1_23 LowCmd / LowState arm control over Unitree SDK2 DDS.
5 DoF per arm (shoulder pitch/roll/yaw, elbow, wrist roll) = 10 total.

Adapted from Unitree's public G1 arm DDS example code (unitreerobotics/
unitree_sdk2_python / xr_teleoperate reference scripts) -- class/method
names and control-loop structure follow that reference closely.
"""

from __future__ import annotations

import logging
import threading
import time
from enum import IntEnum

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState
from unitree_sdk2py.utils.crc import CRC

logger = logging.getLogger(__name__)

kTopicLowCommand_Debug = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

G1_23_Num_Motors = 35
G1_23_ARM_DOF = 10  # 5 per arm


class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None


class G1_23_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_23_Num_Motors)]


class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data


class G1_23_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19

    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26


class G1_23_JointIndex(IntEnum):
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRollNotUsed = 13
    kWaistPitchNotUsed = 14

    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitchNotUsed = 20
    kLeftWristyawNotUsed = 21

    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitchNotUsed = 27
    kRightWristYawNotUsed = 28

    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


G1_23_ARM_JOINT_NAMES = [
    'left_shoulder_pitch',
    'left_shoulder_roll',
    'left_shoulder_yaw',
    'left_elbow',
    'left_wrist_roll',
    'right_shoulder_pitch',
    'right_shoulder_roll',
    'right_shoulder_yaw',
    'right_elbow',
    'right_wrist_roll',
]


class G1_23_ArmController:
    def __init__(
        self,
        motion_mode: bool = True,
        simulation_mode: bool = False,
        dds_already_initialized: bool = False,
    ):
        logger.info("Initialize G1_23_ArmController...")
        self.q_target = np.zeros(G1_23_ARM_DOF)
        self.tauff_target = np.zeros(G1_23_ARM_DOF)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 300.0
        self.kd_high = 5.0
        self.kp_low = 140.0
        self.kd_low = 3.0
        self.kp_wrist = 50.0
        self.kd_wrist = 2.0

        self.all_motor_q = None
        self.arm_velocity_limit = 20.0
        self.control_dt = 1.0 / 250.0

        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None
        self._running = True

        if not dds_already_initialized:
            if self.simulation_mode:
                ChannelFactoryInitialize(1)
            else:
                ChannelFactoryInitialize(0)
        else:
            logger.info(
                "[G1_23_ArmController] DDS already initialized, skipping ChannelFactoryInitialize"
            )

        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        self.subscribe_thread = threading.Thread(
            target=self._subscribe_motor_state, daemon=True
        )
        self.subscribe_thread.start()

        lowstate_timeout_s = 30.0
        wait_start = time.time()
        last_log = wait_start
        while not self.lowstate_buffer.GetData():
            if time.time() - wait_start > lowstate_timeout_s:
                raise TimeoutError(
                    f"[G1_23_ArmController] No rt/lowstate after {lowstate_timeout_s:.0f}s -- "
                    "no G1 (or DDS sim bridge) answering on this domain. Check the robot/DDS "
                    "peer is powered on and reachable, or use --dry-run for IK-only testing."
                )
            time.sleep(0.1)
            if time.time() - last_log > 2.0:
                logger.warning("[G1_23_ArmController] Waiting to subscribe dds...")
                last_log = time.time()
        logger.info("[G1_23_ArmController] Subscribe dds ok.")

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger.info("Current all body motor state q:\n%s", self.all_motor_q)
        logger.info("Current two arms motor state q:\n%s", self.get_current_dual_arm_q())
        logger.info("Lock all joints except two arms...")

        arm_indices = set(member.value for member in G1_23_JointArmIndex)
        for id in G1_23_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q = self.all_motor_q[id]
        logger.info("Lock OK!")

        self.publish_thread = threading.Thread(
            target=self._ctrl_motor_state, daemon=True
        )
        self.ctrl_lock = threading.Lock()
        self.publish_thread.start()
        logger.info("Initialize G1_23_ArmController OK!")

    def _subscribe_motor_state(self):
        while self._running:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_23_LowState()
                for id in range(G1_23_Num_Motors):
                    lowstate.motor_state[id].q = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        return current_q + delta / max(motion_scale, 1.0)

    def _ctrl_motor_state(self):
        if self.motion_mode:
            self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = 1.0

        while self._running:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target = self.q_target
                arm_tauff_target = self.tauff_target

            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(
                    arm_q_target, velocity_limit=self.arm_velocity_limit
                )

            for idx, id in enumerate(G1_23_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / 5.0))

            sleep_time = max(0, self.control_dt - (time.time() - start_time))
            time.sleep(sleep_time)

    def ctrl_dual_arm(self, q_target, tauff_target):
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        return self.lowstate_subscriber.Read().mode_machine

    def get_current_motor_q(self):
        return np.array(
            [self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointIndex]
        )

    def get_current_dual_arm_q(self):
        return np.array(
            [self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointArmIndex]
        )

    def get_current_dual_arm_dq(self):
        return np.array(
            [self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_23_JointArmIndex]
        )

    def ctrl_dual_arm_go_home(self):
        logger.info("[G1_23_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(G1_23_ARM_DOF)
        tolerance = 0.05
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                if self.motion_mode:
                    for weight in np.linspace(1, 0, num=101):
                        self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = weight
                        time.sleep(0.02)
                logger.info("[G1_23_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def speed_gradual_max(self, t=5.0):
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        self.arm_velocity_limit = 30.0

    def shutdown(self):
        self._running = False
        if self.motion_mode:
            for weight in np.linspace(1, 0, num=51):
                self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = weight
                self.msg.crc = self.crc.Crc(self.msg)
                self.lowcmd_publisher.Write(self.msg)
                time.sleep(0.02)

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            G1_23_JointIndex.kLeftAnklePitch.value,
            G1_23_JointIndex.kRightAnklePitch.value,
            G1_23_JointIndex.kLeftShoulderPitch.value,
            G1_23_JointIndex.kLeftShoulderRoll.value,
            G1_23_JointIndex.kLeftShoulderYaw.value,
            G1_23_JointIndex.kLeftElbow.value,
            G1_23_JointIndex.kRightShoulderPitch.value,
            G1_23_JointIndex.kRightShoulderRoll.value,
            G1_23_JointIndex.kRightShoulderYaw.value,
            G1_23_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            G1_23_JointIndex.kLeftWristRoll.value,
            G1_23_JointIndex.kRightWristRoll.value,
        ]
        return motor_index.value in wrist_motors
