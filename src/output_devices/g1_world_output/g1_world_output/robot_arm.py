"""
Unitree G1 arm DDS controller, plus the shared arm joint-name tables.

G1ArmController drives either arm variant over Unitree SDK2 DDS LowCmd /
LowState: G1_23 (5 DoF per arm, 10 total) or G1_29 (7 DoF per arm, 14
total; adds wrist pitch/yaw at unified-motor-array indices 20/21, 27/28).
Unitree's motor array is indexed identically for both variants, so the
variant only selects which slots are arm joints and which get wrist gains.

Adapted from Unitree's public G1 arm DDS example code (unitreerobotics/
unitree_sdk2_python / xr_teleoperate reference scripts); the spec_1
hardware-replay rework replaced the init loop with the slot policy below,
made the write-thread velocity clip per-joint and always-on, moved gains to
config, added the extended lowstate mirror, and added read-only mode.

LowCmd slot policy (spec_1 component 3, pinned):
  - rt/arm_sdk: write the arm slots (15-28 on the 29) and the weight slot
    (29). Nothing else. Slots 0-14 and 30-34 are never written and stay at
    constructor defaults (kp = kd = 0, inert). Per-motor `mode` is never
    set (the vendor arm7 example never sets it). mode_machine is copied
    from lowstate; mode_pr is 0. The waist is uncommanded: holding it at
    kp 300 would put our position loop in contention with the balance
    controller (TUITION 2.3). Stage A confirms arm_sdk holds the waist with
    its slots unwritten.
  - rt/lowcmd: not used by this design, and REFUSED at construction. It
    requires releasing the onboard controller and owning all 29 motors
    every cycle (a suspended-robot regime), and the write-all-35 init that
    made it physically coherent was removed with the slot policy.

Weight (slot 29 q): owned by the caller's state machine via set_weight();
starts at 0 and is never stepped by this class. Engage/release ramps are
the device FSM's job (>= 2 s each, spec_1 section 8).
"""

from __future__ import annotations

import fcntl
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

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

from g1_world_output.joint_tables import (  # noqa: F401 (re-exported)
    G1_23_ARM_JOINT_NAMES,
    G1_29_ARM_JOINT_NAMES,
)
from g1_world_output.replay_safety import rate_limit_step

logger = logging.getLogger(__name__)

kTopicLowCommand_Debug = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

# Only one process may ever write rt/lowcmd/rt/arm_sdk. This turns a second
# writer (e.g. someone accidentally starting g1_world_output twice, or a
# leftover process from an old design with its own DDS writer) into a loud
# startup failure instead of two processes silently interleaving commands.
# A read-only instance writes nothing and deliberately does NOT take this
# lock, so Stage A observation can run beside nothing or beside a writer.
LOWCMD_WRITER_LOCK_PATH = "/tmp/g1_lowcmd_writer.lock"

G1_23_Num_Motors = 35
G1_23_ARM_DOF = 10  # 5 per arm


class MotorState:
    __slots__ = ('q', 'dq', 'tau_est', 'temperature', 'vol')

    def __init__(self):
        self.q = 0.0
        self.dq = 0.0
        self.tau_est = 0.0
        self.temperature = 0
        self.vol = 0.0


class G1LowStateMirror:
    """One received lowstate frame, everything downstream consumers need.

    The old mirror kept only q/dq; engage gating, staleness holds,
    lowstate-loss reset, /g1/status age, joint_states effort, and /g1/imu
    all need more (spec_1 component 3), so the subscribe thread copies it
    here with a monotonic receive time.
    """

    __slots__ = ('motor_state', 'tick', 'mode_machine', 'imu_quaternion',
                 'imu_gyroscope', 'imu_accelerometer', 'imu_rpy',
                 'receive_monotonic')

    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_23_Num_Motors)]
        self.tick = 0
        self.mode_machine = 0
        self.imu_quaternion = (0.0, 0.0, 0.0, 0.0)
        self.imu_gyroscope = (0.0, 0.0, 0.0)
        self.imu_accelerometer = (0.0, 0.0, 0.0)
        self.imu_rpy = (0.0, 0.0, 0.0)
        self.receive_monotonic = 0.0


# Backwards-compatible alias (older scripts referenced G1_23_LowState).
G1_23_LowState = G1LowStateMirror


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


class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristYaw = 21

    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


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



# Name tables live in joint_tables.py (import-light; the replay package's
# parity test guards its own copy against them) and are re-exported here.
# G1_29: 7 DoF per arm (adds wrist pitch/yaw at slots 20/21 and 27/28,
# which the 23-DoF enum above marks NotUsed). Since aae4638
# G1ArmController(arm_type='G1_29') drives these slots over rt/arm_sdk;
# pose IK remains G1_23-only.

ARM_JOINT_NAMES_BY_TYPE = {
    'G1_23': G1_23_ARM_JOINT_NAMES,
    'G1_29': G1_29_ARM_JOINT_NAMES,
}

# Motor-array slots per variant, same order as the name tables above.
ARM_INDICES_BY_TYPE = {
    'G1_23': [m.value for m in G1_23_JointArmIndex],
    'G1_29': [m.value for m in G1_29_JointArmIndex],
}

# Wrist slots get the soft kp/kd tier: 1 wrist joint per arm on the 23,
# 3 per arm on the 29.
WRIST_MOTORS_BY_TYPE = {
    'G1_23': {
        G1_23_JointIndex.kLeftWristRoll.value,
        G1_23_JointIndex.kRightWristRoll.value,
    },
    'G1_29': {
        G1_23_JointIndex.kLeftWristRoll.value,
        G1_23_JointIndex.kLeftWristPitchNotUsed.value,
        G1_23_JointIndex.kLeftWristyawNotUsed.value,
        G1_23_JointIndex.kRightWristRoll.value,
        G1_23_JointIndex.kRightWristPitchNotUsed.value,
        G1_23_JointIndex.kRightWristYawNotUsed.value,
    },
}


@dataclass
class ArmGains:
    """kp/kd tiers for the arm slots (config g1_robot.yaml `gains:`).

    Only arm slots are ever written under the slot policy, so the old hold
    tier (300/5 on non-arm slots) has no consumer and no longer exists.
    """

    arm_kp: float = 140.0
    arm_kd: float = 3.0
    wrist_kp: float = 50.0
    wrist_kd: float = 2.0


class G1ArmController:
    def __init__(
        self,
        motion_mode: bool = True,
        simulation_mode: bool = False,
        dds_already_initialized: bool = False,
        arm_type: str = 'G1_23',
        read_only: bool = False,
        network_interface: Optional[str] = None,
        gains: Optional[ArmGains] = None,
        vel_ceilings: Optional[np.ndarray] = None,
    ):
        """
        read_only: subscribe lowstate only. No writer lock, no publisher,
            no write thread, weight untouched. Required for Stage A (7A).
        network_interface: NIC for the Unitree DDS participant. The SDK
            builds its own CycloneDDS config and ignores CYCLONEDDS_URI, so
            on a multi-NIC host this parameter is the only way to pin the
            robot link.
        vel_ceilings: per-joint hardware velocity ceilings (rad/s), one per
            arm joint in name-table order, from g1_deploy_limits.yaml. The
            250 Hz write-thread clip runs against these, per joint, always
            (simulation_mode included). Required unless read_only.
        """
        if arm_type not in ARM_INDICES_BY_TYPE:
            raise ValueError(
                f"arm_type must be one of {sorted(ARM_INDICES_BY_TYPE)}, got {arm_type!r}"
            )
        self.arm_type = arm_type
        self._arm_indices = ARM_INDICES_BY_TYPE[arm_type]
        self._arm_dof = len(self._arm_indices)
        self._wrist_motors = WRIST_MOTORS_BY_TYPE[arm_type]
        self.read_only = read_only
        self.gains = gains or ArmGains()
        logger.info("Initialize G1ArmController (%s, %d arm DoF%s)...",
                    arm_type, self._arm_dof, ', READ-ONLY' if read_only else '')

        if not read_only:
            if vel_ceilings is None:
                raise ValueError(
                    "vel_ceilings is required for a writing controller: the "
                    "per-joint DDS clip has no defaults. Load them from "
                    "g1_deploy_limits.yaml (hardware_ceilings velocity rows)."
                )
            vel_ceilings = np.asarray(vel_ceilings, dtype=float)
            if vel_ceilings.shape != (self._arm_dof,) or np.any(vel_ceilings <= 0):
                raise ValueError(
                    f"vel_ceilings must be {self._arm_dof} positive values, "
                    f"got {vel_ceilings}"
                )
        self._vel_ceilings = vel_ceilings

        self._lock_file = None
        if not read_only:
            self._lock_file = open(LOWCMD_WRITER_LOCK_PATH, "w")
            try:
                fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self._lock_file.close()
                raise RuntimeError(
                    "Another process already holds the G1 DDS lowcmd writer lock "
                    f"({LOWCMD_WRITER_LOCK_PATH}). Only one process may write "
                    "rt/lowcmd/rt/arm_sdk at a time -- stop it before starting this one."
                )

        self.q_target = np.zeros(self._arm_dof)
        self.tauff_target = np.zeros(self._arm_dof)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self._weight = 0.0

        self.control_dt = 1.0 / 250.0
        self._running = True
        # First write-tick failure, surfaced on /g1/status (the thread holds
        # the previous frame and keeps running; it never dies silently).
        self.write_fault_reason: Optional[str] = None
        self.write_fault_count = 0

        if not dds_already_initialized:
            domain = 1 if self.simulation_mode else 0
            if network_interface:
                ChannelFactoryInitialize(domain, network_interface)
            else:
                ChannelFactoryInitialize(domain)

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
                    f"[G1ArmController] No rt/lowstate after {lowstate_timeout_s:.0f}s -- "
                    "no G1 (or DDS sim bridge) answering on this domain. Check the robot/DDS "
                    "peer is powered on and reachable, or use --dry-run for offline testing."
                )
            time.sleep(0.1)
            if time.time() - last_log > 2.0:
                logger.warning("[G1ArmController] Waiting to subscribe dds...")
                last_log = time.time()
        logger.info("[G1ArmController] Subscribe dds ok.")

        if read_only:
            self.lowcmd_publisher = None
            self.publish_thread = None
            self.ctrl_lock = threading.Lock()
            logger.info("Initialize G1ArmController OK (read-only, no writer).")
            return

        if not self.motion_mode:
            # rt/lowcmd releases the onboard controller and owns all 29
            # motors every cycle: a suspended-robot regime this design does
            # not use (spec_1 slot policy). The old write-all-35 init that
            # made it physically coherent (per-motor mode 1, hold gains on
            # legs and waist) was removed with the slot policy, so a lowcmd
            # session here would leave the lower body limp on the bus.
            # Refusing is safer than half-working.
            raise RuntimeError(
                "motion_mode=False (rt/lowcmd) is not supported by the "
                "replay design; use rt/arm_sdk (motion_mode: true). A "
                "suspended-robot lowcmd session needs its own full-bus "
                "controller, not this one."
            )
        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        self.lowcmd_publisher.Init()

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        # Slot policy: seed kp/kd + measured q on the ARM slots only. All
        # other slots keep constructor defaults (mode 0, kp = kd = 0 --
        # inert), and per-motor mode is never set anywhere.
        measured = self.get_current_dual_arm_q()
        for idx, slot in enumerate(self._arm_indices):
            cmd = self.msg.motor_cmd[slot]
            if slot in self._wrist_motors:
                cmd.kp, cmd.kd = self.gains.wrist_kp, self.gains.wrist_kd
            else:
                cmd.kp, cmd.kd = self.gains.arm_kp, self.gains.arm_kd
            cmd.q = float(measured[idx])
        # Weight slot carries q only (no gains); starts released.
        self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = 0.0

        # The write thread starts against measured targets and zero weight,
        # so its first cycles command exactly where the arm already is with
        # no authority -- never toward zeros (a startup yank in the old code).
        self.q_target = np.asarray(measured, dtype=float).copy()

        self.publish_thread = threading.Thread(
            target=self._ctrl_motor_state, daemon=True
        )
        self.ctrl_lock = threading.Lock()
        self.publish_thread.start()
        logger.info("Initialize G1ArmController OK (slot policy: arm slots + weight only).")

    # ------------------------------------------------------------ threads

    def _subscribe_motor_state(self):
        while self._running:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                mirror = G1LowStateMirror()
                for i in range(G1_23_Num_Motors):
                    src = msg.motor_state[i]
                    dst = mirror.motor_state[i]
                    dst.q = src.q
                    dst.dq = src.dq
                    dst.tau_est = src.tau_est
                    dst.temperature = max(src.temperature)
                    dst.vol = src.vol
                mirror.tick = msg.tick
                mirror.mode_machine = msg.mode_machine
                imu = msg.imu_state
                mirror.imu_quaternion = tuple(imu.quaternion)
                mirror.imu_gyroscope = tuple(imu.gyroscope)
                mirror.imu_accelerometer = tuple(imu.accelerometer)
                mirror.imu_rpy = tuple(imu.rpy)
                mirror.receive_monotonic = time.monotonic()
                self.lowstate_buffer.SetData(mirror)
            time.sleep(0.002)

    def _ctrl_motor_state(self):
        while self._running:
            start_time = time.monotonic()

            try:
                with self.ctrl_lock:
                    arm_q_target = self.q_target.copy()
                    arm_tauff_target = self.tauff_target.copy()
                    weight = self._weight

                # Per-joint hardware-ceiling clip, ALWAYS on (spec_1: the
                # old uniform 20 rad/s clip was skipped entirely in
                # simulation_mode, and there is exactly one write path
                # either way).
                current_q = self.get_current_dual_arm_q()
                clipped = rate_limit_step(
                    current_q, arm_q_target, self._vel_ceilings, self.control_dt
                )

                for idx, slot in enumerate(self._arm_indices):
                    self.msg.motor_cmd[slot].q = float(clipped[idx])
                    self.msg.motor_cmd[slot].dq = 0.0
                    self.msg.motor_cmd[slot].tau = float(arm_tauff_target[idx])
                self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = float(weight)
            except Exception as exc:  # noqa: BLE001
                # The write thread must NEVER die silently: a dead writer
                # at weight 1 is a hold nobody chose and a release ramp
                # nobody can execute. A bad tick (e.g. a NaN in a lowstate
                # frame making rate_limit_step raise) keeps publishing the
                # PREVIOUS frame unchanged -- hold, never zero -- and the
                # fault is surfaced on /g1/status via write_fault_reason.
                if self.write_fault_reason is None:
                    self.write_fault_reason = f'{type(exc).__name__}: {exc}'
                    logger.error('[G1ArmController] write tick failed; '
                                 'holding previous frame: %s', exc)
                self.write_fault_count += 1

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            sleep_time = max(0.0, self.control_dt - (time.monotonic() - start_time))
            time.sleep(sleep_time)

    # ----------------------------------------------------------- commands

    def ctrl_dual_arm(self, q_target, tauff_target):
        if self.read_only:
            raise RuntimeError("read-only controller: commanding is disabled")
        with self.ctrl_lock:
            self.q_target = np.asarray(q_target, dtype=float).copy()
            self.tauff_target = np.asarray(tauff_target, dtype=float).copy()

    def set_weight(self, weight: float) -> None:
        """Raw arm_sdk weight setter. Ramps (>= 2 s engage/release) are the
        device FSM's responsibility; this only clamps and stores."""
        if self.read_only:
            raise RuntimeError("read-only controller: weight is untouchable")
        with self.ctrl_lock:
            self._weight = float(min(max(weight, 0.0), 1.0))

    def get_weight(self) -> float:
        with self.ctrl_lock:
            return self._weight

    # ------------------------------------------------------------ getters

    def _mirror(self) -> Optional[G1LowStateMirror]:
        return self.lowstate_buffer.GetData()

    def get_mode_machine(self):
        m = self._mirror()
        return m.mode_machine if m is not None else None

    def lowstate_age(self) -> Optional[float]:
        """Seconds since the last lowstate frame arrived; None = never."""
        m = self._mirror()
        if m is None:
            return None
        return time.monotonic() - m.receive_monotonic

    def get_lowstate_tick(self) -> Optional[int]:
        m = self._mirror()
        return m.tick if m is not None else None

    def get_imu(self) -> Optional[dict]:
        m = self._mirror()
        if m is None:
            return None
        return {
            'quaternion': m.imu_quaternion,       # (w, x, y, z), Unitree order
            'gyroscope': m.imu_gyroscope,
            'accelerometer': m.imu_accelerometer,
            'rpy': m.imu_rpy,
        }

    def get_current_motor_q(self):
        m = self._mirror()
        return np.array([m.motor_state[i].q for i in range(G1_23_Num_Motors)])

    def get_current_dual_arm_q(self):
        m = self._mirror()
        return np.array([m.motor_state[i].q for i in self._arm_indices])

    def get_current_dual_arm_dq(self):
        m = self._mirror()
        return np.array([m.motor_state[i].dq for i in self._arm_indices])

    def get_current_dual_arm_tau(self):
        m = self._mirror()
        return np.array([m.motor_state[i].tau_est for i in self._arm_indices])

    def get_arm_max_temperature(self) -> Optional[float]:
        m = self._mirror()
        if m is None:
            return None
        return float(max(m.motor_state[i].temperature for i in self._arm_indices))

    def get_arm_min_voltage(self) -> Optional[float]:
        """Min per-motor bus voltage over the arm slots (LowState has no
        pack-voltage field; motor `vol` is the closest published signal)."""
        m = self._mirror()
        if m is None:
            return None
        return float(min(m.motor_state[i].vol for i in self._arm_indices))

    # ----------------------------------------------------------- shutdown

    def shutdown(self):
        if not self.read_only and self.publish_thread is not None:
            # Release through the running write thread: ramp the weight to
            # zero over >= 2 s (the spec's release floor) while still
            # commanding the current target. If the daemon writer dies
            # mid-ramp (interpreter teardown, descheduling on SIGTERM),
            # fall back to writing the ramp frames directly -- with the
            # thread dead there is no writer to race, and leaving the last
            # on-wire frame at weight ~1 would hand back full authority as
            # an instant step when arm_sdk times out.
            try:
                start_w = self.get_weight()
                if start_w > 0.0:
                    steps = 101  # 101 * 0.02 s > 2 s
                    for w in np.linspace(start_w, 0.0, steps):
                        self.set_weight(float(w))
                        if not self.publish_thread.is_alive():
                            self.msg.motor_cmd[
                                G1_23_JointIndex.kNotUsedJoint0].q = float(w)
                            self.msg.crc = self.crc.Crc(self.msg)
                            self.lowcmd_publisher.Write(self.msg)
                        time.sleep(0.02)
            except Exception:
                logger.warning("weight ramp-down failed (ignored)")
        self._running = False
        if self.publish_thread is not None:
            self.publish_thread.join(timeout=1.0)
        if self._lock_file is not None:
            try:
                fcntl.flock(self._lock_file, fcntl.LOCK_UN)
                self._lock_file.close()
            except Exception:
                logger.warning("Failed to release G1 DDS lowcmd writer lock (ignored)")
