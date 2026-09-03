#!/usr/bin/env python3
"""
G1 Cartesian controller — remaps chest poses, solves dual-arm IK, commands DDS.
G1_23: 5 DoF per arm (10 total).
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import numpy as np

from g1_world_output.config_loader import G1Config
from g1_world_output.robot_arm import (
    ARM_JOINT_NAMES_BY_TYPE,
    G1ArmController,
)
from g1_world_output.robot_arm_ik import G1_23_ArmIK
from g1_world_output.transform_utils import chest_pose_to_pelvis


class G1CartesianController:
    """Mirror of Tianji CartesianController API, backed by Unitree G1_23 stack.

    arm_type selects the arm variant: 'G1_23' (5 DoF/arm) or 'G1_29'
    (7 DoF/arm, the rig's robot). Both drive DDS through G1ArmController.
    The pose-IK path is still G1_23-only (10-DoF reduced model), so pose
    mode with G1_29 raises; joint_replay needs no IK.
    """

    def __init__(
        self,
        config: Optional[G1Config] = None,
        motion_mode: Optional[bool] = None,
        simulation_mode: Optional[bool] = None,
        logger=None,
        connect: bool = True,
        arm_type: Optional[str] = None,
    ):
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self._cfg = config if config is not None else G1Config.load()

        self.arm_type = arm_type or self._cfg.arm_type
        if self.arm_type not in ARM_JOINT_NAMES_BY_TYPE:
            raise ValueError(
                f"arm_type must be one of {sorted(ARM_JOINT_NAMES_BY_TYPE)}, "
                f"got {self.arm_type!r}"
            )
        self._joint_names = ARM_JOINT_NAMES_BY_TYPE[self.arm_type]
        self._dof_side = len(self._joint_names) // 2

        motion_mode = self._cfg.motion_mode if motion_mode is None else motion_mode
        simulation_mode = (
            self._cfg.simulation_mode if simulation_mode is None else simulation_mode
        )

        # Lazy: only 'pose' mode needs Pinocchio+CasADi. Built on first use
        # in _ensure_arm_ik() so 'joint_replay'/'idle' modes never pay for
        # (or can fail on) the URDF/IK load.
        self.arm_ik: Optional[G1_23_ArmIK] = None

        self.arm_ctrl: Optional[G1ArmController] = None
        if connect:
            self.logger.info(
                "Connecting %s arm DDS (motion_mode=%s, simulation_mode=%s)...",
                self.arm_type,
                motion_mode,
                simulation_mode,
            )
            self.arm_ctrl = G1ArmController(
                motion_mode=motion_mode,
                simulation_mode=simulation_mode,
                arm_type=self.arm_type,
                network_interface=self._cfg.network_interface,
            )

        left_zsp = self._cfg.default_zsp_para.get('left', [0, -1, -0.5, 0, 0, 0])
        right_zsp = self._cfg.default_zsp_para.get('right', [0, 1, -0.5, 0, 0, 0])
        self.left_zsp_para = list(left_zsp)
        self.right_zsp_para = list(right_zsp)

        self._last_sol_q = np.zeros(2 * self._dof_side)
        self.logger.info(
            "%s dual-arm controller ready (nq=%d)", self.arm_type, 2 * self._dof_side
        )

    def _send_dual_arm(self, q: np.ndarray, tau: np.ndarray) -> None:
        """Single choke point for writing a 10-DoF joint target to DDS.

        Every path that ends up commanding the arms -- IK-solved poses or
        already-solved joint angles from a replayed trajectory -- goes
        through here, so there is exactly one place that talks to
        G1ArmController and exactly one place that updates the IK
        seed (_last_sol_q).
        """
        if self.arm_ctrl is not None:
            self.arm_ctrl.ctrl_dual_arm(q, tau)
        self._last_sol_q = q

    def _ensure_arm_ik(self) -> G1_23_ArmIK:
        if self.arm_ik is None:
            urdf_path = self._cfg.get_urdf_path()
            self.logger.info("Loading G1_23 IK from URDF: %s", urdf_path)
            self.arm_ik = G1_23_ArmIK(urdf_path=urdf_path, mesh_dir=self._cfg.urdf_package_dir)
        return self.arm_ik

    def get_current_joints(self) -> Tuple[Optional[list], Optional[list]]:
        """Return (left, right) per-side joint angles in radians."""
        if self.arm_ctrl is None:
            return None, None
        q = self.arm_ctrl.get_current_dual_arm_q()
        d = self._dof_side
        return list(q[:d]), list(q[d:])

    def move_to_init(self, wait: bool = True, timeout: float = 3.0) -> bool:
        """Move both arms to configured reset wrist poses via IK."""
        if self.arm_type != 'G1_23':
            raise NotImplementedError(
                f"pose-IK path (move_to_init) is G1_23-only; arm_type={self.arm_type}"
            )
        left_T = self._cfg.get_reset_wrist_matrix('left')
        right_T = self._cfg.get_reset_wrist_matrix('right')

        if self.arm_ctrl is not None:
            seed_q = self.arm_ctrl.get_current_dual_arm_q()
            seed_dq = self.arm_ctrl.get_current_dual_arm_dq()
        else:
            seed_q, seed_dq = None, None

        sol_q, sol_tau, ok = self._ensure_arm_ik().solve_ik(left_T, right_T, seed_q, seed_dq)
        if not ok:
            self.logger.warning("Reset IK failed; leaving arm at its current position")
        # Consistent with move_to_pose_direct: never send a failed solve to
        # the arm, so a broken IK config can't jerk the robot on startup.
        if ok:
            self._send_dual_arm(sol_q, sol_tau)
            if wait and self.arm_ctrl is not None:
                import time
                time.sleep(timeout)
        self.logger.info("Moved toward G1_23 reset wrist poses (ok=%s)", ok)
        return ok

    def move_to_pose_direct(
        self,
        left_pose,
        right_pose,
        unit: str = 'matrix',
    ) -> Tuple[bool, bool, Optional[list], Optional[list]]:
        """
        Remap chest-frame poses -> pelvis, solve dual-arm IK, send DDS command.

        Returns:
            (left_success, right_success, left_joints_rad[5], right_joints_rad[5])
        """
        if unit != 'matrix':
            raise ValueError("g1_world_output only supports unit='matrix'")
        if self.arm_type != 'G1_23':
            raise NotImplementedError(
                f"pose-IK path (move_to_pose_direct) is G1_23-only; "
                f"arm_type={self.arm_type}"
            )

        left_pelvis = (
            chest_pose_to_pelvis(left_pose, 'left')
            if left_pose is not None
            else self._cfg.get_reset_wrist_matrix('left')
        )
        right_pelvis = (
            chest_pose_to_pelvis(right_pose, 'right')
            if right_pose is not None
            else self._cfg.get_reset_wrist_matrix('right')
        )

        if self.arm_ctrl is not None:
            current_q = self.arm_ctrl.get_current_dual_arm_q()
            current_dq = self.arm_ctrl.get_current_dual_arm_dq()
        else:
            current_q = self._last_sol_q
            current_dq = np.zeros(2 * self._dof_side)

        sol_q, sol_tau, ok = self._ensure_arm_ik().solve_ik(
            left_pelvis, right_pelvis, current_q, current_dq
        )

        if ok:
            self._send_dual_arm(sol_q, sol_tau)

        d = self._dof_side
        left_joints = list(sol_q[:d]) if (ok and left_pose is not None) else None
        right_joints = list(sol_q[d:]) if (ok and right_pose is not None) else None
        left_ok = ok and left_pose is not None
        right_ok = ok and right_pose is not None
        return left_ok, right_ok, left_joints, right_joints

    def move_to_joints_direct(
        self,
        left_q: Optional[list] = None,
        right_q: Optional[list] = None,
        left_tau: Optional[list] = None,
        right_tau: Optional[list] = None,
    ) -> Tuple[bool, bool]:
        """
        Send already-solved joint angles straight to DDS -- no IK.

        For sources that ship joint angles directly (e.g. a replayed
        reference trajectory) rather than end-effector poses. Merges with
        the arm's current position (measured if connected, else the last
        solve) for whichever side is not provided, then hands off to the
        same _send_dual_arm() choke point move_to_pose_direct() uses -- the
        DDS write loop's velocity clamp (G1ArmController.clip_arm_q_target)
        still applies either way. Works in dry-run (arm_ctrl is None) the
        same way move_to_pose_direct() does, so joint_replay mode is usable
        without hardware.

        Returns:
            (left_success, right_success)
        """
        d = self._dof_side
        if self.arm_ctrl is not None:
            q = np.array(self.arm_ctrl.get_current_dual_arm_q(), copy=True)
        else:
            q = np.array(self._last_sol_q, copy=True)
        tau = np.zeros(2 * d)
        left_ok = left_q is not None
        right_ok = right_q is not None
        if left_ok:
            q[:d] = left_q
            if left_tau is not None:
                tau[:d] = left_tau
        if right_ok:
            q[d:] = right_q
            if right_tau is not None:
                tau[d:] = right_tau

        if left_ok or right_ok:
            self._send_dual_arm(q, tau)
        return left_ok, right_ok

    def joint_names(self, side: str) -> list:
        if side == 'left':
            return self._joint_names[:self._dof_side]
        return self._joint_names[self._dof_side:]

    def disable_and_release(self):
        self.logger.info("Shutting down %s arm controller...", self.arm_type)
        if self.arm_ctrl is not None:
            try:
                self.arm_ctrl.shutdown()
            except Exception as exc:
                self.logger.warning("G1 shutdown error (ignored): %s", exc)
        self.logger.info("%s arm controller released", self.arm_type)
