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
    G1_23_ARM_DOF,
    G1_23_ARM_JOINT_NAMES,
    G1_23_ArmController,
)
from g1_world_output.robot_arm_ik import G1_23_ArmIK
from g1_world_output.transform_utils import chest_pose_to_pelvis


class G1CartesianController:
    """Mirror of Tianji CartesianController API, backed by Unitree G1_23 stack."""

    def __init__(
        self,
        config: Optional[G1Config] = None,
        motion_mode: Optional[bool] = None,
        simulation_mode: Optional[bool] = None,
        logger=None,
        connect: bool = True,
    ):
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self._cfg = config if config is not None else G1Config.load()

        motion_mode = self._cfg.motion_mode if motion_mode is None else motion_mode
        simulation_mode = (
            self._cfg.simulation_mode if simulation_mode is None else simulation_mode
        )

        urdf_path = self._cfg.get_urdf_path()
        self.logger.info("Loading G1_23 IK from URDF: %s", urdf_path)
        self.arm_ik = G1_23_ArmIK(urdf_path=urdf_path, mesh_dir=self._cfg.urdf_package_dir)

        self.arm_ctrl: Optional[G1_23_ArmController] = None
        if connect:
            self.logger.info(
                "Connecting G1_23 arm DDS (motion_mode=%s, simulation_mode=%s)...",
                motion_mode,
                simulation_mode,
            )
            self.arm_ctrl = G1_23_ArmController(
                motion_mode=motion_mode,
                simulation_mode=simulation_mode,
            )

        left_zsp = self._cfg.default_zsp_para.get('left', [0, -1, -0.5, 0, 0, 0])
        right_zsp = self._cfg.default_zsp_para.get('right', [0, 1, -0.5, 0, 0, 0])
        self.left_zsp_para = list(left_zsp)
        self.right_zsp_para = list(right_zsp)

        self._last_sol_q = np.zeros(G1_23_ARM_DOF)
        self.logger.info("G1_23 dual-arm controller ready (nq=%d)", G1_23_ARM_DOF)

    def get_current_joints(self) -> Tuple[Optional[list], Optional[list]]:
        """Return (left_5, right_5) joint angles in radians."""
        if self.arm_ctrl is None:
            return None, None
        q = self.arm_ctrl.get_current_dual_arm_q()
        return list(q[:5]), list(q[5:])

    def move_to_init(self, wait: bool = True, timeout: float = 3.0) -> bool:
        """Move both arms to configured reset wrist poses via IK."""
        left_T = self._cfg.get_reset_wrist_matrix('left')
        right_T = self._cfg.get_reset_wrist_matrix('right')

        if self.arm_ctrl is not None:
            seed_q = self.arm_ctrl.get_current_dual_arm_q()
            seed_dq = self.arm_ctrl.get_current_dual_arm_dq()
        else:
            seed_q, seed_dq = None, None

        sol_q, sol_tau, ok = self.arm_ik.solve_ik(left_T, right_T, seed_q, seed_dq)
        if not ok:
            self.logger.warning("Reset IK failed; leaving arm at its current position")
        # Consistent with move_to_pose_direct: never send a failed solve to
        # the arm, so a broken IK config can't jerk the robot on startup.
        if self.arm_ctrl is not None and ok:
            self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tau)
            if wait:
                import time
                time.sleep(timeout)
        if ok:
            self._last_sol_q = sol_q
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
            current_dq = np.zeros(G1_23_ARM_DOF)

        sol_q, sol_tau, ok = self.arm_ik.solve_ik(
            left_pelvis, right_pelvis, current_q, current_dq
        )

        if self.arm_ctrl is not None and ok:
            self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tau)

        if ok:
            self._last_sol_q = sol_q

        left_joints = list(sol_q[:5]) if (ok and left_pose is not None) else None
        right_joints = list(sol_q[5:]) if (ok and right_pose is not None) else None
        left_ok = ok and left_pose is not None
        right_ok = ok and right_pose is not None
        return left_ok, right_ok, left_joints, right_joints

    @staticmethod
    def joint_names(side: str) -> list:
        if side == 'left':
            return G1_23_ARM_JOINT_NAMES[:5]
        return G1_23_ARM_JOINT_NAMES[5:]

    def disable_and_release(self):
        self.logger.info("Shutting down G1_23 arm controller...")
        if self.arm_ctrl is not None:
            try:
                self.arm_ctrl.shutdown()
            except Exception as exc:
                self.logger.warning("G1 shutdown error (ignored): %s", exc)
        self.logger.info("G1_23 arm controller released")
