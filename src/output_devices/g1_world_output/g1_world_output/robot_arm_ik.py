"""
Unitree G1_23 dual-arm IK (Pinocchio + Casadi).

5 DoF per arm (shoulder pitch/roll/yaw, elbow, wrist roll).
URDF / meshes from g1_wuji2_description; wrist pitch/yaw (+ hands/legs/waist)
are locked so the reduced model matches G1_23.
"""

from __future__ import annotations

import logging
from pathlib import Path

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin

from g1_world_output.weighted_moving_filter import WeightedMovingFilter

logger = logging.getLogger(__name__)

G1_23_ARM_DOF = 10

# Lock everything except the 10 G1_23 arm joints when using g1_23_wuji2.urdf.
_G1_23_LOCK_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # G1_23 has no wrist pitch/yaw — lock them on the full body URDF
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    # Hands
    "left_wuji_l_thumb_cmc_flex",
    "left_wuji_l_thumb_cmc_abd",
    "left_wuji_l_thumb_mcp",
    "left_wuji_l_thumb_ip",
    "left_wuji_l_index_finger_mcp_flex",
    "left_wuji_l_index_finger_mcp_abd",
    "left_wuji_l_index_finger_pip",
    "left_wuji_l_index_finger_dip",
    "left_wuji_l_middle_finger_mcp_flex",
    "left_wuji_l_middle_finger_mcp_abd",
    "left_wuji_l_middle_finger_pip",
    "left_wuji_l_middle_finger_dip",
    "left_wuji_l_ring_finger_mcp_flex",
    "left_wuji_l_ring_finger_mcp_abd",
    "left_wuji_l_ring_finger_pip",
    "left_wuji_l_ring_finger_dip",
    "left_wuji_l_pinky_mcp_flex",
    "left_wuji_l_pinky_mcp_abd",
    "left_wuji_l_pinky_pip",
    "left_wuji_l_pinky_dip",
    "right_wuji_r_thumb_cmc_flex",
    "right_wuji_r_thumb_cmc_abd",
    "right_wuji_r_thumb_mcp",
    "right_wuji_r_thumb_ip",
    "right_wuji_r_index_finger_mcp_flex",
    "right_wuji_r_index_finger_mcp_abd",
    "right_wuji_r_index_finger_pip",
    "right_wuji_r_index_finger_dip",
    "right_wuji_r_middle_finger_mcp_flex",
    "right_wuji_r_middle_finger_mcp_abd",
    "right_wuji_r_middle_finger_pip",
    "right_wuji_r_middle_finger_dip",
    "right_wuji_r_ring_finger_mcp_flex",
    "right_wuji_r_ring_finger_mcp_abd",
    "right_wuji_r_ring_finger_pip",
    "right_wuji_r_ring_finger_dip",
    "right_wuji_r_pinky_mcp_flex",
    "right_wuji_r_pinky_mcp_abd",
    "right_wuji_r_pinky_pip",
    "right_wuji_r_pinky_dip",
]


class G1_23_ArmIK:
    def __init__(
        self,
        urdf_path: str | Path | None = None,
        mesh_dir: str | Path | None = None,
        Visualization: bool = False,
    ):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.Visualization = Visualization

        if urdf_path is None:
            raise ValueError("urdf_path is required for G1_23_ArmIK")
        urdf_path = Path(urdf_path)
        if not urdf_path.exists():
            raise FileNotFoundError(f"G1 URDF not found: {urdf_path}")
        if mesh_dir is None:
            mesh_dir = urdf_path.parent
        mesh_dir = Path(mesh_dir)

        self.robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path), str(mesh_dir))

        model_joint_names = {
            self.robot.model.names[i] for i in range(self.robot.model.njoints)
        }
        self.mixed_jointsToLockIDs = [
            name for name in _G1_23_LOCK_JOINTS if name in model_joint_names
        ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # G1_23 EE frame on wrist_roll with 0.20 m forward offset (matches xr_teleoperate).
        ee_offset = np.array([0.20, 0.0, 0.0]).T
        self.reduced_robot.model.addFrame(
            pin.Frame(
                'L_ee',
                self.reduced_robot.model.getJointId('left_wrist_roll_joint'),
                pin.SE3(np.eye(3), ee_offset),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.model.addFrame(
            pin.Frame(
                'R_ee',
                self.reduced_robot.model.getJointId('right_wrist_roll_joint'),
                pin.SE3(np.eye(3), ee_offset),
                pin.FrameType.OP_FRAME,
            )
        )

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3, 3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3, 3],
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(
                        self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3, :3].T
                    ),
                    cpin.log3(
                        self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3, :3].T
                    ),
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(
            self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        self.rotation_cost = casadi.sumsqr(
            self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        # G1_23 cost weights from xr_teleoperate (rotation down-weighted vs G1_29)
        self.opti.minimize(
            50 * self.translational_cost
            + 0.5 * self.rotation_cost
            + 0.02 * self.regularization_cost
            + 0.1 * self.smooth_cost
        )

        opts = {
            'ipopt': {'print_level': 0, 'max_iter': 50, 'tol': 1e-6},
            'print_time': False,
            'calc_lam_p': False,
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1]), G1_23_ARM_DOF
        )
        self.vis = None

        if self.reduced_robot.model.nq != G1_23_ARM_DOF:
            logger.warning(
                "Expected reduced nq=%d for G1_23, got %d (locked %d joints)",
                G1_23_ARM_DOF,
                self.reduced_robot.model.nq,
                len(self.mixed_jointsToLockIDs),
            )

    def solve_ik(
        self,
        left_wrist,
        right_wrist,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
    ):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            v = np.zeros(self.reduced_robot.model.nv)
            self.init_data = sol_q
            sol_tauff = pin.rnea(
                self.reduced_robot.model,
                self.reduced_robot.data,
                sol_q,
                v,
                np.zeros(self.reduced_robot.model.nv),
            )
            return sol_q, sol_tauff, True

        except Exception as e:
            logger.error("ERROR in IK convergence: %s", e)
            if current_lr_arm_motor_q is not None:
                return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv), False
            return self.init_data.copy(), np.zeros(self.reduced_robot.model.nv), False
