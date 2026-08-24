#!/usr/bin/env python3
"""
Coordinate remapping for G1 world output.

pico_input publishes wrist targets in Tianji-style chest frames
(world_left / world_right). G1 Pinocchio IK expects SE3 targets in the
pelvis / robot-base frame (ROS REP 103: +X forward, +Y left, +Z up).

Remap:
  T_pelvis_ee = T_pelvis_chest @ T_chest_ee

where T_pelvis_chest is a pure rotation (chest axes) plus the configured
chest-origin translation in the pelvis frame.
"""

from __future__ import annotations

import numpy as np

from .config_loader import get_config as _get_config


def _config():
    return _get_config()


def get_world_to_chest_rotation(side: str) -> np.ndarray:
    return _config().get_world_to_chest_rotation(side)


def get_chest_to_world_rotation(side: str) -> np.ndarray:
    return _config().get_chest_to_world_rotation(side)


def transform_world_to_chest(vector_world, side: str) -> np.ndarray:
    x, y, z = vector_world[0], vector_world[1], vector_world[2]
    if side == 'left':
        return np.array([x, -z, y])
    return np.array([x, z, -y])


def transform_chest_to_world(vector_chest, side: str) -> np.ndarray:
    x, y, z = vector_chest[0], vector_chest[1], vector_chest[2]
    if side == 'left':
        return np.array([x, z, -y])
    return np.array([x, -z, y])


def chest_pose_to_pelvis(T_chest: np.ndarray, side: str, arm_scale: float | None = None) -> np.ndarray:
    """Remap a 4x4 chest-frame EE pose into the G1 pelvis frame.

    Args:
        T_chest: 4x4 SE3 in chest frame (meters), as published on
                 /{side}_arm_target_pose.
        side: 'left' or 'right'
        arm_scale: optional override for config arm_scale

    Returns:
        4x4 SE3 in G1 pelvis / IK base frame (meters).
    """
    cfg = _config()
    if arm_scale is None:
        arm_scale = cfg.arm_scale

    R_c2w = get_chest_to_world_rotation(side)
    R_chest = T_chest[:3, :3]
    p_chest = T_chest[:3, 3] * float(arm_scale)

    T_pelvis = np.eye(4)
    T_pelvis[:3, :3] = R_c2w @ R_chest
    T_pelvis[:3, 3] = R_c2w @ p_chest + cfg.chest_origin_in_pelvis[side]
    return T_pelvis


def elbow_direction_chest_to_pelvis(direction_chest: np.ndarray, side: str) -> np.ndarray:
    """Map an elbow direction vector from chest axes into pelvis/world axes."""
    d = transform_chest_to_world(direction_chest, side)
    n = np.linalg.norm(d)
    if n > 1e-6:
        d = d / n
    return d
