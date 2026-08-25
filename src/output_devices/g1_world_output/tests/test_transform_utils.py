#!/usr/bin/env python3
"""Smoke tests for chest -> pelvis remapping (no robot / DDS required)."""

import numpy as np
from scipy.spatial.transform import Rotation as R

from g1_world_output.config_loader import G1Config, reload_config
from g1_world_output.transform_utils import (
    chest_pose_to_pelvis,
    transform_chest_to_world,
    transform_world_to_chest,
)


def test_chest_world_roundtrip():
    v = np.array([0.5, 0.2, 0.3])
    for side in ('left', 'right'):
        back = transform_chest_to_world(transform_world_to_chest(v, side), side)
        assert np.allclose(v, back, atol=1e-9)


def test_chest_pose_to_pelvis_applies_origin():
    reload_config()
    cfg = G1Config.load()
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 0.0]
    left = chest_pose_to_pelvis(T, 'left', arm_scale=1.0)
    assert np.allclose(left[:3, 3], cfg.chest_origin_in_pelvis['left'])


def test_chest_pose_rotation_remap():
    reload_config()
    # Identity chest orientation -> chest_to_world rotation in pelvis frame
    T = np.eye(4)
    T[:3, 3] = [0.1, 0.0, 0.0]
    out = chest_pose_to_pelvis(T, 'left', arm_scale=1.0)
    R_c2w = G1Config.load().get_chest_to_world_rotation('left')
    assert np.allclose(out[:3, :3], R_c2w)
    # Position: R @ [0.1,0,0] + origin
    expected_p = R_c2w @ np.array([0.1, 0.0, 0.0]) + G1Config.load().chest_origin_in_pelvis['left']
    assert np.allclose(out[:3, 3], expected_p)
