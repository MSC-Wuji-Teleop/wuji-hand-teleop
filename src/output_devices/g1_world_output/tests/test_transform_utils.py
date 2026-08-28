#!/usr/bin/env python3
"""Smoke tests for chest -> pelvis remapping (no robot / DDS required).

The two config-loading tests resolve g1_wuji2_description through ament and
skip cleanly outside a sourced workspace (host venv / CI ROS-free lane).
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
    pytest.importorskip('ament_index_python')
    reload_config()
    cfg = G1Config.load()
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 0.0]
    left = chest_pose_to_pelvis(T, 'left', arm_scale=1.0)
    assert np.allclose(left[:3, 3], cfg.chest_origin_in_pelvis['left'])


def test_chest_pose_rotation_remap():
    pytest.importorskip('ament_index_python')
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
