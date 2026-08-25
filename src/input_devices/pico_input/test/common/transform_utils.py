#!/usr/bin/env python3
"""
Coordinate transform utility functions (backward-compatible wrapper)

Actual implementation is in: pico_input/pico_input/transform_utils.py
This file only re-exports, so import paths in test/ scripts stay unchanged.

Usage (in test scripts):
    from common.transform_utils import (
        transform_world_to_chest,
        apply_world_rotation_to_chest_pose,
        transform_pico_rotation_to_world,
        elbow_direction_from_angles,
        get_pico_to_robot,
        get_tf_quaternion,
        # ... other functions
    )
"""

import sys
from pathlib import Path

# Add the pico_input package root to the path so `import pico_input` resolves
# when these scripts run from a source tree with no colcon install sourced.
_pkg_root = Path(__file__).resolve().parents[2]  # common -> test -> pico_input
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

# Re-export all public functions
from pico_input.transform_utils import (  # noqa: F401, E402
    # Position transforms
    transform_world_to_chest,
    transform_chest_to_world,
    # Rotation matrices
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    # TF publishing
    get_tf_quaternion,
    # Direction queries
    get_direction_vector_world,
    get_rotation_axis_world,
    # Configuration queries
    get_pico_to_robot,
    # Pose rotation transforms
    apply_world_rotation_to_chest_pose,
    transform_pico_rotation_to_world,
    # Arm angle control
    elbow_direction_from_angles,
)
