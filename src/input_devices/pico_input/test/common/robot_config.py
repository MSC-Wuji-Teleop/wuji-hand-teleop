#!/usr/bin/env python3
"""
Robot configuration parameters (compatibility wrapper)

This file is a backward-compatibility layer; the actual configuration comes from:
  pico_input/config/robot_frames.yaml

Usage (recommended):
  from pico_input.config_loader import get_config
  config = get_config()

Usage (compatible with legacy scripts):
  from common.robot_config import TIANJI_INIT_POS, TIANJI_INIT_ROT, ...

The TIANJI_* names are kept because the surviving step scripts use them and
because the values genuinely are Tianji-derived: they are FK of the old Tianji
arm's calibrated init pose, carried over verbatim and not yet re-derived for
the G1_23. See the PROVENANCE block in config/robot_frames.yaml.
"""

import sys
from pathlib import Path

# Add the pico_input package root to the path so `import pico_input` resolves
# when these scripts run from a source tree with no colcon install sourced.
_pkg_root = Path(__file__).resolve().parents[2]  # common -> test -> pico_input
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

# Import from the unified config loader
from pico_input.config_loader import get_config  # noqa: E402

# Load configuration
_config = get_config()

# =============================================================================
# Export compatibility variables
# =============================================================================

# Initial pose (chest frame)
TIANJI_INIT_POS = _config.init_pos
TIANJI_INIT_ROT = _config.init_rot

# World -> Chest coordinate transform
WORLD_TO_LEFT_QUAT = _config.world_to_chest_quat['left']
WORLD_TO_RIGHT_QUAT = _config.world_to_chest_quat['right']
WORLD_TO_CHEST_TRANS = _config.world_to_chest_trans

# Elbow-direction / nullspace parameters
ZSP_TYPE = _config.zsp_type
ZSP_PARA = _config.default_zsp_para
ZSP_ANGLE = _config.zsp_angle
DGR = _config.dgr

# Normalized default zsp direction vector (first 3 components, used as the
# elbow direction default)
DEFAULT_ZSP_DIRECTION = {
    'left': _config.get_default_zsp_direction('left'),
    'right': _config.get_default_zsp_direction('right'),
}

# Forearm tracker initial pose (chest frame)
ARM_INIT_POS = _config.arm_init_pos
ARM_INIT_QUAT = _config.arm_init_quat

# World coordinate system reference orientation
import numpy as np  # noqa: E402

WORLD_REFERENCE_ROT = np.eye(3)
