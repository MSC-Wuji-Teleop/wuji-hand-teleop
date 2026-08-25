#!/usr/bin/env python3
"""
PICO frame / anchor configuration loader.

Loads config/robot_frames.yaml and exposes it with typed accessors.

Usage:
    from pico_input.config_loader import get_config

    config = get_config()
    init_pos  = config.init_pos['left']              # numpy [x, y, z]
    init_rot  = config.init_rot['left']              # numpy 3x3
    init_quat = config.init_quat['left']             # numpy [qx, qy, qz, qw]
    w2c_quat  = config.world_to_chest_quat['left']
    raw       = config.raw                           # untouched dict

History: this was tianji_world_output.config_loader (class TianjiConfig). It
moved into pico_input when the Tianji arm packages were removed, because
pico_input was its only remaining consumer and the values it carries are
input-side frame conventions plus incremental-control anchors, not arm-driver
state. Three Tianji-hardware fields were dropped in the move: `robot_ip`,
`init_joints`, and `get_kine_config_path()`. See the PROVENANCE block in
config/robot_frames.yaml before changing any value.
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any, Optional
import numpy as np

try:
    import yaml
except ImportError as exc:
    raise ImportError("PyYAML is required. Install with: pip install pyyaml") from exc

_CONFIG_FILENAME = 'robot_frames.yaml'


@dataclass
class RobotFramesConfig:
    """Frame conventions and incremental-control anchors for the PICO path."""

    # Raw config dictionary
    raw: Dict[str, Any]

    # Initial end-effector pose (chest frame) — incremental-control anchor
    init_pos: Dict[str, np.ndarray]     # [x, y, z] metres
    init_rot: Dict[str, np.ndarray]     # 3x3 rotation matrix
    init_quat: Dict[str, np.ndarray]    # [qx, qy, qz, qw]

    # World -> Chest transform (arm-agnostic)
    world_to_chest_quat: Dict[str, np.ndarray]   # [qx, qy, qz, qw]
    world_to_chest_trans: Dict[str, np.ndarray]  # [x, y, z]

    # Forearm tracker initial pose (chest frame) — incremental-control anchor
    arm_init_pos: Dict[str, np.ndarray]    # [x, y, z] metres
    arm_init_quat: Dict[str, np.ndarray]   # [qx, qy, qz, qw]

    # PICO -> Robot transform (arm-agnostic)
    pico_to_robot: np.ndarray  # 3x3

    # Elbow-direction / nullspace parameters
    zsp_type: int
    default_zsp_para: Dict[str, list]
    zsp_angle: float
    dgr: list

    # Resolved path of the file this was loaded from
    config_path: Path

    @classmethod
    def load(cls, config_path: Optional[str] = None) -> 'RobotFramesConfig':
        """Load the config file.

        Args:
            config_path: explicit path, or None to auto-locate.

        Returns:
            RobotFramesConfig instance
        """
        if config_path is None:
            config_path = cls._find_config_file()

        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")

        with path.open('r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        return cls._parse(raw, path)

    @classmethod
    def _find_config_file(cls) -> str:
        """Locate robot_frames.yaml.

        Two layouts are supported:
          1. Ament install / Docker: `share/pico_input/config/robot_frames.yaml`.
             This is the normal runtime path.
          2. Source-tree pytest: `<pkg>/config/robot_frames.yaml` next to this
             module, for plain `python3 -m pytest` runs with no colcon and no
             AMENT setup, where get_package_share_directory raises.

        Only PackageNotFoundError is caught — any other ament error is a real
        install problem and should propagate rather than be swallowed.
        """
        from ament_index_python.packages import (
            get_package_share_directory,
            PackageNotFoundError,
        )
        try:
            share_dir = get_package_share_directory('pico_input')
        except PackageNotFoundError:
            share_dir = None
        if share_dir is not None:
            config_path = Path(share_dir) / 'config' / _CONFIG_FILENAME
            if config_path.exists():
                return str(config_path)

        # Source-tree fallback (pytest with no ROS2 environment).
        current_dir = Path(__file__).parent
        for path in (
            current_dir / 'config' / _CONFIG_FILENAME,
            current_dir.parent / 'config' / _CONFIG_FILENAME,
        ):
            if path.exists():
                return str(path)

        raise FileNotFoundError(
            f"Cannot find {_CONFIG_FILENAME}. Tried ament "
            "share/pico_input/config/ and the source-tree fallback at "
            "<pkg>/config/. Did `colcon build` succeed and is "
            "AMENT_PREFIX_PATH set?"
        )

    @classmethod
    def _parse(cls, raw: Dict[str, Any], config_path: Path) -> 'RobotFramesConfig':
        """Parse the raw config dictionary."""

        def to_numpy_dict(d: Dict[str, list]) -> Dict[str, np.ndarray]:
            return {k: np.array(v) for k, v in d.items()}

        return cls(
            raw=raw,
            init_pos=to_numpy_dict(raw.get('init_pos', {})),
            init_rot=to_numpy_dict(raw.get('init_rot', {})),
            init_quat=to_numpy_dict(raw.get('init_quat', {})),
            world_to_chest_quat=to_numpy_dict(raw.get('world_to_chest_quat', {})),
            world_to_chest_trans=to_numpy_dict(raw.get('world_to_chest_trans', {})),
            arm_init_pos=to_numpy_dict(raw.get('arm_init_pos', {})),
            arm_init_quat=to_numpy_dict(raw.get('arm_init_quat', {})),
            pico_to_robot=np.array(raw.get('pico_to_robot', [[0, 0, -1], [-1, 0, 0], [0, 1, 0]])),
            zsp_type=raw.get('zsp_type', 1),
            default_zsp_para=raw.get('default_zsp_para', {}),
            zsp_angle=raw.get('zsp_angle', 0.0),
            dgr=raw.get('dgr', [5.0, 5.0, 5.0]),
            config_path=config_path,
        )

    def get_world_to_chest_rotation(self, side: str) -> np.ndarray:
        """World -> Chest rotation matrix."""
        from scipy.spatial.transform import Rotation as R
        quat = self.world_to_chest_quat[side]
        return R.from_quat(quat).as_matrix()

    def get_chest_to_world_rotation(self, side: str) -> np.ndarray:
        """Chest -> World rotation matrix (inverse of the above)."""
        return self.get_world_to_chest_rotation(side).T

    def get_default_zsp_direction(self, side: str) -> np.ndarray:
        """Normalized default zsp direction vector (first 3 components)."""
        zsp = self.default_zsp_para.get(side, [0, -1, -0.5, 0, 0, 0] if side == 'left' else [0, 1, -0.5, 0, 0, 0])
        direction = np.array(zsp[:3], dtype=float)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        return direction


# Global singleton (lazy-loaded)
_config_instance: Optional[RobotFramesConfig] = None


def get_config() -> RobotFramesConfig:
    """Get the config singleton."""
    global _config_instance
    if _config_instance is None:
        _config_instance = RobotFramesConfig.load()
    return _config_instance


def reload_config(config_path: Optional[str] = None) -> RobotFramesConfig:
    """Reload the configuration."""
    global _config_instance
    _config_instance = RobotFramesConfig.load(config_path=config_path)
    return _config_instance
