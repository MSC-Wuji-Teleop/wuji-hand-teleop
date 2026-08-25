#!/usr/bin/env python3
"""
G1 Robot Configuration Loader

Usage:
    from g1_world_output.config_loader import G1Config
    config = G1Config.load()
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

try:
    import yaml
except ImportError as exc:
    raise ImportError(
        "PyYAML is required by g1_world_output (see setup.py install_requires); "
        "install it with `pip install PyYAML`."
    ) from exc


def _load_yaml(path: Path) -> Dict[str, Any]:
    text = path.read_text(encoding='utf-8')
    return yaml.safe_load(text) or {}


@dataclass
class G1Config:
    raw: Dict[str, Any]
    arm_type: str
    motion_mode: bool
    simulation_mode: bool
    urdf_package_dir: str
    urdf_filename: str
    world_to_chest_quat: Dict[str, np.ndarray]
    chest_origin_in_pelvis: Dict[str, np.ndarray]
    arm_scale: float
    reset_wrist_pose: Dict[str, Dict[str, np.ndarray]]
    default_zsp_para: Dict[str, list]
    config_path: Path

    @classmethod
    def load(cls, config_path: Optional[str] = None) -> 'G1Config':
        if config_path is None:
            config_path = cls._find_config_file()
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        raw = _load_yaml(path)
        return cls._parse(raw, path)

    @classmethod
    def _find_config_file(cls) -> str:
        try:
            from ament_index_python.packages import (
                PackageNotFoundError,
                get_package_share_directory,
            )
            try:
                share_dir = get_package_share_directory('g1_world_output')
            except PackageNotFoundError:
                share_dir = None
            if share_dir is not None:
                config_path = Path(share_dir) / 'config' / 'g1_robot.yaml'
                if config_path.exists():
                    return str(config_path)
        except ImportError:
            pass

        current_dir = Path(__file__).parent
        for path in (
            current_dir / 'config' / 'g1_robot.yaml',
            current_dir.parent / 'config' / 'g1_robot.yaml',
        ):
            if path.exists():
                return str(path)

        raise FileNotFoundError(
            "Cannot find g1_robot.yaml. Tried ament share/<pkg>/config/ "
            "and the source-tree fallback at <pkg>/config/."
        )

    @classmethod
    def _resolve_urdf_dir(cls, raw_value: str) -> str:
        """Resolve g1_wuji2_description's location: an absolute
        urdf_package_dir override if given, else the ament package share
        directory. This node always runs under a sourced workspace (via
        `ros2 launch`), so there is deliberately no by-name directory crawl
        fallback here -- see scripts/_mujoco_common.py for the crawl these
        standalone scripts still need, since they're commonly invoked as
        `python3 scripts/foo.py` without install/setup.bash sourced.
        """
        if raw_value:
            urdf_path = Path(raw_value).expanduser()
            if urdf_path.is_absolute():
                return str(urdf_path)

        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
        try:
            return get_package_share_directory('g1_wuji2_description')
        except PackageNotFoundError as exc:
            raise FileNotFoundError(
                "g1_wuji2_description package share directory not found -- build it "
                "with `colcon build --packages-select g1_wuji2_description "
                "g1_world_output` and source install/setup.bash, or set "
                "g1_robot.yaml::urdf_package_dir to an absolute path override."
            ) from exc

    @classmethod
    def _parse(cls, raw: Dict[str, Any], config_path: Path) -> 'G1Config':
        def to_numpy_dict(d: Dict[str, list]) -> Dict[str, np.ndarray]:
            return {k: np.array(v, dtype=float) for k, v in d.items()}

        reset_raw = raw.get('reset_wrist_pose', {})
        reset_wrist_pose: Dict[str, Dict[str, np.ndarray]] = {}
        for side in ('left', 'right'):
            side_cfg = reset_raw.get(side, {})
            reset_wrist_pose[side] = {
                'position': np.array(
                    side_cfg.get(
                        'position',
                        [0.30, 0.25 if side == 'left' else -0.25, 0.05],
                    ),
                    dtype=float,
                ),
                'quat': np.array(side_cfg.get('quat', [0.0, 0.0, 0.0, 1.0]), dtype=float),
            }

        urdf_dir = cls._resolve_urdf_dir((raw.get('urdf_package_dir') or '').strip())

        return cls(
            raw=raw,
            arm_type=raw.get('arm_type', 'G1_23'),
            motion_mode=bool(raw.get('motion_mode', False)),
            simulation_mode=bool(raw.get('simulation_mode', False)),
            urdf_package_dir=urdf_dir,
            urdf_filename=raw.get('urdf_filename', 'g1_23_wuji2.urdf'),
            world_to_chest_quat=to_numpy_dict(raw.get('world_to_chest_quat', {})),
            chest_origin_in_pelvis=to_numpy_dict(raw.get('chest_origin_in_pelvis', {})),
            arm_scale=float(raw.get('arm_scale', 1.0)),
            reset_wrist_pose=reset_wrist_pose,
            default_zsp_para=raw.get('default_zsp_para', {}),
            config_path=config_path,
        )

    def get_world_to_chest_rotation(self, side: str) -> np.ndarray:
        from scipy.spatial.transform import Rotation as R
        return R.from_quat(self.world_to_chest_quat[side]).as_matrix()

    def get_chest_to_world_rotation(self, side: str) -> np.ndarray:
        return self.get_world_to_chest_rotation(side).T

    def get_default_zsp_direction(self, side: str) -> np.ndarray:
        zsp = self.default_zsp_para.get(
            side,
            [0, -1, -0.5, 0, 0, 0] if side == 'left' else [0, 1, -0.5, 0, 0, 0],
        )
        direction = np.array(zsp[:3], dtype=float)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        return direction

    def get_urdf_path(self) -> Path:
        return Path(self.urdf_package_dir) / self.urdf_filename

    def get_reset_wrist_matrix(self, side: str) -> np.ndarray:
        from scipy.spatial.transform import Rotation as R
        T = np.eye(4)
        T[:3, :3] = R.from_quat(self.reset_wrist_pose[side]['quat']).as_matrix()
        T[:3, 3] = self.reset_wrist_pose[side]['position']
        return T


_config_instance: Optional[G1Config] = None


def get_config() -> G1Config:
    global _config_instance
    if _config_instance is None:
        _config_instance = G1Config.load()
    return _config_instance


def reload_config(config_path: Optional[str] = None) -> G1Config:
    global _config_instance
    _config_instance = G1Config.load(config_path=config_path)
    return _config_instance
