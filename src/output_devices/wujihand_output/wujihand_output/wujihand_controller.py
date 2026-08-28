#!/usr/bin/env python3
"""
Wuji Hand Controller.

Talks to the wujihandros2 driver (which wraps the C++ wujihandcpp SDK)
over ROS2. Each instance owns a single hand and supports both raw joint
control and IK-based retargeting.

Multi-core parallelism: one process per hand -> independent retargeter,
independent GIL -> good multi-core CPU utilization.
"""
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple
import numpy as np

try:
    from wujihand_output._internal.hand_interface import WujiHand
except ImportError:
    from ._internal.hand_interface import WujiHand

try:
    from wuji_retargeting import Retargeter
    RETARGETER_AVAILABLE = True
except ImportError:
    RETARGETER_AVAILABLE = False


class WujiHandController:
    """Wuji-hand controller for a single hand.

    Talks to the wujihandros2 driver (1000Hz hardware loop). One instance
    = one hand; multi-core parallelism is provided by process-level
    isolation.

    Two control modes:
    1. Joint angle control: set the 20 joint angles directly.
    2. IK control: take a 21-point hand keypoint set and retarget it.
    """

    NUM_JOINTS = 20  # 5 fingers x 4 joints

    def __init__(
        self,
        side: str,
        hand_name: str,
        input_source: str = "wuji_glove",
        retarget_config: Optional[str] = None,
        retarget_config_dir: Optional[str] = None,
        enable_ik: bool = True,
        logger=None,
        node=None,
    ):
        """
        Initialize a single-hand controller.

        Args:
            side: "left" or "right".
            hand_name: wujihandros2 driver namespace (e.g. "left_hand").
            input_source: input source type (used to pick the IK retarget config).
            retarget_config: explicit retarget config path (optional).
            retarget_config_dir: retarget config directory (optional). When
                set, lookup order is
                retarget_{input_source}_{side}.yaml then
                retarget_{input_source}.yaml — taking priority over the
                wujihand_output package's bundled config/. Used for
                multi-host deployments: launch passes the local test/
                config directory so retarget params follow the deploy host.
            enable_ik: enable IK control.
            logger: external logger.
            node: ROS2 node instance.
        """
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger(f'WujiHandController_{side}')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        self.side = side
        self.hand_name = hand_name
        self.input_source = input_source
        self.node = node
        self._retarget_config_dir = (
            Path(retarget_config_dir) if retarget_config_dir else None
        )

        # Hardware interface
        self.hand: Optional[WujiHand] = None

        # IK retargeter
        self.retargeter: Optional['Retargeter'] = None
        self._ik_enabled = False
        # Optimizer-qpos -> device-order permutation; None means identity.
        self._qpos_perm: Optional[np.ndarray] = None

        if enable_ik and RETARGETER_AVAILABLE:
            self._init_retargeter(retarget_config)
        elif enable_ik and not RETARGETER_AVAILABLE:
            self.logger.warning("wuji_retargeting not installed; IK control unavailable")

        self._init_hand()
        self.logger.info(f"{side}-hand controller initialized (wujihandros2)")

    def _resolve_retarget_config(self) -> Optional[str]:
        """Resolve retarget_{input_source}_{side}.yaml from override dir or package share."""
        candidate_dirs = []
        if self._retarget_config_dir is not None:
            candidate_dirs.append(self._retarget_config_dir)

        try:
            from ament_index_python.packages import get_package_share_directory
            candidate_dirs.append(
                Path(get_package_share_directory("wujihand_output")) / "config"
            )
        except Exception as e:
            self.logger.warning(
                f"Could not locate wujihand_output share dir; "
                f"skipping package default retarget config: {e}")

        for cfg_dir in candidate_dirs:
            per_side = cfg_dir / f"retarget_{self.input_source}_{self.side}.yaml"
            if per_side.exists():
                self.logger.info(f"IK retarget config: {per_side}")
                return str(per_side)
            shared = cfg_dir / f"retarget_{self.input_source}.yaml"
            if shared.exists():
                self.logger.info(f"IK retarget config (shared): {shared}")
                return str(shared)

        return None

    def _init_retargeter(self, config: Optional[str]) -> None:
        config_path = config or self._resolve_retarget_config()
        if config_path and Path(config_path).exists():
            self.retargeter = Retargeter.from_yaml(config_path, self.side)
            self._qpos_perm = self._build_qpos_perm(self.retargeter)
            self._ik_enabled = True
            self.logger.info(f"IK retarget config: {Path(config_path).name}")
        else:
            self.logger.warning("No IK retarget config found")

    @staticmethod
    def _urdf_joint_order(config: dict) -> Optional[List[str]]:
        """Movable joint names in URDF declaration order.

        That order is the vendor's own flat 20-element convention (thumb, index,
        middle, ring, pinky, 4 each) and is what wujihand_driver's JOINT_NAMES
        and the MuJoCo HAND_CODES mapping both index positionally.

        Returns None when the config sets no urdf_path, i.e. the bundled Wuji
        Hand 1 fallback, where Pinocchio's order already matches and no remap is
        wanted.
        """
        rel = (config.get('optimizer') or {}).get('urdf_path')
        if not rel:
            return None
        path = Path(rel)
        if not path.is_absolute():
            path = (Path(config['__yaml_dir']) / path).resolve()
        root = ET.parse(path).getroot()
        return [j.get('name') for j in root.findall('joint')
                if j.get('type') != 'fixed']

    def _build_qpos_perm(self, retargeter) -> Optional[np.ndarray]:
        """Permutation taking optimizer qpos order -> device order.

        Pinocchio orders joints by its own tree walk, which urdfdom sorts by
        child-link name. Wuji Hand 1's finger1..finger5 names happen to sort into
        declaration order, so the two agree and no remap is needed. Wuji Hand 2's
        anatomical names do not: Pinocchio yields index, middle, pinky, ring,
        thumb. Publishing that raw would send thumb angles to the index finger.

        Raises when a urdf_path override is set but the names cannot be aligned.
        Falling back to the identity there would silently drive the wrong
        fingers, which is worse than refusing to start.
        """
        dst = self._urdf_joint_order(retargeter.config)
        if dst is None:
            return None
        src = list(retargeter.optimizer.robot.dof_joint_names)
        idx = {n: i for i, n in enumerate(src)}
        missing = [n for n in dst if n not in idx]
        if missing or len(dst) != len(src):
            raise RuntimeError(
                f"{self.side}-hand qpos remap failed: URDF declares {len(dst)} "
                f"movable joints, optimizer has {len(src)}; unmatched={missing}"
            )
        perm = np.array([idx[n] for n in dst], dtype=int)
        if np.array_equal(perm, np.arange(len(perm))):
            self.logger.info("qpos remap: identity (URDF order matches optimizer)")
            return None
        self.logger.info(f"qpos remap active: optimizer -> device {perm.tolist()}")
        return perm

    def _init_hand(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS2 node was not provided")
        self.hand = WujiHand(
            hand_name=self.hand_name,
            side=self.side,
            node=self.node,
            logger=self.logger,
        )
        self.hand.connect()
        self.logger.info(f"ROS2 interface created -> /{self.hand_name}")

    # ==================== Joint-angle control ====================

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        if self.hand is not None:
            return self.hand.set_joint_positions(positions)
        return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        if self.hand is not None:
            return self.hand.get_joint_positions()
        return None

    # ==================== IK control ====================

    def is_ik_available(self) -> bool:
        return self._ik_enabled

    def retarget(self, keypoints: np.ndarray) -> Optional[np.ndarray]:
        """Retarget hand keypoints to joint angles.

        Args:
            keypoints: hand keypoints, shape (21, 3) or (63,).

        Returns:
            (20,) joint-angle array in device order, or None on failure.
        """
        if self.retargeter is None:
            return None
        try:
            keypoints = np.asarray(keypoints, dtype=np.float32)
            if keypoints.shape == (63,):
                keypoints = keypoints.reshape(21, 3)
            q = self.retargeter.retarget(keypoints)
            return q if self._qpos_perm is None else q[self._qpos_perm]
        except Exception as e:
            self.logger.error(f"{self.side}-hand IK retarget failed: {e}")
            return None

    def set_keypoints(self, keypoints: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Drive the hand from keypoints (IK retarget + hardware command).

        Returns:
            (success, joint_angles).
        """
        angles = self.retarget(keypoints)
        if angles is not None and self.hand is not None:
            success = self.hand.set_joint_positions(angles)
            return success, angles
        return False, angles

    # ==================== Status & release ====================

    def is_connected(self) -> bool:
        return self.hand is not None and self.hand.is_connected()

    def disable_and_release(self) -> None:
        self.logger.info(f"Disabling {self.side} hand...")
        if self.hand is not None:
            self.hand.release()
            self.hand = None
        self.logger.info("Exited cleanly")
