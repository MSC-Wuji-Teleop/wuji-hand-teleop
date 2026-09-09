"""The G1 + Wuji Hand 2 kinematic model this tool solves against.

One class, SanitizeModel: the Pinocchio model built from
src/g1_wuji2_description/g1_29_wuji2.urdf, every index map the rest of the
package needs, and the forward kinematics that turn a frame's 14 arm joints
into the wrist and elbow poses the IK re-solves.

Why the URDF and not the MJCF the clip audit uses: MuJoCo answers a different
question. It reports the reaction force of a contact -- its peak pairs on a
real clip are routinely hand-to-hand -- and never how deep the overlap is.
The URDF carries a <collision> mesh on all 101 links, and coal answers
touching-or-not per pair, which is what a clearance check needs.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# tools/ is on sys.path from the CLI and from conftest, as prepare_clip.py does.
from clip_audit import ARM_JOINT_NAMES, HAND_JOINT_NAMES, SIDES

# Constants. Each one says where its value comes from.

# Repo root, two directories up; the container mounts tools/ and src/ likewise.
REPO_ROOT = Path(__file__).resolve().parents[2]

# The 29-DoF model: the rig's variant, and what the replay path drives.
URDF_REL_PATH = "src/g1_wuji2_description/g1_29_wuji2.urdf"

# Mesh paths are relative, so coal resolves them against the description package.
MESH_PACKAGE_REL_PATH = "src/g1_wuji2_description"

# URDF arm joints are ARM_JOINT_NAMES (the arm_q order) plus this suffix.
URDF_JOINT_SUFFIX = "_joint"

# URDF hand joints are HAND_JOINT_NAMES (already l_/r_) with this prefix.
def hand_joint_urdf_name(side: str, hand_name: str) -> str:
    """'left', 'l_thumb_cmc_flex' -> 'left_wuji_l_thumb_cmc_flex'."""
    return f"{side}_wuji_{hand_name}"

# Task frames: the wrist placement fixes the hand, the elbow centre the swivel.
WRIST_FRAME_PATTERN = "{side}_wrist_yaw_link"
ELBOW_FRAME_PATTERN = "{side}_elbow_link"

# Arm joints per side and in total.
ARM_JOINTS_PER_SIDE = len(ARM_JOINT_NAMES["left"])
NUM_ARM_JOINTS = 2 * ARM_JOINTS_PER_SIDE

# Wuji Hand 2 joints per side (starport_wuji_hand joint_map.NUM_JOINTS).
NUM_HAND_JOINTS = len(HAND_JOINT_NAMES["left"])

# Legs and waist, held at the audit's "stand" keyframe value and never written.
LOCKED_JOINT_VALUE_RAD = 0.0


def urdf_path() -> Path:
    """The 29-DoF URDF, resolved from the repo root."""
    return REPO_ROOT / URDF_REL_PATH


def mesh_package_dirs() -> List[str]:
    """Directories coal resolves the URDF's relative mesh filenames against."""
    return [str(REPO_ROOT / MESH_PACKAGE_REL_PATH)]


@dataclass(frozen=True)
class ArmTargets:
    """What one frame of one arm asks of the solver.

    wrist is the full placement of {side}_wrist_yaw_link and elbow is the
    origin of {side}_elbow_link, both in the pelvis frame, both read off the
    source joint angles by forward kinematics.
    """
    wrist: object          # pinocchio.SE3
    elbow: np.ndarray      # (3,) position


class SanitizeModel:
    """Model, geometry and index maps, built once per run.

    Building the collision geometry loads 90 STL meshes and takes a few
    seconds, so callers that only need kinematics pass build_geometry=False.
    """

    def __init__(self,
                 urdf: Optional[Path] = None,
                 package_dirs: Optional[Sequence[str]] = None,
                 build_geometry: bool = True) -> None:
        import pinocchio as pin  # imported here so the module is testable without it

        self.pin = pin
        self.urdf = Path(urdf) if urdf is not None else urdf_path()
        if not self.urdf.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf}")
        self.package_dirs = list(package_dirs) if package_dirs is not None else mesh_package_dirs()

        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.data = self.model.createData()

        # Joint -> index. All 1-DoF revolute, so one index per joint is enough.
        self.arm_joint_ids: Dict[str, List[int]] = {}
        self.arm_q_idx: Dict[str, np.ndarray] = {}
        self.hand_joint_ids: Dict[str, List[int]] = {}
        self.hand_q_idx: Dict[str, np.ndarray] = {}
        for side in SIDES:
            self.arm_joint_ids[side] = [self._joint_id(n + URDF_JOINT_SUFFIX)
                                        for n in ARM_JOINT_NAMES[side]]
            self.arm_q_idx[side] = np.array([self.model.idx_qs[j] for j in self.arm_joint_ids[side]])
            self.hand_joint_ids[side] = [self._joint_id(hand_joint_urdf_name(side, n))
                                         for n in HAND_JOINT_NAMES[side]]
            self.hand_q_idx[side] = np.array([self.model.idx_qs[j] for j in self.hand_joint_ids[side]])

        # Both arms in one vector, so a cross-side row can move either of them.
        self.arm_q_idx_all = np.concatenate([self.arm_q_idx[s] for s in SIDES])

        # Frames the tasks are written on.
        self.wrist_frame: Dict[str, int] = {}
        self.elbow_frame: Dict[str, int] = {}
        for side in SIDES:
            self.wrist_frame[side] = self._frame_id(WRIST_FRAME_PATTERN.format(side=side))
            self.elbow_frame[side] = self._frame_id(ELBOW_FRAME_PATTERN.format(side=side))

        # Limits, read off the URDF so there is one source of truth.
        self.arm_lower = self.model.lowerPositionLimit[self.arm_q_idx_all].copy()
        self.arm_upper = self.model.upperPositionLimit[self.arm_q_idx_all].copy()
        self.arm_velocity = self.model.velocityLimit[self.arm_q_idx_all].copy()
        self.hand_lower: Dict[str, np.ndarray] = {}
        self.hand_upper: Dict[str, np.ndarray] = {}
        for side in SIDES:
            self.hand_lower[side] = self.model.lowerPositionLimit[self.hand_q_idx[side]].copy()
            self.hand_upper[side] = self.model.upperPositionLimit[self.hand_q_idx[side]].copy()

        # The joints the clip drives; tells a movable geometry from a locked one.
        self.movable_joint_ids = set()
        for side in SIDES:
            self.movable_joint_ids.update(self.arm_joint_ids[side])
            self.movable_joint_ids.update(self.hand_joint_ids[side])

        self.geom_model = None
        self.geom_data = None
        if build_geometry:
            self.geom_model = pin.buildGeomFromUrdf(
                self.model, str(self.urdf), pin.GeometryType.COLLISION,
                package_dirs=self.package_dirs)

    # -- lookups that refuse a missing name -------------------------------

    def _joint_id(self, name: str) -> int:
        if not self.model.existJointName(name):
            raise KeyError(f"joint '{name}' not in {self.urdf.name}")
        return self.model.getJointId(name)

    def _frame_id(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise KeyError(f"frame '{name}' not in {self.urdf.name}")
        return self.model.getFrameId(name)

    # -- configuration assembly -------------------------------------------

    def neutral(self) -> np.ndarray:
        """All joints at LOCKED_JOINT_VALUE_RAD (the model's neutral)."""
        return self.pin.neutral(self.model)

    def configuration(self,
                      arm: Optional[Dict[str, np.ndarray]] = None,
                      hand: Optional[Dict[str, np.ndarray]] = None,
                      arm_all: Optional[np.ndarray] = None) -> np.ndarray:
        """One full model configuration.

        Legs and waist stay at LOCKED_JOINT_VALUE_RAD. Pass either arm
        ({side: (7,)}) or arm_all (the 14-vector), not both. Omitted hands
        stay open, which is the replay path's resting pose.
        """
        q = self.neutral()
        if arm_all is not None:
            arm_all = np.asarray(arm_all, dtype=float)
            if arm_all.shape != (NUM_ARM_JOINTS,):
                raise ValueError(f"arm_all must be ({NUM_ARM_JOINTS},), got {arm_all.shape}")
            q[self.arm_q_idx_all] = arm_all
        if arm is not None:
            for side, values in arm.items():
                values = np.asarray(values, dtype=float)
                if values.shape != (ARM_JOINTS_PER_SIDE,):
                    raise ValueError(
                        f"arm[{side!r}] must be ({ARM_JOINTS_PER_SIDE},), got {values.shape}")
                q[self.arm_q_idx[side]] = values
        if hand is not None:
            for side, values in hand.items():
                values = np.asarray(values, dtype=float)
                if values.shape != (NUM_HAND_JOINTS,):
                    raise ValueError(
                        f"hand[{side!r}] must be ({NUM_HAND_JOINTS},), got {values.shape}")
                q[self.hand_q_idx[side]] = values
        return q

    def split_arm_all(self, arm_all: np.ndarray) -> Dict[str, np.ndarray]:
        """The 14-vector back into {'left': (7,), 'right': (7,)}."""
        arm_all = np.asarray(arm_all, dtype=float)
        return {side: arm_all[i * ARM_JOINTS_PER_SIDE:(i + 1) * ARM_JOINTS_PER_SIDE].copy()
                for i, side in enumerate(SIDES)}

    def join_arm(self, arm: Dict[str, np.ndarray]) -> np.ndarray:
        """{'left': (7,), 'right': (7,)} into the 14-vector."""
        return np.concatenate([np.asarray(arm[s], dtype=float) for s in SIDES])

    # -- forward kinematics ------------------------------------------------

    def update(self, q: np.ndarray) -> None:
        """Place every frame for this configuration."""
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)

    def targets(self, q: np.ndarray) -> Dict[str, ArmTargets]:
        """The wrist placement and elbow position this configuration implies.

        This is the "extract wrist and elbow poses" step of the spec: the
        poses, not the joint angles, are what the sanitized clip preserves.
        """
        self.update(q)
        return {side: ArmTargets(wrist=self.data.oMf[self.wrist_frame[side]].copy(),
                                 elbow=self.data.oMf[self.elbow_frame[side]].translation.copy())
                for side in SIDES}

    def frame_pose(self, q: np.ndarray, frame_id: int):
        """One frame's placement, for callers that want a single lookup."""
        self.update(q)
        return self.data.oMf[frame_id].copy()

    # -- kinematic-tree helpers used by the collision pair set -------------

    def joint_ancestors(self, joint_id: int) -> List[int]:
        """joint_id and every parent up to (but not including) the root."""
        out: List[int] = []
        j = int(joint_id)
        while j > 0:
            out.append(j)
            j = self.model.parents[j]
        return out

    def joints_within(self, a: int, b: int, depth: int) -> bool:
        """True when one joint is the other, or an ancestor within depth steps.

        Used to drop collision pairs between links that are neighbours in the
        tree: their meshes touch by construction at every configuration, so
        checking them reports geometry, not motion.
        """
        for start, goal in ((a, b), (b, a)):
            j = int(start)
            for _ in range(depth + 1):
                if j == goal:
                    return True
                if j == 0:
                    break
                j = self.model.parents[j]
        return False

    def geometry_is_movable(self, geom) -> bool:
        """True when a clip joint is on this geometry's path to the root."""
        return any(a in self.movable_joint_ids for a in self.joint_ancestors(geom.parentJoint))

    def geometry_pairs(self) -> List[Tuple[int, int]]:
        """Every unordered geometry index pair, in a stable order."""
        if self.geom_model is None:
            raise RuntimeError("geometry not built; construct with build_geometry=True")
        return list(itertools.combinations(range(len(self.geom_model.geometryObjects)), 2))
