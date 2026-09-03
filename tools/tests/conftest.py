"""Shared fixtures: a synthetic bundle method dir and a fake retargeter.

The RobotSTAR bundle is not on every machine, so every test builds its own
bundle under tmp_path with make_bundle(): the same file layout, names and
dtypes as the real one, a smooth arm motion around the model's stand pose,
one injected single-frame spike, and optionally a >= 90 deg step (an
orientation flip). The fake retargeter exposes exactly what prepare_clip.py
reads from the real one and returns a known pattern, so the retarget stage
and the end-to-end path run without wuji_retargeting.

Also used, outside pytest, to generate the bundle for the CLI check in the
container (see the module docstring of tools/tests/__init__.py).
"""

from __future__ import annotations

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional

import numpy as np
import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import clip_audit  # noqa: E402

REPO_ROOT = TOOLS_DIR.parent

# The replay package, for load_clip in the end-to-end test.
REPLAY_PKG_DIR = REPO_ROOT / "src" / "input_devices" / "replay"

# Bundle body_actuators (29 names, no _joint suffix). Legs and waist as the
# G1 29-DoF names them; the arm blocks are deliberately NOT in the clip's
# order (right before left) so a tool that indexed by position would fail.
LEG_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
]
WAIST_NAMES = ["waist_yaw", "waist_roll", "waist_pitch"]
BODY_ACTUATORS = (LEG_NAMES + WAIST_NAMES
                  + clip_audit.ARM_JOINT_NAMES["right"] + clip_audit.ARM_JOINT_NAMES["left"])

# Arm values of the "stand" keyframe in g1_29_wuji2_fixed.xml
# (AuditRig.ctrl_stand at the arm actuators), the pose the audit starts from.
STAND_ARM_Q = {
    "left": [0.2, 0.2, 0.0, 1.28, 0.0, 0.0, 0.0],
    "right": [0.2, -0.2, 0.0, 1.28, 0.0, 0.0, 0.0],
}

# Default synthetic motion: shoulder pitch and elbow swing that stays clear
# of the torso (checked against the audit: passes at 1.0, 0.5, 0.25 with no
# contact), plus a single-frame spike on the left shoulder pitch.
MOTION_FREQ_HZ = 0.5
SHOULDER_PITCH_AMPLITUDE_RAD = 0.3
ELBOW_AMPLITUDE_RAD = 0.2
ELBOW_PHASE_RAD = 0.5

# Pinocchio's joint order for the Hand 2 URDF (probed in the container):
# fingers by child-link name, index, middle, pinky, ring, thumb.
OPTIMIZER_FINGER_ORDER = ("index_finger", "middle_finger", "pinky", "ring_finger", "thumb")

# Joint range the fake tests clip into: the URDF's common finger range.
FAKE_HAND_RANGE = np.tile(np.array([[-1.047, 1.57]]), (len(clip_audit.HAND_JOINT_NAMES["left"]), 1))

# Default fake output, in hardware order. Stays inside every model joint range
# (the tightest is the abduction joints' +-0.698) so nothing is clipped.
FAKE_PATTERN_HW = np.linspace(-0.5, 0.6, len(clip_audit.HAND_JOINT_NAMES["left"]))


def optimizer_joint_names(side: str) -> List[str]:
    """HAND_JOINT_NAMES[side] regrouped into the optimizer's finger order."""
    names = clip_audit.HAND_JOINT_NAMES[side]
    prefix = names[0].split("_")[0] + "_"
    out: List[str] = []
    for finger in OPTIMIZER_FINGER_ORDER:
        out.extend(n for n in names if n.startswith(prefix + finger + "_"))
    assert sorted(out) == sorted(names)
    return out


def flat_hand_keypoints(side: str) -> np.ndarray:
    """A plausible open hand: (21, 3) float32 meters, MediaPipe order, wrist at the origin.

    Fingers extend along +x, spread along +y (mirrored for the right hand),
    joints 2 to 3 cm apart.
    """
    kp = np.zeros((21, 3), dtype=np.float32)
    finger_y = {1: -0.03, 5: -0.02, 9: 0.0, 13: 0.02, 17: 0.035}
    for base, y in finger_y.items():
        for k in range(4):
            idx = base + k
            if base == 1:  # thumb: shorter, angled away
                kp[idx, 0] = 0.02 + 0.02 * (k + 1)
                kp[idx, 1] = -0.03 - 0.01 * k
            else:
                kp[idx, 0] = 0.04 + 0.025 * (k + 1)
                kp[idx, 1] = y * (1.0 + 0.1 * k)
    if side == "right":
        kp[:, 1] *= -1.0
    return kp


def synthetic_arm_q(frames: int, target_fps: float) -> Dict[str, np.ndarray]:
    """The default smooth motion around the stand pose, per side (frames, 7)."""
    t = np.arange(frames) / target_fps
    out = {}
    for side in clip_audit.SIDES:
        q = np.tile(np.array(STAND_ARM_Q[side]), (frames, 1))
        q[:, 0] += SHOULDER_PITCH_AMPLITUDE_RAD * np.sin(2 * np.pi * MOTION_FREQ_HZ * t)
        q[:, 3] += ELBOW_AMPLITUDE_RAD * np.sin(2 * np.pi * MOTION_FREQ_HZ * t + ELBOW_PHASE_RAD)
        out[side] = q
    return out


def make_bundle(root: Path, sample: str = "synth", method: str = "Ours", frames: int = 120,
                source_frames: int = 48, target_fps: float = 50.0, source_fps: float = 20.0,
                time_scale: int = 1, spike_frame: Optional[int] = 40, spike_rad: float = 0.3,
                flip_frame: Optional[int] = None, flip_deg: float = 100.0,
                write_manifest: bool = True, body_q_override: Optional[np.ndarray] = None,
                keypoints_shape_override: Optional[tuple] = None,
                omit_source_frames: bool = False) -> Path:
    """Write <root>/samples/<sample>/<method>/... and <root>/MANIFEST.sha256; return the method dir.

    spike_frame: a single-frame bump of spike_rad on left_shoulder_pitch.
    flip_frame: from that frame on, right_shoulder_pitch is offset by flip_deg
    (a persistent step, the shape of an estimator orientation flip).
    """
    root = Path(root)
    method_dir = root / "samples" / sample / method
    (method_dir / "g1_reference").mkdir(parents=True, exist_ok=True)
    (method_dir / "hand2_input").mkdir(parents=True, exist_ok=True)

    arm = synthetic_arm_q(frames, target_fps)
    if spike_frame is not None:
        arm["left"][spike_frame, 0] += spike_rad
    if flip_frame is not None:
        arm["right"][flip_frame:, 0] += np.radians(flip_deg)
    body_q = np.zeros((frames, len(BODY_ACTUATORS)))
    for side in clip_audit.SIDES:
        for j, name in enumerate(clip_audit.ARM_JOINT_NAMES[side]):
            body_q[:, BODY_ACTUATORS.index(name)] = arm[side][:, j]
    if body_q_override is not None:
        body_q = body_q_override

    npz_path = method_dir / "g1_reference" / "controller_reference_v7.npz"
    np.savez(npz_path, body_q=body_q.astype(np.float32))
    meta = {
        "frames": frames,
        "source_frames": source_frames,
        "source_fps": source_fps,
        "target_fps": target_fps,
        "time_scale": time_scale,
        "end_behavior": "hold_last_target",
        "joint_actuator_order": {"body_actuators": BODY_ACTUATORS},
    }
    if omit_source_frames:
        # RobotSTAR_demos/sweep-test writes its keypoints on the body frame grid
        # and its generator emits no source_frames/source_fps.
        meta.pop("source_frames")
        meta.pop("source_fps")
    meta_path = method_dir / "g1_reference" / "target_meta.json"
    meta_path.write_text(json.dumps(meta, indent=1))

    # Keypoints: the open hand with a slow finger curl so consecutive frames differ.
    ts = np.arange(source_frames) / source_fps
    kp = {}
    for side in clip_audit.SIDES:
        base = flat_hand_keypoints(side)
        frames_kp = np.repeat(base[None], source_frames, axis=0)
        curl = 0.01 * (0.5 - 0.5 * np.cos(2 * np.pi * 0.5 * ts))
        frames_kp[:, 5:, 2] -= curl[:, None] * np.arange(1, 17)[None, :] / 16.0
        kp[side] = frames_kp.astype(np.float32)
    if keypoints_shape_override is not None:
        kp["left"] = np.zeros(keypoints_shape_override, dtype=np.float32)
    kp_path = method_dir / "hand2_input" / f"{method.lower()}_human_targets_v5.npz"
    np.savez(kp_path, left_hand_keypoints21=kp["left"], right_hand_keypoints21=kp["right"])
    (method_dir.parent / "DO_NOT_COMMAND_HAND2.txt").write_text("synthetic\n")

    if write_manifest:
        lines = [f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(root).as_posix()}"
                 for p in (meta_path, npz_path, kp_path)]
        (root / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")
    return method_dir


def write_fake_urdf(path: Path, joint_names: List[str]) -> Path:
    """A URDF with the given movable joints in that order, plus one fixed joint."""
    robot = ET.Element("robot", name="fake_hand")
    ET.SubElement(robot, "link", name="base")
    ET.SubElement(robot, "link", name="mount")
    fixed = ET.SubElement(robot, "joint", name="mount_fixed", type="fixed")
    ET.SubElement(fixed, "parent", link="base")
    ET.SubElement(fixed, "child", link="mount")
    parent = "mount"
    for name in joint_names:
        ET.SubElement(robot, "link", name=f"{name}_link")
        j = ET.SubElement(robot, "joint", name=name, type="revolute")
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=f"{name}_link")
        ET.SubElement(j, "limit", lower="-1.047", upper="1.57")
        parent = f"{name}_link"
    path = Path(path)
    path.write_bytes(ET.tostring(robot))
    return path


class FakeRetargeter:
    """Stands in for wuji_retargeting.Retargeter.

    Exposes what prepare_clip reads: config with __yaml_dir and
    optimizer.urdf_path, optimizer.robot.dof_joint_names in the real
    optimizer order, reset(), and retarget(kp) -> (20,) in that order. The
    values are a pattern defined in HARDWARE order (pattern_hw, plus kp_gain
    times the keypoints' mean so frames differ), permuted into optimizer order,
    so a correct permutation returns pattern_hw exactly.
    """

    def __init__(self, side: str, urdf_path: Path, pattern_hw: Optional[np.ndarray] = None,
                 kp_gain: float = 0.0):
        self.side = side
        urdf_path = Path(urdf_path)
        self.config = {"__yaml_dir": str(urdf_path.parent), "optimizer": {"urdf_path": urdf_path.name}}
        names_opt = optimizer_joint_names(side)
        self.optimizer = SimpleNamespace(robot=SimpleNamespace(dof_joint_names=names_opt))
        hw = clip_audit.HAND_JOINT_NAMES[side]
        self._hw_index_of_opt = np.array([hw.index(n) for n in names_opt])
        self.pattern_hw = (FAKE_PATTERN_HW.copy() if pattern_hw is None
                           else np.asarray(pattern_hw, dtype=np.float64))
        self.kp_gain = kp_gain
        self.reset_calls = 0
        self.retarget_calls = 0
        self.seen_shapes = set()

    def reset(self) -> None:
        self.reset_calls += 1

    def retarget(self, kp) -> np.ndarray:
        kp = np.asarray(kp)
        self.seen_shapes.add(kp.shape)
        self.retarget_calls += 1
        hw = self.pattern_hw + self.kp_gain * float(kp.mean())
        return hw[self._hw_index_of_opt]


@pytest.fixture
def fake_retargeter_factory(tmp_path):
    """factory(config_path, side) -> FakeRetargeter; factory.made lists them.

    Set factory.kwargs (pattern_hw, kp_gain) before the run to shape the output.
    """
    made = []

    def factory(config_path, side):
        urdf = write_fake_urdf(tmp_path / f"fake_{side}.urdf", clip_audit.HAND_JOINT_NAMES[side])
        r = FakeRetargeter(side, urdf, **factory.kwargs)
        made.append(r)
        return r

    factory.made = made
    factory.kwargs = {}
    return factory


@pytest.fixture(scope="session")
def rig():
    """The real AuditRig on the composed model, loaded once for the whole session."""
    pytest.importorskip("mujoco")
    return clip_audit.AuditRig()


@pytest.fixture
def bundle_root(tmp_path):
    return tmp_path / "bundle"
