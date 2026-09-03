#!/usr/bin/env python3
"""MuJoCo dynamic audit of a replay clip (docs/spec/spec1.md, step 3).

A clip is a pair of joint trajectories: 14 arm joints and 40 hand joints at
a fixed frame rate. This module replays such a clip through mj_step on the
composed G1 + Wuji Hand 2 model and measures what our controller would ask of
the robot: peak contact force and which bodies touch, peak arm torque as a
fraction of the joint's clamp, saturation, and tracking error. Nothing here
decides anything about the real robot. It produces numbers that
prepare_clip.py records in clip.json.

Importable API:
    AuditRig(model_path, gains)       loads the model once, re-gains the arms
    AuditRig.run(...) -> AuditResult  one speed
    audit_clip(...) -> dict           one speed, one call, returns the
                                      per_speed block of clip.json
    render_video(clip_dir, speed, out_mp4)

Command line: re-audit an existing clip directory and print the per_speed
block. Does not modify the clip.

    python3 tools/clip_audit.py clips/safe/<clip> [--speeds 1.0 0.5 0.25]
                                [--video OUT.mp4 --video-speed S]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import struct
import subprocess
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# Constants. Each one says where its value comes from.
# ---------------------------------------------------------------------------

# Repo root is one directory above tools/. In the teleop container tools/ is
# mounted at ~/ros2_ws/tools and src/ at ~/ros2_ws/src, so the same relative
# resolution works there.
REPO_ROOT = Path(__file__).resolve().parents[1]

# The composed model with a fixed base and the wrist roll/yaw contact exclude.
MODEL_REL_PATH = "src/g1_wuji2_description/g1_29_wuji2_fixed.xml"

# The model compiles with a 2 ms timestep. spec1 fixes it, and the hand slew
# step below depends on it, so a different model timestep is an error.
EXPECTED_TIMESTEP_S = 0.002

# Keyframe the audit resets to and holds legs and waist at.
STAND_KEYFRAME = "stand"

# Arm PD gains the G1 node writes to DDS: G1ArmController.kp_low / kd_low for
# shoulder and elbow joints, kp_wrist / kd_wrist for wrist roll, pitch, yaw
# (src/output_devices/g1_world_output/g1_world_output/robot_arm.py). The
# model's own kp 500 servos are replaced by these for the 14 arm actuators.
ARM_KP = 140.0
ARM_KD = 3.0
WRIST_KP = 50.0
WRIST_KD = 2.0

# Joint name fragments that select the wrist gain tier.
WRIST_JOINT_KEYS = ("wrist_roll", "wrist_pitch", "wrist_yaw")

# The hand driver slews its command toward the target at this joint speed
# (starport hand_node max_joint_velocity default). The audit applies the same
# limit to the hand actuator ctrl each physics step.
HAND_SLEW_RAD_S = 2.0

# Timeline (spec1, step 3): the first frame is approached from the stand pose
# over APPROACH_S with metrics off, the clip plays, then the last frame is
# held for HOLD_S with metrics on.
APPROACH_S = 2.0
HOLD_S = 0.5

# Judge thresholds (spec1, step 4). 80 N is the bundle authors' own
# deployment gate, kept as a starting number. 0.8 leaves 20 percent of the
# joint clamp for what the simulation does not model.
DEFAULT_MAX_ARM_TORQUE_RATIO = 0.8
DEFAULT_MAX_CONTACT_FORCE_N = 80.0

# Speeds audited by default (spec1). The publisher refuses speeds above 1.
DEFAULT_SPEEDS = (1.0, 0.5, 0.25)

# A clamped force equals its limit exactly in MuJoCo. Comparing at 0.999 of
# the limit avoids a float equality test and also counts forces a rounding
# error below the clamp.
SATURATION_TOL = 0.999

# How many contact pairs clip.json lists, by peak force (contract).
TOP_CONTACT_PAIRS = 5

# MJCF joint and actuator names for the G1 body are <name>_joint. The clip
# and the G1 node use <name> (G1_29_ARM_JOINT_NAMES in robot_arm.py).
MJCF_JOINT_SUFFIX = "_joint"

SIDES = ("left", "right")

# Arm joint names per side, the G1 node's order (robot_arm.py
# G1_29_ARM_JOINT_NAMES). This is the arm_q.npz column order.
ARM_JOINT_NAMES: Dict[str, List[str]] = {
    side: [
        f"{side}_shoulder_pitch",
        f"{side}_shoulder_roll",
        f"{side}_shoulder_yaw",
        f"{side}_elbow",
        f"{side}_wrist_roll",
        f"{side}_wrist_pitch",
        f"{side}_wrist_yaw",
    ]
    for side in SIDES
}

# Hand joint names per side in the driver's hardware order. This equals the
# movable-joint declaration order of src/wujihand_urdf/wujihand_{side}.urdf
# and the order of the hand joints in the MJCF once the "{side}_wuji_" prefix
# is removed. This is the hand_q20.npz column order.
HAND_JOINT_NAMES: Dict[str, List[str]] = {
    side: [
        f"{p}_thumb_cmc_flex", f"{p}_thumb_cmc_abd", f"{p}_thumb_mcp", f"{p}_thumb_ip",
        f"{p}_index_finger_mcp_flex", f"{p}_index_finger_mcp_abd",
        f"{p}_index_finger_pip", f"{p}_index_finger_dip",
        f"{p}_middle_finger_mcp_flex", f"{p}_middle_finger_mcp_abd",
        f"{p}_middle_finger_pip", f"{p}_middle_finger_dip",
        f"{p}_ring_finger_mcp_flex", f"{p}_ring_finger_mcp_abd",
        f"{p}_ring_finger_pip", f"{p}_ring_finger_dip",
        f"{p}_pinky_mcp_flex", f"{p}_pinky_mcp_abd", f"{p}_pinky_pip", f"{p}_pinky_dip",
    ]
    for side, p in (("left", "l"), ("right", "r"))
}

# MJCF hand actuator names are {side}_wuji_{l|r}_{code} with these codes, in
# hardware order: thumb (TH), first/index (FF), middle (MF), ring (RF),
# little/pinky (LF), four joints each.
HAND_ACTUATOR_CODES = (
    "THJ0", "THJ1", "THJ2", "THJ3",
    "FFJ0", "FFJ1", "FFJ2", "FFJ3",
    "MFJ0", "MFJ1", "MFJ2", "MFJ3",
    "RFJ0", "RFJ1", "RFJ2", "RFJ3",
    "LFJ0", "LFJ1", "LFJ2", "LFJ3",
)

# MJCF hand joint names carry this prefix in front of the hardware name.
HAND_MJCF_PREFIX = {"left": "left_wuji_", "right": "right_wuji_"}

# Video (contract): mujoco.Renderer at 480x640, 25 fps.
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 25

# Free camera framing the upper body from the front. Checked by rendering the
# stand pose: azimuth 180 faces the robot, 1.8 m fits both arms at full
# reach, a slight downward elevation keeps the hands in frame when they drop.
VIDEO_CAM_LOOKAT_BODY = "torso_link"
VIDEO_CAM_LOOKAT_Z_OFFSET_M = 0.05
VIDEO_CAM_DISTANCE_M = 1.8
VIDEO_CAM_AZIMUTH_DEG = 180.0
VIDEO_CAM_ELEVATION_DEG = -10.0

# x264 constant-rate-factor for the review video. 18 is the usual
# "visually lossless" setting; file size is not a concern for one clip.
FFMPEG_CRF = 18

# Small positive slack when converting simulation time to a frame index, so
# k * dt / period lands on the right integer at frame boundaries.
FRAME_INDEX_EPS = 1e-9


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def default_model_path() -> Path:
    """The composed fixed-base model, resolved from the repo root."""
    return REPO_ROOT / MODEL_REL_PATH


def sha256_file(path: Path) -> str:
    """Hex SHA-256 of a file, read in one go (files here are small)."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def name_id(model: mujoco.MjModel, objtype: mujoco.mjtObj, name: str) -> int:
    """mj_name2id that refuses a missing name.

    mj_name2id returns -1 for an unknown name, and -1 silently indexes the
    last element of every model array. Every lookup goes through here.
    """
    idx = mujoco.mj_name2id(model, objtype, name)
    if idx < 0:
        raise KeyError(f"{objtype.name} '{name}' not in model")
    return idx


def speed_key(speed: float) -> str:
    """clip.json per_speed key for a speed: '1.0', '0.5', '0.25'."""
    return str(float(speed))


def slew_toward(current: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    """Move current toward target by at most max_step per element."""
    return current + np.clip(target - current, -max_step, max_step)


@dataclass(frozen=True)
class ArmGains:
    """PD gains written to the 14 arm actuators. Defaults are the G1 node's."""
    kp: float = ARM_KP
    kd: float = ARM_KD
    kp_wrist: float = WRIST_KP
    kd_wrist: float = WRIST_KD

    def as_dict(self) -> dict:
        return {"kp": float(self.kp), "kd": float(self.kd),
                "kp_wrist": float(self.kp_wrist), "kd_wrist": float(self.kd_wrist)}


@dataclass(frozen=True)
class Thresholds:
    """Judge thresholds. A speed passes when both peaks are at or below."""
    max_arm_torque_ratio: float = DEFAULT_MAX_ARM_TORQUE_RATIO
    max_contact_force_n: float = DEFAULT_MAX_CONTACT_FORCE_N

    def as_dict(self) -> dict:
        return {"max_arm_torque_ratio": float(self.max_arm_torque_ratio),
                "max_contact_force_n": float(self.max_contact_force_n)}


def passes(peak_arm_torque_ratio: float, peak_contact_force_n: float,
           thresholds: Thresholds) -> bool:
    """spec1 step 4: pass when both peaks are within the thresholds."""
    return (peak_arm_torque_ratio <= thresholds.max_arm_torque_ratio
            and peak_contact_force_n <= thresholds.max_contact_force_n)


@dataclass
class AuditResult:
    """One speed's audit.

    summary is the per_speed block written to clip.json. The frame arrays
    are per clip frame and feed --auto-trim; they are not written.
    """
    summary: dict
    frame_torque_ratio: np.ndarray
    frame_contact_force_n: np.ndarray


# ---------------------------------------------------------------------------
# The rig
# ---------------------------------------------------------------------------

class AuditRig:
    """The model, loaded once, with the arm actuators re-gained.

    Builds every index it needs by name and checks each one, because the
    model's qpos order (hands nest under the wrists) is not the actuator
    order, and a wrong index would measure the wrong joint without error.
    """

    def __init__(self, model_path: Optional[Path] = None, gains: ArmGains = ArmGains()):
        self.model_path = Path(model_path) if model_path else default_model_path()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.model_sha256 = sha256_file(self.model_path)
        self.gains = gains
        m = self.model
        if not math.isclose(m.opt.timestep, EXPECTED_TIMESTEP_S):
            raise ValueError(f"model timestep {m.opt.timestep} != {EXPECTED_TIMESTEP_S}")
        if not np.all(m.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT):
            raise ValueError("every actuator must drive a joint")
        self.key_id = name_id(m, mujoco.mjtObj.mjOBJ_KEY, STAND_KEYFRAME)
        self._map_arms()
        self._map_hands()
        self._apply_arm_gains()
        # ctrl that holds the stand keyframe: each actuator's joint qpos.
        jnt_of_act = m.actuator_trnid[:, 0]
        self.ctrl_stand = m.key_qpos[self.key_id, m.jnt_qposadr[jnt_of_act]].copy()

    # -- index maps ----------------------------------------------------------

    def _map_arms(self) -> None:
        m = self.model
        aids, dofs, qposs, limits, wrist = [], [], [], [], []
        for side in SIDES:
            for name in ARM_JOINT_NAMES[side]:
                mjcf = name + MJCF_JOINT_SUFFIX
                jid = name_id(m, mujoco.mjtObj.mjOBJ_JOINT, mjcf)
                aid = name_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mjcf)
                if m.actuator_trnid[aid, 0] != jid:
                    raise ValueError(f"actuator {mjcf} does not drive joint {mjcf}")
                if not m.jnt_actfrclimited[jid] or m.jnt_actfrcrange[jid, 1] <= 0:
                    raise ValueError(f"joint {mjcf} has no actuatorfrcrange clamp")
                aids.append(aid)
                dofs.append(int(m.jnt_dofadr[jid]))
                qposs.append(int(m.jnt_qposadr[jid]))
                limits.append(float(m.jnt_actfrcrange[jid, 1]))
                wrist.append(any(k in name for k in WRIST_JOINT_KEYS))
        self.arm_aid = np.array(aids)
        self.arm_dof = np.array(dofs)
        self.arm_qpos = np.array(qposs)
        self.arm_frc_limit = np.array(limits)
        self.arm_is_wrist = np.array(wrist)

    def _map_hands(self) -> None:
        m = self.model
        aids, qposs, limits, lo, hi = [], [], [], [], []
        self.hand_jnt_range: Dict[str, np.ndarray] = {}
        for side in SIDES:
            prefix = HAND_MJCF_PREFIX[side]
            act_prefix = f"{side}_wuji_{side[0]}_"
            jids = []
            for k, code in enumerate(HAND_ACTUATOR_CODES):
                aid = name_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, act_prefix + code)
                jid = int(m.actuator_trnid[aid, 0])
                jname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
                expect = prefix + HAND_JOINT_NAMES[side][k]
                if jname != expect:
                    raise ValueError(f"actuator {act_prefix + code} drives {jname}, expected {expect}")
                if not m.actuator_forcelimited[aid]:
                    raise ValueError(f"hand actuator {act_prefix + code} is not forcelimited")
                if not np.allclose(m.actuator_ctrlrange[aid], m.jnt_range[jid]):
                    raise ValueError(f"hand actuator {act_prefix + code} ctrlrange != joint range")
                aids.append(aid)
                jids.append(jid)
                qposs.append(int(m.jnt_qposadr[jid]))
                limits.append(float(m.actuator_forcerange[aid, 1]))
                lo.append(float(m.actuator_ctrlrange[aid, 0]))
                hi.append(float(m.actuator_ctrlrange[aid, 1]))
            self.hand_jnt_range[side] = m.jnt_range[np.array(jids)].copy()
        self.hand_aid = np.array(aids)
        self.hand_qpos = np.array(qposs)
        self.hand_frc_limit = np.array(limits)
        self.hand_ctrl_lo = np.array(lo)
        self.hand_ctrl_hi = np.array(hi)

    def _apply_arm_gains(self) -> None:
        """Replace the arm actuators' position servo gains.

        MuJoCo's affine servo is force = gain[0]*ctrl + bias[0] + bias[1]*q
        + bias[2]*qdot. A PD position servo is kp*(ctrl - q) - kd*qdot, so
        gain[0] = kp, bias[1] = -kp, bias[2] = -kd. The joint-level
        actuatorfrcrange clamp is left as the model has it.
        """
        m = self.model
        g = self.gains
        for aid, wrist in zip(self.arm_aid, self.arm_is_wrist):
            if m.actuator_gaintype[aid] != mujoco.mjtGain.mjGAIN_FIXED:
                raise ValueError("arm actuator gain type is not fixed")
            if m.actuator_biastype[aid] != mujoco.mjtBias.mjBIAS_AFFINE:
                raise ValueError("arm actuator bias type is not affine")
            kp, kd = (g.kp_wrist, g.kd_wrist) if wrist else (g.kp, g.kd)
            m.actuator_gainprm[aid, 0] = kp
            m.actuator_biasprm[aid, 0] = 0.0
            m.actuator_biasprm[aid, 1] = -kp
            m.actuator_biasprm[aid, 2] = -kd

    # -- the replay --------------------------------------------------------

    def run(self, arm_q: Dict[str, np.ndarray], hand_q20: Dict[str, np.ndarray],
            rate_hz: float, speed: float, thresholds: Thresholds = Thresholds(),
            on_step: Optional[Callable[[mujoco.MjData], None]] = None) -> AuditResult:
        """Replay one clip at one speed and measure.

        arm_q[side] is (T, 7) in ARM_JOINT_NAMES order, hand_q20[side] is
        (T, 20) in HAND_JOINT_NAMES order. Frames advance every
        1 / (rate_hz * speed) seconds. on_step, if given, is called after
        every physics step (approach included) with the MjData.
        """
        m = self.model
        arm14 = np.concatenate([np.asarray(arm_q[s], dtype=np.float64) for s in SIDES], axis=1)
        hand40 = np.concatenate([np.asarray(hand_q20[s], dtype=np.float64) for s in SIDES], axis=1)
        n_frames = arm14.shape[0]
        if arm14.shape != (n_frames, 14) or hand40.shape != (n_frames, 40):
            raise ValueError(f"bad clip shapes {arm14.shape} {hand40.shape}")
        if n_frames < 1 or rate_hz <= 0 or speed <= 0:
            raise ValueError("need at least one frame, rate_hz > 0, speed > 0")

        dt = m.opt.timestep
        period = 1.0 / (rate_hz * speed)
        n_approach = int(round(APPROACH_S / dt))
        n_clip = int(round(n_frames * period / dt))
        n_hold = int(round(HOLD_S / dt))
        slew_step = HAND_SLEW_RAD_S * dt

        data = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, data, self.key_id)
        data.ctrl[:] = self.ctrl_stand
        arm_stand = self.ctrl_stand[self.arm_aid]
        hand_ctrl = self.ctrl_stand[self.hand_aid].copy()

        def write_ctrl(arm_target: np.ndarray, hand_target: np.ndarray) -> np.ndarray:
            nonlocal hand_ctrl
            hand_ctrl = np.clip(slew_toward(hand_ctrl, hand_target, slew_step),
                                self.hand_ctrl_lo, self.hand_ctrl_hi)
            data.ctrl[self.arm_aid] = arm_target
            data.ctrl[self.hand_aid] = hand_ctrl
            return hand_ctrl

        # Approach: metrics off.
        for k in range(n_approach):
            s = (k + 1) / n_approach
            write_ctrl(arm_stand + s * (arm14[0] - arm_stand), hand40[0])
            mujoco.mj_step(m, data)
            if on_step is not None:
                on_step(data)

        # Clip and hold: metrics on.
        frame_torque = np.zeros(n_frames)
        frame_force = np.zeros(n_frames)
        frame_contact = np.zeros(n_frames, dtype=bool)
        frame_arm_sat = np.zeros(n_frames, dtype=bool)
        frame_hand_sat = np.zeros(n_frames, dtype=bool)
        pair_peak: Dict[tuple, float] = {}
        arm_sq = 0.0
        hand_sq = 0.0
        n_steps = 0
        force6 = np.zeros(6)
        body_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]

        for k in range(n_clip + n_hold):
            t = k * dt
            frame = min(int((t + FRAME_INDEX_EPS) / period), n_frames - 1)
            if frame == 0:
                arm_target = arm14[0]
            else:
                # One publish period behind (contract decision 7): while
                # frame k is the newest published frame, interpolate from
                # frame k-1 to frame k.
                alpha = min(max((t - frame * period) / period, 0.0), 1.0)
                arm_target = arm14[frame - 1] + alpha * (arm14[frame] - arm14[frame - 1])
            hand_target = hand40[frame]
            write_ctrl(arm_target, hand_target)
            mujoco.mj_step(m, data)
            if on_step is not None:
                on_step(data)

            # Arms: the joint-level clamp is what the robot would feel.
            ratio = np.abs(data.qfrc_actuator[self.arm_dof]) / self.arm_frc_limit
            peak_ratio = float(ratio.max())
            frame_torque[frame] = max(frame_torque[frame], peak_ratio)
            if peak_ratio >= SATURATION_TOL:
                frame_arm_sat[frame] = True
            # Hands: force-limited servos; actuator_force is the clamped value.
            hand_force = np.abs(data.actuator_force[self.hand_aid])
            if np.any(hand_force >= SATURATION_TOL * self.hand_frc_limit):
                frame_hand_sat[frame] = True
            # Contacts: 3D force norm per contact, pair as sorted body names.
            if data.ncon > 0:
                frame_contact[frame] = True
                for i in range(data.ncon):
                    c = data.contact[i]
                    mujoco.mj_contactForce(m, data, i, force6)
                    f = float(np.linalg.norm(force6[:3]))
                    pair = tuple(sorted((body_names[m.geom_bodyid[c.geom1]],
                                         body_names[m.geom_bodyid[c.geom2]])))
                    if f > pair_peak.get(pair, 0.0):
                        pair_peak[pair] = f
                    frame_force[frame] = max(frame_force[frame], f)
            # Tracking: arms against the interpolated target, hands against
            # the clip frame (the slew is plant lag, not target).
            arm_sq += float(np.sum((arm_target - data.qpos[self.arm_qpos]) ** 2))
            hand_sq += float(np.sum((hand_target - data.qpos[self.hand_qpos]) ** 2))
            n_steps += 1

        ranked = sorted(pair_peak.items(), key=lambda kv: kv[1], reverse=True)
        peak_force = float(ranked[0][1]) if ranked else 0.0
        peak_pair = list(ranked[0][0]) if ranked else []
        peak_ratio_all = float(frame_torque.max())
        summary = {
            "pass": bool(passes(peak_ratio_all, peak_force, thresholds)),
            "peak_arm_torque_ratio": peak_ratio_all,
            "peak_contact_force_n": peak_force,
            "peak_contact_pair": peak_pair,
            "contact_frame_fraction": float(frame_contact.mean()),
            "arm_saturation_fraction": float(frame_arm_sat.mean()),
            "hand_saturation_fraction": float(frame_hand_sat.mean()),
            "tracking_rmse_rad": {
                "arms": float(np.sqrt(arm_sq / (n_steps * 14))),
                "hands": float(np.sqrt(hand_sq / (n_steps * 40))),
            },
            "top_contact_pairs": [[a, b, float(f)] for (a, b), f in ranked[:TOP_CONTACT_PAIRS]],
        }
        return AuditResult(summary=summary, frame_torque_ratio=frame_torque,
                           frame_contact_force_n=frame_force)

    def audit_meta(self, speeds, thresholds: Thresholds, note: str = "") -> dict:
        """The fixed part of clip.json's audit block (everything but per_speed)."""
        return {
            "model": self.model_path.name,
            "model_sha256": self.model_sha256,
            "mujoco_version": mujoco.mj_versionString(),
            "timestep": float(self.model.opt.timestep),
            "arm_gains": self.gains.as_dict(),
            "hand_command_slew_rad_s": float(HAND_SLEW_RAD_S),
            "thresholds": thresholds.as_dict(),
            "speeds": [float(s) for s in speeds],
            "note": note,
        }


def audit_clip(arm_q: Dict[str, np.ndarray], hand_q20: Dict[str, np.ndarray],
               rate_hz: float, speed: float, model_path: Optional[Path] = None,
               gains: ArmGains = ArmGains(), thresholds: Thresholds = Thresholds()) -> dict:
    """One clip, one speed, one call. Returns the per_speed block of clip.json.

    Loads the model each time. Use AuditRig directly for several speeds.
    """
    rig = AuditRig(model_path, gains)
    return rig.run(arm_q, hand_q20, rate_hz, speed, thresholds).summary


# ---------------------------------------------------------------------------
# Clip directory reading
# ---------------------------------------------------------------------------

def load_clip_dir(clip_dir: Path):
    """Read arm_q.npz, hand_q20.npz and clip.json. Checks names and shapes."""
    clip_dir = Path(clip_dir)
    meta = json.loads((clip_dir / "clip.json").read_text())
    arm_npz = np.load(clip_dir / "arm_q.npz")
    hand_npz = np.load(clip_dir / "hand_q20.npz")
    arm_q = {s: np.asarray(arm_npz[s], dtype=np.float64) for s in SIDES}
    hand_q20 = {s: np.asarray(hand_npz[s], dtype=np.float64) for s in SIDES}
    n = int(meta["frames"])
    for s in SIDES:
        if meta["arm_joint_names"][s] != ARM_JOINT_NAMES[s]:
            raise ValueError(f"{clip_dir}: arm_joint_names[{s}] differ from the audit's")
        if meta["hand_joint_names"][s] != HAND_JOINT_NAMES[s]:
            raise ValueError(f"{clip_dir}: hand_joint_names[{s}] differ from the audit's")
        if arm_q[s].shape != (n, 7) or hand_q20[s].shape != (n, 20):
            raise ValueError(f"{clip_dir}: bad shapes for {s}")
    return arm_q, hand_q20, meta


# ---------------------------------------------------------------------------
# Video
# ---------------------------------------------------------------------------

def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal PNG writer (8-bit RGB) so the fallback needs no image library."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw))
           + chunk(b"IEND", b""))
    Path(path).write_bytes(png)


def render_video(clip_dir: Path, speed: float, out_mp4: Path, model_path: Optional[Path] = None,
                 png_dir: Optional[Path] = None, png_stride: int = 1) -> Path:
    """Render one audited replay (approach, clip, hold) at VIDEO_FPS.

    Encodes with ffmpeg when it is on PATH. Otherwise writes PNG frames to
    png_dir (default: <out_mp4 stem>_frames next to out_mp4), keeping every
    png_stride-th video frame. Returns the path written.
    """
    arm_q, hand_q20, meta = load_clip_dir(clip_dir)
    rig = AuditRig(model_path)
    m = rig.model
    out_mp4 = Path(out_mp4)
    steps_per_frame = int(round(1.0 / (VIDEO_FPS * m.opt.timestep)))

    renderer = mujoco.Renderer(m, height=VIDEO_HEIGHT, width=VIDEO_WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = VIDEO_CAM_DISTANCE_M
    cam.azimuth = VIDEO_CAM_AZIMUTH_DEG
    cam.elevation = VIDEO_CAM_ELEVATION_DEG
    lookat_body = name_id(m, mujoco.mjtObj.mjOBJ_BODY, VIDEO_CAM_LOOKAT_BODY)
    stand = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, stand, rig.key_id)
    mujoco.mj_forward(m, stand)
    cam.lookat[:] = stand.xpos[lookat_body] + np.array([0.0, 0.0, VIDEO_CAM_LOOKAT_Z_OFFSET_M])

    ffmpeg = shutil.which("ffmpeg")
    proc = None
    if ffmpeg:
        out_mp4.parent.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{VIDEO_WIDTH}x{VIDEO_HEIGHT}", "-r", str(VIDEO_FPS), "-i", "-",
             "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-crf", str(FFMPEG_CRF),
             str(out_mp4)],
            stdin=subprocess.PIPE)
        target = out_mp4
    else:
        target = Path(png_dir) if png_dir else out_mp4.parent / f"{out_mp4.stem}_frames"
        target.mkdir(parents=True, exist_ok=True)

    counter = {"step": 0, "frame": 0}

    def on_step(data: mujoco.MjData) -> None:
        counter["step"] += 1
        if counter["step"] % steps_per_frame:
            return
        idx = counter["frame"]
        counter["frame"] += 1
        if proc is None and idx % png_stride:
            return
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render()
        if proc is not None:
            proc.stdin.write(np.ascontiguousarray(rgb).tobytes())
        else:
            write_png(target / f"frame_{idx:05d}.png", rgb)

    try:
        rig.run(arm_q, hand_q20, float(meta["rate_hz"]), speed, on_step=on_step)
    finally:
        renderer.close()
        if proc is not None:
            proc.stdin.close()
            proc.wait()
    if proc is not None and proc.returncode != 0:
        raise RuntimeError(f"ffmpeg exited with {proc.returncode}")
    return target


# ---------------------------------------------------------------------------
# Command line: re-audit an existing clip directory
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Re-audit a clip directory and print its per_speed block.")
    p.add_argument("clip_dir", type=Path)
    p.add_argument("--speeds", type=float, nargs="+", default=None,
                   help="speeds to audit (default: the clip's audit.speeds)")
    p.add_argument("--model", type=Path, default=None)
    p.add_argument("--max-arm-torque-ratio", type=float, default=None,
                   help="default: the clip's threshold")
    p.add_argument("--max-contact-force-n", type=float, default=None,
                   help="default: the clip's threshold")
    p.add_argument("--video", type=Path, default=None, help="render one speed to this mp4")
    p.add_argument("--video-speed", type=float, default=None,
                   help="speed for --video (default: fastest audited)")
    p.add_argument("--png-dir", type=Path, default=None,
                   help="where PNG frames go when ffmpeg is missing")
    p.add_argument("--png-stride", type=int, default=1,
                   help="keep every N-th video frame when writing PNGs")
    args = p.parse_args(argv)

    arm_q, hand_q20, meta = load_clip_dir(args.clip_dir)
    audit = meta["audit"]
    speeds = args.speeds or audit["speeds"]
    thr = Thresholds(
        max_arm_torque_ratio=(args.max_arm_torque_ratio
                              if args.max_arm_torque_ratio is not None
                              else audit["thresholds"]["max_arm_torque_ratio"]),
        max_contact_force_n=(args.max_contact_force_n
                             if args.max_contact_force_n is not None
                             else audit["thresholds"]["max_contact_force_n"]))
    rig = AuditRig(args.model)
    per_speed = {}
    for s in speeds:
        per_speed[speed_key(s)] = rig.run(arm_q, hand_q20, float(meta["rate_hz"]), s, thr).summary
    print(json.dumps(per_speed, indent=1))
    if args.video is not None:
        vs = args.video_speed if args.video_speed is not None else max(speeds)
        written = render_video(args.clip_dir, vs, args.video, args.model,
                               args.png_dir, args.png_stride)
        print(f"video: {written} (speed {vs})", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
