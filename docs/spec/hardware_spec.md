# Hardware Spec: This Rig

Fork is NOT general purpose. Currently drives rig: Unitree G1_23 + dual Wuji hand 2 teleoped from Wuji Gloves and PICO 4 headset. 

Upstream supported hardware we do not have (Tianji arm, HTC trackers, MANUS glove, D405/HBVCAM cameras). That code has now been removed; see [cleanup.md](cleanup.md).

This file is the source of truth for what our hardware is and what the code assumed. 

- [Inventory](#inventory): what we have.
- [Not in this rig](#not-in-this-rig): upstream hardware we don't use.
- [Where G1_23 is baked into the code](#where-g1_23-is-baked-into-the-code): per-file variant assumptions.
- [Open hardware items](#open-hardware-items): unresolved gaps, with owners.

## Inventory

### Robot: Unitree G1, 23-DoF variant

Confirmed 2026-08-24: the robots are the **23-DoF** G1, not the 29-DoF.

| Property | Value |
|---|---|
| DoF layout | 12 leg + 1 waist (yaw only) + 5 per arm (shoulder pitch/roll/yaw, elbow, wrist roll) |
| Not present vs. 29-DoF | wrist pitch, wrist yaw, waist roll, waist pitch; link inertias also differ |
| Control interface | `unitree_sdk2py` DDS (`rt/lowcmd` or `rt/arm_sdk`) |

#### Joint indices and limits (G1_23)

| Index | Joint Name | Limit (rad) |
|---|---|---|
| 0 | L_LEG_HIP_PITCH | -2.5307~2.8798 |
| 1 | L_LEG_HIP_ROLL | -0.5236~2.9671 |
| 2 | L_LEG_HIP_YAW | -2.7576~2.7576 |
| 3 | L_LEG_KNEE | -0.087267~2.8798 |
| 4 | L_LEG_ANKLE_PITCH | -0.87267~0.5236 |
| 5 | L_LEG_ANKLE_ROLL | -0.2618~0.2618 |
| 6 | R_LEG_HIP_PITCH | -2.5307~2.8798 |
| 7 | R_LEG_HIP_ROLL | -2.9671~0.5236 |
| 8 | R_LEG_HIP_YAW | -2.7576~2.7576 |
| 9 | R_LEG_KNEE | -0.087267~2.8798 |
| 10 | R_LEG_ANKLE_PITCH | -0.87267~0.5236 |
| 11 | R_LEG_ANKLE_ROLL | -0.2618~0.2618 |
| 12 | WAIST_YAW | -2.618~2.618 |
| 13 | WAIST_ROLL | -0.52~0.52 |
| 14 | WAIST_PITCH | -0.52~0.52 |
| 15 | L_SHOULDER_PITCH | -3.0892~2.6704 |
| 16 | L_SHOULDER_ROLL | -1.5882~2.2515 |
| 17 | L_SHOULDER_YAW | -2.618~2.618 |
| 18 | L_ELBOW | -1.0472~2.0944 |
| 19 | L_WRIST_ROLL | -1.972222054~1.972222054 |
| 20 | L_WRIST_PITCH | -1.614429558~1.614429558 |
| 21 | L_WRIST_YAW | -1.614429558~1.614429558 |
| 22 | R_SHOULDER_PITCH | -3.0892~2.6704 |
| 23 | R_SHOULDER_ROLL | -2.2515~1.5882 |
| 24 | R_SHOULDER_YAW | -2.618~2.618 |
| 25 | R_ELBOW | -1.0472~2.0944 |
| 26 | R_WRIST_ROLL | -1.972222054~1.972222054 |
| 27 | R_WRIST_PITCH | -1.614429558~1.614429558 |
| 28 | R_WRIST_YAW | -1.614429558~1.614429558 |

Note: indices 13, 14, 20, 21, 27, 28 (waist roll/pitch, wrist pitch/yaw)
are the 29-DoF joints not present on this rig's 23-DoF robot; see
[Where G1_23 is baked into the code](#where-g1_23-is-baked-into-the-code).

### End effectors: 2x Wuji Hand 2

| Property | Value |
|---|---|
| Model | Wuji Hand 2, 20 actuated DoF per hand |
| Connection | USB (VID:PID `0483:2000`); firmware v1.2.1+ recommended (upstream README) |
| Milestone 1 | Sim-only; no physical hand in the loop |

### Mounting adapter: does not exist yet

Measured 2026-08-22: the vendor's `unitree-g1-docking-adapter.stl` is a **Wuji
Hand v1 part and does not fit Hand 2**. A Hand 2 adapter redesign is pending;
printing is on hold. Until the CAD lands, `g1_wuji2_description` uses a
provisional mount: hand at the ICP-located palm flange on the wrist-roll link
(the 23's terminal arm link), wrist_roll + [0.1220, +-0.003, 0], zero plate
thickness. The earlier 29-DoF mount (x = 0.0415 m on `wrist_yaw_link`) is
obsolete: the 23-DoF robot has no wrist yaw link.

### Input devices

| Device | Details |
|---|---|
| Wuji Glove, left + right | UDP on the glove LAN (factory default `192.168.1.100` left, `.101` right, port 50001); calibration via Wuji Studio 5.18 into `~/.wuji/sdk/params/<SN>.toml`; serials bound in `wuji_glove.yaml` |
| PICO 4 headset | XRoboToolkit APK + PC-Service (vendored); ADB link to host |
| PICO Motion Trackers, **4 required** | `pico_input` binds four unique serials: left/right **wrist** + left/right **forearm** (`pico_input.yaml`); an empty or duplicate SN makes the node fail. Milestone 1 names only the headset; the current `pico_input` cannot produce wrist poses without all four trackers |

### Host

Ubuntu 22.04 LTS x86_64, Docker (the only supported runtime). No GPU
required: NVENC only accelerates camera streaming, and cameras are not in
this rig.

## Not in this rig

Tianji Arm (and the Monitor `brake` tool), HTC Vive Tracker / SteamVR, and
the MANUS glove were upstream-only and **their code has been removed** — see
[cleanup.md](cleanup.md). Our arm output is `g1_world_output`, consuming the
PICO-path topic contract (`/left_arm_target_pose`,
`/right_arm_target_pose`).

The camera package `src/camera/` is **kept but unwired**. It targets D405 wrist
+ HBVCAM head hardware we do not have, and the G1's own head cameras (RealSense
D435i built-in, D455 attachment) are planned but not yet integrated. Nothing
launches it; see [wuji-camera-topics.md](wuji-camera-topics.md).

## Where G1_23 is baked into the code

Both sides target G1_23 as of 2026-08-24: the controller solves the 10
arm DoF the robot has, and `g1_wuji2_description` is composed from
Unitree's `g1_23dof_rev_1_0` sources (bare wrist module derived, since
Unitree only ships the wrist fused with the rubber hand). Remaining
29-DoF residue is limited to two config-level values noted below.

<details>
<summary>Per-file assumptions</summary>

| File | Assumption |
|---|---|
| `g1_world_output/config/g1_robot.yaml` | `arm_type: "G1_23"`. Also `chest_origin_in_pelvis` was derived from the 29-DoF body URDF ("waist_roll z + shoulder pitch xyz"); re-derivation against the 23-DoF description is still pending |
| `g1_world_output/g1_world_output/robot_arm.py` | Unitree's unified 35-slot motor array with the 23-DoF gaps: arms at indices 15-19 / 22-26; waist roll/pitch (13, 14) and wrist pitch/yaw (20, 21, 27, 28) declared NotUsed; `rt/arm_sdk` weight flag at motor 29. DDS write loop at 250 Hz; per-joint kp/kd tiers (300/5 high, 140/3 low, 50/2 wrist); velocity clip ramps 20 to 30 deg/s over 5 s |
| `g1_world_output/g1_world_output/robot_arm_ik.py` | Loads `g1_23_wuji2.urdf` (23-DoF composed model), then locks legs, waist, and fingers to the 10 arm DoF; the lock list filters by joint presence, so the absent wrist pitch/yaw entries are simply skipped. EE frames `L_ee`/`R_ee` sit on the wrist-roll links with a +0.20 m forward offset (xr_teleoperate convention); this shifts the achieved palm pose by a constant wrist-frame vector, fix planned alongside the adapter regeneration. Cost weights are xr_teleoperate's G1_23 set (rotation down-weighted) |
| `src/g1_wuji2_description/` | Composed from `g1_23dof_rev_1_0` + 2x Wuji Hand 2 (2026-08-24, matches hardware): floating nq 70 / nv 69 / nu 63, fixed-base 63. Fused wrist+rubber-hand link replaced with a derived bare wrist module; flange at wrist_roll + [0.1220, +-0.003, 0]. The 29-DoF files were removed (kept in msc_research as `g1_29_wuji2*`). Generated files; do not hand-edit (see the package README) |

</details>
