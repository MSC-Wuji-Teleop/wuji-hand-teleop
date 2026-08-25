# Hardware Spec: This Rig

Fork is NOT general purpose. Currently drives rig: Unitree G1_23 + dual Wuji hand 2 teleoped from Wuji Gloves and PICO 4 headset. 

Upstream docs describe hardware we do not have (Tianji arm, HTC trackers, cameras). 

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
provisional mount: hand at the Unitree palm flange plane (x = 0.0415 m on
`wrist_yaw_link`), zero plate thickness. Note the flange parent itself changes
with the 23-DoF rebuild (the 23's terminal arm link is wrist roll, there is no
wrist yaw link).

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

Tianji Arm (and the Monitor `brake` tool), HTC Vive Tracker / SteamVR, the
head stereo + D405 wrist cameras, and the MANUS glove are upstream-only; our
arm output is `g1_world_output`, consuming the PICO-path topic contract
(`/left_arm_target_pose`, `/right_arm_target_pose`).

## Where G1_23 is baked into the code

The controller side already targets G1_23; the robot description does not
yet. IK solutions and DDS commands match the real robot's kinematics today
(wrist pitch/yaw never move), but the sim model's inertias, terminal wrist
link, and actuator count are 29-DoF artifacts until the rebuild.

<details>
<summary>Per-file assumptions</summary>

| File | Assumption |
|---|---|
| `g1_world_output/config/g1_robot.yaml` | `arm_type: "G1_23"`. Also `chest_origin_in_pelvis` was derived from the 29-DoF body URDF ("waist_roll z + shoulder pitch xyz"); re-derive after the description rebuild |
| `g1_world_output/g1_world_output/robot_arm.py` | Unitree's unified 35-slot motor array with the 23-DoF gaps: arms at indices 15-19 / 22-26; waist roll/pitch (13, 14) and wrist pitch/yaw (20, 21, 27, 28) declared NotUsed; `rt/arm_sdk` weight flag at motor 29. DDS write loop at 250 Hz; per-joint kp/kd tiers (300/5 high, 140/3 low, 50/2 wrist); velocity clip ramps 20 to 30 deg/s over 5 s |
| `g1_world_output/g1_world_output/robot_arm_ik.py` | Loads `g1_23_wuji2.urdf` (23-DoF composed model), then locks legs, waist, and fingers to the 10 arm DoF; the lock list filters by joint presence, so the absent wrist pitch/yaw entries are simply skipped. EE frames `L_ee`/`R_ee` sit on the wrist-roll links with a +0.20 m forward offset (xr_teleoperate convention); this shifts the achieved palm pose by a constant wrist-frame vector, fix planned alongside the adapter regeneration. Cost weights are xr_teleoperate's G1_23 set (rotation down-weighted) |
| `src/g1_wuji2_description/` | Composed from `g1_23dof_rev_1_0` + 2x Wuji Hand 2 (2026-08-24, matches hardware): floating nq 70 / nv 69 / nu 63, fixed-base 63. Fused wrist+rubber-hand link replaced with a derived bare wrist module; flange at wrist_roll + [0.1220, +-0.003, 0]. The 29-DoF files were removed (kept in msc_research as `g1_29_wuji2*`). Generated files; do not hand-edit (see the package README) |

</details>
