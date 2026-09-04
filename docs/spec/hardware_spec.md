# Hardware Spec: This Rig

Fork is NOT general purpose. Currently drives rig: Unitree G1 (29-DoF primary;
23-DoF supported secondary) + dual Wuji hand 2 teleoped from Wuji Gloves and
PICO 4 headset.

Upstream supported hardware we do not have (Tianji arm, HTC trackers, MANUS glove, D405/HBVCAM cameras). That code has now been removed; see [cleanup.md](../deprecated/cleanup.md).

This file is the source of truth for what our hardware is and what the code assumed. 

- [Inventory](#inventory): what we have.
- [Not in this rig](#not-in-this-rig): upstream hardware we don't use.
- [Where the DoF variant is baked into the code](#where-the-dof-variant-is-baked-into-the-code): per-file variant assumptions.

## Inventory

### Robot: Unitree G1, 23-DoF and 29-DoF

History: the units seen up to 2026-08-24 were the **23-DoF** G1 (confirmed that
day); the 29-DoF variant came back into scope on 2026-08-26. **Resolved
2026-08-27: the rig's robot is the 29-DoF G1.** The 23-DoF variant remains a
supported secondary target. Code, models, and configs carry both; defaults are
G1_29 (commit 5ce3ea8).

| Property | 23-DoF | 29-DoF |
|---|---|---|
| Legs | 12 | 12 |
| Waist | yaw only | yaw, roll, pitch |
| Arm, per side | 5: shoulder pitch/roll/yaw, elbow, wrist roll | 7: adds wrist pitch, wrist yaw |
| Total | 23 | 29 |
| Control interface | `unitree_sdk2py` DDS (`rt/lowcmd` or `rt/arm_sdk`) | same |

Link inertias also differ. The 23-DoF robot is not the 29-DoF robot with
joints removed.

Network and firmware, measured on the rig 2026-09-01: the robot answers at
`192.168.123.161` and the host NIC sits on the `192.168.123.0/24` subnet,
pinned by `g1_robot.yaml` `network_interface`:

- black diamond linux `enx00e04c3a0398`
- nathan linux `enx00051bc62afa`.

The Unitree SDK builds its own CycloneDDS config and ignores `CYCLONEDDS_URI`,
so that parameter is the only thing that binds the robot link to the right
interface on a multi-NIC host; left empty, the SDK takes the first interface
and the only symptom is the lowstate timeout. All 14 arm joints including
wrist pitch and yaw track over `rt/arm_sdk` (which confirms the 29-DoF
variant), the lowstate tick is 1000 Hz, `mode_machine` is 5 in the standing
mode used for replay, and `unitree-sdk2py` is 1.0.1. **This rig has no
dedicated e-stop:** the physical stop is the remote damp command or main
power.

#### Joint indices and limits

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

Note: this is Unitree's motor array, and it is indexed the same way for both
variants. On a 29-DoF robot, indices 13, 14, 20, 21, 27, 28 (waist roll/pitch,
wrist pitch/yaw) are real joints. On a 23-DoF robot they are absent, and
`robot_arm.py` declares those slots NotUsed. See
[Where the DoF variant is baked into the code](#where-the-dof-variant-is-baked-into-the-code).

### End effectors: 2x Wuji Hand 2

| Property | Value |
|---|---|
| Model | Wuji Hand 2, 20 actuated DoF per hand |
| Connection | **Ethernet** (decided 2026-09-02): each hand has a static IP on its own subnet, is discovered by UDP broadcast, and is selected by serial number; driver `starport_wuji_hand` over `wuji_sdk` ([spec1.md](spec1.md)). The tree still runs the USB driver (`wujihandros2`, VID:PID `0483:2000`) until the swap lands. IPs, subnet, and firmware versions: unrecorded |
| Serial numbers, IPs, revision (Beta 1 or Beta 2), firmware | unrecorded; fill from the rig |

### Mounting adapter: does not exist yet

Measured 2026-08-22: the vendor's `unitree-g1-docking-adapter.stl` is a **Wuji
Hand v1 part and does not fit Hand 2**. A Hand 2 adapter redesign is pending;
printing is on hold. Until the CAD lands, `g1_wuji2_description` uses a
provisional mount: hand at the ICP-located palm flange on the wrist-roll link
(the 23's terminal arm link), wrist_roll + [0.1220, +-0.003, 0], zero plate
thickness. The flange is per variant: the terminal arm link is `wrist_roll` on
the 23-DoF arm and `wrist_yaw_link` on the 29-DoF arm (earlier mount there:
x = 0.0415 m). A composed model for either variant needs its own mount
transform.

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
[cleanup.md](../deprecated/cleanup.md). Our arm output is `g1_world_output`, consuming the
PICO-path topic contract (`/left_arm_target_pose`,
`/right_arm_target_pose`).

The camera package `src/camera/` is **kept but unwired**. It targets D405 wrist
+ HBVCAM head hardware we do not have, and the G1's own head cameras (RealSense
D435i built-in, D455 attachment) are planned but not yet integrated. Nothing
launches it; see [wuji-camera-topics.md](../wuji-camera-topics.md).

## Where the DoF variant is baked into the code

Update 2026-08-27 (commit 5ce3ea8): `G1ArmController(arm_type)` now drives
either variant over DDS and the default is `G1_29`; pose IK remains G1_23-only.
The table below predates that commit and stands as the record of what each file
assumed; the "For 29-DoF" column of the `robot_arm.py` row is now landed.
`g1_wuji2_description`'s 23-DoF files are composed from Unitree's
`g1_23dof_rev_1_0` sources (bare wrist module derived, since Unitree only ships
the wrist fused with the rubber hand); the package carries both variants as of
2026-08-26.

<details>
<summary>Per-file assumptions</summary>

| File | Assumption | For 29-DoF |
|---|---|---|
| `g1_world_output/config/g1_robot.yaml` | `arm_type: "G1_23"`. Also `chest_origin_in_pelvis` was derived from the 29-DoF body URDF ("waist_roll z + shoulder pitch xyz"); re-derivation against the 23-DoF description is still pending | Needs a `G1_29` value and the 7-DoF arm joint list. `chest_origin_in_pelvis` is already the 29-DoF derivation |
| `g1_world_output/g1_world_output/robot_arm.py` | Unitree's unified 35-slot motor array with the 23-DoF gaps: arms at indices 15-19 / 22-26; waist roll/pitch (13, 14) and wrist pitch/yaw (20, 21, 27, 28) declared NotUsed; `rt/arm_sdk` weight flag at motor 29. DDS write loop at 250 Hz; per-joint kp/kd tiers (300/5 high, 140/3 low, 50/2 wrist); velocity clip fixed at 20 **rad**/s (a 20-to-30 rad/s ramp exists in code but is never invoked) | The six NotUsed slots become real joints: the arm index enum grows from 10 to 14 entries, `G1_23_ARM_DOF` stops being 10, and the wrist kp/kd tier covers 3 wrist joints per arm instead of 1 |
| `g1_world_output/g1_world_output/robot_arm_ik.py` | Loads `g1_23_wuji2.urdf` (23-DoF composed model), then locks legs, waist, and fingers to the 10 arm DoF; the lock list filters by joint presence, so the absent wrist pitch/yaw entries are simply skipped. EE frames `L_ee`/`R_ee` sit on the wrist-roll links with a +0.20 m forward offset (xr_teleoperate convention); this shifts the achieved palm pose by a constant wrist-frame vector, fix planned alongside the adapter regeneration. Cost weights are xr_teleoperate's G1_23 set (rotation down-weighted) | Loads the 29-DoF URDF and solves 14 DoF instead of 10. The lock list filters by joint presence, so it already tolerates either joint set; the cost weights would need revisiting |
| `src/g1_wuji2_description/` | Composed from `g1_23dof_rev_1_0` + 2x Wuji Hand 2 (2026-08-24, matches the 23-DoF units seen): floating nq 70 / nv 69 / nu 63, fixed-base 63. Fused wrist+rubber-hand link replaced with a derived bare wrist module; flange at wrist_roll + [0.1220, +-0.003, 0]. Generated files; do not hand-edit (see the package README) | Present since 2026-08-26: `g1_29_wuji2{,_fixed}.xml`, `g1_29_wuji2.urdf`, `scene_g1_29_wuji2.xml`; floating nq 76 / nv 75 / nu 69, fixed-base 69. Hand mounts on `wrist_yaw_link` + [0.0415, 0, 0]. `meshes/g1/` is shared by both variants |

</details>
