# SignAR → G1 + Wuji Hand 2 integration handoff

Generated: 2026-08-23T00:07:08.018220+00:00
Source batch: `/data/yujiazeng/DexterousGPT/experiments/signar_gt_ours15_physical_v7_2_normal_complete_visualization_r1`
Samples: 15

## Safety and model identity

This is a **reference/integration bundle**, not a directly executable robot command package.
The SignAR simulation that produced the included robot videos uses the legacy Wuji 20-DoF
MuJoCo model. The real platform is Wuji Hand 2. Therefore:

- Do not publish any file under `legacy_wuji_sim_only/` to Wuji Hand 2.
- Recompute hand joint angles from each method's `*_human_targets_v5.npz` using the current
  Wuji SDK with `HandModel.WujiHand2`, preserving the exact sample timing.
- Use the official Hand 2 URDF/MJCF revision matching the delivered hardware and exact
  left/right mount transforms.
- For a free-standing G1, keep a balance-capable Unitree controller active; do not replay the
  fixed-base MuJoCo torque controller on hardware.
- Begin on a protective rack or other supported setup with conservative speed/amplitude,
  an operator on the E-stop, and a cleared workspace.

## Per-method files

`GT/` and `Ours/` each contain:

- `hand2_input/*_human_targets_v5.npz`: source-space MediaPipe-order 21-point hand keypoints,
  palm bases, elbow geometry, contact flags, and SMPL-X body reference. Important keys include
  `left_hand_keypoints21` and `right_hand_keypoints21` with shape `[T,21,3]`, in metres.
- `g1_reference/controller_reference_v7.npz`: retimed G1 body q/dq/ddq reference and timing.
- `g1_reference/motor_targets.csv`: readable 29-joint simulation reference.
- `g1_reference/target_meta.json`: target FPS, timing profile, joint/model metadata.
- `g1_reference/retime_report_v7.json`: timing/path-preservation report.
- `audits/`: kinematic and retiming gate outputs plus physical summary/force trace.
- `videos/`: kinematic target and physical-controller simulation videos.
- `legacy_wuji_sim_only/hand_targets.csv`: legacy-hand simulation reference only.

The G1 29-joint ordering in the current reference is:
12 leg joints, 3 waist joints, and 14 arm joints. On a standing G1, the real controller should
normally consume the reviewed waist/arm subset while the vendor balance controller remains active;
it should not replay the leg portion from this fixed-base simulation.

## Required information from the hardware team

Return or record before motion tests:

1. G1 serial/model variant, firmware versions, `unitree_sdk2` commit/version, and selected control mode.
2. Whether the test is protective-rack/supported or free-standing.
3. Wuji Hand 2 hardware revision (Beta 1/Beta 2), left/right serial numbers, firmware version,
   `wuji-sdk` version, and the exact official URDF/MJCF used.
4. Hand 2 joint labels and indices reported by the SDK; online-joint count must be 20 per hand.
5. Neutral-state logs, calibrated zero/origin values, side assignment, and G1-to-hand mount transforms.
6. Per-joint position, velocity, effort/current and temperature limits; watchdog and E-stop behavior.
7. Timestamped command/state logs from every hardware run.

## Recommended first hardware test

Do not start with all 15 clips. Select one low-contact, low-motion sample from the batch audit,
then test: read-only state → neutral hold → one arm at reduced amplitude/speed → both arms with
hands open → Hand 2 articulation → full synchronized clip → bimanual contact only after prior stages pass.
