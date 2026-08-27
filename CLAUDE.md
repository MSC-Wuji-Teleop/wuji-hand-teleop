# CLAUDE.md

## Overview

ROS2 (Humble) teleoperation stack for one rig: a **Unitree G1 (23-DoF)** with **2x Wuji Hand 2**, driven by **Wuji Gloves** for the hands and a **PICO 4** headset with 4 Motion Trackers for the arms. The repo lives inside a colcon workspace (`~/ros2_ws/src/wuji-hand-teleop`); its `src/` bind-mounts into the `wuji-hand-teleop` Docker container as the workspace package source.

**Docker is the only supported runtime.** `docker/Dockerfile` is the source of truth for the environment (apt/pip versions, SDKs). One exception to the single-container picture: `g1_world_output` runs as its own image/container because its Pinocchio+CasADi build needs NumPy 1.x while the rest of the stack needs 2.x.

The hardware source of truth is [docs/spec/hardware_spec.md](docs/spec/hardware_spec.md): the physical G1 is the **23-DoF** variant; `g1_wuji2_description` carries both the matching `g1_23_wuji2*` files and, since 2026-08-27, `g1_29_wuji2*` (29-DoF, used for SOT bundle replay in sim only). The upstream Tianji arm, HTC/SteamVR, and MANUS code was removed on 2026-08-25; what went and why is in [docs/deprecated/cleanup.md](docs/deprecated/cleanup.md). `src/camera/` is kept but unwired, pending the G1 head cameras.

## Commands

Full reference: [docs/usage.md](docs/usage.md). The essentials, inside the container:

```bash
colcon build --symlink-install                  # after code changes
colcon test --packages-select <package_name>
colcon test-result --verbose
python3 -m pytest src/controller/test/          # direct pytest also works
```

Host code edits are live via the bind-mount + symlink install; rerun `colcon build --symlink-install` after adding new files or templates. Rebuild the image (`cd docker && docker compose build`) only for `Dockerfile` or `prebuilt/` changes. If a package fails to import after a pull: `git submodule update --init --recursive`, then rebuild.

## Architecture

Data flow: input device, standard topic/TF interface, output controller, hardware. Full map with per-package detail: [docs/architecture.md](docs/architecture.md).

- `src/input_devices/`: `wuji_glove` (in-process `wuji_sdk` UDP, no topic hop), `pico_input` (chest-frame `PoseStamped` topics; also owns the PICO frame math in `transform_utils.py` + `config_loader.py`), `replay` (replays a SOT bundle sample: named arm joint targets + hand keypoints on one timer).
- `src/controller/`: `wujihand_controller` runs as two independent processes (left/right), each with its own GIL, retargeting, and IK. Input selected by `wujihand_ik.yaml::input_source` (`wuji_glove` or `keypoints_topic`); dispatch in `controller/wujihand_node.py`, controller class in `src/output_devices/wujihand_output/wujihand_controller.py`.
- `src/output_devices/`: `wujihand_output` (hand IK), `g1_world_output` (G1 arms, own container; `mode` parameter selects `pose` — target-pose topics through IK — or `joint_replay` — named `/left_arm/joint_targets` + `/right_arm/joint_targets`, interpolated, no IK; `arm_type` selects `G1_23` (real rig) or `G1_29` (replay sim only, refuses DDS)).
- `src/wuji_teleop_bringup/`: launch files per preset (`wuji_teleop_hand.launch.py`, `pico_teleop.launch.py`). `src/wuji_teleop_monitor/`: Qt5 GUI, single `monitor` entry point, the reference for preset-to-launch mapping.

## Rules and invariants

- No launch file in the `teleop` container can start the G1 arms: `g1_world_output` is a separate image (Pinocchio + CasADi need NumPy 1.x). It is always a second terminal, and the Monitor GUI cannot reach it.
- The hand controller never opens hand USB; only the separate `wujihand_driver` process does. Hand sim mode: `enable_hand_driver:=false`. G1 sim mode: `dry_run:=true` (real IK, no DDS).
- Configs are `.yaml.template` in git; `docker/entrypoint.sh` seeds the real gitignored `.yaml` so serials/IPs never land in the repo. New template: rerun `colcon build --symlink-install`.
- `src/input_devices/pico_input/vendor/` is pinned upstream code under its own licenses. Do not modify it as first-party.
- For any coordinate-frame bug on the PICO path, read `src/input_devices/pico_input/ARCHITECTURE.md` before touching code.
- Docs here are team-facing: never document hardware location or access logistics, and never reference Alex's private research repo.

## RobotSTAR SOT handoff bundle

`RobotSTAR_demos/` is offline reference data (GT/Ours motion samples, hand keypoints, 29-DoF G1 joint trajectories, audits/videos). Its `HANDOFF_README.md` and `TUITION.md` are the authority on what may touch hardware. A sim replay pipeline exists and is validated in MuJoCo (runbook: [docs/usage.md](docs/usage.md#sot-bundle-replay-sim); design: [docs/architecture.md](docs/architecture.md#sot-bundle-replay)):

- `replay` publishes a sample's arm joints (named `JointState`, by-name matching end to end) and hand keypoints on one timer; `g1_world_output` `mode:=joint_replay arm_type:=G1_29` interpolates the arms; the hand controllers (`input_source: "keypoints_topic"`) retarget the keypoints live through the production path.
- **Never** use the bundle's precomputed hand joints (`controller_reference_v7.npz` `left_q`/`right_q`, `legacy_wuji_sim_only/hand_targets.csv`) on a real Hand 2 — they target the legacy hand model (`DO_NOT_COMMAND_HAND2.txt` in every sample). Hand joints are always regenerated from `hand2_input/*_human_targets_v5.npz` keypoints.
- **Hardware replay is deliberately not wired up**: `arm_type=G1_29` refuses DDS/pose-IK (no 29-DoF DDS controller or IK exists; the rig's robot is 23-DoF), and the Hand 2 mount adapter still doesn't exist (vendor STL is a Hand v1 part), which also blocks the measured flange→wrist transform TUITION.md §4 requires.
- Before any first hardware run, follow TUITION.md's staged test sequence and pick the sample from the batch audits — at least one sample failed its deployment audit (§7 names it) and several ship with `safe_timing_at_requested_scale: false`.
