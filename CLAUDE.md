# CLAUDE.md

## Overview

ROS2 (Humble) teleoperation stack for one rig: a **Unitree G1 (29-DoF)** with **2x Wuji Hand 2**, driven by **Wuji Gloves** for the hands and a **PICO 4** headset with 4 Motion Trackers for the arms. The repo lives inside a colcon workspace (`~/ros2_ws/src/wuji-hand-teleop`); its `src/` bind-mounts into the `wuji-hand-teleop` Docker container as the workspace package source.

**Docker is the only supported runtime.** `docker/Dockerfile` is the source of truth for the environment (apt/pip versions, SDKs). One exception to the single-container picture: `g1_world_output` runs as its own image/container because its Pinocchio+CasADi build needs NumPy 1.x while the rest of the stack needs 2.x.

The hardware source of truth is [docs/spec/hardware_spec.md](docs/spec/hardware_spec.md): the physical G1 is the **29-DoF** variant (decided 2026-08-27); `g1_wuji2_description` carries `g1_29_wuji2*` and the `g1_23_wuji2*` secondary. The upstream Tianji arm, HTC/SteamVR, and MANUS code was removed on 2026-08-25; what went and why is in [docs/deprecated/cleanup.md](docs/deprecated/cleanup.md). `src/camera/` is kept but unwired, pending the G1 head cameras.

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

- `src/input_devices/`: `wuji_glove` (in-process `wuji_sdk` UDP, no topic hop), `pico_input` (chest-frame `PoseStamped` topics; also owns the PICO frame math in `transform_utils.py` + `config_loader.py`), `replay` (plays a prepared clip directory: named arm joint targets to the G1 node and named hand joints to the hand drivers, one timer).
- `src/controller/`: `wujihand_controller` runs as two independent processes (left/right), each with its own GIL, retargeting, and IK. Input selected by `wujihand_ik.yaml::input_source` (`wuji_glove` or `keypoints_topic`); dispatch in `controller/wujihand_node.py`, controller class in `src/output_devices/wujihand_output/wujihand_controller.py`.
- `src/output_devices/`: `wujihand_output` (hand IK), `g1_world_output` (G1 arms, own container; `mode` parameter selects `pose` — target-pose topics through IK — or `joint_replay` — named `/left_arm/joint_targets` + `/right_arm/joint_targets`, interpolated, no IK; `arm_type` selects `G1_29` (the rig, default) or `G1_23` (secondary); pose mode is `G1_23`-only).
- `src/wuji_teleop_bringup/`: launch files per preset (`wuji_teleop_hand.launch.py`, `pico_teleop.launch.py`). `src/wuji_teleop_monitor/`: Qt5 GUI, single `monitor` entry point, the reference for preset-to-launch mapping.

## Rules and invariants

- No launch file in the `teleop` container can start the G1 arms: `g1_world_output` is a separate image (Pinocchio + CasADi need NumPy 1.x). It is always a second terminal, and the Monitor GUI cannot reach it.
- The hand controller never opens the hand link; only the separate hand driver process does. The hand driver is `starport_wuji_hand` `hand_node`, one per side at `/{side}/wuji_hand`, over Ethernet via `wuji_sdk` ([docs/spec/spec1.md](docs/spec/spec1.md)). The USB driver (`wujihand_driver`, `wujihandros2` submodule) is still in the tree and is what the teleop launch files spawn until the swap lands. Hand sim mode: `enable_hand_driver:=false`. G1 sim mode: `dry_run:=true` (real IK, no DDS).
- Configs are `.yaml.template` in git; `docker/entrypoint.sh` seeds the real gitignored `.yaml` so serials/IPs never land in the repo. New template: rerun `colcon build --symlink-install`.
- `src/input_devices/pico_input/vendor/` is pinned upstream code under its own licenses. Do not modify it as first-party.
- For any coordinate-frame bug on the PICO path, read `src/input_devices/pico_input/ARCHITECTURE.md` before touching code.
- Docs here are team-facing: never document hardware location or access logistics, and never reference Alex's private research repo.

## RobotSTAR SOT handoff bundle

`RobotSTAR_demos/` (gitignored) is offline reference data: GT/Ours motion samples, hand keypoints, 29-DoF G1 joint trajectories, audits and videos. Its `HANDOFF_README.md` describes the file layout. Sim runbook: [docs/usage.md](docs/usage.md#sot-bundle-replay-sim); hardware runbook: [docs/replay.md](docs/replay.md); design: [docs/spec/spec1.md](docs/spec/spec1.md) and [docs/architecture.md](docs/architecture.md#sot-bundle-replay).

- `tools/prepare_clip.py` turns a bundle sample into a clip directory (`clips/safe/` tracked, `clips/rejected/` and `clips/candidate/` ignored): smooths the arms, retargets the hands to Hand 2 with the production retargeter, replays the clip dynamically in MuJoCo with the G1 node's gains (`tools/clip_audit.py`), and judges it per speed. Magic numbers and their justification sit at the top of those files and in spec1.
- `replay_publisher` reads a safe clip and publishes named `JointState` targets on one timer: arms to `g1_world_output` (`mode:=joint_replay arm_type:=G1_29`, interpolates one publish period behind), hand joints straight to the `starport_wuji_hand` drivers. The hand controller is not on this path.
- **Never** send the bundle's precomputed hand joints (`controller_reference_v7.npz` `left_q`/`right_q`, `legacy_wuji_sim_only/hand_targets.csv`) to a real Hand 2: they target the legacy hand model (`DO_NOT_COMMAND_HAND2.txt` in every sample). Hand joints are always regenerated from `hand2_input/*_human_targets_v5.npz` keypoints.
- `scripts/replay.sh --home` is the rehome ([docs/spec/spec1_1.md](docs/spec/spec1_1.md)): it captures the measured arm pose, generates and audits a slow clip from it to all-zeros with `tools/make_home_clip.py`, and plays it through the same publisher. Separate operator command, no runtime state, not an e-stop. All-zeros is Unitree's own `arm_sdk` zero posture; the home pose and the dropped retract waypoint are settled by [docs/issues/home-audit-matrix-2026-09-03.md](docs/issues/home-audit-matrix-2026-09-03.md), so do not re-litigate either without new numbers.
- The replay path is the publisher and the two device nodes, nothing else, by decision. Do not add runtime checks, modes, or trip conditions to it. Clip quality is decided offline before a run; the runtime plays the clip once and holds the last frame.
- Hardware replay: `arm_type=G1_29` (the default) drives DDS through `G1ArmController`; pose-IK stays `G1_23`-only. The Hand 2 mount adapter is still unconfirmed (the vendor STL is a Hand v1 part).
