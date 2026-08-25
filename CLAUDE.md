# CLAUDE.md

## Overview

ROS2 (Humble) teleoperation stack for one rig: a **Unitree G1 (23-DoF)** with **2x Wuji Hand 2**, driven by **Wuji Gloves** for the hands and a **PICO 4** headset with 4 Motion Trackers for the arms. The repo lives inside a colcon workspace (`~/ros2_ws/src/wuji-hand-teleop`); its `src/` bind-mounts into the `wuji-hand-teleop` Docker container as the workspace package source.

**Docker is the only supported runtime.** `docker/Dockerfile` is the source of truth for the environment (apt/pip versions, SDKs). One exception to the single-container picture: `g1_world_output` runs as its own image/container because its Pinocchio+CasADi build needs NumPy 1.x while the rest of the stack needs 2.x.

The hardware source of truth is [docs/hardware_spec.md](docs/hardware_spec.md): the G1 is the **23-DoF** variant and `g1_wuji2_description` matches it (g1_23_wuji2* files, rebuilt 2026-08-24). The upstream Tianji arm, HTC/SteamVR, and MANUS code was removed on 2026-08-25; what went and why is in [docs/cleanup.md](docs/cleanup.md). `src/camera/` is kept but unwired, pending the G1 head cameras.

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

- `src/input_devices/`: `wuji_glove` (in-process `wuji_sdk` UDP, no topic hop), `pico_input` (chest-frame `PoseStamped` topics; also owns the PICO frame math in `transform_utils.py` + `config_loader.py`).
- `src/controller/`: `wujihand_controller` runs as two independent processes (left/right), each with its own GIL, retargeting, and IK. Input selected by `wujihand_ik.yaml::input_source`; dispatch + custom-input reference pattern in `src/output_devices/wujihand_output/wujihand_controller.py`.
- `src/output_devices/`: `wujihand_output` (hand IK), `g1_world_output` (G1 arms; consumes `/left_arm_target_pose` + `/right_arm_target_pose`, runs in its own container).
- `src/wuji_teleop_bringup/`: launch files per preset (`wuji_teleop_hand.launch.py`, `pico_teleop.launch.py`). `src/wuji_teleop_monitor/`: Qt5 GUI, single `monitor` entry point, the reference for preset-to-launch mapping.

## Rules and invariants

- No launch file in the `teleop` container can start the G1 arms: `g1_world_output` is a separate image (Pinocchio + CasADi need NumPy 1.x). It is always a second terminal, and the Monitor GUI cannot reach it.
- The hand controller never opens hand USB; only the separate `wujihand_driver` process does. Hand sim mode: `enable_hand_driver:=false`. G1 sim mode: `dry_run:=true` (real IK, no DDS).
- Configs are `.yaml.template` in git; `docker/entrypoint.sh` seeds the real gitignored `.yaml` so serials/IPs never land in the repo. New template: rerun `colcon build --symlink-install`.
- `src/input_devices/pico_input/vendor/` is pinned upstream code under its own licenses. Do not modify it as first-party.
- For any coordinate-frame bug on the PICO path, read `src/input_devices/pico_input/ARCHITECTURE.md` before touching code.
- Docs here are team-facing: never document hardware location or access logistics, and never reference Alex's private research repo.
