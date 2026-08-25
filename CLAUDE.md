# CLAUDE.md

## Overview

ROS2 (Humble) teleoperation stack for the **Wuji Hand**, driven by the **Wuji Glove**, with optional dual-arm teleop (HTC Vive Tracker or PICO 4) and a **Unitree G1** arm output. The repo lives inside a colcon workspace (`~/ros2_ws/src/wuji-hand-teleop`); its `src/` bind-mounts into the `wuji-hand-teleop` Docker container as the workspace package source.

**Docker is the only supported runtime.** `docker/Dockerfile` is the source of truth for the environment (apt/pip versions, SDKs). One exception to the single-container picture: `g1_world_output` runs as its own image/container because its Pinocchio+CasADi build needs NumPy 1.x while the rest of the stack needs 2.x.

This fork drives one specific rig (G1 + Wuji Hand 2, gloves + PICO 4 input). The hardware source of truth is [docs/hardware_spec.md](docs/hardware_spec.md): the G1 is the **23-DoF** variant, `g1_wuji2_description` is still 29-DoF-based (rebuild pending), and the Tianji/HTC/camera hardware does not exist here.

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

- `src/input_devices/`: `wuji_glove` (in-process `wuji_sdk` UDP, no topic hop), `openvr_input` (TF), `pico_input` (chest-frame `PoseStamped` topics), `manus_input` (community-supported, feature-frozen).
- `src/controller/`: `wujihand_controller` runs as two independent processes (left/right), each with its own GIL, retargeting, and IK. Input selected by `wujihand_ik.yaml::input_source`; dispatch + custom-input reference pattern in `src/output_devices/wujihand_output/wujihand_controller.py`.
- `src/output_devices/`: `wujihand_output` (hand IK), `tianji_output` (TF mode, HTC path), `tianji_world_output` (topic mode, PICO path), `g1_world_output` (same topic contract as `tianji_world_output`, drop-in output alternative).
- `src/wuji_teleop_bringup/`: launch files per preset. `src/wuji_teleop_monitor/`: Qt5 GUI (`monitor` / `brake` / `camera`); `monitor` is the reference for preset-to-launch mapping.

## Rules and invariants

- `brake` and `monitor` teleop must never run concurrently: the Tianji cabinet allows a single TCP session.
- The hand controller never opens hand USB; only the separate `wujihand_driver` process does. Hand sim mode: `enable_hand_driver:=false`. G1 sim mode: `dry_run:=true` (real IK, no DDS).
- Configs are `.yaml.template` in git; `docker/entrypoint.sh` seeds the real gitignored `.yaml` so serials/IPs never land in the repo. New template: rerun `colcon build --symlink-install`.
- `src/input_devices/pico_input/vendor/` is pinned upstream code under its own licenses. Do not modify it as first-party.
- For any coordinate-frame bug on the PICO path, read `src/input_devices/pico_input/ARCHITECTURE.md` before touching code.
- Docs here are team-facing: never document hardware location or access logistics, and never reference Alex's private research repo.
