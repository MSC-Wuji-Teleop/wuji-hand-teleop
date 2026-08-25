# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 (Humble) teleoperation stack for the **Wuji Hand**, driven by the **Wuji Glove** by default, with optional dual-arm teleop via an **HTC Vive Tracker** (SteamVR) or **PICO 4** VR headset, and an alternative **Unitree G1** arm output. This repo is `src/` inside a colcon workspace — the workspace root is one level up (`~/ros2_ws/`), and this repo's `src/` bind-mounts into a Docker container as the workspace's package source.

**Docker is the only supported deployment/runtime path** — there are no maintained bare-metal install instructions. If you need to reason about the runtime environment (installed apt/pip versions, SDKs), `docker/Dockerfile` is the canonical source of truth.

## Common commands

All development/runtime commands run **inside** the `wuji-hand-teleop` container unless noted otherwise.

```bash
# Build image / start container (from docker/)
cd docker && docker compose build
docker compose up -d
docker exec -it wuji-hand-teleop bash

# Build ROS2 packages (inside container, after code changes)
colcon build --symlink-install

# Run a single package's tests (inside container)
colcon test --packages-select <package_name>
colcon test-result --verbose

# Or run a package's pytest directly, e.g.:
python3 -m pytest src/controller/test/
python3 -m pytest src/output_devices/g1_world_output/tests/

# Launch (hand-only, CLI)
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py

# Launch (hand + arm, HTC Tracker path)
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=true arm_input:=tracker

# Launch (hand + arm, PICO path — separate launch file, different arm controller)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_robot:=true

# One-click GUI (preferred entry point over raw launch files)
ros2 run wuji_teleop_monitor monitor      # device dashboard + preset launch + live joint preview
ros2 run wuji_teleop_monitor brake        # direct-SDK Tianji arm brake/recovery (teleop must be OFF)
ros2 run wuji_teleop_monitor camera       # 2x2 camera feed preview (read-only diagnostic)

# Verify hand pipeline is running
ros2 topic hz /left_hand/joint_commands   # target ~120 Hz
ros2 topic hz /right_hand/joint_commands
```

Code changes on the host are live via the `src/` bind-mount + `colcon build --symlink-install`; only the `Dockerfile` itself or a `prebuilt/` deb change requires `docker compose build`.

## Submodules

`git clone --recurse-submodules` is required. Three Wuji/Unitree-owned submodules live under `src/`:
- `wujihandros2` — ROS2 driver for the Wuji Hand (pulls in `external/wuji-description`)
- `wuji-retargeting` — hand-pose retargeting algorithm (pip-installed into the image)
- `unitree_sdk2_python` — only needed to build the G1 arm image (`docker compose build g1_world_output`)

If a package fails to import after pulling, re-run `git submodule update --init --recursive` and rebuild.

## Architecture

**Data flow**: input device → standardized topic/TF interface → output controller → hardware.

- **Input devices** (`src/input_devices/`) turn hardware into a standard interface:
  - `wuji_glove/` — default hand input. Connects **in-process** via `wuji_sdk` UDP (no ROS2 topic hop) directly from the hand controller.
  - `openvr_input/` — HTC Vive Tracker arm input (SteamVR). Publishes TF: `world → chest`, `world → wrist`.
  - `pico_input/` — PICO 4 arm/hand tracking. Publishes `PoseStamped` to `/left_arm_target_pose` / `/right_arm_target_pose` (chest-frame poses; converts from PICO's world-frame internally). See `pico_input/ARCHITECTURE.md` for the full coordinate-transform derivation — read this first for any coordinate-frame bug.
  - `manus_input/` — MANUS Glove, community-supported and feature-frozen; not surfaced in the Monitor GUI.

- **Controllers** (`src/controller/`) — `wujihand_controller` runs as **two independent processes** (`wujihand_controller_left` / `wujihand_controller_right`), each on its own GIL, each doing its own retargeting + IK. `input_source` in `wujihand_ik.yaml` selects which input device feeds it. Dispatch logic + the reference integration pattern for custom hand input lives in `src/output_devices/wujihand_output/wujihand_controller.py`. It never touches the physical Wuji Hand SDK itself — it always publishes `/left_hand|right_hand/joint_commands` regardless of hardware; only the separate `wujihand_driver` process (`wujihandros2`, C++) opens the real USB connection. `wuji_teleop_hand.launch.py`'s `enable_hand_driver:=false` skips that process for sim mode (real glove input, no physical hand) — pair with `g1_world_output/scripts/mujoco_visualizer.py --focus hands`.

- **Output devices** (`src/output_devices/`):
  - `wujihand_output/` — Wuji Hand IK controller.
  - `tianji_output/` — Tianji Arm controller, TF-mode (consumes `openvr_input`'s TF frames), used by the HTC path.
  - `tianji_world_output/` — Tianji Arm controller, topic-mode (consumes `/left_arm_target_pose` etc.), used by the PICO path. `transform_utils.py` has the chest-frame transform utilities a custom input would need.
  - `g1_world_output/` — Unitree G1 dual-arm controller (Pinocchio + CasADi IK over `unitree_sdk2py` DDS). Consumes the **same** topic contract as `tianji_world_output` and does its own chest→pelvis remap (`transform_utils.py::chest_pose_to_pelvis`), so it's a drop-in output alternative, not a new input path. Runs as its **own container/image** (`docker compose up -d g1_world_output`), separate from the main `teleop` container — Pinocchio+CasADi with working Python bindings only ships via an apt build (`robotpkg`) linked against NumPy 1.x, which conflicts with the NumPy 2.x the rest of the stack needs. URDF/MJCF/meshes live in `src/g1_wuji2_description/`. `--dry-run` (or `dry_run:=true` on its launch file) is the hardware/sim toggle — solves real IK but never opens DDS, meant to pair with `scripts/mujoco_visualizer.py` or `scripts/sweep_and_visualize.py` (run in the `teleop` container, which has `mujoco`) for MuJoCo-based testing without a physical G1; see that package's README.

- **Camera** (`src/camera/`) — unified stereo head camera (USB UVC, `/dev/stereo_camera` udev symlink) + dual RealSense D405 wrist cameras. Head camera splits to `/stereo/{left,right}/compressed` for ROS2 and separately H.264-encodes (NVENC if available, else libx264) for PICO streaming.

- **`wuji_teleop_bringup/`** — launch files (`wuji_teleop_hand.launch.py`, `wuji_teleop.launch.py`, `pico_teleop.launch.py`) that wire the above together per preset.

- **`wuji_teleop_monitor/`** — Qt5 GUI with three entry points (`monitor`, `brake`, `camera`); `monitor` is the primary one-click flow and should be treated as the reference for how presets map to launch files/flags.

**Key invariant**: `brake` and `monitor` (teleop) must never run concurrently — the Tianji controller cabinet allows only a single TCP session.

**Config files** are `.yaml.template` in git; `docker/entrypoint.sh` seeds the real `.yaml` on first container start, and the real files are gitignored so serials/IPs never land in the repo. When adding a new templated config, rerun `colcon build --symlink-install` so the `install/share/` symlink picks it up.

## Third-party / vendored code

`src/input_devices/pico_input/vendor/` contains vendored upstream sources (XRoboToolkit PC-Service, Apache-2.0/MIT) — each subdirectory keeps its own upstream `LICENSE`. The repo's own MIT `LICENSE` applies only to files outside `vendor/`. Do not modify vendored code as if it were first-party; treat it as a pinned external dependency.
