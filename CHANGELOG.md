# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Removed

- **Fork cleanse to the actual rig (2026-08-25).** Removed the upstream packages
  for hardware this lab does not have: `tianji_output`, `tianji_world_output`
  (Tianji arm), `openvr_input` (HTC Vive Tracker / SteamVR), and `manus_input`
  (MANUS glove). Also removed the HTC launch preset, the Monitor's `brake` and
  `camera` entry points and its `/tianji_arm/lifecycle_state` gating, the
  SteamVR/OpenVR plumbing in the Docker layer, the upstream release workflows,
  and the HTC docs and demo videos. Full inventory with rationale, per folder
  and per file: [docs/deprecated/cleanup.md](docs/deprecated/cleanup.md).
- The entrypoint's Tianji `libKine.so` / `libMarvinSDK.so` health checks are
  gone. They called `exit 1`, so they would have failed the container outright
  once `tianji_output` was removed.

### Changed

- **`pico_input` now owns the PICO frame math.** It imported
  `tianji_world_output.config_loader` and `.transform_utils` in production code,
  so those two modules moved into the package unchanged as
  `pico_input/transform_utils.py` and `pico_input/config_loader.py`, with the
  new `config/robot_frames.yaml` replacing `tianji_robot.yaml`. The Tianji
  hardware fields (`robot_ip`, `kine_config_file`, `init_joints`) were dropped.
  The frame math was verified bit-identical to the original before the switch.
  The `init_pos` / `init_rot` / `arm_init_*` anchors are still Tianji-derived FK
  values, carried over verbatim and **not yet re-derived for the G1_23**.
- `pico_teleop.launch.py` now starts PICO input and the hands only. It no longer
  starts an arm output or the camera stack. The G1 arm controller runs in its own
  container, so it cannot be a node in this launch file; it is a second terminal.
- Hand `input_source` collapsed to `wuji_glove`, its only remaining value.
- Monitor presets reduced to "Hand only (Wuji Glove)" and "Hand + PICO input".
- `src/camera/` is **kept but unwired**, pending the G1 head cameras (RealSense
  D435i built-in, D455 attachment). Nothing launches it. `d435i` is already a
  supported type in `camera_launch.py`; the migration notes are in
  `src/camera/README.md`.
- Deleted the `pico_input` test scripts that drove the Tianji arm directly
  (`step1`, `step3`, `step5`, `step6`, and all of `test/tool/`). `step2` and
  `step4` remain, repointed at the ported modules.

### Added

- Added **Unitree G1 dual-arm support** (`src/output_devices/g1_world_output/`) as an
  alternative to the Tianji arm, consuming the same PICO topic contract as
  `tianji_world_output`. Pinocchio/CasADi IK + Unitree DDS LowCmd control; runs in
  its own Docker image/service (`docker compose --profile g1 up -d g1_world_output`)
  since its Pinocchio build needs NumPy 1.x, incompatible with the rest of the
  stack's NumPy 2.x. Vendored via the new `src/unitree_sdk2_python` submodule.
- Added `src/g1_wuji2_description/` — composed G1 + dual Wuji Hand 2 URDF/MJCF/meshes.
- Added a **MuJoCo sim mode** for G1 teleop: `g1_world_output_node --dry-run` (or
  `dry_run:=true` on the launch file) solves real IK without ever touching DDS/
  hardware, paired with `scripts/mujoco_visualizer.py` (mirrors real teleop's
  joint-command topics live in MuJoCo) or `scripts/sweep_and_visualize.py`
  (synthetic sweep, for when no glove/tracker is attached) run from the main
  `teleop` container.
- Extended the same sim mode to the **Wuji Hand**: `wuji_teleop_hand.launch.py`
  gained an `enable_hand_driver` arg — set `false` to skip `wujihand_driver`
  (the only process touching the physical hand) while `wujihand_controller`
  keeps running off real Wuji Glove input and publishing
  `/left_hand|right_hand/joint_commands` unchanged, then mirror it with
  `mujoco_visualizer.py --focus hands` (new `--focus`/camera-framing option).

### Changed

- Added `mujoco==3.12.0` to the main `teleop` image's pip dependencies, needed by
  the new G1 sim-mode scripts.
- Rewrote `docs/spec/spec_1.md` from a question list into the proposed hardware
  replay design (29-DoF G1 primary, 23-DoF secondary): conditioning artifacts,
  LowCmd slot policy, run/device state machines, safety envelope layers, and a
  bring-up staged against TUITION §7. hardware_spec.md records the variant
  decision (2026-08-27).

## [2026.6.13]

Open-sourcing the four Wuji-owned pillars (Wuji Glove + Wuji Hand + `wujihandcpp` SDK + `wuji-retargeting` IK) as a single-machine, Docker-only teleop stack. Wuji Glove is the new default hand input. The Tianji arm path (HTC Vive Tracker default, PICO 4 alternative) is optional and runs on top of the same image — see `docs/STEAMVR.md` / `docs/PICO.md`.

### Added

- Added **Wuji Glove** as the new default hand input — wireless, no per-session calibration, per-side SN config via `wuji-sdk` (PyPI), with a LAN-leak guard so the Glove SN is no longer advertised on the local network. MANUS demoted to community-supported alternative.
- Vendored **`wuji-retargeting`** (hand IK algorithm) as a submodule. Docker now builds from the local checkout instead of cloning at image-build time. **Breaking**: `git clone --recurse-submodules` required.
- Moved **`wujihandros2` submodule** from `wuji-hand-description` to lineup-wide `wuji-description`. **Breaking**: run `git submodule update --init --recursive` after pulling.
- Added **per-machine config seeding via `.yaml.template`**: repo tracks placeholder templates only, and container start auto-copies any missing live yaml from its template sibling, so real SNs/IPs never enter the public repo. Monitor `Scan SNs` dialog and launch helpers read/write the same yaml path. Covers `wujihand_ik.yaml`, `wuji_glove.yaml`, `openvr_input.yaml`, `pico_input.yaml`, `camera_config.yaml`.

### Changed

- Upgraded **Monitor GUI** — Wuji-first device dashboard (Glove + Hand reachability above the fold), one-click `Scan SNs` writes detected SNs back to the live yaml, three hand-first launch presets. Sibling `brake` and `camera` console entries: `ros2 run wuji_teleop_monitor {monitor|brake|camera}` or desktop shortcuts.
- Upgraded **Tianji arm path** — HTC Vive Tracker (default) and PICO 4 (alternative) arm inputs, both with state-driven lifecycle. Gated by the `enable_arm` launch arg.
- Renamed **Docker image and container** from `wuji-teleop` to `wuji-hand-teleop`. **Breaking**: run `docker stop wuji-teleop && docker rm wuji-teleop` before `docker compose up -d`.
- Switched to **Docker-only deployment**: README's `Bare-Metal Install` removed, and `docker/README.md` folded into the main `README.md`. Dockerfile stays open-source as the canonical host-dependency reference.

### Removed

- Removed multi-machine lifecycle gating, including the Monitor's matching subscriptions and arm-manager service clients.
- Removed the multi-process state machine from the Tianji arm path: INFERENCE mode, `switch_mode` / `get_mode` services, and the `default_mode` launch param.
- **Breaking**: removed the `hand_input` launch param and the legacy `wuji_teleop_{arm,camera,single}.launch.py` and `pico_teleop_minimal.launch.py` files. Input source is now read from `wujihand_ik.yaml::input_source`, and `wuji_teleop.launch.py` covers all paths via params.
- Removed the unused `common_input` package, 43 MB `pico_input/record/` data dump, 200 KB `trackingData_sample.txt`, and the cross-process clock-sync dependency no longer needed for single-machine deployment.

### Security

- Scrubbed two real D405 wrist-camera SNs from `src/camera/config/udev/99-teleop-cameras.rules`, replaced with `YOUR_LEFT_WRIST_CAM_SERIAL` / `YOUR_RIGHT_WRIST_CAM_SERIAL` placeholders.
- Added Wuji Glove LAN-leak guard (see Added).

### Fixed

- Stabilised **Tianji v37 disable path** — the arm no longer hangs half-powered when Ctrl-C interrupts shutdown, and the Marvin SDK session releases cleanly.
- Restored Tianji arm impedance parameters to validated production values. Tool kinematics intentionally live in URDF and the chest TF rather than the SDK's tool-frame matrix, so IK solves at the flange.
- Bumped `wujihandros2` submodule pin to a commit present on the new `wuji-description` remote (the previous pin was missing after the submodule move).
- Fixed Wuji Studio calibration files (`~/.wuji/sdk/params/<SN>.toml`) being invisible inside the Docker container — `docker-compose.yml` now bind-mounts `${HOME}/.wuji` with the right ownership.
- Fixed `import pinocchio` failure caused by a missing shared library — upgraded the pinocchio dependency, which also rebuilds eigenpy against the NumPy 2 ABI.
- Fixed Tianji arm crash with `libKine.so: cannot open shared object file` from a `.gitignore` path error. Container startup now also detects Git LFS pointer stubs for all four vendored SDKs (`libKine.so`, `libMarvinSDK.so`, `libPXREARobotSDK.so`, `libManusSDK.so`) and fails fast, instead of crashing at runtime when `git lfs pull` was skipped.
- Fixed PICO input node boot crash from a parameter-declaration collision. PICO arm controller is now state-driven and publishes `/{side}_arm/joint_states` at 100 Hz for the Monitor preview.
- Fixed `brake` UI Read Status crash after the fault-code helper was simplified to a single English dictionary.
- Fixed outer launch arguments from `camera_launch.py` leaking into `rs_launch.py`.
- Fixed five runtime crashes caused by incomplete cleanup from the previous multi-process layout — all built successfully but failed at runtime, including stale console-script paths and leftover packages that should have been removed in [2026.04.28].

## [2026.04.28] - 2026-04-28

### Changed

- Hand controller now runs as one process per side (no shared GIL/timer).
- **Breaking**: custom hand input now publishes `manus_ros2_msgs/ManusGlove` on `/manus_glove_{0,1}` (was `Float32MultiArray` on `/hand_input`).
- **Breaking**: hand mode services are now per-side at `/wuji_hand/{left,right}/{switch_mode,get_mode}`.
- Hand and HTC/Tianji arm controllers now default to 120 Hz (PICO arm stays at 90 Hz).
- New launch params `control_rate` and `nlopt_max_eval` (`0` keeps library default).

### Fixed

- Manus udev rule now also covers the wireless transceiver (`1915:83fd`) — without it the BLE skeleton stream was silently empty.
- Incomplete Manus frames are now dropped instead of substituted with the origin — the hand no longer snaps to `(0, 0, 0)`.

### Removed

- Removed the `manus_input` Python wrapper; controllers subscribe to MANUS topics directly.

## [2026.04.18] - 2026-04-18

### Added

- Added HTC Vive Tracker wearing guide with visual reference
- Added teleoperation demo video
- Added system dataflow architecture diagram
- Added udev rule so the MANUS USB dongle no longer requires sudo
- Added installation instructions for the Wuji Hand C++ SDK
- Documented minimum firmware version requirement with a link to the Wuji Hand firmware upgrade tool
- Added per-hand pinch threshold configuration for both left and right hands
- Added placeholder serial numbers to config templates so users fill in their own hardware values

### Changed

- Rewrote README with a linear step-by-step setup flow
- Replaced sudo-based USB permissions with udev rules so launch files no longer require elevated privileges
- Updated PICO XRoboToolkit APK download links to match current release filenames
- Changed the hand retargeting install command from editable mode to a regular install

### Fixed

- Fixed a package name conflict that could block the workspace build
- Fixed a missing system dependency required for the MANUS build
- Fixed documentation for the `/hand_input` topic QoS policy
- Fixed several inaccurate default values in configuration documentation
- Fixed a crash in the Tianji Arm controller when a required file was missing
- Fixed the MANUS calibration guide to reflect separate left- and right-hand calibration files
- Fixed camera launch documentation to reflect the actual default behavior

## [0.1.0] - 2026-03-10

### Added

- ROS2-based hand and arm teleoperation system
- Input support for MANUS data glove, HTC Vive Tracker, and PICO VR controller/tracker
- Output support for Wuji Hand retargeting and Tianji Arm IK

[Unreleased]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v2026.6.13...HEAD
[2026.6.13]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v2026.04.28...v2026.6.13
[2026.04.28]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v2026.04.18...v2026.04.28
[2026.04.18]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v0.1.0...v2026.04.18
[0.1.0]: https://github.com/wuji-technology/wuji-hand-teleop/releases/tag/v0.1.0
