# Cleanup: what was removed from the upstream fork, and why

This repo was forked from `wuji-technology/wuji-hand-teleop` and detached on
2026-08-25. Upstream supports hardware this lab does not have. This file records
what came out and the reasoning, so nobody has to reconstruct it from `git log`.

The rig is exactly: **Unitree G1 (23-DoF) + 2x Wuji Hand 2, teleoped from Wuji
Gloves and a PICO 4 headset with 4 Motion Trackers.** The hardware source of
truth is [hardware_spec.md](../spec/hardware_spec.md).

**Nothing here is lost.** History was not rewritten. Every file below is
recoverable from the pre-cleanse commit `b065f3f`, e.g.:

```bash
git show b065f3f:src/output_devices/tianji_output/tianji_output/fx_kine.py
git checkout b065f3f -- docs/teleop-demo.mp4
```

The `backup_8_24` branch also points at that commit.

- [Folders removed](#folders-removed)
- [Files removed](#files-removed)
- [Changed, not removed](#changed-not-removed)
- [Kept on purpose](#kept-on-purpose)
- [Known follow-ups](#known-follow-ups)

## Folders removed

| Folder | What it does | Hardware it assumes | Why removed |
|---|---|---|---|
| `src/output_devices/tianji_output/` | Arm controller in **TF mode**. Read `world -> chest` / `world -> wrist` TF from `openvr_input`, solved IK, drove the arm over the Marvin TCP protocol. Shipped `libMarvinSDK.so` + `libKine.so` (25 MB, LFS) | Tianji arm controller cabinet, which accepts a single TCP client | No Tianji arm. Our arm is the G1 |
| `src/output_devices/tianji_world_output/` | Same arm, **topic mode**. Subscribed `/left_arm_target_pose` + `/right_arm_target_pose` (the PICO path's contract) instead of TF. Also held the shared frame-math and config modules that `pico_input` imported | Tianji arm | Same. `g1_world_output` consumes the identical topic contract and replaces it. Its two shared modules were **moved into `pico_input`**, not deleted; see [Changed](#changed-not-removed) |
| `src/input_devices/openvr_input/` | Published tracker poses as TF, via the `openvr` Python bindings against a SteamVR runtime in null-driver mode | HTC Vive Trackers, base stations, USB dongles, a SteamVR install | No Vive hardware. Our arm input is the PICO 4 |
| `src/input_devices/manus_input/` | C++ ROS2 node publishing `/manus_glove_{0,1}` from the MANUS SDK | MANUS glove + USB dongle | No MANUS glove. Upstream had already frozen it. Our hand input is the Wuji Glove |

`src/camera/` is deliberately **not** in this list — see
[Kept on purpose](#kept-on-purpose).

## Files removed

### Code

| File | What it does | Hardware it assumes | Why removed |
|---|---|---|---|
| `src/controller/controller/tianji_arm_node.py` | The HTC-path arm controller, entry point `tianji_arm_controller`. Ran an enable/disable state machine and published `/tianji_arm/lifecycle_state`, which the Monitor gated its Stop button on | Tianji arm | Tianji only. Its sibling `wujihand_node.py`, the hand controller, stays |
| `src/wuji_teleop_bringup/launch/wuji_teleop.launch.py` | `wuji_teleop_bringup` is the launch-file package: one file per rig preset, wiring input + controller + output together. This file was the **HTC preset**: `openvr_input` -> TF -> `tianji_output`, plus the hand stack | Vive Trackers + Tianji arm | Both gone. `wuji_teleop_hand.launch.py` (hand-only) and `pico_teleop.launch.py` (PICO) stay |
| `src/wuji_teleop_bringup/wuji_teleop_bringup/tf_utils.py` | Built `static_transform_publisher` nodes for the chest and wrist-to-arm frames | Tianji arm geometry | Dead: `wuji_teleop.launch.py` was its only caller |
| `src/wuji_teleop_bringup/config/static_transforms.yaml` | The `wrist_to_tianji` offsets those TF nodes read | Tianji arm geometry | Dead: `tf_utils.py` was its only reader |
| `.../ui/run_brake.py` | `wuji_teleop_monitor` is the Qt5 operator GUI. This entry point was the **brake / recovery tool**: cleared arm faults and released brakes after an e-stop, talking to the SDK directly | Tianji cabinet. Could never run concurrently with teleop, since the cabinet accepts one TCP client | No cabinet to brake. This is also why the "single Tianji TCP session" invariant is gone from `architecture.md` |
| `.../ui/run_camera.py` | The **camera preview** entry point: a 2x2 tile view of the head and wrist feeds | D405 wrists + HBVCAM head | The camera pipeline is unwired, so the preview has nothing to show. Rebuild it against the G1 head cameras |
| `.../ui/arm_constants.py` | Tianji SDK constants: `INIT_LEFT`/`INIT_RIGHT` 7-DoF reference poses, plus the `STATE_NAMES` and `ERR_DESCRIPTIONS` error tables | Tianji arm, 7 DoF per side | Tianji-specific. The pose arrays are also simply wrong for a G1_23, which has 5 arm DoF per side |
| `brake-control.desktop.template`, `camera-preview.desktop.template` | Desktop shortcuts for the two GUIs above | as above | Their targets are gone |
| `.../wujihand_output/config/retarget_manus_{left,right}.yaml` | Retargeting parameters mapping MANUS skeleton joints onto Wuji Hand joints | MANUS glove | No MANUS glove. The `retarget_wuji_glove_*` pair stays |
| `.github/workflows/auto-release.yml`, `create_release_tag.yaml` | Upstream's release tagging automation | none | Tied to the `wuji-technology` repo. This fork is detached and does not cut releases |

### `pico_input` test scripts

These drove the Tianji arm directly, importing `fx_kine.Marvin_Kine`,
`fx_robot.Marvin_Robot`, or `cartesian_controller.CartesianController`. With the
SDK gone they cannot run at all.

| File | What it did |
|---|---|
| `test/step1_direct_joint_control.py` | Joint-space trajectories straight to the arm over the Marvin SDK |
| `test/step3_visualize_in_rviz.py` | RViz frame visualization, but connected to the live arm for FK |
| `test/step5_incremental_control_with_robot.py` | Full PICO incremental control against the real arm |
| `test/step6_arm_angle_stability_test.py` | Arm-angle / nullspace stability sweeps on the real arm |
| `test/tool/verify_fk_values.py` | Compared `init_joints` FK against the config anchors |
| `test/tool/move_to_init_pose.py` | Drove the arm to its calibrated init pose |
| `test/tool/diagnose_zsp_para.py` | FK -> IK closed-loop grid search for `zsp_para` |
| `test/common/robot_lifecycle.py` | Power-on/off sequencing for a `Marvin_Robot` instance |

`step2_pose_topic_control.py` (publishes topics only) and
`step4_visualize_recorded_data.py` (visualizes recordings) survive, repointed at
the ported modules.

### Docs and media

| File | What it was | Why removed |
|---|---|---|
| `docs/STEAMVR.md` | Operator setup for the SteamVR null driver, base stations, dongles, tracker pairing | `openvr_input` is gone |
| `docs/tracker-wearing-guide.md` | How to strap Vive trackers to chest, wrists, and upper arms | Vive-specific. The PICO path also uses trackers, but they are PICO Motion Trackers and are covered by [PICO.md](../PICO.md) |
| `docs/images/tracker-wearing-combined.jpg` | Figure for the guide above | Its only reference went |
| `docs/images/dataflow.png` | Upstream's data-flow diagram | Depicts the Tianji pipeline. The mermaid graph in [architecture.md](../architecture.md) is the live diagram and has been redrawn |
| `docs/teleop-demo.mp4` (15 MB) | Upstream demo clip | Shows hardware we do not have. Also the single largest file removed |
| `docs/tracker-wearing-demo.mp4` (2.5 MB) | Companion clip for the wearing guide | Its guide went |

> `README.md` was left untouched on purpose, for a separate rewrite (it still
> referenced `docs/images/dataflow.png` and `docs/teleop-demo.mp4`, so those
> two links were broken). That rewrite, drafted as `MSC_README.md`, has since
> been promoted: `README.md` is now that draft, and the old upstream README
> lives on as [`/deprecated/DEPRECATED_README.md`](..//deprecated/DEPRECATED_README.md) for historical
> reference only.

## Changed, not removed

- **`pico_input` now owns the PICO frame math.** This was the one entanglement
  that made the cleanup more than a deletion: `pico_input_node.py` and
  `incremental_controller.py` imported `tianji_world_output.config_loader` and
  `.transform_utils` in **production** code, so removing the Tianji packages
  broke `pico_input` outright. Both modules moved into the package unchanged:
  - `tianji_world_output/transform_utils.py` -> `pico_input/pico_input/transform_utils.py`
    (verbatim; no function body altered)
  - `tianji_world_output/config_loader.py` -> `pico_input/pico_input/config_loader.py`
    (class `TianjiConfig` renamed `RobotFramesConfig`; the Tianji hardware
    fields `robot_ip`, `kine_config_file`, `init_joints` and the
    `get_kine_config_path()` method dropped)
  - `tianji_world_output/config/tianji_robot.yaml` -> `pico_input/config/robot_frames.yaml`
    (same values, now a plain tracked yaml rather than a `.template`, since it
    holds no serials or IPs once `robot_ip` is gone)

  The port was verified: the four functions `pico_input` actually calls, plus
  the config fields it reads, were compared against the originals over ~100
  probe inputs and are bit-identical.

- **The hand `input_source` collapsed to `wuji_glove`.** MANUS was an
  `input_source` option threaded through `wujihand_node.py`, so deleting the
  package alone would have left a branch that lazy-imports `manus_ros2_msgs`
  and dies at runtime. The key itself stays: `WujiHandController` uses it to
  pick the retarget config (`retarget_{input_source}_{side}.yaml`), so it is
  load-bearing, not vestigial.

- **`pico_teleop.launch.py` starts input and hands only.** It no longer starts
  an arm output or the camera stack. `g1_world_output` runs in a separate
  image, so it cannot be a `Node` in a `teleop`-container launch file.

- **The Monitor lost its arm gating.** `/tianji_arm/lifecycle_state` was how the
  GUI decided when "starting" became "running". Subprocess liveness plus a 2 s
  warm-up is now the only signal, which is what its docstring already claimed.

- **The Docker entrypoint lost its Tianji health checks.** They called `exit 1`
  on a missing `libKine.so` or `libMarvinSDK.so`, so leaving them would have
  failed the container outright. The SteamVR/OpenVR path registration, the
  MANUS SDK and udev blocks, and `pid: host` (SteamVR IPC) went with them.

## Kept on purpose

- **`src/camera/`** — kept and unwired, not deleted. It targets D405 wrists and
  an HBVCAM head we do not have, but the G1's own head cameras (RealSense
  **D435i** built-in, **D455** attachment) are planned, and `d435i` is *already*
  a supported and default type in `camera_launch.py`. Deleting it would mean
  deleting working D435i support and restoring it later. Its `xrobo_protocol.py`
  is also the only thing in this repo that knows how to get video into the PICO
  headset, and it is camera-agnostic. Migration notes:
  `src/camera/README.md`. Docker keeps `ros-humble-realsense2-camera`, `ffmpeg`,
  the `c 81:*` V4L2 cgroup rule (librealsense uses the V4L2 backend, so this is
  not UVC-only), and the commented NVENC block.
- **`docs/wuji-camera-topics.md`** — the topic map is still accurate for the code
  as written, so it stays with a status banner rather than being deleted.
- **`src/wujihand_urdf/`** — referenced only by a README tree listing, but it
  holds the Wuji Hand URDFs, which are our hardware.
- **`TIANJI_INIT_POS` / `TIANJI_INIT_ROT`** in `test/common/robot_config.py` —
  the names are kept because the values genuinely are Tianji-derived. Renaming
  them would hide that.
- **`get_default_qos()`** in `src/controller/controller/common.py` — now unused
  (its only caller was the MANUS subscription), kept as a generic ROS2 helper.

## Known follow-ups

None of these are regressions from the cleanup; they are pre-existing gaps it
made visible.

| Item | Detail |
|---|---|
| **Incremental-control anchors are still Tianji FK** | `init_pos`, `init_rot`, `init_quat`, `arm_init_pos`, `arm_init_quat` in `pico_input/config/robot_frames.yaml` are FK of the Tianji 7-DoF arm's calibrated pose. They set where the arms sit at session start. Not re-derived for the G1_23 (5 arm DoF per side, different reach). Carried over verbatim so the cleanup changed no behavior |
| **PICO -> G1 has never been run** | The topic contract matches and cross-container DDS is configured, but nobody has driven the G1 from the PICO. Needs the rig |
| **Monitor cannot start the G1** | The GUI runs inside the `teleop` container via `docker exec`; `g1_world_output` is another container. A one-click preset needs a mounted docker socket or a host-side helper |
| **Joint panel still shows 7 arm columns** | `joint_panel.py` was built for the Tianji 7-DoF arm. The G1_23 has 5 per side, so the last two cells stay `--`. Noted inline at the column-count loop |
| **`controller/package.xml` over-declares** | `std_msgs`, `std_srvs`, `tf2_ros`, `geometry_msgs` were `tianji_arm_node.py`'s dependencies. Left declared because they are installed ROS core packages and removing a needed one without a build to verify is the worse risk |
| **`wuji_teleop_bringup` under-declares** | It launches `pico_input`, `controller`, and `wujihand_driver` without declaring them as `exec_depend`. Pre-existing; not touched to keep this pass to deletions |
| **Submodule init is a build prerequisite** | A recursive submodule init is required before `docker compose build`, not just after a pull as `CLAUDE.md` implies. The image build fails at `COPY src/wujihandros2/external/wuji-description/...` without it |
