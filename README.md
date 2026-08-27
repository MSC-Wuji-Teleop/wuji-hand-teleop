# wuji-hand-teleop

ROS2 Humble teleoperation stack for one rig: a **Unitree G1 (23-DoF)** with **two
Wuji Hand 2** end effectors, driven by **Wuji Gloves** for the hands and a
**PICO 4** headset with 4 Motion Trackers for the arms. Docker on Ubuntu 22.04
x86_64 is the only supported runtime, and the hardware is fixed: this is a
single-rig fork, not a general-purpose framework.

Hardware source of truth: [docs/hardware_spec.md](docs/hardware_spec.md). What
was removed from upstream and why: [docs/deprecated/cleanup.md](docs/deprecated/cleanup.md).

---

## Run it

Both flows assume the image is built and the container is up. First time
through, open [Install](#install) below.

### Flow 1 — Hands only

One terminal. Wuji Gloves drive both Wuji Hands. No arms involved.

```bash
docker exec -it wuji-hand-teleop bash
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py
```

Add `enable_hand_driver:=false` to run off real glove input with **no physical
hand** attached, then watch it in MuJoCo:

```bash
python3 src/output_devices/g1_world_output/scripts/mujoco_visualizer.py --focus hands
```

### Flow 2 — PICO arms + hands

**Two terminals, and that is structural, not a convenience.** The G1 arm
controller lives in its own image because Pinocchio with working CasADi
bindings needs NumPy 1.x while the rest of the stack needs 2.x. Nothing in the
`teleop` container can start it.

```bash
# terminal 1 — PICO input + both hands (teleop container)
docker exec -it wuji-hand-teleop bash
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

```bash
# terminal 2 — the G1 arms (own container, run from the host)
cd docker
docker compose run --rm g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py
#   append dry_run:=true for sim: real IK, no DDS, no robot touched
```

The two containers share host networking, `ROS_DOMAIN_ID`, and
`docker/cyclonedds.xml`, so `/left_arm_target_pose` and
`/right_arm_target_pose` cross between them directly.

> **This end-to-end path has not been verified on hardware.** The topic
> contract matches and the cross-container DDS is configured, but nobody has
> driven the G1 from the PICO yet. The sim smoke test below exercises
> everything except the PICO itself.

---

<details id="install">
<summary><strong>Install</strong> — first-time host setup and image build</summary>

Ubuntu 22.04 x86_64. No GPU required.

```bash
# 1. Docker CE + compose plugin, then re-login (or `newgrp docker`)
sudo apt-get install -y docker.io docker-compose-plugin
sudo usermod -aG docker "$USER"

# 2. Clone
git clone <this-repo> ~/ros2_ws/src/wuji-hand-teleop
cd ~/ros2_ws/src/wuji-hand-teleop
```

**Two fetch steps are hard prerequisites for the build, not troubleshooting.**
Skip either and `docker compose build` fails:

```bash
# 3a. Submodules. Without this the build dies at
#     COPY src/wujihandros2/external/wuji-description/...
git submodule update --init --recursive

# 3b. Git LFS. The PC-Service .deb, the PICO APK, and several .so files
#     ship via LFS; dpkg reports "archive corrupt" on the pointer stubs.
sudo apt-get install -y git-lfs
git lfs install
git lfs pull
```

```bash
# 4. Build and start
cd docker
docker compose build
docker compose up -d
```

`docker/Dockerfile` is the source of truth for the environment (apt and pip
versions, SDKs). Rebuild the image only for `Dockerfile` or `prebuilt/`
changes; host source edits are live via the `../src` bind-mount.

</details>

<details id="configure-serial-numbers">
<summary><strong>Configure serial numbers</strong> — gloves and hands</summary>

Config files are tracked as `*.yaml.template` only. On first container start
`docker/entrypoint.sh` seeds each missing real `.yaml` from its template
sibling; the real files are gitignored, so serials and IPs never land in the
repo and a pull never conflicts with local values.

| File | Holds |
|---|---|
| `src/input_devices/wuji_glove/config/wuji_glove.yaml` | Glove serial numbers, per side |
| `src/output_devices/wujihand_output/config/wujihand_ik.yaml` | Hand serial numbers, driver rates, retarget params |
| `src/input_devices/pico_input/config/pico_input.yaml` | The four PICO tracker serials (left/right wrist + left/right forearm) |

Finding the serials:

```bash
# Wuji Hands
lsusb -v -d 0483:2000 | grep iSerial

# Wuji Gloves — printed on the device, and shown in Wuji Studio
```

The Monitor GUI has a **Scan SNs** button that writes these files for you.

After adding a *new* template, rerun `colcon build --symlink-install` so the
`install/share/` symlink picks it up.

Glove calibration is done in Wuji Studio 5.18, which writes `<SN>.toml` into
the host's `~/.wuji/sdk/params/`. That directory is bind-mounted into the
container, so the SDK reads the same calibration.

</details>

<details id="monitor-gui">
<summary><strong>Monitor GUI</strong> — one-click launcher</summary>

Qt5 dashboard with preset launch, live joint preview, and SN scanning. It runs
*inside* the teleop container and paints on the host X server.

```bash
# from the host
xhost +local:docker
src/wuji_teleop_monitor/scripts/launch_ui_docker.sh monitor

# or inside the container
ros2 run wuji_teleop_monitor monitor
```

Two presets: `Hand only (Wuji Glove)` and `Hand + PICO input`.

**Neither starts the G1 arms.** The GUI cannot reach the `g1-world-output`
container, so Flow 2's second terminal is still manual. A one-click preset is
tracked in [docs/issues/](docs/issues/).

`./install_desktop.sh` from `src/wuji_teleop_monitor/` installs a desktop
shortcut.

</details>

<details id="simulation">
<summary><strong>Simulation</strong> — no robot, no PICO, no hands</summary>

The hand and arm sim toggles are independent: run either, both, or neither.

- **Hands**: `enable_hand_driver:=false` skips `wujihand_driver`, the only
  process that opens hand USB. The controller still runs off real glove input
  and publishes `/left_hand/joint_commands` unchanged.
- **Arms**: `dry_run:=true` makes `g1_world_output` solve real IK from the
  target-pose topics and publish joint commands, without ever opening DDS.

The one check that needs **no hardware at all** — it generates its own input
and exercises the full cross-container round trip:

```bash
# terminal 1 — real IK, no DDS
cd docker && docker compose run --rm g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true

# terminal 2 — sweeps both hands through their ranges and both arm targets
# through a Lissajous pattern, then mirrors the result in MuJoCo
docker exec -it wuji-hand-teleop python3 \
    src/output_devices/g1_world_output/scripts/sweep_and_visualize.py
```

Add `--no-viewer` for a headless topic-only smoke test.

</details>

<details id="build-and-test">
<summary><strong>Build and test</strong> — daily commands inside the container</summary>

```bash
colcon build --symlink-install                 # after adding files or templates
colcon test --packages-select <package_name>
colcon test-result --verbose

python3 -m pytest src/controller/test/         # direct pytest also works
python3 -m pytest src/output_devices/g1_world_output/tests/
```

Host code edits are live via the bind-mount plus symlink install. Rerun
`colcon build --symlink-install` after adding new files or templates. If a
package fails to import after a pull, re-run the submodule init and rebuild.

Full reference: [docs/usage.md](docs/usage.md).

</details>

<details id="architecture">
<summary><strong>Architecture</strong> — packages and the topic contract</summary>

Data flows: input device, standard topic interface, output controller,
hardware. Inputs and outputs meet only at the standard interface, which is what
makes them swappable. Full map: [docs/architecture.md](docs/architecture.md);
every hop from device to MuJoCo with its file and line is in
[Hardware to sim data flow](docs/architecture.md#hardware-to-sim-data-flow).

| Package | Role |
|---|---|
| `src/input_devices/wuji_glove/` | Glove config. The SDK is imported in-process by each hand controller over UDP, so glove data never crosses a topic |
| `src/input_devices/pico_input/` | PICO headset + 4 trackers -> chest-frame `PoseStamped`. Also owns the PICO frame math (`transform_utils.py`, `config_loader.py`, `config/robot_frames.yaml`) |
| `src/controller/` | `wujihand_controller`, run as two independent processes (left/right) so the sides never block each other |
| `src/output_devices/wujihand_output/` | Hand retargeting + IK |
| `src/output_devices/g1_world_output/` | G1 arms: Pinocchio + CasADi IK, Unitree DDS. **Own container** |
| `src/g1_wuji2_description/` | Composed G1 + 2x Wuji Hand 2 URDF / MJCF / meshes. Generated; do not hand-edit |
| `src/wuji_teleop_bringup/` | Launch files, one per preset |
| `src/wuji_teleop_monitor/` | Qt5 GUI |
| `src/camera/` | **Staged, not wired.** Targets the planned G1 head cameras (D435i / D455). Nothing launches it |

Arm topic contract, which any arm output can consume:

```
/left_arm_target_pose      /right_arm_target_pose        geometry_msgs/PoseStamped
/left_arm_elbow_direction  /right_arm_elbow_direction    geometry_msgs/Vector3Stamped
```

Hand topic contract, ~120 Hz:

```
/left_hand/joint_commands  /right_hand/joint_commands    sensor_msgs/JointState
```

**Invariants**

- The hand controller never opens hand USB. Only the separate
  `wujihand_driver` process does.
- No launch file in the `teleop` container can start the G1 arms.
- `src/input_devices/pico_input/vendor/` is pinned upstream code under its own
  licenses. Do not modify it as first-party.
- For any coordinate-frame bug on the PICO path, read
  `src/input_devices/pico_input/ARCHITECTURE.md` before touching code.

</details>

<details id="troubleshooting">
<summary><strong>Troubleshooting</strong></summary>

| Symptom | Cause | Fix |
|---|---|---|
| Build fails at `COPY src/wujihandros2/external/...` | Submodules not initialized | Recursive submodule init, then rebuild |
| `dpkg: archive corrupt`, or `file format not recognized` on a `.so` | LFS objects are still pointer stubs | `git lfs install && git lfs pull`, then rebuild |
| Glove is discovered but connect times out | Multi-NIC routing | [docs/wuji-glove-network.md](docs/wuji-glove-network.md) |
| `pico_input` node exits immediately | An empty or duplicated tracker serial in `pico_input.yaml`. All four are required | Re-check the four `tracker_serial_*` entries |
| `No module named 'xrobotoolkit_sdk'` | PICO Pybind SDK not installed | `src/input_devices/pico_input/install_sdk.sh` |
| Arms do not move on the PICO path | Flow 2's second terminal is not running | Start `g1_world_output` in its own container |
| Cross-container DDS silent | `ROS_DOMAIN_ID` mismatch between containers | Both read it from the same env; check `docker compose config` |

Per-device setup: [docs/PICO.md](docs/PICO.md).

</details>

<details id="known-gaps">
<summary><strong>Known gaps</strong></summary>

- **PICO -> G1 is unverified end to end.** Needs the rig.
- **Incremental-control anchors are still Tianji-derived.** `init_pos` /
  `init_rot` / `arm_init_*` in `pico_input/config/robot_frames.yaml` are FK of
  the old Tianji 7-DoF arm and set where the arms sit at session start. Not
  re-derived for the G1_23. Carried over verbatim so the cleanup changed no
  behavior.
- **`chest_origin_in_pelvis`** in `g1_world_output/config/g1_robot.yaml` was
  derived from the 29-DoF body URDF; re-derivation against the 23-DoF
  description is pending.
- **EE frames** `L_ee` / `R_ee` sit on the wrist-roll links with a +0.20 m
  forward offset (an xr_teleoperate convention), which shifts the achieved palm
  pose by a constant wrist-frame vector.
- **Hand 2 mounting adapter does not exist yet.** The vendor STL is a Hand v1
  part. `g1_wuji2_description` uses a provisional flange.
- **Monitor cannot start the G1**, and the **joint panel still shows 7 arm
  columns** (Tianji's DoF count; the G1_23 has 5 per side).

Full list with detail: [docs/deprecated/cleanup.md](docs/deprecated/cleanup.md#known-follow-ups).

</details>
