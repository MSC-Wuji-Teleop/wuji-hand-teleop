# g1_world_output

Unitree G1_23 arm output package for PICO teleoperation. Same ROS topic contract
as `pico_input` publishes; remaps chest-frame wrist poses into the G1 pelvis
frame, then runs Pinocchio/Casadi IK and Unitree DDS LowCmd control.

## Modes

`g1_world_output_node` is the **only** node that talks to DDS -- see
`robot_arm.py`'s writer lockfile (`/tmp/g1_lowcmd_writer.lock`), which turns
a second DDS writer into a startup failure instead of two processes
silently fighting over `rt/lowcmd`/`rt/arm_sdk`. Everything else (live
teleop, offline replay) feeds this one node through topics; which topics it
listens to is controlled by the `mode` ROS parameter, switchable at
runtime:

| mode | input | how it's driven |
|---|---|---|
| `pose` (default) | `/left_arm_target_pose`, `/right_arm_target_pose` | chest->pelvis remap -> `G1_23_ArmIK.solve_ik` -> DDS |
| `joint_replay` | `/left_arm/joint_targets`, `/right_arm/joint_targets` | no IK -- joints matched by name, linearly interpolated one publish period behind between the two newest samples ([Interpolation](#interpolation)), sent straight to DDS |
| `idle` | (none) | holds the arms at their current measured position |

A message arriving on a topic that doesn't match the active mode is
dropped with a throttled warning, not queued. Switching modes always seeds
the next command from the arm's current measured position first (bumpless
transfer) so the switch itself can't produce a step.

Orthogonal to `mode`, the `arm_type` parameter selects the joint-name
layout: `G1_29` (the config default; 7 DoF/arm, the rig's robot) or
`G1_23` (5 DoF/arm). Both drive DDS through `G1ArmController`
(`robot_arm.py`); `g1_controller.py` selects the motor slots and wrist
gains per variant. The pose IK is `G1_23`-only, so the node refuses
`mode=pose` with `G1_29`. `joint_replay` and `idle` work with either.

```
pico_input                                    (mode=pose)
  -> /left_arm_target_pose   (PoseStamped, chest frame)
  -> /right_arm_target_pose  (PoseStamped, chest frame)
  -> g1_world_output
       chest -> pelvis remap  (transform_utils.chest_pose_to_pelvis)
       G1_23_ArmIK.solve_ik   (robot_arm_ik.py, 5 DoF/arm, lazy-loaded)
       G1ArmController        (robot_arm.py -> rt/lowcmd or rt/arm_sdk)

replay_publisher (or any joint source)    (mode=joint_replay)
  -> /left_arm/joint_targets  (JointState, named)
  -> /right_arm/joint_targets (JointState, named)
  -> g1_world_output
       match by name against joint_names(side); SideBuffer interpolates one publish period behind
       G1CartesianController.move_to_joints_direct  (no IK)
       G1ArmController        (robot_arm.py -> rt/lowcmd or rt/arm_sdk)
```

### Interpolation

Frames arrive at the publish rate (50 Hz times the replay speed) and the
control loop runs at `control_rate` (250 Hz on the rig). Each side has a
`SideBuffer` (`g1_world_output/side_buffer.py`, pure Python, no ROS). It
holds the newest sample until two real samples exist, then commands
`q_prev + alpha * (q_next - q_prev)` with
`alpha = (now - t_next) / (t_next - t_prev)` clamped to `[0, 1]`: linear
between the two newest samples, one publish period behind, where `t` is
arrival time. The first frame is a step from the measured pose (the seed
written at a mode switch is not a sample), and every later frame is
continuous, one publish period late, at any speed. The rule before
2026-09-02 measured `alpha` from `t_prev`, which is at or past 1 at every
poll, so it was a zero-order hold at the publish rate. Tests:
`tests/test_side_buffer.py`.

### Replaying a SOT bundle sample (`joint_replay` mode)

The full replay pipeline -- arms **and** hands, mirrored in MuJoCo on the
29-DoF model -- is driven by the `replay` package (see
[its README](../../input_devices/replay/README.md)); the complete
four-process runbook is
[docs/usage.md -- SOT bundle replay](../../../docs/usage.md#sot-bundle-replay-sim).
This node's part of it:

```bash
# host, from docker/ -- joint_replay + 29-DoF names, no DDS/hardware
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    dry_run:=true mode:=joint_replay arm_type:=G1_29 control_rate:=250.0
```

Any publisher of named `/left_arm/joint_targets` / `/right_arm/joint_targets`
works, not just `replay`: joints are matched by name against
`joint_names(side)`, unknown names are warned about and ignored, and a
message missing a required joint is dropped whole rather than partially
applied. `scripts/joint_replay_publisher.py` remains as an arms-only debug
source (same wire format, reads the npz directly; see its docstring).

**Hands are out of scope for this node.** The bundle's hand columns target
the legacy 20-DoF hand model, not the real Wuji Hand 2, and must never be
sent to it directly (`legacy_wuji_sim_only/`, marked `DO_NOT_COMMAND_HAND2.txt`
in every sample). `tools/prepare_clip.py` regenerates Hand 2 joints offline
from each sample's 21-point keypoints, and `replay_publisher` sends them
straight to the hand drivers on `/left/wuji_hand/joint_command` and
`/right/wuji_hand/joint_command`; under `--sim` no driver runs and
`scripts/mujoco_visualizer.py` mirrors those two topics instead
([Sim mode](#sim-mode-vs-hardware-mode), [docs/spec/spec1.md](../../../docs/spec/spec1.md)).

## Topics (the standard arm-output contract)

| Direction | Topic | Type |
|-----------|-------|------|
| sub | `/left_arm_target_pose` | `geometry_msgs/PoseStamped` (mode=pose) |
| sub | `/right_arm_target_pose` | `geometry_msgs/PoseStamped` (mode=pose) |
| sub | `/left_arm_elbow_direction` | `geometry_msgs/Vector3Stamped` (mode=pose, echo only) |
| sub | `/right_arm_elbow_direction` | `geometry_msgs/Vector3Stamped` (mode=pose, echo only) |
| sub | `/left_arm/joint_targets` | `sensor_msgs/JointState`, named (mode=joint_replay) |
| sub | `/right_arm/joint_targets` | `sensor_msgs/JointState`, named (mode=joint_replay) |
| pub | `/left_arm/joint_states` | `sensor_msgs/JointState` (rad, named, per `arm_type`) |
| pub | `/right_arm/joint_states` | `sensor_msgs/JointState` (rad, named, per `arm_type`) |
| pub | `/left_arm/joint_commands` | `sensor_msgs/JointState` (rad, named, per `arm_type`) |
| pub | `/right_arm/joint_commands` | `sensor_msgs/JointState` (rad, named, per `arm_type`) |
| pub | `/left_arm/zsp_para` | `std_msgs/Float64MultiArray` (mode=pose) |
| pub | `/right_arm/zsp_para` | `std_msgs/Float64MultiArray` (mode=pose) |

Arm joints per side: shoulder pitch/roll/yaw, elbow, wrist roll (`G1_23`,
5/side), plus wrist pitch/yaw under `arm_type:=G1_29` (7/side). The
`/left_arm/joint_targets`/`/right_arm/joint_targets` input accepts extra
names beyond the active layout and warns-and-ignores them -- see
[Modes](#modes).

### Both arms are always commanded together

The topics are per-side, but the control is not — this is the one place where
behaviour diverges from the removed `tianji_world_output`, which treated each arm as an
independent controller. Here, the control loop runs as soon as *either*
`/left_arm_target_pose` or `/right_arm_target_pose` has been received, and
`G1CartesianController.move_to_pose_direct` then solves **both** arms in a single
IK problem: a side with no pose yet is substituted with its configured
`reset_wrist_pose` and driven there, rather than being left uncommanded. Two
consequences follow. First, publishing only one side's pose still moves the other
arm — to its home pose, not nowhere. Second, IK feasibility is shared: the solver
returns one success flag for all 10 joints, so an unreachable target on one side
suppresses the command for both, and the arms hold the last good solution. This
is a consequence of the G1's single DDS `LowCmd`, which carries every motor in one
CRC-stamped message, and of formulating the IK over both arms at once. Note that
`LowCmd`'s per-motor `kp`/`kd`/`q` fields *would* permit addressing one arm alone
(zero its gains, or command its measured position); the current controller does
not use that, and writes all 10 arm joints on every cycle.

## Build / run

```bash
cd ~/Projects/ros2_ws
colcon build --packages-select g1_world_output g1_wuji2_description
source install/setup.bash

ros2 launch g1_world_output g1_world_output.launch.py
```

`g1_wuji2_description` must be built alongside `g1_world_output` — the URDF is
resolved via `get_package_share_directory('g1_wuji2_description')`
(`config_loader.py`), which only exists once that package is built and
`install/setup.bash` is sourced.

## Sim mode vs. hardware mode

`--dry-run` (CLI flag or `dry_run:=true` launch arg) is the hardware/sim
toggle: the node still solves real IK from `/left_arm_target_pose` /
`/right_arm_target_pose` and publishes `/left_arm/joint_commands` /
`/right_arm/joint_commands`, it just never opens a DDS connection, so no
physical G1 (or DDS sim bridge) is required. Pair it with
`scripts/mujoco_visualizer.py` (run in the main `teleop` container, which has
`rclpy` + `mujoco`) to see the result live instead of on the real robot:

```bash
# Hardware: real DDS to the physical G1
docker compose run --rm g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py

# Sim: no physical G1 touched, watch it in MuJoCo instead
docker compose run --rm g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true
docker exec -it wuji-hand-teleop python3 \
    src/output_devices/g1_world_output/scripts/mujoco_visualizer.py
```

`motion_mode` / `simulation_mode` can be overridden the same way
(`motion_mode:=false`, `simulation_mode:=true`) — see [Config](#config) for
what each one actually does; neither replaces `--dry-run` as the "no
hardware" switch. `mujoco_visualizer.py` only subscribes (it publishes
nothing), so it also mirrors real teleop's `/left_hand/joint_commands` /
`/right_hand/joint_commands` (the glove controller's positional 20-vectors)
if `wujihand_controller` happens to be running too, and the replay
publisher's `/left/wuji_hand/joint_command` / `/right/wuji_hand/joint_command`
(the hand drivers' own command topics, 20 named joints). Arm commands and
the driver-topic hand commands are matched by joint name against the loaded
model (`left_elbow` -> MJCF `left_elbow_joint`; `l_thumb_ip` -> MJCF
`left_wuji_l_thumb_ip`, then the actuator driving that joint); names the
model does not have are skipped and logged once per distinct set. The
default model is `g1_23_wuji2_fixed.xml`; clip replay runs on the 29-DoF
model, which is what `--sim` in [docs/replay.md](../../../docs/replay.md)
passes:

```bash
# Clip replay sim: G1 node in joint_replay on the 29 names, no DDS; viewer on the 29 model
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    dry_run:=true mode:=joint_replay arm_type:=G1_29 control_rate:=250.0
docker exec -it wuji-hand-teleop python3 \
    src/output_devices/g1_world_output/scripts/mujoco_visualizer.py \
    --mjcf src/g1_wuji2_description/g1_29_wuji2_fixed.xml
```

That hand side has its own hardware/sim split, independent of G1: the Wuji
Glove → retargeting → `/left_hand/joint_commands` publish
(`wujihand_controller`, in `src/controller/`) never touches the physical
Wuji Hand SDK itself; only the separate hand driver process does
(`starport_wuji_hand` `hand_node`, Ethernet; see `docs/spec/spec1.md`).
So real glove input can drive `mujoco_visualizer.py` with no Wuji Hand
plugged in at all:

```bash
# Real glove input, no physical Wuji Hand
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py enable_hand_driver:=false
python3 src/output_devices/g1_world_output/scripts/mujoco_visualizer.py --focus hands
```

`--focus hands` just tightens the initial camera framing for a hands-only
session (`--focus full`, the default, is the whole-body G1 framing) — it's
still freely orbitable once the viewer is open. This is independent of the
G1 arm toggle above: run either, both, or neither — each topic pair only
moves in the viewer if something is actually publishing it.

### Testing without any input at all (sweep + MuJoCo visualization)

`scripts/sweep_and_visualize.py` is the same idea but also *generates* the
input: it sweeps both Wuji Hands through their joint ranges and both arm
target poses through a small Lissajous pattern, publishes them on the topics
above, and mirrors the result live in MuJoCo — useful when you don't have a
glove/tracker handy and just want to see the whole pipeline move.

```bash
# terminal 1 — real IK, no hardware/DDS required
cd docker && docker compose run --rm g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true

# terminal 2 — sweep + viewer (needs rclpy + mujoco, e.g. the main teleop container)
python3 src/output_devices/g1_world_output/scripts/sweep_and_visualize.py
```

`--no-viewer` publishes only (headless topic smoke test); `--period`,
`--pos-amplitude`, `--rot-amplitude-deg` tune the sweep. See the script's
module docstring for the full topic contract it exercises. Both scripts
share their MuJoCo plumbing via `scripts/_mujoco_common.py`; its
joint-name -> ctrl mapping is pinned by `tests/test_mujoco_common.py`
against both composed models (runs where `rclpy` is installed).

Two things make the cross-container round trip (either script) work out of
the box:

- `docker/cyclonedds.xml` (shared by both images) lists `127.0.0.1` as a
  static DDS peer. SPDP multicast discovery between two `network_mode: host`
  containers on the same machine is unreliable on Wi-Fi APs that drop or
  throttle multicast; the peer entry adds a unicast discovery path over
  loopback so `teleop` and `g1_world_output` find each other quickly without
  touching multicast behavior for a real, separately-networked G1 (that
  topology itself isn't validated either way — see the comment in that file).
- Both scripts default `LIBGL_ALWAYS_SOFTWARE=1` before importing `mujoco`.
  On a host GPU newer than the container's Mesa build supports, hardware
  GL context creation fails (`libGL error: failed to load driver: iris`)
  and the fallback path can block Ctrl-C/SIGTERM while pegging a CPU core.
  Set it to `0` beforehand if your setup has a working hardware GL driver.

## Config

`config/g1_robot.yaml`:

- `arm_type: G1_29`; `G1_23` (5 DoF/arm) stays supported
- `urdf_package_dir: ""` — blank resolves via `get_package_share_directory('g1_wuji2_description')` (requires that package built and `install/setup.bash` sourced); set an absolute path only to override
- `urdf_filename: g1_23_wuji2.urdf`
- `motion_mode` / `simulation_mode` — DDS channel selection (`rt/arm_sdk` vs `rt/lowcmd`, DDS domain 0 vs 1). `motion_mode` defaults to `true` (`rt/arm_sdk`): the G1's onboard controller keeps the legs. Set `false` (`rt/lowcmd`, full low-level bus) only with the robot hanging on a stand. Both assume *some* DDS peer answers `rt/lowstate` — neither is a "no hardware needed" switch; that's `--dry-run` (see [Sim mode vs. hardware mode](#sim-mode-vs-hardware-mode))
- `reset_wrist_pose` — home EE targets used at startup

## Dependencies

- `unitree_sdk2py` (DDS LowCmd / LowState) — vendored via the `src/unitree_sdk2_python` git submodule (not on PyPI)
- `pinocchio` (built with CasADi Python bindings), `casadi`, `numpy==1.26.4`, `scipy`, `PyYAML`
- URDF + meshes from `src/g1_wuji2_description/`

### Why this package has its own Docker image

`robot_arm_ik.py` needs `from pinocchio import casadi as cpin`. No PyPI `pin`
wheel (checked 4.1.0, 4.0.0, 3.8.0) ships that binding — it's headers-only on
PyPI, regardless of whether `casadi` is also installed. The only prebuilt
package that does ship a working `pinocchio.casadi` for this target
(Ubuntu 22.04 / Python 3.10) is the robotpkg apt build
(`robotpkg-py310-pinocchio` + `robotpkg-py310-casadi`), and that build is
linked against NumPy 1.x — it fails to import under NumPy 2.x. The rest of
wuji-hand-teleop pins NumPy 2.2.6 for its own (pip) Pinocchio build. Since
those two requirements can't coexist in one Python environment, this node
runs in its own container — see `docker/Dockerfile` — talking to the rest of
the stack purely over ROS2/DDS (shared `ROS_DOMAIN_ID` + host networking),
not through a shared filesystem or interpreter.

Build / run this container specifically:

```bash
cd ../../../docker   # wuji-hand-teleop/docker/
docker compose build g1_world_output
docker compose up -d g1_world_output
```
