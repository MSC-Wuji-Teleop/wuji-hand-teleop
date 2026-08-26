# Architecture

How data flows from input device to robot hardware, which process does what,
and the conventions that keep input and output packages interchangeable.

Audience: a developer working on the code. Operator setup lives in the
[guides index](README.md); daily commands in [usage.md](usage.md). For the
PICO coordinate-transform and incremental-control math, read
[pico_input/ARCHITECTURE.md](../src/input_devices/pico_input/ARCHITECTURE.md);
that is the reference for any coordinate-frame bug on the PICO path.

- [Data flow](#data-flow): the full pipeline in one diagram.
- [Input devices](#input-devices): how each device becomes a standard interface.
- [Hand controller](#hand-controller): the two-process retargeting + IK core.
- [Output devices](#output-devices): the four controllers and what each consumes.
- [Camera](#camera): head stereo + wrist RealSense pipeline.
- [Bringup and Monitor](#bringup-and-monitor): launch files and the GUI.
- [Process and container model](#process-and-container-model): what runs where, and why G1 is separate.
- [Configuration convention](#configuration-convention): the `.yaml.template` seeding scheme.
- [Invariants](#invariants): rules that must hold across changes.

## Data flow

Every path follows the same shape: **input device, standardized topic/TF
interface, output controller, hardware**. Inputs and outputs only meet at the
standard interface, which is what makes them swappable.

```mermaid
graph LR
    subgraph Inputs
        WG["Wuji Glove"]
        PICO["PICO 4<br/>+ 4 trackers"]
    end

    WG -. "wuji_sdk UDP<br/>in-process" .-> HC["wujihand_controller<br/>(left + right processes)"]
    PICO --> PI["pico_input"]

    PI --> TP["/left_arm_target_pose<br/>/right_arm_target_pose"]

    HC -- "/left_hand/joint_commands<br/>/right_hand/joint_commands" --> DRV["wujihand_driver<br/>(C++, USB)"]
    DRV --> HAND["2x Wuji Hand 2"]

    TP -.->|"cross-container DDS"| G1O["g1_world_output<br/>(own container)"]
    G1O --> G1["Unitree G1 (23-DoF)"]
```

The two dashed edges are the ones that are not a plain ROS2 topic hop inside
one process tree. The glove SDK is imported in-process by each hand
controller, so glove data never crosses a topic. The arm target poses do cross
a topic, but between *containers*: `g1_world_output` runs in its own image, so
nothing in the `teleop` container can start it. See
[Process and container model](#process-and-container-model).

## Input devices

All under `src/input_devices/`. Each turns hardware into one of the standard
interfaces:

| Package | Device | Interface it produces |
|---|---|---|
| `wuji_glove/` | Wuji Glove (default hand input) | None. Connects in-process via `wuji_sdk` UDP directly inside each hand controller |
| `pico_input/` | PICO 4 headset + 4 Motion Trackers | `PoseStamped` on `/left_arm_target_pose`, `/right_arm_target_pose`. These are chest-frame poses; the node converts from PICO's world frame internally, using `pico_input/transform_utils.py` and the anchors in `config/robot_frames.yaml` |

The topic contract for plugging in a custom input is specified in the
[Architecture](../README.md#architecture) section of the main README.

## Hand controller

`src/controller/` provides `wujihand_controller`, which runs as **two
independent processes** (`wujihand_controller_left`, `wujihand_controller_right`).
Each has its own GIL and runs its own retargeting + IK, so the sides never
block each other.

Key properties:

- **Input selection**: `wujihand_ik.yaml::input_source` picks which input
  device feeds it. Dispatch logic, and the reference integration pattern for a
  custom hand input, live in
  `src/output_devices/wujihand_output/wujihand_controller.py`.
- **Never touches hand hardware**: it always publishes
  `/left_hand/joint_commands` and `/right_hand/joint_commands`
  (`sensor_msgs/JointState`, ~120 Hz) regardless of whether a physical hand is
  attached. Only the separate `wujihand_driver` process (from the
  `wujihandros2` submodule, C++) opens the real USB connection.
- **Sim mode**: `wuji_teleop_hand.launch.py enable_hand_driver:=false` skips
  the driver process (real glove input, no physical hand). Pair with
  `g1_world_output/scripts/mujoco_visualizer.py --focus hands`.

## Output devices

All under `src/output_devices/`:

| Package | Hardware | Consumes |
|---|---|---|
| `wujihand_output/` | Wuji Hand | Hand IK controller config + retargeting parameters (see [Hand controller](#hand-controller)) |
| `g1_world_output/` | Unitree G1 (dual arm) | `/left_arm_target_pose`, `/right_arm_target_pose`, plus the elbow-direction hints. Does its own chest-to-pelvis remap (`transform_utils.py::chest_pose_to_pelvis`). Runs in its own container |

`g1_world_output` solves IK with Pinocchio + CasADi and talks to the robot
over `unitree_sdk2py` DDS. Its `--dry-run` flag (or `dry_run:=true` on the
launch file) is the hardware/sim toggle: real IK, no DDS connection. URDF,
MJCF, and meshes live in `src/g1_wuji2_description/`. Full details, including
the sim/hardware toggle and the MuJoCo scripts, are in the
[package README](../src/output_devices/g1_world_output/README.md).

## Camera

`src/camera/` runs the unified camera pipeline: a stereo head camera (USB UVC,
`/dev/stereo_camera` udev symlink) plus dual RealSense D405 wrist cameras. The
head camera splits into `/stereo/{left,right}/compressed` for ROS2 and
separately H.264-encodes (NVENC when available, else libx264) for PICO
streaming. Topic map and camera troubleshooting:
[wuji-camera-topics.md](wuji-camera-topics.md).

## Bringup and Monitor

- `src/wuji_teleop_bringup/` holds the launch files that wire packages
  together per preset: `wuji_teleop_hand.launch.py` (hand-only),
  `pico_teleop.launch.py` (PICO input + hands). Neither starts an arm
  output, because the G1 controller is in a different container.
- `src/wuji_teleop_monitor/` is the Qt5 GUI with three entry points:
  `monitor`, `brake`, `camera`. `monitor` is the primary one-click flow and is
  the reference for how presets map to launch files and flags.

## Process and container model

Two containers, one ROS2 graph (host networking, same `ROS_DOMAIN_ID`):

- **`teleop`** (`wuji-hand-teleop`): everything except the G1 controller. The
  host `src/` bind-mounts in as the colcon workspace source.
- **`g1_world_output`** (`g1-world-output`): the G1 controller only. It is a
  separate image because Pinocchio with working CasADi Python bindings only
  ships via a `robotpkg` apt build linked against NumPy 1.x, which conflicts
  with the NumPy 2.x the rest of the stack needs. The two sides talk purely
  over ROS2/DDS.

Within `teleop`, the hand pipeline is per-side processes end to end: two
controller processes, plus the driver process that owns USB.

## Configuration convention

Config files are tracked in git as `.yaml.template` only. On first container
start, `docker/entrypoint.sh` seeds each missing real `.yaml` from its
template sibling. The real files are gitignored, so serials and IPs never land
in the repo and `git pull` never conflicts with local values. After adding a
new template, rerun `colcon build --symlink-install` so the `install/share/`
symlink picks it up. The full config list is in
[Configure serial numbers](../README.md#configure-serial-numbers).

## Invariants

- **Controller/driver split**: only `wujihand_driver` opens the hand USB
  connection. The hand controller stays hardware-agnostic and always publishes
  joint-command topics.
- **Vendored code is pinned**: `src/input_devices/pico_input/vendor/` is
  upstream XRoboToolkit source under its own licenses. Treat it as an external
  dependency; do not modify it as first-party code.
