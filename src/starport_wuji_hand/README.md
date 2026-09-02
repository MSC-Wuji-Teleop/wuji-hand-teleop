# starport_wuji_hand

ROS 2 driver for the Wuji Hand 2 over Ethernet. One `hand_node` per hand. It
takes joint targets in radians on a `JointState` topic, runs them through a
guard chain, writes them to the hand through `wuji_sdk`, and publishes
measured state and health. It carries no retargeting and no grasp semantics.
On the clip replay path its only producer is `replay_publisher`
(`src/input_devices/replay`); see [docs/replay.md](../../docs/replay.md).

## Origin

Vendored from Multiply Labs' `starport_wuji_hand`, BSD-3-Clause. The
`package.xml` maintainer line and license tag are kept as attribution; the
source copy carried no LICENSE file.

Pruned in the copy, with the reason: the RViz three-ghost view
(`hand_view.launch.py`, `hand_view.rviz`, `description.py`,
`test_description.py`) and its `rviz2`, `robot_state_publisher` and
`tf2_ros` dependencies, because nothing here runs RViz; the Beta 1 URDFs,
because this repo carries the Beta 2 URDFs in `src/wujihand_urdf`; the USB
udev rule, because the hand is on Ethernet; `generate_limits_yaml.py`,
because it read a per-hand MJCF this repo does not have; and two bench replay
scripts that hard-coded a clip path on another machine. Two tests that read
files outside the package now read this repo's URDFs and composed MJCF.
No parameter default and no guard behaviour changed.

## How the hand is found

Each hand is an Ethernet device with a static IP. `wuji_sdk` discovers hands
by UDP broadcast on their subnet, so the host NIC must hold an address on
that subnet. A hand on another subnet is invisible; `scan()` returning
nothing with the hand powered almost always means that, not a firewall.

The driver filters the scan to Hand 2 devices, then to `serial_number` when
one is given, connects to the first match, and checks the handedness the
device reports against its own `hand_side`. A wrong-side hand is refused.

`scripts/set_hand_ip.py --list` prints what answers the scan.
`scripts/set_hand_ip.py --ip A.B.C.D --execute` moves one hand's static IP.
Read its docstring first: the hand reboots onto the new address, and the
host needs an address on both subnets during the move.

## Node layout

`hand.launch.py` starts one node per selected side:

```bash
ros2 launch starport_wuji_hand hand.launch.py side:=both \
    left_serial_number:=<SN> right_serial_number:=<SN>
```

Each node is `Node(package='starport_wuji_hand', executable='hand_node',
name='wuji_hand', namespace='/{side}')`, so `~` resolves to
`/{side}/wuji_hand`. The launch arguments:

| argument | default | meaning |
|---|---|---|
| `side` | `right` | `left`, `right` or `both`; refused otherwise, before any node starts |
| `{side}_serial_number` | `""` | selects that hand; empty takes any Hand 2 the scan finds, handedness still checked |
| `{side}_limits_file` | `config/joint_limits_hand2_beta1_{side}.yaml` | the envelope the guard chain clamps to |
| `{side}_joint_sign` | twenty `1.0` | per-joint `+1.0` or `-1.0`, a wiring-direction correction |
| `{side}_joint_offset` | twenty `0.0` | per-joint zero correction, rad |
| `max_joint_velocity` | `2.0` | slew limit, rad/s |
| `effort_limit_a` | `0.6` | per-joint current ceiling, A |
| `setpoint_velocity_filter_hz` | `10.0` | low-pass on the commanded velocity, Hz |
| `home_on_start` | `true` | sweep to the zero pose on connect |
| `max_connect_attempts` | `10` | give up after this many; `0` waits forever |

Arguments are typed. Write float arrays as floats (`-1.0`, not `-1`); an
integer array is refused. Launching a side whose hand is absent does not
stop the other: that node reports the missing hand and gives up after
`max_connect_attempts`.

## Topics

| direction | topic | type | notes |
|---|---|---|---|
| sub | `/{side}/wuji_hand/joint_command` | `sensor_msgs/JointState` | radians. Named joints (unnamed ones hold the last safe target) or a bare 20-vector in hardware order. A joint name from the other hand is refused. Default QoS: RELIABLE, depth 10, so publishers must be RELIABLE; a BEST_EFFORT publisher never matches |
| pub | `/joint_states` | `sensor_msgs/JointState` | measured position, derived velocity, effort. Global: both nodes publish here with `l_` and `r_` names |
| pub | `/{side}/wuji_hand/commanded_joint_states` | `sensor_msgs/JointState` | the post-guard target written to the SDK |
| pub | `/{side}/wuji_hand/connected` | `std_msgs/Bool` | true while the link is up and the motors are enabled; false after an idle release |
| pub | `/{side}/wuji_hand/diagnostics` | `diagnostic_msgs/DiagnosticArray` | link, guard activity, error codes; `fatal` once the node has refused to run |

All publishers are RELIABLE, depth 10. `/joint_states` and `~/connected`
publish at `publish_rate`; diagnostics at 10 Hz. `JointState.velocity` is
derived by differencing positions and low-passing at 20 Hz; the hand
measures position and current only.

## Parameters

Read once at construction. A bad value refuses to start; `ros2 param set`
on a running node changes nothing. Defaults as declared in `hand_node.py`:

| parameter | default | meaning |
|---|---|---|
| `hand_side` | `right` | fixed per node by the launch file |
| `serial_number` | `""` | selects a hand; empty takes any Hand 2 the scan finds |
| `limits_file` | `""` | required; the launch file passes the packaged YAML |
| `limit_margin` | `0.02` | rad shaved off each end of the envelope to form the soft limits |
| `effort_limit_a` | `0.6` | A; per-joint current ceiling, written once at connect |
| `kp`, `kd` | `10.0`, `0.2` | MIT impedance gains, written once at connect |
| `max_joint_velocity` | `2.0` | rad/s; the slew guard's budget per tick is this times the measured tick |
| `command_timeout` | `0.25` | s without a command before the watchdog reports stale and holds |
| `idle_release_s` | `5.0` | s without a command before the motors are released; `0` holds forever |
| `command_rate` | `100.0` | Hz; the tick that runs the guard chain and writes one setpoint |
| `publish_rate` | `100.0` | Hz; `/joint_states` and `~/connected` |
| `home_on_start` | `true` | sweep to the zero pose on connect |
| `home_duration_s` | `3.0` | length of that sweep, s |
| `joint_sign` | twenty `1.0` | `+1.0` or `-1.0` per joint, applied at the SDK boundary |
| `joint_offset` | twenty `0.0` | rad per joint, same boundary |
| `max_connect_attempts` | `10` | `0` retries forever |
| `reconnect_interval` | `2.0` | s between connect attempts |

Also declared, defaults as vendored: `setpoint_velocity_filter_hz` 10.0,
`measured_velocity_filter_hz` 20.0, `diagnostics_rate` 10.0,
`link_timeout` 0.5, `friction_file` `""` (off), `friction_scale` 1.0,
`friction_velocity_deadzone` 0.02.

## What the driver does on its own

These are the driver's behaviours, set by its parameters. The replay path
adds nothing to them and mirrors the slew limit in the offline audit.

- On connect: sets the effort ceiling and gains, enables the motors, seeds
  the guard chain from the measured pose, then homes to the zero pose over
  3 s (`home_on_start`, `home_duration_s`).
- Every command passes four guards in order: finite check (NaN or Inf drops
  the whole message), clamp to the soft limits, slew limit at 2 rad/s,
  watchdog. A rejected command leaves the held target unchanged.
- No command for 0.25 s: the watchdog holds the last safe target and reports
  `stale`. No command for 5 s: the motors are released and the hand goes
  limp. The next command re-enables (about 0.7 s) and re-seeds from the
  measured pose. `~/connected` is false while released.
- No state frame from the hand for 0.5 s: the link counts as down, the node
  disconnects and retries.
- On shutdown or a crash inside the node: the hand is disabled.
- Latched refusals, never retried, reported as `refusing to run` with a
  `fatal` key in diagnostics: the hand reports the other handedness; the
  hand reports fewer than 20 joints online; `max_connect_attempts` is
  exhausted.

Two bench tools ship as executables and are not on the replay path:
`wave_check` curls one finger at a time; `replay_clip` plays an NPZ clip in
the original package's own format. `scripts/` holds bench scripts that talk
to `wuji_sdk` with no ROS (`scripts/DIRECT_SDK.md`).

## Joint order

Hardware order is the order in `joint_map.JOINT_NAMES_RIGHT`, mirrored to
`l_` for the left hand: thumb, index, middle, ring, pinky; per finger the
base flex, abduction, middle flex, tip. Position `i` on a topic is the slot
the SDK reads and writes. The same order is the movable-joint declaration
order of `src/wujihand_urdf/wujihand_{left,right}.urdf` and the order of the
`left_wuji_l_*` and `right_wuji_r_*` joints in
`src/g1_wuji2_description/g1_29_wuji2_fixed.xml`. `test/test_joint_map.py`
asserts both, and `test/test_limits_match_mjcf.py` asserts that each hand's
limits YAML equals that model's joint ranges, all 20 joints to 1e-6 rad.
Clip directories written by `tools/prepare_clip.py` carry `hand_q20` in this
order and name every joint on the wire.

## Tests

`test/conftest.py` imports `rclpy`, so on a machine without ROS run the pure
modules with `--noconftest`:

```bash
cd src/starport_wuji_hand
PYTHONPATH=. python3 -m pytest --noconftest \
    test/test_joint_map.py test/test_safety.py test/test_limits_match_mjcf.py -q
```

The full suite (node callbacks against a fake SDK, launch entities, the
bench tools) needs ROS. In the teleop container, after `colcon build` and
sourcing the workspace:

```bash
python3 -m pytest src/starport_wuji_hand/test -q
```

The conftest blocks `import wuji_sdk` for every test, so no test can reach a
hand on the network.

## Open items that need the rig

- The pinned `wuji-sdk` must expose what `hand_node.py` calls:
  `SdkManager.instance().scan()`, `DeviceType.WujiHand2`,
  `connect(sn=..., device_name=...)`, `handedness`, `online_joints_count`,
  `joint_states()`, `joint_diagnostics()`, `effort_limit().set()`,
  `mit_params().set()`, `enable()`, `disable()`,
  `joint_command().publish().send()`, and `JointCommand(q, qd, ff)`. This
  was not checked here; the package is not resolvable outside the
  container.
- The hands' IP addresses, and that both sit on a subnet the host NIC holds
  an address on.
- The hands' hardware revision. The limits YAMLs are named `beta1`; the Beta
  2 URDF and the composed MJCF carry the same 20 ranges, and the test above
  holds the YAMLs to the model. Whether the hands enforce that envelope is a
  bench question; `scripts/calibrate_joint_limits.py` measures it.
- `scripts/check_hand_link.py` and `test/test_check_hand_link.py` are from
  the original USB version (`wujihandpy`). They do not apply to the Ethernet
  hand and are kept as vendored.
