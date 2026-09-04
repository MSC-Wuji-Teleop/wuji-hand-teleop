# replay

Plays one prepared clip directory on the G1 arms and the two Wuji Hand 2
units, and checks that the device nodes are reporting before a run. Pure
`rclpy` + `numpy`: runs in the teleop container, never touches DDS or a
device SDK, never retargets. Design and clip format:
[docs/spec/spec1.md](../../../docs/spec/spec1.md). Operator commands
(`scripts/replay.sh`, launch, sim): [docs/replay.md](../../../docs/replay.md).

## What it publishes, and to whom

`replay_publisher` reads `clips/safe/<clip>/` (`arm_q.npz`, `hand_q20.npz`,
`clip.json`, written by `tools/prepare_clip.py`) and publishes every selected
side on one timer, so arms and hands stay time-aligned. All four publishers
are `RELIABLE`, `KEEP_LAST`, depth 10, matching the consumers.

| topic | type | consumer |
|---|---|---|
| `/left_arm/joint_targets`, `/right_arm/joint_targets` | `sensor_msgs/JointState`, 7 named joints, rad | `g1_world_output` with `mode:=joint_replay arm_type:=G1_29` (own container); matches by name, interpolates one publish period behind |
| `/left/wuji_hand/joint_command`, `/right/wuji_hand/joint_command` | `sensor_msgs/JointState`, 20 named joints, rad | `starport_wuji_hand` `hand_node`, one per side at `/{side}/wuji_hand`; matches by name, refuses the other hand's names |

Behaviour: wait for the selected consumers (same sources as `replay_check`;
`--ready-timeout` default 30 s, `0` skips), then a quintic approach from the
measured pose to frame 0 (`--ramp` default 2 s, matching the clip's start
velocity so the join is C1), then play at 100 Hz with linear interpolation
between clip frames. After the last frame it keeps publishing that frame
until killed. `--speed` scales clip time, not the publish rate, so a slow
replay is not a staircase. Nothing else: no run-time checks, no loop. Clip
quality is decided offline, before the run.

## Refusals

Checked once, before the first message, by `replay/clip.py` (pure, tested
without ROS). Each refusal exits with status 2 and a message that names the
file or value at fault.

- The clip directory's parent is not named `safe`. Only directories
  `prepare_clip.py` filed under `clips/safe/` are played; a copy elsewhere is
  refused whatever its `clip.json` says.
- `clip.json` `verdict` is not `"safe"`, or `safe_speeds` is empty.
- `arm_q.npz` / `hand_q20.npz` do not carry both sides as `(T, 7)` / `(T, 20)`
  finite arrays with `T` equal to `clip.json` `frames`.
- Joint names: not 7 arm and 20 hand names per side, wrong side prefix
  (`left_`/`right_` for arms, `l_`/`r_` for hands), or duplicates.
- `--speed` is not in `(0, 1]` or is above the largest `safe_speeds` entry.
- `--arms none` together with `--hands none`.

## replay_publisher

```bash
ros2 run replay replay_publisher -- --clip clips/safe/<clip> \
    [--arms none|left|right|both] [--hands none|left|right|both] \
    [--speed S] [--ready-timeout S] [--ramp S]
```

| flag | default | meaning |
|---|---|---|
| `--clip DIR` | required | a directory under `clips/safe/` |
| `--arms` | `both` | which arm topics are published; `none` publishes nothing to the G1 node |
| `--hands` | `both` | which hand driver topics are published |
| `--speed S` | largest `safe_speeds` entry | same frames published slower; peak velocity scales by `S`, acceleration by `S^2` |
| `--ready-timeout S` | `30` | wait for selected consumers before the first command; `0` skips (required when no drivers run, e.g. `sim:=true`) |
| `--ramp S` | `2` | quintic approach from the measured pose to frame 0; `0` skips |

## replay_check

What `scripts/replay.sh --check` runs in place of the publisher once the G1
node and the hand drivers are up. It publishes nothing; it subscribes
(`BEST_EFFORT`, depth 10, so it matches the G1 node's `BEST_EFFORT` state
publishers as well as the drivers' `RELIABLE` ones) and waits for:

- `/{side}_arm/joint_states` for each selected arm side (the G1 node writing);
- `/joint_states` carrying each selected hand side's 20 names, and
  `/{side}/wuji_hand/connected` having been `true` at least once (the driver
  reports `false` again after its 5 s idle release, which happens during a
  check since nothing commands the hands).

```bash
ros2 run replay replay_check -- [--arms none|left|right|both] \
    [--hands none|left|right|both] [--timeout S]
```

When every source has reported it prints the rates and exits 0. At
`--timeout` (default 30 s, matching the publisher's ready wait: two-hand
scan plus the driver's blocking 3 s home) it prints the table with the
missing rows marked and exits 1.

```
/left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose
/right_arm/joint_states       ~250 Hz
/joint_states                 ~100 Hz    both hands, 40 names (l_*, r_*)
/left/wuji_hand/connected     true
/right/wuji_hand/connected    true
```

A missing row reads `missing    no message in 30.0 s` (or `no r_* names in
30.0 s` when `/joint_states` is alive but one hand's names never appeared, or
`false      never true in 30.0 s` for a driver that is up without its hand).

## Layout and tests

```
replay/clip.py              clip directory loader and the refusals (pure numpy + json)
replay/check.py             connection-check rules and the table (pure Python)
replay/motion.py            clip lerp and the quintic approach (pure numpy)
replay/replay_publisher.py  the publisher node
replay/replay_check.py      the check node
test/                       pytest; needs numpy and pytest only, ROS is stubbed in test/conftest.py
```

```bash
# teleop container, ~/ros2_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider src/input_devices/replay/test -q
colcon build --symlink-install --packages-select replay    # after adding files
```
