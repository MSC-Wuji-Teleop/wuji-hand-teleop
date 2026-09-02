# Replay runbook

Exact commands for preparing and playing clips on the rig. Design, clip
format, and build status: [spec/spec1.md](spec/spec1.md). Everything below
is built; the online half has not yet run in the container or on the rig.

Where things run. Every code block below starts with the line that gets
you to the right place:

- `# host` commands run on the rig host, from the repo root, with the
  containers up (`cd docker && docker compose up -d`, once per boot).
- `# teleop container` commands run inside the `wuji-hand-teleop` container.
  Enter it with `docker exec -it wuji-hand-teleop bash`; you land in
  `~/ros2_ws` with the workspace sourced. `clips/` and `RobotSTAR_demos/`
  from the repo root are bind-mounted there under the same relative paths.
- The G1 node never runs in the teleop container. `scripts/replay.sh`
  starts it in its own container for you; the two-terminal form shows the
  `docker compose run` line it uses.

## Clips: where and what

The publisher only plays directories under `clips/safe/`. Each is one
trajectory, named `<sample>_<GT|Ours>`, and holds `arm_q.npz`,
`hand_q20.npz`, and `clip.json`. `clip.json` carries the verdict, the
`safe_speeds` list, and the audit numbers (peak contact force and pair,
peak arm torque ratio, saturation, tracking error) per speed. Read it before
the first run of a new clip. Full format: [spec1.md](spec/spec1.md#clip-directory).

## 1. Prepare clips (offline, no hardware)

One trajectory:

```bash
# teleop container
docker exec -it wuji-hand-teleop bash
python3 tools/prepare_clip.py \
    --method-dir RobotSTAR_demos/samples/<sample>/Ours \
    --out clips
```

Every trajectory in the bundle (15 samples x GT and Ours):

```bash
# teleop container (same shell as above)
python3 tools/prepare_clip.py --all RobotSTAR_demos/samples --out clips
cat clips/summary.md      # one row per trajectory: verdict, safe speeds, peak force and pair, torque ratio
```

Options, all recorded in `clip.json`:

| option | default | use |
|---|---|---|
| `--speeds` | `1.0 0.5 0.25` | speeds to audit; a clip is safe if any passes |
| `--cutoff-hz`, `--max-step-deg` | `6`, `15` | arm smoothing |
| `--trim-start N`, `--trim-end N` | `0` | drop frames |
| `--auto-trim --min-seconds S` | off, `3` | keep the longest window that passes |
| `--allow-flips` | off | smooth through a 90 deg single-frame step instead of refusing |
| `--max-arm-torque-ratio`, `--max-contact-force-n` | `0.8`, `80` | pass thresholds |
| `--note "..."` | | reason, when a threshold was changed |

A rejected clip lands in `clips/rejected/` with the same `clip.json`; the
`per_speed` block says which number failed at which speed. Rerun slower,
trim, or change a threshold with a note.

## 2. Check hardware connections

Robot powered, host NIC on the hands' subnet, `cd docker && docker compose
up -d` done.

```bash
# host, repo root
scripts/replay.sh --check
```

Starts the G1 node and the hand drivers with no publisher, waits up to 20 s
for state from each, prints the rates, and exits 0 when every source
reported, 1 otherwise. `--arms` and `--hands` narrow what is started and
checked:

```
/left_arm/joint_states        ~250 Hz    G1 node writing, arms holding measured pose
/right_arm/joint_states       ~250 Hz
/joint_states                 ~100 Hz    both hands, 40 names (l_*, r_*)
/left/wuji_hand/connected     true
/right/wuji_hand/connected    true
```

A hand missing here is a network fact: the driver discovers hands by UDP
broadcast on their subnet. Serials and network facts:
[spec/hardware_spec.md](spec/hardware_spec.md).

What the check itself does to the robot: the G1 node position-holds the
arms at their measured pose and raises the `arm_sdk` weight; each hand
driver homes its hand to the zero pose over 3 s on connect.

## 3. Single-device replays

One command, four scopes. Each plays the clip once on one device and holds
the last frame. Nothing else is started: with `--arms none` the G1
container does not run, with `--hands none` no hand driver runs.

```bash
# host, repo root
scripts/replay.sh clips/safe/<clip> --arms left  --hands none
scripts/replay.sh clips/safe/<clip> --arms right --hands none
scripts/replay.sh clips/safe/<clip> --arms none  --hands left
scripts/replay.sh clips/safe/<clip> --arms none  --hands right
```

## 4. Full replay

```bash
# host, repo root
scripts/replay.sh clips/safe/<clip>                    # --arms both --hands both, fastest safe speed
scripts/replay.sh clips/safe/<clip> --speed 0.25       # slower
```

Ctrl-C stops the publisher and the hand drivers, then the G1 container. The
G1 node releases the `arm_sdk` weight on shutdown, so the onboard controller
takes the arms back. The hands go limp after the driver's idle timeout (5 s
without commands).

## Flags

| flag | values | meaning |
|---|---|---|
| `--arms` | `none` `left` `right` `both` (default `both`) | which arm topics the publisher writes; `none` also skips starting the G1 container |
| `--hands` | `none` `left` `right` `both` (default `both`) | which hand driver topics the publisher writes; `none` skips the hand drivers |
| `--speed S` | `0 < S <= 1`; default: fastest value in the clip's `safe_speeds` | same frames published slower. Amplitudes do not change; peak velocity scales by `S`, acceleration by `S^2`. A speed above the clip's fastest safe speed is refused |
| `--check` | | connection check only (section 2) |
| `--sim` | | G1 node with `dry_run:=true`, no hand drivers, MuJoCo viewer on the composed model instead |

## Two-terminal form

What `replay.sh` runs. Use it when the G1 log should have its own window.

```bash
# T1: host. The G1 node in its own container. dry_run:=true for sim.
cd docker
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    mode:=joint_replay arm_type:=G1_29 control_rate:=250.0
```

```bash
# T2: host, then into the teleop container. Hand drivers for the selected
# sides + the publisher.
docker exec -it wuji-hand-teleop bash
ros2 launch wuji_teleop_bringup replay.launch.py \
    clip:=clips/safe/<clip> arms:=both hands:=both speed:=0.5
```

Stop order: Ctrl-C in T2 first (publisher and hand drivers), then T1 (the
G1 node releases the `arm_sdk` weight on shutdown).

## Sim

```bash
# host, repo root
scripts/replay.sh clips/safe/<clip> --sim
```

The G1 node runs with `dry_run:=true`, no hand driver starts, and
`mujoco_visualizer.py` opens on `g1_29_wuji2_fixed.xml`, mirroring the G1
node's arm commands and the publisher's hand commands. Details:
[usage.md](usage.md#sot-bundle-replay-sim).
