# Replay runbook

Exact commands for preparing and playing clips on the rig. Design, clip
format, and build status: [spec/spec1.md](spec/spec1.md). Everything below is
built, and everything except the hardware sections has been run: the offline
half on the whole bundle, the online half in sim. Nothing has run on the rig.
Start at [Which clip to run first](#which-clip-to-run-first).

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

## Which clip to run first

`clips/safe/90_sweep_joints_GT`, from `RobotSTAR_demos/sweep-test`. It is built
for this: the left arm's seven joints ramp together, then the right arm's, then
a one-second stop, then each thumb flexes on its own, with amplitudes capped at
0.2 rad. It audits at peak arm torque ratio 0.33 and zero contact force, and it
passes at all three speeds.

The sign-language clips are a different proposition. Of the bundle's 30
trajectories, 4 are safe, 23 are rejected and 3 are refused for estimator
orientation flips, and what rejects them is almost always **wrist pitch or wrist
yaw**: those two joints per arm carry a 5 Nm clamp against 25 Nm elsewhere, and
in these clips the hands press on each other hard enough to hold that clamp at
any speed. The bundle authors' own physical audit agrees, saturating a wrist
actuator on all 30. Playing a rejected clip slower does not fix it; re-solving
the arm retarget against this model is what would. See
[the handoff note](issues/replay-handoff-2026-09-02.md#verified-2026-09-03).

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
cat clips/summary.md      # one row per trajectory: verdict, safe speeds, and the audit numbers at the speed named in the `at` column
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
`per_speed` block says which number failed at which speed. Rerun slower, trim,
or change a threshold with a note. Read the failing number before reaching for
`--speeds`: a torque ratio of 1.00 that stays at 1.00 as the speed drops is a
contact reaction the wrist cannot hold, and no speed will clear it.

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
scripts/replay.sh clips/safe/<clip> --speed 0.25       # slower -- only if 0.25 is in the clip's safe_speeds
```

`--speed` takes one of the clip's `safe_speeds` and nothing else. A speed the
audit did not pass is refused even when it is slower than the default, because
slower is not reliably safer here (see [Flags](#flags)).

Ctrl-C stops the publisher and the hand drivers, then the G1 container. The
G1 node releases the `arm_sdk` weight on shutdown, so the onboard controller
takes the arms back. The hands go limp after the driver's idle timeout (5 s
without commands).

## Flags

| flag | values | meaning |
|---|---|---|
| `--arms` | `none` `left` `right` `both` (default `both`) | which arm topics the publisher writes; `none` also skips starting the G1 container |
| `--hands` | `none` `left` `right` `both` (default `both`) | which hand driver topics the publisher writes; `none` skips the hand drivers |
| `--speed S` | one of the clip's `safe_speeds`; default: the fastest | same frames published slower. Amplitudes do not change; peak velocity scales by `S`, acceleration by `S^2`. A speed the audit did not pass is refused, **including a slower one**: on `05_test_G42xKICVj9U_5-5-rgb_front_GT` the audit passes 0.5 and fails 0.25. To play a speed that is not listed, audit it first with `prepare_clip.py --speeds` |
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
