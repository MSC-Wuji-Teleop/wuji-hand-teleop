# Replay runbook

Exact commands for preparing and playing clips on the rig. Design, clip
format, and build status: [spec/spec1.md](spec/spec1.md). Everything below is
built, and everything except the hardware sections has been run: the offline
half on the whole bundle, the online half in sim. Nothing has run on the rig.
On a host that has not run this before, start at
[Per-machine setup](#0-per-machine-setup-once-per-host); otherwise start at
[Which clip to run first](#which-clip-to-run-first).

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

`clips/safe/90_sweep_joints_GT`. It is generated rather than recorded, by
`tools/generate_sweep_sample.py`, and built for this: the left arm's seven
joints ramp together, then the right arm's, then a one-second stop, then each
thumb flexes on its own. It audits at peak arm torque ratio 0.33 and
essentially zero contact force, and passes at all three speeds.

Two things to expect. The 0.2 rad amplitude cap is on the **arm** joints; the
hands start the clip already curled, with the middle, ring and pinky PIP joints
near 1.4 rad. The publisher approaches that pose from the measured (homed)
joints over 2 s with a min-jerk ramp before clip time starts, so the first
command is no longer a 1.4 rad step. And only the thumbs move after that,
deliberately: whole-hand motion on this donor pose presses adjacent fingers
into each other, which the generator's docstring explains. Regenerating it:
[SOURCE.md](../clips/safe/90_sweep_joints_GT/SOURCE.md).

The sign-language clips are a different proposition. Of the bundle's 30
trajectories, 4 are safe, 23 are rejected and 3 are refused for estimator
orientation flips, and what rejects them is almost always **wrist pitch or wrist
yaw**: those two joints per arm carry a 5 Nm clamp against 25 Nm elsewhere.
Two mechanisms load them, and which one binds depends on the speed. At full
speed it is the motion itself: the arms still peak near 13 rad/s after
smoothing, and at kd 2 the damping term alone reaches the clamp at 2.5 rad/s.
Re-auditing `11_..._Ours` with every contact removed from the model still
saturates at 1.0x, and drops to 0.38 at 0.25x. At the slow end contact is what
holds the clamp, which is why slowing down often does not help. The bundle
authors' own physical audit saturates a wrist actuator on all 30, and names
wrist pitch or yaw as the worst joint in every one, so this is a property of
the source trajectories rather than of our audit. Re-solving the arm retarget
against this model is the fix. See
[the handoff note](issues/replay-handoff-2026-09-02.md#verified-2026-09-03).

## 0. Per-machine setup, once per host

Two values are not in git, because they name this host's hardware rather than
the robot's. Both must be right before section 2 can pass.

**The G1's network interface.** The Unitree SDK builds its own CycloneDDS
config and ignores `CYCLONEDDS_URI`, so this parameter is the only thing that
binds the robot link to the right adapter. Wrong or empty on a multi-NIC host,
the SDK takes the first interface and the only symptom is a lowstate timeout.

```bash
# host: which interface holds an address on the robot's subnet
ip -br addr | grep 192.168.123.
ip link                       # if nothing matches, the adapter is unplugged
```

Put that interface name in `network_interface` in
[g1_robot.yaml](../src/output_devices/g1_world_output/config/g1_robot.yaml).
It is committed with this rig's adapter already set. A name that does not
exist stops the node at startup with `<name>: does not match an available
interface`, which is the intended failure: it never quietly binds Wi-Fi. The
robot's address and subnet are in
[spec/hardware_spec.md](spec/hardware_spec.md).

**The hand serial numbers.** `wujihand_ik.yaml` is gitignored and seeded from
its template on first container start, with placeholders. Left as
`YOUR_LEFT_HAND_SERIAL` the driver scans, finds no hand with that serial, and
gives up after ten attempts.

```bash
# teleop container: what is on the hands' subnet right now
python3 src/starport_wuji_hand/scripts/set_hand_ip.py --list
```

Write the two serials into `left_hand.serial_number` and
`right_hand.serial_number` in
`src/output_devices/wujihand_output/config/wujihand_ik.yaml`, then
`colcon build --symlink-install --packages-select wujihand_output` so the
launch files read the new values. Leaving a serial empty makes the driver take
the first Hand 2 that answers, which is only safe with one hand connected: with
both, each node refuses the hand whose reported handedness does not match its
side.

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
| `--hand-lp-alpha` | the configs' `0.2` | the retargeter's low-pass on the fingers. 0.2 keeps the gross shape and flattens fast detail; 0.5 roughly doubles the retained finger motion at the same verdict. The hand driver's own 2 rad/s slew bounds what any value reaches the hardware as |
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
/left_arm/joint_states        ~100 Hz    G1 node writing, arms holding measured pose
/right_arm/joint_states       ~100 Hz
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

One command, four scopes. The publisher waits until the selected devices
have reported state (so one terminal is enough: drivers can still be
scanning and homing when launch starts), approaches frame 0 from the
measured pose over 2 s, then plays the clip once at 100 Hz with
interpolated frames and holds the last. Nothing else is started: with
`--arms none` the G1 container does not run, with `--hands none` no hand
driver runs.

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

## 5. Rehome the arms

Brings the arms slowly to a known pose. Use it when a clip has ended with the
arms somewhere awkward, when you stopped one mid-clip, or before powering down.
Design: [spec/spec1_1.md](spec/spec1_1.md).

```bash
# host, repo root. The G1 container must not already be running.
scripts/replay.sh --home
scripts/replay.sh --home --arms left                       # one arm at a time
scripts/replay.sh --home --sim --from stand                # rehearsal, no hardware
```

What it does to the robot, in order. Note that the publisher's usual 2 s
approach to frame 0 is switched off here (`ramp:=0`): a rehome clip already
starts at the measured pose, so there is nothing to approach.

1. Starts the G1 node exactly as a replay does. It takes the arms and holds the
   pose it measured at startup.
2. Reads that pose off `/{side}_arm/joint_states` and writes it to a file.
   Commands nothing.
3. Generates a clip: a half-cosine move from that pose to all-zeros, sized so no
   joint exceeds 0.2 rad/s, and audits it in MuJoCo the same way
   `prepare_clip.py` audits a recording. Prints the duration, the travel per
   arm, the peak arm torque ratio and the peak contact force with its body pair.
   If the audit rejects it the clip is filed under `clips/rejected/`, the
   command exits non-zero, and **the arms do not move**.
4. Plays that clip once with the ordinary publisher, then holds the home pose
   until Ctrl-C. Duration is typically 3 to 16 s and is printed before it starts.

All-zeros is the arms hanging straight down, wrists neutral. It is Unitree's own
`arm_sdk` zero posture, so releasing the weight afterwards hands the arms back
at the pose the onboard controller expects. No hand driver is started: the hands
stay limp.

`--from SPEC` skips reading the robot and uses a start pose you name (`stand`,
`zeros`, `clip:<dir>@last`, or 14 numbers). Required with `--sim`, because a
dry-run G1 node publishes no arm state to read. It is also how the audit matrix
in [issues/home-audit-matrix-2026-09-03.md](issues/home-audit-matrix-2026-09-03.md)
was produced.

### What `--home` does not protect against

**It is not an e-stop.** It is a slow deliberate motion and it takes the
duration the generator printed. The fast stop is the remote's damp command or
main power ([spec/hardware_spec.md](spec/hardware_spec.md)).

- **It does not open the hands.** They are limp, which is the right rest state,
  but if the fingers are interlocked, separate them before homing: damp from the
  remote and part them by hand, or start the drivers with `scripts/replay.sh
  --check`, which homes each hand to the zero pose over 3 s.
- **It cannot run while a replay holds the arms.** `G1ArmController` takes an
  exclusive lock on `/tmp/g1_lowcmd_writer.lock`, so a second writer is refused
  at startup. Stop the replay first. Two things never command the arms at once.
- **It refuses a start pose that is already in hard contact**, and that refusal
  is the whole answer for that case. Measured from a pose folded across the
  torso: 42.6 N of contact and a saturated wrist actuator before anything moves,
  rising to 133 N during the motion. No slower speed or path shape changes that.
  Damp from the remote and move the arms by hand.
- **The audit behind it models a fixed base, our gains, an assumed hand pose,
  and no harness.** It does not know real contact stiffness, the unconfirmed
  Hand 2 mount adapter, or the firmware's behaviour at a torque clamp. Read
  `peak_contact_pair` as much as the number.
- **It commands the arms for the whole motion.** If the audit was wrong about
  contact, the arms will push, and only the remote stops that.
- **If the arms are moved by hand between the capture and the play**, frame 0 is
  no longer the measured pose and the first frame becomes a step. The capture
  refuses a pose that moved more than 0.01 rad while it was being read, which
  catches the arms drifting, not someone moving them a minute later.
- **It does nothing about the legs, the waist, or an unstable robot.** Those
  stay with the onboard controller.

## Flags

| flag | values | meaning |
|---|---|---|
| `--arms` | `none` `left` `right` `both` (default `both`) | which arm topics the publisher writes; `none` also skips starting the G1 container |
| `--hands` | `none` `left` `right` `both` (default `both`) | which hand driver topics the publisher writes; `none` skips the hand drivers |
| `--speed S` | one of the clip's `safe_speeds`; default: the fastest | same frames published slower. Amplitudes do not change; peak velocity scales by `S`, acceleration by `S^2`. A speed the audit did not pass is refused, **including a slower one**: on `05_test_G42xKICVj9U_5-5-rgb_front_GT` the audit passes 0.5 and fails 0.25. To play a speed that is not listed, audit it first with `prepare_clip.py --speeds` |
| `--check` | | connection check only (section 2) |
| `--home` | | rehome the arms (section 5). Takes no clip, no `--speed` and no `--hands`, and starts no hand driver. Not an e-stop |
| `--from` | a start pose | `--home` only: use this start pose instead of reading the robot. `stand`, `zeros`, `clip:<dir>[@first\|@last]`, or 14 numbers. Required with `--sim` |
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
