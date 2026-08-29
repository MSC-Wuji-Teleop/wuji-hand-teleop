# Spec 1 hardware bring-up runbook

The operator guide for the first hardware campaign: exact commands per
stage, what each stage proves, what to record and where. Requirements:
[spec_1.md](spec_1.md) and [TUITION.md](../../RobotSTAR_demos/TUITION.md)
(sections 6-9 bind every step). Runtime contracts:
[spec_1_interfaces.md](spec_1_interfaces.md). Sim gate:
[spec_1_stage0.md](spec_1_stage0.md).

Two independent tracks until the mount adapter exists: the **arm track**
(G1 on the rig) and the **hand track** (both hands benchtop on the rig
host). Combined runs are blocked (hard blocker 1). Solo operation: one
launch terminal per container, one `run_ctl` terminal, one hand on the
physical e-stop, always.

Realistic one-day target: Stage A both tracks, Stage B hands and at least
a few arm joints, first Stage C scoped runs at 0.25x. The stages are
ordered so partial progress is durable; do not skip gates to get further.

---

## 0. Before touching hardware

Ten minutes of setup that prevents hours of debugging.

### 0.1 Fill in the blanks

| what | where | how |
|---|---|---|
| Hand serial numbers | `wujihand_ik.yaml` (gitignored, seeded from [the template](../../src/output_devices/wujihand_output/config/wujihand_ik.yaml.template) on container start) — `left_hand.serial_number`, `right_hand.serial_number` | `lsusb -v -d 0483:2000 \| grep iSerial` with one hand plugged at a time, so you know which serial is which physical unit |
| G1 network interface | [`g1_robot.yaml`](../../src/output_devices/g1_world_output/config/g1_robot.yaml) `network_interface:` | `ip addr` on the host; the interface on the robot's subnet. `CYCLONEDDS_URI` does NOT reach the Unitree participant; this field is the only way to pin it |
| Hardware manifest | copy [`hardware_manifest_template.json`](../../src/input_devices/replay/templates/hardware_manifest_template.json) to `~/wuji_runs/hardware_manifest.json` | filled during Stage A; every `null` is a field you owe (TUITION section 6, section 11 items 1-4) |
| Operator name | pass `--operator <name>` to every `run_ctl load` | lands in `run_manifest.json` |

### 0.2 Rebuild after the merge

```bash
# host, repo root
git pull && git submodule update --init --recursive
mkdir -p wuji_clips wuji_runs          # host bind mounts for artifacts/runs
cd docker && docker compose up -d && docker compose build g1_world_output
# inside the teleop container
colcon build --symlink-install && source install/setup.bash
python3 -m pytest src/input_devices/replay/test \
    src/output_devices/g1_world_output/tests \
    src/output_devices/wujihand_output/tests -q     # expect 223 passed
```

### 0.3 Stage 0 sanity on the rig host (15 min, no hardware)

The Linux CI build passed, but the sim flow has not RUN anywhere yet. Run
it once before hardware: [spec_1_stage0.md](spec_1_stage0.md) steps 2-5
(conditioning sweep, one load/arm/start traversal in MuJoCo, the
piecewise-linear assert) -- its step 3 carries the full transition-by-
transition command set, which is the same operator sequence every
hardware stage below uses. If the traversal works in sim, every gate,
service name, and topic below is known-wired.

While the sweep runs, note each clip's verdict, `k`, and
`max_allowed_speed_scale` from `~/wuji_clips/verdict_table.json`; Stage C
picks from these.

<details>
<summary>Debug: sim traversal problems</summary>

- `run_ctl` prints "service not available": the supervisor is not up, or
  node names diverged. `ros2 node list` must show `/replay_supervisor`,
  `/replay_publisher`, `/wujihand_controller_left`, `/right`, and (own
  container) `/g1_world_output`. The names are pinned contracts.
- Supervisor faults at ARMED with "hand diagnostics silent": you launched
  the sim profile without `expect_hand_diagnostics:=false`; use
  `replay_sim.launch.py` (it sets it) rather than hand-rolling nodes.
- Bag error event `bag record failed to start`: the mcap plugin is
  missing. `apt list --installed | grep mcap`; install
  `ros-humble-rosbag2-storage-mcap` in the container. Non-fatal for sim
  (the run continues; events.jsonl still records).
</details>

---

## Terminal layout (hardware, both tracks)

| terminal | where | runs |
|---|---|---|
| T1 | host, `docker/` | the G1 arm node in its own container (per-stage command below) |
| T2 | teleop container | `ros2 launch wuji_teleop_bringup replay_hw.launch.py` (drivers + q20 controllers + publisher + supervisor) |
| T3 | teleop container | `run_ctl`, topic echoes, `condition_clip` |
| — | your hand | the physical e-stop |

`run_ctl status -w` in a spare pane is worth having the whole day.

---

## Stage A: read-only (TUITION 7A)

No motion command is sent by anything. Connect, observe both devices,
fill the hardware manifest, soak the comms for 10 minutes, and physically
exercise the e-stop and watchdog while only reading.

### A.1 Arm track

```bash
# T1 -- arm node, READ-ONLY: no writer lock, no DDS publisher, no write
# thread; every motion service refuses.
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    read_only:=true mode:=joint_replay arm_type:=G1_29
```

```bash
# T3 -- observe
ros2 topic echo /g1/status --once      # mode_machine, tick, lowstate_age_s,
                                       # max_motor_temp_c, voltage_v
ros2 topic hz /g1/imu                  # ~10 Hz
ros2 topic hz /left_arm/joint_states   # ~100 Hz, velocity+effort filled
```

Record into `~/wuji_runs/hardware_manifest.json` (g1 block):
`mode_machine_at_arm_time` (the `mode_machine` value while the onboard
controller is in its normal standing/damping mode — note WHICH mode),
`lowstate_identity` (tick rate, firmware/SDK versions from the robot app),
`control_frequency`. Leave the app/firmware fields you read off the
Unitree app or robot label filled in the same pass.

**Comm soak (10 min):** watch `lowstate_age_s` in `/g1/status`. It must
stay at ~0.002-0.01 s with no excursions past 0.1 s. Any sustained gap is
a network problem to fix now, not during a powered run.

**E-stop test (read-only):** press the e-stop. Watch `/g1/status`: does
`lowstate_age_s` grow (lowstate stops) or do values freeze with the tick
still advancing (robot depowered, DDS alive)? Release, watch recovery.
Record the observed behavior verbatim in
`e_stop_effect_on_lowstate_and_write_path`. This single observation
decides how the lowstate-loss reset behaves for the rest of the campaign.

### A.2 Hand track (benchtop)

```bash
# T2 -- full teleop-side stack; controllers are gated and publish nothing
ros2 launch wuji_teleop_bringup replay_hw.launch.py
```

```bash
# T3 -- observe per hand
ros2 topic echo /left_hand/status --once   # handedness, joints_online=20,
                                           # fsm_state=hold, fault=null
ros2 topic hz /left_hand/joint_states      # ~1000 Hz
ros2 topic echo /left_hand/hand_diagnostics --once
cat ~/wuji_runs/joint_mapping_left.json    # written by the q20 branch;
                                           # handedness_matches_namespace: true
```

Checks (7A list): both hands enumerate, driver-reported handedness matches
each namespace (a mismatch latches a fault in `/left_hand/status` —
that is the alarm working, swap the serials in `wujihand_ik.yaml`),
`joints_online: 20` per hand, zero error codes, temperatures reasonable.
Record serials-to-side, firmware versions, and SDK versions into the
manifest hand blocks; the joint tables are already in
`joint_mapping_{side}.json`.

**Gate out:** manifest g1+hand blocks filled, 10-min soak clean on both
tracks, e-stop and watchdog rows signed in the `stage_a` block.

<details>
<summary>Debug: no lowstate / arm node exits</summary>

- `TimeoutError: No rt/lowstate after 30s`: wrong NIC. Fill
  `network_interface` in g1_robot.yaml (the g1 container bind-mounts its
  package, so the edit is live on next start). Confirm the robot answers
  on that link (`ping` its IP from the host).
- Node starts but `mode_machine` is None / ages huge: the DDS domain is
  wrong; `simulation_mode` must be false on hardware (domain 0).
- `ros2 topic list` shows `rt/lowstate` etc. as unknown types: harmless;
  Unitree's raw DDS participant shares the domain with the ROS graph.
- Writer-lock error in read-only: cannot happen (read-only takes no
  lock); if you see it you are not on the new build.
</details>

<details>
<summary>Debug: hands missing or wrong</summary>

- Driver cannot open the device: check `lsusb -d 0483:2000` shows two
  units; the compose file mounts `/dev` rw, but a hand plugged in AFTER
  container start sometimes needs a re-plug or container restart.
- Both drivers grab the same hand: serials unset (template placeholders).
  Fill `wujihand_ik.yaml`, then restart T2.
- `/left_hand/status` shows `fault: joint_states stale` mid-session: the
  driver stopped publishing (USB drop). The controller is holding; fix
  the link, then `ros2 service call /wujihand_controller_left/clear_fault
  std_srvs/srv/Trigger`.
</details>

---

## Stage B: single joints, robot supported (TUITION 7B)

One joint at a time, tiny amplitude, through the exact load path real
clips use. Hands first (benchtop), then arms with the G1 suspended or
firmly supported — never free-standing. This stage proves index, sign,
zero, feedback agreement, the arm_sdk slot policy, and the gains.

```bash
# T3 -- generate all 54 artifacts once (audited, verdict pass)
bash tools/make_stage_b_clips.sh          # -> ~/wuji_clips/stage_b
```

### B.1 Hands (all 20 joints per side; the sides wiggle test is first)

Restart T1 is not needed (arm not involved). For each joint, left hand
first:

```bash
# T3 -- example: left thumb flexion (the sides wiggle test, 7A item)
ros2 run replay run_ctl load \
    ~/wuji_clips/stage_b/single_joint_left_hand_thumb_cmc_flex/conditioned_clip_v1.npz \
    --arms '' --hands left --operator alex
ros2 run replay run_ctl arm       # publish_first -> hand approach -> barrier
ros2 run replay run_ctl start     # the ramp plays; watch the physical hand
ros2 run replay run_ctl park      # hand slews to neutral
ros2 run replay run_ctl release   # closes the run dir + bag
```

Confirm per joint and tick a row in your notes: the COMMANDED joint moved
(the wiggle on `thumb_cmc_flex` must move the physical thumb — the only
check that catches an off-by-one across all five fingers), direction
matches positive = flex/abduct per the URDF convention, zero pose looks
neutral, `ros2 topic echo /left_hand/status` shows small
`max_target_error_rad`, no error codes, temperature flat. Record
`finger1_is_thumb_confirmed_physically` and
`sides_wiggle_test_{left,right}` in the manifest.

### B.2 Arms (robot suspended/supported; 7B order: left arm, right arm)

```bash
# T1 -- arm node, WRITING (this is the first command authority)
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    mode:=joint_replay arm_type:=G1_29 control_rate:=250.0
```

```bash
# T3 -- example: left elbow
ros2 run replay run_ctl load \
    ~/wuji_clips/stage_b/single_joint_arm_left_elbow/conditioned_clip_v1.npz \
    --hands '' --operator alex
ros2 run replay run_ctl arm
#   engage: weight ramps 0 -> 1 over 2 s at the measured pose. WATCH THE
#   WHOLE ROBOT during the first engage: legs and waist must not stiffen,
#   twitch, or fight the onboard controller (the slot-policy check --
#   their slots are never written). Any waist/leg reaction: e-stop,
#   record, stop the campaign (spec: one-line exception with a reason).
ros2 run replay run_ctl start
ros2 run replay run_ctl park && ros2 run replay run_ctl release
#   release: weight 1 -> 0 over 2 s at the engage snapshot; the arm must
#   hand back smoothly, no droop-snap.
```

Per joint record: index/sign/zero correct, measured tracks commanded
(`make_artifacts --run-dir ~/wuji_runs/<run>` gives per-joint RMSE), no
abnormal current (`effort` in `/left_arm/joint_states`) or temperature
(`/g1/status`). Collect the per-joint rows into
`~/wuji_runs/stage_b_report.json` (free-form; it is the stage's gate-out
evidence). If tracking is sloppy or oscillatory, try the vendor gains
profile: `gains: profile: vendor` in g1_robot.yaml, restart T1, repeat
one joint.

**Gate out:** every hand joint and every arm joint confirmed; slot policy
confirmed on real firmware (waist held by the onboard controller with its
slots unwritten); `stage_b_report.json` written.

<details>
<summary>Debug: arm sequence stalls before ARMED</summary>

Read `/run/events` — the supervisor logs every refused call verbatim.
Common causes, in order:

- `engage gate: need 50 consecutive fresh+still ticks`: the arm is still
  settling or lowstate dq is noisy. Wait 2 s and re-arm; if it never
  fills, raise `-p engage_dq_max:=0.1` on T1 (default 0.05 rad/s).
- `approach requires engage complete`: engage ramp still running (2 s);
  the supervisor retries via its sequence — if it timed out instead,
  re-arm after a `clear-fault`.
- Approach runs but never completes: `approach_done` needs max measured
  error < 0.05 rad AND |dq| < 0.05. Static gravity sag at kp 140 can
  exceed 0.05 rad on shoulder pitch. Raise on T1:
  `-p approach_done_err:=0.1` (and note it in the run manifest notes).
  The threshold is a completion check, not a safety clamp.
- `no fresh in-scope target stream`: publisher not in first_frame;
  check `/replay/status` (state must be `first_frame` after arm starts).
- Barrier timeout -> FAULT: the reason names the stalled stage. Fix,
  `run_ctl clear-fault`, reload from scratch (no resume, by design).
</details>

<details>
<summary>Debug: faults during a run</summary>

- Divergence fault (`|measured - command| above 0.35 rad`): the arm is
  not following. At Stage B amplitudes this is a sign/index problem or a
  wildly wrong gain, not a tuning knob — stop and inspect. The recovery
  path after ANY fault: `run_ctl park`, `run_ctl release`,
  `run_ctl clear-fault`, then reload from the start.
- `write_fault` non-null in `/g1/status`: the write thread hit a bad tick
  (e.g. NaN lowstate) and is holding the previous frame. Treat as a
  comms/firmware anomaly; e-stop, record, investigate.
- Layer-3 liveness fault the moment ARMED is reached, naming a device
  you did not scope: check the load's `--arms`/`--hands` matched what is
  actually running.
- Hand FSM faulted and holds: read `fault.reason` in
  `/{side}_hand/status`; error codes and over-temperature are hardware
  facts, not software retries.
</details>

---

## Stage C: separate full clips (TUITION 7C)

First real clips, one device group at a time, in the pinned order:
hands-left, hands-right, arms-left, arms-right, arms-both. Step 6
(combined) is blocked on the adapter. Every run starts at 0.25x.

```bash
# T3 -- condition everything once (during Stage A soak is a good time)
for s in RobotSTAR_demos/samples/*/; do for m in GT Ours; do
  ros2 run replay condition_clip --method-dir "$s$m" --out-dir ~/wuji_clips || true
done; done
ros2 run replay choose_first_clip --clips-dir ~/wuji_clips \
    --bundle RobotSTAR_demos --json ~/wuji_clips/verdict_table.json
```

Pick the top eligible clip (sample 01 is refused by the load gate as a
first clip; the chooser already excludes it). Then per scope, GT first:

```bash
# hands-left example; then hands-right; then arms-left; arms-right; arms-both
ros2 run replay run_ctl load \
    ~/wuji_clips/<sample>_GT/conditioned_clip_v1.npz \
    --arms '' --hands left --speed 0.25 --operator alex
ros2 run replay run_ctl arm && ros2 run replay run_ctl start
# ... clip ends; devices end_hold automatically ...
ros2 run replay run_ctl park && ros2 run replay run_ctl release
ros2 run replay make_artifacts --run-dir ~/wuji_runs/<run_dir>
```

**Gate out per scope:** `tracking_summary.json` says `pass: true` (zero
faults, arm RMSE <= 0.15 rad, max <= 0.35 rad, hand RMSE <= 0.15 rad —
proposed criteria, note deviations rather than silently accepting), and
your eyes agreed with the motion. Compare against the sample's reference
video (TUITION 3.3 checklist) and note it.

---

## Stage D: GT before Ours (TUITION 7D)

Enforced by the load gate: loading an `Ours` clip requires a PASSING GT
`tracking_summary.json` for the same sample at the same scope and a
speed at least as high (the gate scans `~/wuji_runs/`). Just run GT
first; the gate refuses anything else. `--override-gt-gate` exists for a
deliberate, recorded exception only.

## Stage E: the speed ladder (TUITION 7E)

Per clip that passed at 0.25x: rerun at 0.5x, then 1.0x, each rung capped
by the artifact's `max_allowed_speed_scale` (the load gate enforces it;
`verdict_table.json` lists it per clip). Time redistribution only — the
baked `k` already slowed sustained motion to the deploy rows; spiky clips
have `fail` verdicts and never load. The passing list at 0.5x and 1.0x is
TUITION section 11 item 11 — it may legitimately be short or empty at the
0.5 rad/s provisional deploy limit.

If Stage A recorded OFFICIAL deployment limits: edit the `deploy:` rows in
[`g1_deploy_limits.yaml`](../../src/output_devices/g1_world_output/config/g1_deploy_limits.yaml)
(the ceilings never change), re-run the conditioning sweep (deterministic,
new `k` and allowed scales), and restart T1/T2 so the runtime chains
reload. Record the new limits in the manifest.

## Stage F: contact clips — blocked

Blocked on the mount adapter (combined runs) and a fresh audit for sample
01. Not part of this campaign day.

---

## End of day: the return package

For TUITION section 11: `~/wuji_runs/` run directories (bags, manifests,
events, tracking summaries, `command_vs_actual.npz`),
`hardware_manifest.json`, `joint_mapping_{left,right}.json`,
`~/wuji_clips/` (item 6, the q20 trajectories, by construction),
`verdict_table.json`, `stage_b_report.json`, and the item-11 speed list.
All of it lives on host bind mounts and survives the containers.
