# Spec 1 Stage 0 checklist (all-sim gate)

The bring-up table's Stage 0 row, expanded into runnable steps. Exit gate:
CI smoke green; artifacts generate and validate. Everything runs with no
hardware: the g1 node under `dry_run:=true`, hands with
`wujihand_ik_q20_sim.yaml` (no driver).

What sim CANNOT exercise (unit-test-only coverage, by design): divergence
faults, engage-gate rejection, lowstate-loss reset, and every hand feedback
watchdog. Those live in `tests/test_device_fsm.py`,
`tests/test_hand_fsm.py`, and `tests/test_replay_safety.py`; dry_run
synthesizes measured := command, so nothing physical can disagree.

## Steps

1. **CI smoke.** `.github/workflows/ci.yml` green (223 ROS-free tests on
   both container numpy majors), plus in-container
   `colcon test --packages-select replay g1_world_output wujihand_output
   controller`.

2. **Conditioning sweep, all 30 clips.** In the teleop container:

   ```bash
   for s in RobotSTAR_demos/samples/*/; do
     for m in GT Ours; do
       ros2 run replay condition_clip --method-dir "$s$m" \
           --out-dir ~/wuji_clips || true
     done
   done
   ros2 run replay choose_first_clip --clips-dir ~/wuji_clips \
       --bundle RobotSTAR_demos --json ~/wuji_clips/verdict_table.json
   ```

   Expected shape (measured facts): every clip needs k > 1 at the 0.5 rad/s
   screening deploy row; the spike clips (01, 02, 03, 14, 15 at minimum)
   come back `fail` with the branch-flip reason; nothing passes at
   speed_scale 1.0 without k. Record the verdict table.

3. **Full state-machine traversal.** Every transition is an operator
   service call; the only automatic exits from RUNNING are clip end and
   fault. Three terminals, then load -> arm -> start -> (clip end) ->
   park -> release, watching `/run/status` throughout.

   ```bash
   # T1 (host, from docker/) -- arm node, no DDS
   docker compose run --rm --name g1-world-output g1_world_output \
       ros2 launch g1_world_output g1_world_output.launch.py \
       dry_run:=true mode:=joint_replay arm_type:=G1_29 control_rate:=250.0

   # T2 (teleop container) -- publisher, both q20 hand controllers,
   # supervisor, MuJoCo viewer
   ros2 launch wuji_teleop_bringup replay_sim.launch.py

   # T3 (teleop container) -- the status pane, left running all session
   ros2 run replay run_ctl status -w
   ```

   <details>
   <summary>T4: the traversal itself, transition by transition</summary>

   Run these in a fourth pane, checking the stated `/run/status` field in
   T3 after each one before continuing. `<clip>` is any artifact whose
   verdict is `pass` from step 2 (or the synthetic single-joint one from
   drill 6e).

   ```bash
   # --- load: gates run here (verdict, allowed scale, scope, 7F, 7D) ---
   ros2 run replay run_ctl load \
       ~/wuji_clips/<sample>_GT/conditioned_clip_v1.npz \
       --speed 0.25 --operator <name>
   #   run_state: idle, and the run directory is created + bag started.
   #   Check: ls ~/wuji_runs/  -> a new <UTC>_<sample>_GT_0.25_full/
   #   /replay/status state: loaded
   ```

   ```bash
   # --- arm: publish_first -> engage -> approach -> frame-0 barrier ---
   ros2 run replay run_ctl arm
   #   Watch T3 progress through arm_seq: publish_first, engage,
   #   approach, barrier, then run_state: armed (a few seconds).
   #   /replay/status state: first_frame (frame 0 repeating, stamps
   #   advancing). Devices: g1 fsm_state engage -> approach with
   #   approach_done true; hands the same.
   #   In dry_run the engage weight ramp is simulated: /g1/status weight
   #   goes 0 -> 1 over 2 s.
   ```

   ```bash
   # --- start: the only command that advances the clip ---
   ros2 run replay run_ctl start
   #   run_state: running. Devices go fsm_state: track.
   #   The MuJoCo window now shows arms AND hands moving (step 4).
   #   /replay/status tick counts up toward total.
   ```

   ```bash
   # --- clip end is automatic: RUNNING -> IDLE, devices end_hold ---
   #   Nothing to type. In T3, watch for:
   #     /replay/status clip_done: true, state: finished
   #     run_state: idle
   #     devices fsm_state: end_hold  (holding the last target, not zero)
   #   /run/events logs "clip end: publisher holding last frame".
   ```

   ```bash
   # --- park: arm re-approaches its engage snapshot; hands slew neutral ---
   ros2 run replay run_ctl park
   #   Devices fsm_state: approach (approach_target snapshot / neutral).
   #   Wait for approach_done true on every in-scope device before
   #   releasing -- release is refused until the arm reaches the snapshot.
   ```

   ```bash
   # --- release: weight 1 -> 0 over 2 s, then the run closes ---
   ros2 run replay run_ctl release
   #   /g1/status weight ramps to 0, fsm_state returns to ready.
   #   Only THEN does the supervisor close the bag and the run dir
   #   (/run/events: "release complete; run directory closed").
   #   Confirm: ls ~/wuji_runs/<run>/  -> bag/ run_manifest.json events.jsonl
   ```

   ```bash
   # --- post-run evidence ---
   ros2 run replay make_artifacts --run-dir ~/wuji_runs/<run>/
   #   writes command_vs_actual.npz, tracking_summary.json, fault_log.jsonl
   ```

   **Refusal checks (prove the gates, no hardware risk).** Each of these
   must be REFUSED; the message says why:

   ```bash
   ros2 run replay run_ctl start        # before arm: "start requires ARMED"
   ros2 run replay run_ctl load <fail-verdict clip>   # "artifact verdict is 'fail'"
   ros2 run replay run_ctl load <clip> --speed 1.0    # if the clip's
                                        # max_allowed_speed_scale is lower
   ros2 run replay run_ctl load <Ours clip>           # "GT-before-Ours" with
                                        # no passing GT run in ~/wuji_runs
   ros2 run replay run_ctl load ~/wuji_clips/01_*/conditioned_clip_v1.npz
                                        # "banned as the first clip (7F)"
   ```
   </details>

4. **Hand q20 branch drives MuJoCo.** During step 3's `start`, confirm the
   viewer's hands move, not just the arms (the q20 branch publishes
   `/left,right_hand/joint_commands`; the visualizer maps them
   positionally via HAND_CODES).

   ```bash
   ros2 topic hz /left_hand/joint_commands    # ~200 Hz while tracking
   ros2 topic echo /left_hand/status --once   # fsm_state: track
   ```

5. **Piecewise-linear assert** (the ZOH fix, spec_1 known defect 1):

   ```bash
   python3 src/output_devices/g1_world_output/scripts/check_piecewise_linear.py \
       record --topic /left_arm/joint_commands --duration 20 --out /tmp/l.npz
   python3 src/output_devices/g1_world_output/scripts/check_piecewise_linear.py \
       analyze /tmp/l.npz --vel-limit 0.5
   ```

6. **Fault drills.** Each drill is: get to RUNNING (step 3 through
   `start`), break something, confirm the response, then recover with the
   standard sequence. There is no resume by design (section 9): recovery
   always means park, release, clear-fault, reload from the start.

   <details>
   <summary>The five drills, with commands and expected responses</summary>

   **Recovery sequence, used after every drill below:**

   ```bash
   ros2 run replay run_ctl park          # devices approach snapshot/neutral
   ros2 run replay run_ctl release       # weight down; run dir closes
   ros2 run replay run_ctl clear-fault   # unlatches; next step must be a
                                         # fresh load (no resume)
   ```

   **6a. Kill the publisher mid-run.** Proves Layer 1 holds with the pacer
   dead, and that Layer 3 notices.

   ```bash
   # while RUNNING:
   pkill -f 'replay.replay_publisher'          # or: kill <pid>
   ```
   Expect: both device `fsm_state` stay `track` but commands FREEZE at the
   last value (target staleness -> hold, never zero — watch the MuJoCo
   model stop dead, not collapse); within ~1 s the supervisor faults with
   `liveness: replay publisher silent`; `/run/fault` latches; `run_state:
   fault`. A `run_ctl load` now is refused ("FAULT latched"). Recover as
   above, then relaunch T2's publisher (restart the launch file).

   **6b. Operator stop.** The same latch through the intended path.

   ```bash
   ros2 run replay run_ctl stop
   ```
   Expect: `/run/events` logs `FAULT_HOLD: operator stop (run_ctl)`;
   publisher freezes its frame (`/replay/status state: fault`); every
   in-scope device freezes its command and (arm) its weight.

   **6c. Stale stream, then resume.** Proves the interpolator ramps from
   the held command instead of jumping.

   ```bash
   PID=$(pgrep -f 'replay.replay_publisher')
   kill -STOP $PID && sleep 1 && kill -CONT $PID
   ```
   Expect: commands hold flat during the gap; on resume the command ramps
   toward the newest target rather than stepping to it (the 1 s gap
   exceeds the 0.25 s target-staleness bound, so the supervisor may also
   fault on liveness — either outcome is correct; note which happened).

   **6d. Frame jump / spike clip.** Proves the rate limits clip a
   discontinuity that got past the offline gate.

   ```bash
   # Load a FAIL-verdict spike clip deliberately, sim-only bypass:
   #   T2 must be running with force_sim:=true for this drill (arms BOTH
   #   bypasses: publisher --force-sim and the supervisor's load gates):
   #   ros2 launch wuji_teleop_bringup replay_sim.launch.py force_sim:=true
   ros2 run replay run_ctl load \
       ~/wuji_clips/02_<sample>_GT/conditioned_clip_v1.npz --speed 0.25
   ros2 run replay run_ctl arm && ros2 run replay run_ctl start
   ```
   Expect: the clip plays without the command stream ever stepping; record
   `/left_arm/joint_commands` through the step-5 analyzer and confirm
   `piecewise_linear: true` and `max_tick_step_rad` at or under the deploy
   budget. The spike is absorbed by the safety-chain rate limit, and (on
   hardware) again by the always-on 250 Hz DDS clip.
   **Return T2 to the normal profile afterwards** — `--force-sim` must
   never be used with hardware attached.

   **6e. Single-joint artifact through the normal path.** Validates the
   Stage B path Stage C depends on.

   ```bash
   ros2 run replay condition_clip --single-joint arm:left_elbow \
       --amplitude 0.2 --out-dir ~/wuji_clips/stage_b
   ros2 run replay run_ctl load \
       ~/wuji_clips/stage_b/single_joint_arm_left_elbow/conditioned_clip_v1.npz \
       --hands '' --speed 1.0
   ros2 run replay run_ctl arm && ros2 run replay run_ctl start
   ```
   Expect: verdict `pass`, only `left_elbow` moves in MuJoCo, and the run
   completes to `clip_done` with no fault. This is the exact command shape
   Stage B uses on hardware, so a failure here is a Stage B blocker.
   </details>

7. **Section 3.3 visual comparison.** Record the MuJoCo replay (screen
   capture) of at least one conditioned clip beside the bundle's
   `*_Physical.mp4` reference video; check the 7 items (left/right, elbow
   half-space, palm orientation, inter-hand distance, finger open/close,
   body contact, start/end consistency). Operator judgment, recorded in
   the Stage 0 notes.

## Sign-off

| item | result | by |
|---|---|---|
| CI smoke green | | |
| conditioning sweep verdict table recorded | | |
| traversal load->release clean | | |
| hands move in MuJoCo via q20 | | |
| piecewise-linear assert passes | | |
| fault drill 6a publisher kill (devices hold, Layer 3 faults) | | |
| fault drill 6b operator stop | | |
| fault drill 6c stale stream + resume (no jump) | | |
| fault drill 6d spike clip clipped, stream stays piecewise-linear | | |
| fault drill 6e single-joint artifact through load/arm/start | | |
| gate refusals all refused (fail verdict, overspeed, GT-before-Ours, sample 01) | | |
| section 3.3 comparison recorded | | |
