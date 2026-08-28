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

1. **CI smoke.** `.github/workflows/ci.yml` green (205 ROS-free tests on
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

3. **Full state-machine traversal.** `replay_sim.launch.py` + the g1
   container terminal; then load -> arm -> start -> (clip end) -> park ->
   release through `run_ctl`, watching `/run/status`. Every transition
   must be operator-initiated; the only automatic exits from RUNNING are
   clip end and fault.

4. **Hand q20 branch drives MuJoCo.** During step 3, confirm the viewer's
   hands move (the q20 branch publishes `/left,right_hand/joint_commands`;
   the visualizer maps them positionally via HAND_CODES).

5. **Piecewise-linear assert** (the ZOH fix, spec_1 known defect 1):

   ```bash
   python3 src/output_devices/g1_world_output/scripts/check_piecewise_linear.py \
       record --topic /left_arm/joint_commands --duration 20 --out /tmp/l.npz
   python3 src/output_devices/g1_world_output/scripts/check_piecewise_linear.py \
       analyze /tmp/l.npz --vel-limit 0.5
   ```

6. **Fault drills.**
   - Kill the publisher mid-run (`kill` its pid): devices must hold via
     target staleness (arm chain holds last command; hand FSM holds);
     supervisor faults on replay liveness; `/run/fault` latches; further
     loads refused until `run_ctl clear-fault`.
   - `run_ctl stop` mid-run: same latch through the operator path.
   - Inject a stale stream (SIGSTOP the publisher for 1 s, SIGCONT):
     devices hold through the gap, no jump on resume (the buffer ramps
     from the held command).
   - Frame jump: condition a synthetic clip with a spike
     (`test/conftest.py::make_bundle_sample(spike=...)` shape), load with
     `--force-sim` on the publisher profile, and confirm the arm-side rate
     limit clips the step while the DDS-thread clip stays under the
     ceiling rows.
   - Single-joint artifact through the normal path:
     `condition_clip --single-joint arm:left_elbow --amplitude 0.2` then
     load/arm/start -- validates the Stage B path Stage C depends on.

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
| fault drills (publisher kill, stop, stall, spike, single-joint) | | |
| section 3.3 comparison recorded | | |
