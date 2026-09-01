# replay

The conditioned-clip replay pipeline (spec:
[docs/spec/spec_1.md](../../../docs/spec/spec_1.md), runtime contracts:
[docs/spec/spec_1_interfaces.md](../../../docs/spec/spec_1_interfaces.md)).
One offline stage conditions a bundle sample into an audited artifact; one
gated publisher paces both devices from one timer; one supervisor owns the
run state machine, the load gates, the Layer-3 monitors, and the run
directory.

## Pieces

| Entry point | What it does |
|---|---|
| `condition_clip` | offline: bundle sample -> `conditioned_clip_v1.{npz,json}`. Extracts the 14 arm joints by name, retargets hands from keypoints (Retargeter reset per clip, PCHIP-retimed onto the arm grid), audits FD velocities/accelerations against `g1_deploy_limits.yaml` + `hand_limits.yaml`, bakes integer time redistribution `k`, computes `max_allowed_speed_scale`, writes a `pass`/`fail` verdict. Deterministic. `--single-joint arm:left_elbow` emits Stage B artifacts through the same schema and audit |
| `replay_publisher` | gated pacer. Publishes nothing on spin; `load` (JSON via the `load_request` parameter) / `publish_first` / `start` / `fault` Triggers; refuses `fail` verdicts (`--force-sim` bypasses, sim only). Publishes named, stamped `JointState` on `/left,right_arm/joint_targets` (q7/side) and `/left,right_hand/joint_targets` (q20) from one timer; stamps are `t0 + j * dt_play`, `dt_play = k / (target_fps * speed_scale)`. No pause, no resume, no loop |
| `supervisor` | run FSM (IDLE/ARMED/RUNNING/FAULT latched), load gates (verdict, allowed scale, sample-01 first-clip ban, GT-before-Ours), the arm sequence (publish_first -> engage -> approach -> frame-0 barrier), the six Layer-3 detectors with FAULT_HOLD fan-out, rosbag2 mcap recording, run directories under `~/wuji_runs/` |
| `run_ctl` | operator CLI: `load / arm / start / stop / park / release / clear-fault / status` |
| `choose_first_clip` | ranks conditioned clips against the TUITION 7F first-clip bar (contact-pair classification, amplitude); excludes sample 01 |
| `make_artifacts` | post-run: `command_vs_actual.npz`, `tracking_summary.json` (proposed, unratified pass criteria), `fault_log.jsonl` from a run directory |

Core logic is ROS-free and unit-tested (`test/`): `clip_artifact.py`
(schema), `conditioning.py` (audits, k, verdict), `pacer.py` (publisher
FSM), `run_gates.py` (gates, monitors, arm sequence), plus the pure halves
of the post-run tools. The rclpy files are thin adapters.

## The run state machine, operator view

One run is **single-use**: `load` through `release`, in order, then the
next run starts from a fresh `load`. Three commands only *begin* something
asynchronous — the machine tells you when it is done via `run_ctl status`
(keep `status -w` in a spare pane; every "wait" below is a field in it).

```
load     gates run; clip loaded; run dir + bag open        run_state: idle
arm      BEGINS ~5 s sequence: publish frame 0 -> engage
         (weight 0->1) -> approach frame 0 -> barrier      WAIT: run_state armed
start    plays the clip; refused unless ARMED              run_state: running
(clip end is automatic: publisher holds the last frame,
 devices hold pose)                                        replay: finished,
                                                           devices: end_hold
park     BEGINS the slew back to the engage snapshot       WAIT: g1 approach_done true
release  weight 1->0; closes the bag and the run dir.
         Refused until park's slew completes.              g1: ready
                                                           THE RUN IS OVER
```

Rules that follow:

- **Never chain `arm && start`**: `arm` returns before ARMED exists, so
  `start` is refused. Wait for `run_state: armed` between them.
- **Never re-`arm` a finished run** ("publish_first requires state loaded,
  is finished", then a barrier-timeout FAULT). After `release`, the next
  step is always a new `load`.
- **A refusal is the gate speaking, not an error**: `start` before ARMED
  and `release` before the park slew completes are the two you will see
  most; both mean "wait", not "retry harder".
- **FAULT is latched, from any powered state.** Recovery is always:
  `park` -> `release` -> `clear-fault` -> fresh `load`. There is no resume,
  by design.
- `make_artifacts` runs **after** `release` only — never while a run is
  live (a >1 s host CPU stall trips the Layer-3 liveness fault).

The design rationale behind these states is
[docs/spec/spec_1.md](../../../docs/spec/spec_1.md) ("Run state machine",
"Device state machine"); the per-stage hardware sequences that use them are
[docs/spec/spec_1_bringup.md](../../../docs/spec/spec_1_bringup.md).

## Sim quickstart (Stage 0)

```bash
# teleop container
ros2 launch wuji_teleop_bringup replay_sim.launch.py
# own container (second terminal, from docker/)
docker compose run --rm --name g1-world-output g1_world_output \
    ros2 launch g1_world_output g1_world_output.launch.py \
    dry_run:=true mode:=joint_replay arm_type:=G1_29 control_rate:=250.0
# third terminal (teleop container)
ros2 run replay condition_clip \
    --method-dir RobotSTAR_demos/samples/<sample>/GT --out-dir ~/wuji_clips
ros2 run replay run_ctl load ~/wuji_clips/<sample>_GT/conditioned_clip_v1.npz
ros2 run replay run_ctl arm      # wait for run_state: armed (status -w) ...
ros2 run replay run_ctl start    # ... THEN start (see the state machine above)
```

The bundle's precomputed hand joint columns are **never** used: hands are
regenerated offline from the 21-point keypoints via the production
retargeter (TUITION 3.1). `keypoints21` is teleop-only; this pipeline does
not publish it. The old live-keypoint sim path (Flow 3) remains available
via `wujihand_ik_replay.yaml` until the artifact flow fully replaces it.

Artifacts land in `~/wuji_clips/`, run directories in `~/wuji_runs/`. Both
are host bind mounts (docker-compose), so they survive container recreation.
