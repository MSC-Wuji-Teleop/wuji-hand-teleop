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
ros2 run replay run_ctl arm && ros2 run replay run_ctl start
```

The bundle's precomputed hand joint columns are **never** used: hands are
regenerated offline from the 21-point keypoints via the production
retargeter (TUITION 3.1). `keypoints21` is teleop-only; this pipeline does
not publish it. The old live-keypoint sim path (Flow 3) remains available
via `wujihand_ik_replay.yaml` until the artifact flow fully replaces it.

Artifacts land in `~/wuji_clips/`, run directories in `~/wuji_runs/`. Both
are host bind mounts (docker-compose), so they survive container recreation.
