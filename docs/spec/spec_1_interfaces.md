# Spec 1 runtime interfaces

Pinned contracts between the replay pipeline's processes. spec_1.md says
what each piece must do; this file says exactly what it exposes, so the
supervisor, publisher, and device nodes can be built and tested against one
table. Everything below uses stock interfaces only: std_srvs/Trigger,
std_msgs/String (JSON payload), sensor_msgs/JointState, sensor_msgs/Imu
(spec_1 component 6: no new interface packages).

## Conventions

- **Trigger semantics.** Every transition service validates its
  preconditions and returns immediately: `success` = transition accepted
  and begun, never "completed". Progress happens in the node's control
  loop; completion is observed on the status topic. No service handler
  blocks on motion (single-threaded executors would deadlock the ramp the
  handler waits for).
- **JSON responses.** `Trigger.message` carries a JSON object; on
  rejection it includes `{"error": <reason>}`.
- **Load requests.** A node that needs a payload with its trigger exposes a
  string parameter `load_request` holding a JSON object. The caller sets
  the parameter, then calls `~/load`. One set = one request.
- **Stamps.** The publisher stamps tick i with `t0 + i * dt_play`,
  `dt_play = k / (target_fps * speed_scale)`. `publish_first` repeats
  frame 0 with **advancing** stamps `t0 + j * dt_play`; `start` continues
  the same series without discontinuity. Device interpolators: if there is
  no previous stamp or the stamp delta is <= 0, hold the newest target;
  ramp periods are clamped to [1 ms, 4 * dt_play]; never divide by a stamp
  delta unchecked.
- **Target-stream QoS.** All four `joint_targets` topics are RELIABLE,
  KEEP_LAST, depth 10, on the publisher and on both subscribers. The hand
  branch must not inherit `get_default_qos()` (BEST_EFFORT); the driver-
  facing `joint_commands` topics keep their existing SensorData QoS.
- **Status topics.** std_msgs/String, JSON object, published at >= 5 Hz
  and on every state change.

## Conditioned clip artifact (condition_clip output)

`conditioned_clip_v1.npz`:

| key | shape / type | meaning |
|---|---|---|
| `arm_q` | [T, 14] float64 | arm targets on the play grid |
| `arm_joint_names` | [14] str | unsuffixed names, robot_arm.py convention |
| `left_hand_q20` | [T, 20] float64 | Hand 2 angles, driver flat order |
| `right_hand_q20` | [T, 20] float64 | same |
| `target_fps` | scalar float | bundle grid fps (50) |
| `k` | scalar int | baked integer time redistribution (>= bundle k) |

`conditioned_clip_v1.json` (same basename): `schema_version`, `sample`,
`method` (GT/Ours), `source_dir`, `input_sha256` (per input file, checked
against MANIFEST.sha256), `retargeter` (submodule commit, config hash, hand
model id) or `null` for arm-only artifacts, `limits` (files used + their
provenance fields), `audit` (per-joint FD velocity/acceleration stats on
the play grid, spike list, ceiling violations, hand stats),
`max_allowed_speed_scale` (largest speed_scale that keeps FD peaks inside
the deploy rows after baked k; consumers never re-derive it), `verdict`
(`pass` | `fail`), `verdict_reasons`, `first_frame` / `last_frame` (arm +
hand poses, for approach and park planning), `tool_version`. No wall-clock
fields anywhere: conditioning is deterministic, same inputs = same output
hashes (spec_1 component 1); creation time lives in the filesystem mtime.

Single-joint artifacts (Stage B) use the identical schema with
`sample: "single_joint"`, method `null`, and a generated ramp clip.

## replay_publisher

| surface | name | notes |
|---|---|---|
| param | `load_request` | JSON: `{"clip": <path>, "speed_scale": float, "arms": ["left","right"], "hands": ["left","right"]}`; empty side lists scope the run (spec 7C) |
| service | `~/load` (Trigger) | consumes `load_request`; refuses verdict `fail` unless the node ran with `--force-sim`; refuses while RUNNING or FAULT |
| service | `~/publish_first` (Trigger) | repeat frame 0, advancing stamps, do not advance the frame index |
| service | `~/start` (Trigger) | begin advancing; refused unless publish_first is active |
| service | `~/fault` (Trigger) | freeze the tick (keep repeating the frozen frame); only a fresh `load` after supervisor clear-fault unfreezes |
| pub | `/left_arm/joint_targets`, `/right_arm/joint_targets` | JointState, named q7/side, stamped |
| pub | `/left_hand/joint_targets`, `/right_hand/joint_targets` | JointState, named q20 (side-prefixed URDF names), stamped |
| pub | `/replay/status` | `{"state": "unloaded"\|"loaded"\|"first_frame"\|"running"\|"finished"\|"fault", "clip": path\|null, "sample", "method", "speed_scale", "tick": int, "total": int, "clip_done": bool, "scope": {...}}` |

No pause, no resume, no loop. keypoints21 is not published by this node.

## g1_world_output (joint_replay mode)

| surface | name | notes |
|---|---|---|
| service | `~/engage` (Trigger) | ready->engage; requires N fresh lowstate frames, measured \|dq\| below threshold; snapshots the release target |
| service | `~/approach` (Trigger) | engage/track/end_hold->approach toward the current stream target (frame 0); requires a fresh in-scope target |
| service | `~/track` (Trigger) | approach->track; requires approach_done |
| service | `~/end_hold` (Trigger) | track->end_hold (hold last target) |
| service | `~/park` (Trigger) | alias: re-enters approach with target = engage snapshot (no separate state) |
| service | `~/release` (Trigger) | approach(snapshot)->release; weight 1->0 over >= 2 s at the snapshot |
| service | `~/fault` (Trigger) | force FAULT_HOLD (supervisor Layer 3 fan-out) |
| service | `~/clear_fault` (Trigger) | operator-only; devices return to ready |
| pub | `/g1/status` | `{"fsm_state", "mode_machine", "tick", "lowstate_age_s", "weight", "approach_done": bool, "max_target_error_rad", "fault": null\|{...}, "max_motor_temp_c", "voltage_v", "read_only": bool}` |
| pub | `/g1/imu` | sensor_msgs/Imu from lowstate |
| pub | `/left,right_arm/joint_states` | velocity and effort now filled |

Holds are constant snapshots, never live measured (a measured-chasing hold
at weight 1 is kd-only and droops under gravity). `--read-only` publishes
status/imu/joint_states and refuses every motion service.

## wujihand_controller (input_source = q20_topic)

| surface | name | notes |
|---|---|---|
| service | `~/approach` (Trigger) | hold->approach toward current stream target; requires a fresh target |
| service | `~/track` (Trigger) | approach->track; requires approach_done |
| service | `~/end_hold` (Trigger) | track->end_hold |
| service | `~/park` (Trigger) | alias: re-enters approach with target = neutral pose (spec: hands slew to neutral at clip end under approach limits) |
| service | `~/release` (Trigger) | acknowledgment only: succeeds on a parked (holding) hand, refused otherwise; no weight, nothing ramps |
| service | `~/fault` (Trigger) | force FAULT_HOLD |
| service | `~/clear_fault` (Trigger) | operator-only |
| pub | `/{hand}/status` | `{"fsm_state", "target_age_s", "state_age_s", "diagnostics_age_s", "approach_done": bool, "max_target_error_rad", "fault": null\|{...}, "handedness", "joints_online": int, "max_joint_temp_c"}` |

States: hold, approach, track, end_hold (+ latched fault-hold overlay).
Commands are unnamed full-20 position-only arrays every cycle (existing
hand_interface path); a publish with != 20 elements is a bug, asserted.

## Supervisor

| surface | name | notes |
|---|---|---|
| param | `load_request` | forwarded to the publisher after the gates pass |
| param | `force_sim` | sim-only drills: load-gate problems are logged (warn event) and bypassed instead of refused; mirrors the publisher's `--force-sim`, armed together by `replay_sim.launch.py force_sim:=true` |
| service | `~/load`, `~/arm`, `~/start` (Trigger) | run FSM; `arm` sequence: publish_first -> per-device engage -> approach -> frame-0 barrier |
| service | `~/stop` (Trigger) | operator stop = the fault path (spec: one stop path) |
| service | `~/park` (Trigger) | post-run, fans out to devices |
| service | `~/release` (Trigger) | refused until every in-scope device is parked (arm at approach(snapshot) done or ready, hands holding); then fans out to the arm and hands |
| service | `~/clear_fault` (Trigger) | unlatches FAULT, allows the next load |
| pub | `/run/status` | run state + per-device state fields (not mirrored FSMs) |
| pub | `/run/events` | one JSON event per message, mirror of events.jsonl |
| pub | `/run/fault` | latched fault description |

Layer 3 detectors (supervisor-only signals): cross-device liveness,
alignment-barrier timeout, hand joint offline / error codes, effort
saturation > 1 s, temperature warn and trip, mode_machine change. Response
is always FAULT_HOLD: latch, then call `~/fault` on the publisher and every
in-scope device. All supervisor service calls to other nodes are
`call_async` + status polling; no synchronous call inside a callback.
