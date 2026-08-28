# Spec 1: hardware replay on the 29-DoF G1 (23-DoF secondary)

**Status:** proposed, 2026-08-27. Supersedes the 2026-08-26 question list.

Play a conditioned SignAR clip onto the real G1 arms over Unitree DDS and both
Wuji Hand 2 units over USB, on one clock, under an explicit safety envelope.
Not teleop; that is spec_2. `RobotSTAR_demos/TUITION.md` governs: its sections
3, 4, 5, 7, 8, 9, 10 are requirements on this design, cited below as §N.

The rig's robot is the **29-DoF G1** (waist yaw/roll/pitch, 7-DoF arms). The
**23-DoF** variant (waist yaw only, 5-DoF arms) stays a supported secondary
target. Decided 2026-08-27; hardware_spec.md carries the dated record.

## Measured facts

These replace the 2026-08-26 draft's facts. All were recomputed from the bundle
on 2026-08-27; the commands live with the audit tooling (section: conditioning).

1. **Waist does not move.** Columns 12/13/14 of `body_q` (waist yaw/roll/pitch,
   matched by name) are 0.0 in every frame of all 30 trajectories (15 samples,
   GT and Ours). The earlier claim that "arms plus waist yaw" moves is wrong.
   Legs are frozen at a stand pose. What moves is 14 arm joints.
2. **The npz velocity column understates the truth.** Finite differences of
   `body_q` at 50 fps give a peak of 161.4 rad/s on sample 02 GT where the
   `body_dq` column reports 96.2 (about 1.7x low). Use position differences for
   every rate decision; never trust `body_dq`.
3. **No clip is speed-safe, and the failures split in two.** Sustained speed
   (p99.5 of |dq|) runs 7.8 to 17.2 rad/s across the 30 trajectories: time
   redistribution fixes this. On top of that sit isolated single-frame spikes,
   up to 3.23 rad in one 20 ms frame (samples 01, 02, 03, 14, 15 at minimum),
   consistent with IK wrist-branch flips. §7 Stage E is explicit that slowing
   down does not fix branch flips: those clips stop at the gate until the
   trajectories are regenerated upstream, or run with wrists excluded.
4. Every sample ships `real_robot_ready: false` and `diagnostic_only: true`.
   Sample 01 is banned as a first clip (§7F: 142.6 N shoulder-torso contact in
   the bundle's own physical audit).
5. Hand columns `left_q`/`right_q` in `controller_reference_v7.npz` are legacy
   Wuji solutions and must never reach Hand 2 (§2.1). Hand 2 angles are
   regenerated from `hand2_input/*_human_targets_v5.npz` keypoints (§3.1).

## Current state (HEAD 5ce3ea8)

Commit 5ce3ea8 landed the G1_29 DDS control MVP: `G1ArmController(arm_type)`
drives either variant over `rt/arm_sdk` or `rt/lowcmd` (250 Hz write thread,
CRC, weight at slot 29, writer lockfile), and `arm_type` defaults to `G1_29`.
Pose IK stays G1_23-only; joint_replay needs no IK. The commit message itself
records that no safety envelope exists yet (§5/§8/§9/§10 pending). Two
leftovers from that commit are work items here:

- The comment above `G1_29_ARM_JOINT_NAMES` (`robot_arm.py`, ~164) still says
  the 29 DDS controller does not exist. Stale; fix with the next code change.
- The init loop still writes all 35 motor slots with `mode = 1` and hold gains
  (kp 300 / kd 5 on non-arm slots). On the 29 that actively commands real waist
  roll/pitch (13/14) and the legs through arm_sdk. Replaced by the slot policy
  below.

The rest of the sim replay path works as documented in usage.md Flow 3:
`replay_publisher` paces one clip from one timer; `g1_world_output` in
`joint_replay` consumes arm targets by name; two `wujihand_controller` nodes
retarget keypoints live; MuJoCo mirrors the four `joint_commands` topics.
Known defects this spec fixes:

- `_SideBuffer.interpolate()` keys on arrival time, so alpha always clamps to
  1 under the single-threaded executor: the output is a zero-order hold at the
  publish rate, exactly the 50 fps stepping §5 forbids. Its docstrings describe
  interpolation and a ramp that never occur; both get fixed with the code.
- The 250 Hz DDS velocity clip (20 rad/s, uniform scaling) is skipped entirely
  when `simulation_mode` is true, and there is no position clamp, staleness
  watchdog, or stop hook anywhere in the replay path.
- The live hand path never calls `Retargeter.reset()`, so filter and warm-start
  state leak across clips (§3.1 requires a reset per clip).
- `ChannelFactoryInitialize(domain)` is called without a network interface. The
  Unitree SDK builds its own CycloneDDS config and ignores `CYCLONEDDS_URI`, so
  on a multi-NIC host the env var cannot pin the robot link.
- `wujihand_driver` clamps nothing (its limit parameters are 0 to 1.57
  placeholders), has no command watchdog, and its **named** command path
  zero-fills any joint whose name does not match: a named JointState against
  the unpatched driver is more dangerous than the unnamed full-20 arrays the
  teleop path sends. `wujihandros2` is a pinned upstream submodule.

## Components

Three existing seams stay where they are: input (`replay`), controllers, and
device writers (`g1_world_output` for DDS, `wujihand_driver` for USB). Two
additions: an offline conditioning stage and one supervisor node. Rationale:
each device writer must be safe standalone (stale input means hold, never
zero), so safety never depends on the supervisor being alive; final clamps
belong at the last hop before each bus; replay stays a dumb pacer.

Sim compatibility is at the `joint_commands` level: `mujoco_visualizer`
subscribes the four `/{left,right}_{arm,hand}/joint_commands` topics, and both
the teleop path and this replay path produce them. The hardware replay pipeline
does **not** publish `keypoints21`; that topic is teleop-only. Flow 3's live
keypoint replay remains as a legacy sim path until the artifact flow below
replaces it.

### 1. `condition_clip` (new; offline; replay package)

Turns one bundle sample (GT or Ours) into a **conditioned clip artifact**. Runs
in the teleop container. Deterministic: same inputs, same output hashes.

Arms:
- Extract the 14 arm joints from `body_q` by name (waist is measured zero; see
  the waist paragraph below).
- Audit with position finite differences against a curated deploy-limits YAML
  (`g1_deploy_limits.yaml`, values copied from the hardware_spec.md joint
  table plus per-joint velocity/acceleration rows). The bundle's 0.5 rad/s
  figure is a screening parameter (§6 says so), not a deployment limit; the
  deployment numbers are parameters, revised on-site once §6 official limits
  are recorded.
- Sustained overspeed gets integer time redistribution `k` (same mechanism as
  the bundle's own retiming: waypoints untouched, §7E compliant), capped at a
  configured maximum.
- Spikes and branch flips are never smoothed. They go into the audit and drive
  the verdict: `go`, `conditional` (residual spikes under a hard threshold;
  manual kinematic preview required), or `no-go` (position violation or any
  flip/step above the threshold).

Hands:
- Per side: `Retargeter.from_yaml(...)`, then `reset()` (§3.1), then step all
  source-rate keypoint frames to q20. PCHIP-retime q20 onto the arm frame grid
  so both devices share one timeline. Audit against a hand-limits YAML (values
  from the hand URDF, e.g. finger1_joint1 [-0.045, 1.651], velocity 8.59).
- **Named deviation from §3.1.** TUITION and the handoff README specify the
  official SDK `RetargetSession` with `HandModel.WujiHand2`. Neither symbol
  exists in this repo. This design substitutes the vendored `wuji-retargeting`
  NLopt `Retargeter`: same 21-keypoint-to-20-joint contract, deterministic,
  offline, and already the mapping the teleop path trusts. Its hand description
  is a pinned submodule whose match to the delivered hardware revision (Beta 1
  vs Beta 2, §2.2) is unconfirmed. The substitution, the pin, and the config
  hash are recorded in `hand2_retarget_meta.json`. Action item: once the
  hardware revision is confirmed, evaluate the official SDK route on the rig
  host and either adopt it or re-affirm this substitution.

Artifact:
- `conditioned_clip_v1.npz`: arm q [T,14] + names, left/right q20 [T,20] on the
  arm grid, fps, k.
- `conditioned_clip_v1.json`: input sha256s checked against `MANIFEST.sha256`,
  retargeter submodule commit + config hash + hand model id, limits used and
  their sources, the full audit, the verdict, and first/last frame poses (for
  approach and park planning).
- This satisfies the §3.1 metadata list and is §11 item 6 (the q20
  trajectories to return) by construction.

### 2. `replay_publisher` (modified)

- Consumes artifacts (`--clip`); refuses `no-go` verdicts on hardware profiles
  (`--force-sim` exists for simulation only).
- Service-gated: `load` (String JSON request: artifact path, speed_scale, arm
  sides), `publish_first` (repeat frame 0, do not advance), `start`. Publishes
  nothing on spin. **No pause and no mid-clip resume**: §9 forbids continuing
  after an abnormal event, so a faulted run is parked, inspected, and rerun
  from the start.
- Publishes both devices' target streams: arm q14 on
  `/left,right_arm/joint_targets` and hand q20 on
  `/left,right_hand/joint_targets` (named, stamped), all from the one timer.
  keypoints21 is not part of this pipeline. One source pacing both devices
  makes arms and hands play the same clip by construction; no cross-check
  handshake is needed.
- Stamps tick i with `t0 + i * dt_play`, where `dt_play = k / (target_fps *
  speed_scale)`, and latches a transport message (String JSON: t0, dt_play,
  frame count, artifact sha256). `speed_scale` is time redistribution only,
  layered on the baked k.
- Per-side and per-device scope for §7C runs is part of the `load` request:
  the publisher publishes only the in-scope target topics.

### 3. `g1_world_output` (modified)

**LowCmd slot policy** (pinned; today's write-all-35 behavior is replaced):

- `rt/arm_sdk`: write slots 12 to 14 (waist, hold at the pose measured at
  engage, gains from the `waist_hold` row), 15 to 28 (arms), and 29 (weight).
  Slots 0 to 11 and 30 to 34 are never written; they stay at constructor
  defaults (kp = kd = 0, inert). Per-motor `mode` is not set (the vendor arm7
  example never sets it). `mode_machine` is copied from lowstate; `mode_pr` is
  0. Stage A/B verify waist-hold behavior under arm_sdk on this firmware.
- `rt/lowcmd`: not used by this design. It requires releasing the onboard
  controller and owning all 29 motors every cycle, a suspended-robot regime.
  Documented so nobody discovers the difference on hardware.

**Waist.** Measured zero in every clip, and the owner decision is to skip
commanding it. The waist stays hold-at-measured under the slot policy above.
`/waist/joint_targets` is reserved as the topic name for a future clip set that
moves the waist; nothing publishes or subscribes it today.

**ZOH fix.** `_SideBuffer` keeps its shape; `interpolate()` changes to
interpolate from the currently commanded value toward the newest sample over
one inter-arrival period (about 10 lines). Costs 20 ms of latency, irrelevant
open-loop. Stage 0 asserts the command stream is piecewise-linear. Docstrings
fixed in the same change.

**Safety chain** (new `replay_safety.py`, pure numpy, unit-testable without
ROS): position clamp (curated limits, margin parameter), per-joint rate limit
(uniform scaling, preserves path direction), staleness tracker (stale input
means hold last command), divergence monitor (|measured - last command| above
threshold for M consecutive ticks raises a fault). Runs in the joint_replay
loop. The 250 Hz DDS-thread clip becomes per-joint, parametric from the same
limits file, and **always on, simulation_mode included**. A latched
`/soft_estop` (std_msgs/Bool) freezes the node into hold.

**Device state machine** (§8):

```
ready (hold measured)
  -> engage   weight 0 -> 1 over >= 2 s, commanding the measured pose;
              that pose is snapshotted as the park pose
  -> approach measured -> frame 0 under approach limits;
              done when max error < 0.05 rad and measured dq ~ 0
  -> track    follow the stamped stream
  -> end_hold hold the last target; confirm dq ~ 0 for >= 1 s
  -> park     slew back to the snapshot under approach limits
  -> release  weight 1 -> 0 over >= 2 s while commanding the snapshot
```

Release happens at the snapshot because that is the pose the onboard
controller was itself maintaining at engage; it minimizes the takeover jump.
Fault in any powered state: hold the last safe command at the current weight;
never zero a command, never auto-release (§8, §9). **Fault mid-engage: freeze
the weight at its current ramp value** and keep commanding the measured pose;
de-escalation is operator-only.

**Read-only mode** (`--read-only`): subscribe lowstate, never start the write
thread, never touch the weight. Required for §7A; today the constructor
immediately position-holds and enables the weight, which makes Stage A
impossible.

**New publications** (for §10): `velocity` and `effort` filled in the arm
`joint_states`; `/g1/imu` (sensor_msgs/Imu); `/g1/status` (String JSON:
mode_machine, tick, lowstate age, max motor temperature, voltage).

**Network.** New `network_interface` parameter threaded to
`ChannelFactoryInitialize(domain, iface)`. YAML and README note that
`CYCLONEDDS_URI` governs only the ROS graph; this parameter is the only way to
pin the Unitree participant's NIC. The docker cyclonedds.xml peer config is
validated same-host only; a separate-host G1 is untested topology.

**Gains** move from hardcoded constants to a `gains:` table in `g1_robot.yaml`:
today's tiers (shoulders/elbows 140/3, wrists 50/2, hold 300/5, `waist_hold`
300/5) as default, the vendor example's uniform 60/1.5 as a selectable
fallback profile. Retuning becomes a config change, validated in Stage B.

### 4. `wujihand_controller` q20 branch (modified; controller package)

No new node. The hand side mirrors the arm side's answer to the same
problem: `g1_world_output` gained `joint_replay` as a mode, so
`wujihand_node` gains a third `input_source` value, `q20_topic`, beside
`wuji_glove` and `keypoints_topic`. The node keeps sole ownership of
`/{hand}/joint_commands` in every mode, so a replay/teleop double-writer is
structurally impossible and the launch-profile discipline a separate
streamer node would need disappears. This branch is the §5 implementation.
Per hand:

- Subscribes `/{hand_name}/joint_targets` (named q20, stamped by the
  publisher). The retargeter is not constructed in this branch
  (`enable_ik=False`), so replay never pays for or depends on NLopt.
- Fixed-rate loop on the existing `control_rate` parameter, raised to 200 Hz
  for replay (§5 range is 200 Hz to 1 kHz; final rate decided on-site);
  interpolates between the two most recent stamped targets with the same
  interpolate-toward-newest scheme as the arm ZOH fix; clamps to the
  hand-limits YAML; rate-limits; runs its own approach phase and mirrors the
  §8 device state machine.
- **Publishes unnamed full-20 arrays in driver order** every cycle. Named
  publishing is forbidden until the driver's named-path zero-fill is fixed
  upstream. A preflight assert compares the driver's `joint_states` names
  against the retargeter's URDF order and refuses to run on mismatch
  (recorded in `joint_mapping.json`).
- **Named deviation from §5.** The driver consumes position only; velocity
  and effort in commands are ignored end to end. The branch therefore
  publishes position-only rather than implying a contract the driver does
  not honor. The wujihandcpp realtime layer (1 kHz loop, 10 Hz low-pass) is
  the fixed-frequency execution §5 asks for; feedback is read via
  `joint_states` (1 kHz) and `hand_diagnostics` (10 Hz).
- Subscribes `joint_states` and `hand_diagnostics`: state staleness, any
  nonzero `error_codes[i]`, joint offline, or over-temperature means hold
  last command and raise a fault.

The `wuji_glove` and `keypoints_topic` branches are untouched.

### 5. `wujihand_driver`: no fork now

The zero-fill hazard cannot trigger on unnamed full-20 commands, and patching a
pinned submodule does not fit the bind-mounted workspace. Plan: file the
upstream PR (named-path zero-fill fix, clamp to real URDF limits, command-age
diagnostic, observe mode); rely on the preflight name assert and the q20
branch's clamps meanwhile; fork and repoint `.gitmodules` only if upstream declines.

### 6. Supervisor (new node; replay package)

One node, no new interface package: std_msgs/String JSON plus std_srvs/Trigger
cover the whole surface, which keeps interface builds out of the containers and
lets logs be read anywhere without message definitions. It owns the run state
machine, the load-time gates, one cross-device "all aligned" barrier, the
operator start, and a latched fault. Beside it, an operator CLI (`run_ctl`) and
four tools: `single_joint_test.py` (Stage B), `choose_first_clip.py` (scans the
30 audit JSONs for the §7F first-clip criteria), `collect_manifest.py`
(Stage A), `make_artifacts.py` (post-run, offline).

## Run state machine

Device state machines (above) carry §8. The run level is deliberately small:

```
IDLE -> LOADED -> CONNECTED -> ENGAGED -> ALIGNED -> RUNNING
                                                        |
                       clip end: END_HOLD -> PARKED -> RELEASED -> IDLE

any powered state -> FAULT_HOLD (latched)
```

- **LOADED** gates: artifact verdict is go/conditional, joint names match the
  rig variant, requested speed is within the per-clip allowed scale, sample 01
  refused as first clip, GT-before-Ours (loading Ours requires a passing GT
  `tracking_summary.json` at the same scope and scale, or an explicit
  override) (§7D, §7F).
- **CONNECTED** preflight (§7A): lowstate fresh, mode_machine recorded
  (MotionSwitcher CheckMode), 20+20 hand joints online and enabled, zero error
  codes, sides correct, hand name-order assert passed, comm soak clean, e-stop
  and watchdog physically exercised. Operator signs the checklist.
- **ENGAGED / ALIGNED**: devices run engage then approach; the barrier waits
  for every in-scope device to report frame-0 hold.
- **RUNNING**: operator start; the only automatic exits are clip end and fault.
- **FAULT_HOLD**: devices freeze per their state machines (weight frozen at its
  current value if mid-engage), the publisher stops advancing, everything keeps
  streaming its hold command. No resume: operator inspects, parks, releases,
  and reruns from the start. Further loads are refused until an explicit
  operator clear-fault (§9: do not continue after an abnormal event).

Every motion-initiating transition is an operator service call. Every stop is
automatic or operator. Solo operation: one launch terminal, one `run_ctl`
terminal, one hand on the physical e-stop.

## Safety envelope: where it lives

- **Layer 0, offline**: the conditioning gate. Cheapest place to stop a bad
  clip; produces the per-clip allowed speed.
- **Layer 1, last software hop** (authoritative): the arm safety chain plus the
  always-on per-joint DDS clip; the hand q20 branch's clamps, rate limit, and
  staleness hold. These act even if everything else dies: stale input means
  hold last command, never zero. `/soft_estop` lives here: operator-owned
  (`run_ctl` publishes it latched), consumed directly by the arm node and both
  hand nodes, no supervisor in the path.
- **Layer 2, device boundary**: driver behavior as-is plus the upstream PR;
  on the arm side, `rt/arm_sdk` with the onboard balance controller active
  (§2.3) and the weight semantics above.
- **Layer 3, supervisor**: the §9 detector table. Divergence (measured vs the
  post-limiter command, not the raw target), per-tick command jump, topic-age
  watchdogs (lowstate 200 ms, hand states 100 ms, diagnostics 500 ms), hand
  joint offline, error codes, effort saturation for over 1 s, temperature warn
  and trip thresholds, mode_machine change and IMU drift as a balance proxy.
  Thresholds are per-stage YAML profiles. Response is always FAULT_HOLD. Two
  named gaps: no direct balance-alarm flag is exposed on lowstate, and
  collision detection is a heuristic (effort spike plus deviation); the primary
  defense for both is §7 stage ordering and the operator's eyes.
- **Layer 4, physical**: the operator e-stop. It is **the designed force-relief
  path for collision and effort-saturation faults**, because the software
  response is a position hold by design (§8 forbids zeroing commands). A
  retarget-to-measured relief is a possible later addition, not in the first
  campaign.

## One clock

One publisher stamps both devices' target streams from one timeline
(t0 + i * dt_play) and latches the transport message (artifact sha256, for
the log). The arm node and both hand nodes interpolate their stamped streams
the same way. All processes run on one host, so one ROS clock. §10's unified
monotonic timestamps are the bag's receive times, with the lowstate tick
recorded in `/g1/status` for a DDS-side cross-check.

## Logging (§10) and return package (§11)

rosbag2 (mcap), spawned by the supervisor at load, stopped at release.
Allowlist: arm `joint_targets`/`joint_commands`/`joint_states`, hand
`joint_commands`/`joint_states`/`hand_diagnostics`, `/g1/imu`, `/g1/status`,
`/replay/status`, `/run/{status,events,fault}`, `/rosout`. keypoints21 is
teleop-only and not recorded here.

Run directory `~/wuji_runs/<UTC>_<sample>_<method>_<scale>_<scope>/`:

```
run_manifest.json      clip, method, scale, scope, git SHA, image digests,
                       operator, threshold profile
bag/                   mcap
events.jsonl           live-written by the supervisor (survives bag loss)
fault_log.jsonl        live-written                          (§10)
command_vs_actual.npz  post-run, make_artifacts.py           (§10)
tracking_summary.json  per-joint RMSE, max error, lag, pass/fail
real_run.mp4           operator camera; a start-marker event syncs it
```

Manifests: `hardware_manifest.json` (Stage A: driver read-only params, lowstate
identity, §6 checklist), `joint_mapping.json` (name tables plus the hand
name-order assert), `hand2_retarget_meta.json` (from artifact provenance),
`mount_transform.yaml` (blocked on the adapter). §11 items 1 to 11 map onto
these plus the run directories; item 11 (samples safe at 0.5x and 1.0x) may
legitimately come back empty, which is a data property, not a tooling gap.

## Bring-up, staged against §7

Until the mount adapter exists the hands cannot ride the arms, so stages A to C
run as two independent tracks in parallel: the **arm track** (G1 on the rig)
and the **hand track** (both hands benchtop on the rig host). Combined runs
(C step 6, combined D/E, all of F) are blocked on the adapter.

| Stage | Gate in | Work | Gate out |
|---|---|---|---|
| 0: all-sim (runnable now) | none | `replay_sim.launch.py` collapses the manual terminals; conditioning over all 30 clips (determinism check, verdict table); full state-machine traversal; fault drills (kill publisher mid-run, inject stale input and a frame jump); **the hand q20 branch drives the MuJoCo hand model** via its joint_commands; assert arm commands piecewise-linear; same stream against `arm_type:=G1_23` dry-run | CI smoke green; artifacts generate and validate |
| A: read-only (§7A) | rig host wired; physical e-stop present | arm `--read-only`; hands observed without motion commands; `collect_manifest.py`; 10 min comm soak; e-stop and watchdog physically tested | signed §7A checklist; `hardware_manifest.json`, `joint_mapping.json` |
| B: single joint, supported (§7B) | A passed; G1 suspended or supported | `single_joint_test.py` through the supervisor: all 20+20 hand joints, then each arm joint; verify index, sign, zero, feedback agreement, current, temperature; verify the arm_sdk slot policy (legs unaffected, waist held); gain tuning | `stage_b_report.json` per device |
| C: separate (§7C) | B passed on that track; first clip from `choose_first_clip.py` (01 excluded) | order: hands-left, hands-right, arms-left, arms-right, arms-both; step 6 combined is blocked on the adapter | scoped `tracking_summary.json` in bounds, zero faults, operator sign-off |
| D: GT before Ours (§7D) | C scoped passes | enforced by the load gate, per track before the adapter, combined after | GT passing before any Ours, per sample |
| E: slow then normal (§7E) | D passing at the current rung | ladder 0.25x, 0.5x, 1.0x; each rung capped by the per-clip allowed scale (FD peaks vs deploy limits); time redistribution only | §11 item 11 list. Arithmetic: sustained 7.8 to 17.2 rad/s against 3 to 6 rad/s proposed limits gives k of 2 to 6; spike clips stay wrist no-gos until regenerated |
| F: contact last (§7F) | E passed for non-contact clips; adapter installed; C6 done | contact clips identified from the physical audits; sample 01 only after a fresh audit clears the 142.6 N contact | full §11 return package |

## DoF variant policy

- **G1_29 is primary.** `arm_type: "G1_29"` is the default (5ce3ea8). The
  bundle replays as recorded: 14 arm joints, wrist pitch/yaw included (rms
  0.88 rad in the data, so this content matters).
- **G1_23 stays supported** with zero extra plumbing: every hop matches joints
  by name, and the arm node warns-and-ignores names its table lacks, so the
  same 7-joint-per-side stream drives a 5-joint table. Known limitation:
  wrist pitch/yaw content is dropped on the 23, so §3.3 palm-orientation
  checks will differ; record it in the run manifest when a 23 rig is used.
- Both composed models (`g1_29_wuji2*`, `g1_23_wuji2*`) live in
  `g1_wuji2_description` for the §4 revalidation work once the real mount
  transform is measured.

## Answers to the 2026-08-26 questions

M1: (1) 29 primary, 23 secondary, above. (2) Joint space; landed as
`joint_replay` plus the safety chain. (3) Baked integer k per clip plus a run
`speed_scale`, both time redistribution. (4) Do not trust `body_dq`; position
FD only (measured 1.7x understatement). (5) Yes via the vendored retargeter,
recorded as a named §3.1 deviation; revision confirmation blocked on hardware.
(6) Waist measured all-zero; hold, topic name reserved, not implemented.
(7) Curated limits YAML sourced from hardware_spec.md; uncommanded DoF are not
written (slot policy). (8) Layer 1: node-level safety chain plus the always-on
parametric DDS clip; hand q20-branch clamps. (9) Engage/approach/track/end_hold/
park/release, §8 above; abort is FAULT_HOLD. (10) `condition_clip` and the
supervisor live in the replay package; the hand q20 branch lives in the
existing `wujihand_controller` node (controller package).

M2: (1) joint_replay reaches DDS through the existing choke point; the 29
controller landed in 5ce3ea8. (2) Weight: 0 to 1 over at least 2 s at the held
measured pose on engage; untouched mid-run; 1 to 0 over at least 2 s at the
park snapshot on release; frozen at its current value on a mid-engage fault.
(3) `set_enabled` at ENGAGED after a clean preflight; mid-run fault holds,
operator inspects, `reset_error`, rerun from start. (4) The physical e-stop,
held by the operator; software never zeroes or auto-releases. (5) First clip
and scale from `choose_first_clip.py` against the audits, at the per-clip
allowed scale. (6) `command_vs_actual.npz` and `tracking_summary.json` against
`rt/lowstate` and hand `joint_states`. (7) Proposed pass numbers, to ratify:
zero faults; arm RMSE <= 0.15 rad and max error <= 0.35 rad; hand RMSE <=
0.15 rad; all comm ages inside watchdog bounds for the full clip.

## Hard blockers

1. **Mount adapter does not exist** (the vendor STL is a Hand v1 part).
   Blocks combined stages and the §4 flange-transform measurement.
2. **Hand 2 revision, serials, firmware unconfirmed** (§2.2). Blocks the
   retarget model pin and hand hardware stages beyond A.
3. **G1 identity unrecorded** (firmware, SDK commit, onboard mode hosting
   arm_sdk; §6). Gate for any DDS write; waist-hold-under-arm_sdk behavior
   unverified on this firmware.
4. **Wrist branch flips** in several clips need upstream regeneration or a
   per-clip wrist no-go; a decision with the trajectory authors.
5. **Official deployment limits unrecorded** (§6); screening-derived values
   govern until then, and 1.0x may be unreachable for spiky clips.

## Out of scope

Teleop (spec_2), the camera pipeline (operator camera covers `real_run.mp4`),
any change to the balance or whole-body controller, pause/resume, waist
command mode, the driver fork (upstream PR first), autonomous return-to-neutral
beyond the park slew, and monitor GUI changes.
