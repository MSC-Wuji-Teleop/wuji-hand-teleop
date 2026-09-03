# Spec 1: clip replay on the G1 and two Wuji Hand 2

**Status:** built 2026-09-02, verified 2026-09-03. The offline half has run on
the whole bundle in the container (30 trajectories: 4 safe, 23 rejected, 3
refused) and the online half has run in sim end to end. Nothing has run on the
rig. Per-piece state: [build status](#build-status); what the runs showed:
[the handoff note](../issues/replay-handoff-2026-09-02.md#verified-2026-09-03).
Operator commands: [replay.md](../replay.md).

Play every recorded clip in the handoff bundle on the real 29-DoF G1 arms
and both Wuji Hand 2 units. Two halves, one file boundary between them:

- **Offline** (`tools/prepare_clip.py`): turn a bundle sample into a clip
  directory. Smooth the arms, retarget the hands, then replay the whole thing
  dynamically in MuJoCo with the hands actuated and measure what the contacts
  do under our controller. Contact is expected in these clips (the hands
  touch each other and the body); the question the audit answers is how hard.
- **Online** (`replay_publisher` + two device nodes): publish the clip.
  Nothing else runs. No run-time checks of any kind; those can be added later
  behind the same file boundary without touching this graph.

## Clip directory

The boundary. One directory per trajectory, written by `prepare_clip.py`,
read by `replay_publisher`.

```
clips/
  safe/       <sample>_<GT|Ours>/     verdict safe at one or more speeds; the publisher searches here
  rejected/   <sample>_<GT|Ours>/     verdict rejected; kept for the report
  candidate/  ...                     scratch, gitignored
```

`clips/safe/` is tracked in git (a clip is about 300 KB). `rejected/` and
`candidate/` are gitignored. The whole `clips/` tree is bind-mounted into
the teleop container at the same relative path, and `tools/` is mounted
read-only next to it (`docker-compose.yml`).

| file | contents |
|---|---|
| `arm_q.npz` | `left`, `right`: `(T, 7)` float64 radians, one row per frame. Column order is `clip.json` `arm_joint_names` |
| `hand_q20.npz` | `left`, `right`: `(T, 20)` float64 radians, same `T`, in the hand driver's hardware order (`clip.json` `hand_joint_names`) |
| `clip.json` | see below |

`clip.json`:

```json
{
  "tool": "prepare_clip/1",
  "source": {"sample": "11_val_a5yNwUSiYpA_9-3-rgb_front", "method": "Ours", "bundle_manifest_sha256": "..."},
  "frames": 190, "rate_hz": 50.0,
  "arm_joint_names": {"left": ["left_shoulder_pitch", "..."], "right": ["..."]},
  "hand_joint_names": {"left": ["l_thumb_cmc_flex", "..."], "right": ["r_thumb_cmc_flex", "..."]},
  "sanitize": {"cutoff_hz": 6.0, "max_step_deg": 15.0, "trim_start": 0, "trim_end": 0, "allow_flips": false,
               "auto_trim": false, "min_seconds": 3.0,
               "before": {"max_step_deg": 26.0, "peak_vel_rad_s": 13.7, "peak_acc_rad_s2": 900.0}, "after": {"...": "..."},
               "arm_rmse_rad": 0.013},
  "hand_retarget": {"config": "retarget_keypoints_topic_{side}.yaml", "config_sha256": {"left": "...", "right": "..."},
                    "clipped_fraction": {"left": 0.0, "right": 0.01}},
  "audit": {
    "model": "g1_29_wuji2_fixed.xml", "model_sha256": "...", "mujoco_version": "3.12.0", "timestep": 0.002,
    "arm_gains": {"kp": 140.0, "kd": 3.0, "kp_wrist": 50.0, "kd_wrist": 2.0},
    "hand_command_slew_rad_s": 2.0,
    "thresholds": {"max_arm_torque_ratio": 0.8, "max_contact_force_n": 80.0},
    "speeds": [1.0, 0.5, 0.25], "note": "",
    "per_speed": {
      "1.0":  {"pass": false, "peak_arm_torque_ratio": 0.93, "peak_contact_force_n": 142.0,
               "peak_contact_pair": ["right_shoulder_yaw_link", "torso_link"], "contact_frame_fraction": 0.31,
               "arm_saturation_fraction": 0.04, "hand_saturation_fraction": 0.12,
               "tracking_rmse_rad": {"arms": 0.09, "hands": 0.11},
               "top_contact_pairs": [["right_shoulder_yaw_link", "torso_link", 142.0], "..."]},
      "0.5":  {"pass": true,  "...": "..."},
      "0.25": {"pass": true,  "...": "..."}
    }
  },
  "safe_speeds": [0.5, 0.25],
  "verdict": "safe"
}
```

Arm joint names carry no `_joint` suffix: they are the names the G1 node
matches on (`G1_29_ARM_JOINT_NAMES` in `robot_arm.py`); the MJCF joints and
actuators are `<name>_joint`. `verdict` is `safe` when at least one audited
speed passes; `safe_speeds` is the list that did. Every number the audit
used to decide is in the file, so a clip can be re-judged with different
thresholds without rerunning the simulation, and the operator can read what
the arms will be asked for before the first run.

## Offline: `tools/prepare_clip.py`

Runs inside the teleop container: it needs `numpy`, `scipy`, `mujoco`, the
retargeter (`wuji_retargeting`), the Hand 2 URDF (`wujihand_urdf`), and the
composed model (`g1_wuji2_description`), all of which the image has. It
also runs on any machine with those Python packages and the bundle; the
model is loaded from `src/` relative to the repo root. The MuJoCo audit
lives in `tools/clip_audit.py`, which also re-audits an existing clip
directory on its own.

```bash
python3 tools/prepare_clip.py --method-dir RobotSTAR_demos/samples/<sample>/Ours --out clips
python3 tools/prepare_clip.py --all RobotSTAR_demos/samples --out clips     # all 30, writes clips/summary.md
```

```mermaid
graph LR
    SRC["bundle sample<br/>g1_reference/controller_reference_v7.npz + target_meta.json<br/>hand2_input/*_human_targets_v5.npz"]
    SAN["1 sanitize arms<br/>zero-phase Butterworth 6 Hz<br/>per-frame step clamp 15 deg<br/>optional trim; refuse 90 deg flips"]
    RT["2 retarget hands<br/>21 keypoints to q20 per body frame<br/>production retargeter + configs"]
    AUD["3 dynamic audit, per speed<br/>MuJoCo g1_29_wuji2_fixed.xml, fixed base<br/>arms: PD kp 140/3, wrists 50/2, torque clamps<br/>hands: actuated, vendor servos, 2 rad/s slew<br/>contact force, torque ratio, saturation, tracking"]
    JUDGE{"4 judge<br/>per speed against thresholds"}
    SAFE[("clips/safe/&lt;clip&gt;/")]
    REJ[("clips/rejected/&lt;clip&gt;/")]

    SRC --> SAN --> RT --> AUD --> JUDGE
    JUDGE -->|some speed passes| SAFE
    JUDGE -->|none| REJ
```

**1. Sanitize arms.** The algorithm from the earlier sanitizer: zero-phase
second-order Butterworth at `--cutoff-hz` (6) on the 14 arm columns, then a
forward and backward per-frame step clamp at `--max-step-deg` (15).
`--trim-start N` and `--trim-end N` drop frames first. A single-frame step
of 90 deg or more in what remains is an estimator orientation flip;
smoothing it produces a slow sweep through the same wrong path, so the tool
refuses unless `--allow-flips` is given, and records that it was. Legs and waist columns are read but not written to the
clip: `g1_world_output` commands arm joints only, and the waist stays under
the robot's onboard controller. The audit holds the waist at zero for the
same reason.

**2. Retarget hands.** For each body frame `i`, take hand keypoint frame
`round(i * (T_hand - 1) / (T_body - 1))` (the same mapping the sim publisher
uses) and run it through the production retargeter with the
`retarget_keypoints_topic_{side}.yaml` configs, with `reset()` at frame 0.
Output is in the driver's hardware order, which is the URDF's declaration
order; the permutation the hand controller applies at runtime is applied
here instead. This is the only place hands are retargeted; the online path
never runs the retargeter.

**3. Dynamic audit, per speed.** For each speed in `--speeds` (default
`1.0 0.5 0.25`), replay the clip through `mujoco.mj_step` on
`g1_29_wuji2_fixed.xml` (fixed base, 2 ms timestep, waist and legs held at
the stand keyframe):

- Arm actuators are re-gained to what `g1_world_output` sends: kp 140 / kd 3
  on shoulder and elbow joints, kp 50 / kd 2 on the three wrist joints. The
  composed model ships the menagerie servo (kp 500, critically damped),
  which is a stand-alone sim setting and not the robot; the audit overwrites
  the gain and bias terms of the 14 arm actuators with the values
  `G1ArmController` writes into every DDS command (`robot_arm.py`). The
  joint-level `actuatorfrcrange` stays (25 Nm arm joints, 5 Nm wrist pitch
  and yaw): MuJoCo clamps `qfrc_actuator` with it, so that array is what the
  torque ratio reads. All of these constants sit at the top of
  `tools/clip_audit.py` with the same justification.
- Arm targets advance one clip frame every `1 / (rate_hz * speed)` seconds
  and are linearly interpolated one publish period behind, which is what the
  fixed G1 node does (see [Arms](#online-play-a-clip)).
- Hands are **actuated**, on the vendor's force-limited position servos as
  the model ships them, with the hand driver's behaviour modelled on the
  command stream: targets slew-limited at 2 rad/s and clamped to the joint
  ranges. A clip with the hands frozen at neutral would show contacts the
  real hands move out of, and miss the ones they move into.
- The first frame is approached from the stand keyframe over 2 s before the
  clock starts, so the audit measures the clip and not the startup step.
  The clip is followed by a 0.5 s hold.

Recorded per speed: peak contact force and its body pair, the five largest
pairs, fraction of frames with any contact, peak arm joint torque as a
fraction of its clamp and the fraction of frames any arm joint sits at the
clamp, hand servo saturation fraction, arm and hand tracking RMSE.
Definitions: contact force is the norm of the 3D force part of
`mj_contactForce` per contact, in newtons; a frame counts toward a fraction
when any physics step inside its period met the condition; saturation means
at or above 0.999 of the clamp; arm RMSE is against the interpolated target,
hand RMSE against the clip's q20 for the frame, so the driver's slew counts
as plant lag.

**4. Judge.** A speed passes when `peak_arm_torque_ratio <=
--max-arm-torque-ratio` (default 0.8) and `peak_contact_force_n <=
--max-contact-force-n` (default 80, the bundle authors' own deployment gate,
kept only as a starting number). Hand saturation is reported, not judged:
the real driver's 0.6 A ceiling bounds it and the servo model's units are
not the driver's. The thresholds are choices. The audit's job is to make
the numbers visible; deciding what a stalled shoulder against the torso
shell may do is not something the simulation knows.

**Where the audit is honest and where it is not.** It uses our gains, our
model with the wrist contact-exclude, and actuated Hand 2 units. It does not
know real contact stiffness, the hand mount adapter's strength (still an
unconfirmed part), harness snags, or the G1 firmware's behaviour at a torque
clamp. Read `peak_contact_pair` as much as the number.

**If every clip is rejected.** Expected for some of the 30: on the corrected
model, 29 of 30 penetrate deeper than 10 mm somewhere when the hands are
frozen at neutral. The ladder, cheapest first, each an option `clip.json`
records: run slower (`--speeds` down to 0.1); trim (`--trim-start`,
`--trim-end`, or `--auto-trim` for the longest passing window of at least
`--min-seconds` 3); raise a threshold with a reason written in
`--note`. `--auto-trim` audits the full clip once, takes the fastest
audited speed that has a passing window of that length, trims to the
longest such window, and audits the trimmed clip again at every speed;
the second audit is what the file records. Nudging contacting frames with a local IK and re-solving the arm
retarget against this model are real fixes for presses and flips
respectively, and are separate work items, not part of this tool.

## Online: play a clip

```mermaid
graph LR
    CLIP[("clips/safe/&lt;clip&gt;/")]
    PUB["replay_publisher<br/>one timer at rate_hz * speed<br/>--arms --hands --speed<br/>plays once, holds last frame"]
    G1["g1_world_output<br/>mode=joint_replay arm_type=G1_29<br/>interpolates one frame behind<br/>own container"]
    HL["hand_node left<br/>starport_wuji_hand, Ethernet"]
    HR["hand_node right<br/>starport_wuji_hand, Ethernet"]
    DDS["G1 arms<br/>DDS rt/arm_sdk"]
    WL["left Hand 2"]
    WR["right Hand 2"]

    CLIP --> PUB
    PUB -->|"/left_arm/joint_targets<br/>/right_arm/joint_targets"| G1 --> DDS
    PUB -->|"/left/wuji_hand/joint_command"| HL --> WL
    PUB -->|"/right/wuji_hand/joint_command"| HR --> WR
```

Four processes, two containers. `wujihand_controller` (the glove-teleop
retargeter) is not on this path.

**`replay_publisher`.** Arguments:

| flag | values | meaning |
|---|---|---|
| `--clip DIR` | a directory under `clips/safe/` | refuses a directory that is not under `clips/safe/` or whose `clip.json` `verdict` is not `safe` |
| `--arms` | `none`, `left`, `right`, `both` (default `both`) | which arm topics are published. `none` publishes nothing to the G1 node |
| `--hands` | `none`, `left`, `right`, `both` (default `both`) | which hand driver topics are published |
| `--speed S` | one of the clip's `safe_speeds`; default: the largest | timer period is `1 / (rate_hz * S)`. Same frames, published slower. Amplitudes unchanged, peak velocity scales by `S`, acceleration by `S^2`. Any speed that is not an audited passing speed is refused, including a slower one: see [Slower is not always safer](#slower-is-not-always-safer) |

Behaviour: one timer; at each tick publish frame `i` for every selected
side, with joint names on every message; at the end hold the last frame
(keep publishing it) until killed. The first published frame is a step
from wherever the robot is to frame 0: on the arm side the G1 node's
velocity clip is the only limiter, on the hand side the driver's slew limit.
No approach ramp here, by decision; if one is wanted it is clip content
written by `prepare_clip.py`.

### Slower is not always safer

`check_speed` requires the requested speed to *be* one of `safe_speeds`, not
merely to sit at or below the largest. The first draft of this spec refused only
speeds above the fastest safe one, on the assumption that a slower replay of the
same frames is always gentler. The bundle falsifies that:
`05_test_G42xKICVj9U_5-5-rgb_front_GT` passes at 0.5 with a peak arm torque ratio
of 0.736 and fails at both 1.0 (1.00) and **0.25 (0.846)**. Most of the load on
the wrist joints in these clips is a hand-to-hand contact reaction, which barely
changes when the clip is played slower, while the poses it is reacting to do
change. Under the old rule `--speed 0.25` on that clip was accepted and would
have played a speed the audit had rejected, which is the one thing the offline
gate exists to prevent. Auditing another speed is one `prepare_clip.py --speeds`
run away, so nothing is lost by refusing.

**Arms.** `g1_world_output` in `joint_replay` mode, `arm_type=G1_29` (the
config default), own container. It matches joints by name and writes DDS
through `G1ArmController` at `control_rate` 250 Hz. Before 2026-09-02 its
interpolation parameter was `(now - t_prev) / (t_next - t_prev)` with
`t_next` the newest sample's arrival time, so at every tick it was at or
past 1 and the output was a zero-order hold at the publish rate; at
`--speed 0.25` that is a 12.5 Hz staircase into a stiff PD. The buffer
(`side_buffer.py`) now interpolates one publish period behind:
`alpha = (now - t_next) / (t_next - t_prev)` between `q_prev` and `q_next`,
clamped to `[0, 1]`. One frame of latency, continuous command at any speed.
Until two real samples have arrived it holds the newest one, so the first
frame is the step described above and not a ramp over however long the node
sat idle before the publisher started.

**Hands.** One `starport_wuji_hand` `hand_node` per side, namespaced
`/left/wuji_hand` and `/right/wuji_hand`. The driver finds its hand by UDP
broadcast scan and serial number (the host NIC must have an address on the
hands' subnet), homes it on connect (3 s), and accepts named `JointState`
in radians on `~/joint_command`. It publishes measured state on the global
`/joint_states` (both hands; `l_` and `r_` prefixes) and `~/connected`.
What the driver does between a received command and the SDK (2 rad/s slew,
soft-limit clamp, hold on stale, idle release after 5 s without commands)
is the driver's own and is set by its launch arguments; this spec adds
nothing to it and mirrors the slew in the offline audit.

**Topics.**

| topic | type | producer | consumer |
|---|---|---|---|
| `/left_arm/joint_targets`, `/right_arm/joint_targets` | `sensor_msgs/JointState`, 7 named | `replay_publisher` | `g1_world_output` |
| `/left/wuji_hand/joint_command`, `/right/wuji_hand/joint_command` | `sensor_msgs/JointState`, 20 named | `replay_publisher` | `hand_node` |
| `/left_arm/joint_states`, `/right_arm/joint_states` | `sensor_msgs/JointState` | `g1_world_output` | connection check |
| `/joint_states` | `sensor_msgs/JointState`, both hands | `hand_node` x2 | connection check |
| `/left/wuji_hand/connected`, `/right/wuji_hand/connected` | `std_msgs/Bool` | `hand_node` | connection check |

**Launch and the single terminal.** `replay.launch.py` (teleop container)
starts the hand drivers for the sides in `hands` and the publisher; its
arguments are `clip`, `arms`, `hands`, `speed` (empty means the clip's
default), `check`, and `sim`. The G1 node cannot be in that file (other
container). `scripts/replay.sh` on the host does both: starts the G1
container detached unless `--arms none`, runs the teleop launch in the
foreground, stops the G1 container on exit. Its flags are the publisher's
plus `--check` and `--sim`. `--check` starts the drivers and the G1 node
with no publisher and runs `replay_check` in place of it: it waits up to
20 s for state from every selected source (the driver gives up on an absent
hand after ten connect attempts at 2 s), prints the topic rates, and exits 0
when all reported, 1 otherwise; `--arms` and `--hands` narrow what is
started and checked. `--sim` runs the G1 node with `dry_run`, starts no hand
driver, and opens `mujoco_visualizer.py` on the composed model, which
mirrors the G1 node's arm commands and the publisher's hand commands. Exact
commands: [replay.md](../replay.md).

## Out of scope, by decision

Nothing runs between the publisher and the device nodes: no run-time checks
or trip conditions (temperature, effort, contact), no e-stop logic, no
approach ramp. Teleop (glove, PICO) is untouched and shares only
`g1_world_output` with this path.

The rehome ([spec1_1.md](spec1_1.md)) does not change that. It is a separate
operator command, `scripts/replay.sh --home`, which generates and audits a clip
before it runs and then plays it through this same publisher and this same
graph. It adds no runtime state, no mode and no second motion path, and nothing
on the replay path can reach it: `--home` takes no clip and a replay invocation
never sets it.

## Build status

Verified 2026-09-03 in the teleop container unless a row says otherwise.

| piece | state | remaining |
|---|---|---|
| `tools/prepare_clip.py`, `tools/clip_audit.py` | run on the whole bundle: 30 trajectories in 6 min, `clips/summary.md` written; 69 tests | none |
| `scipy` in the teleop image | present (`scipy==1.14.1`, Dockerfile section 5) | none |
| `clips/`, `tools/` mounts | built and in use | none |
| `replay_publisher`, `replay_check` | both run; publisher plays a clip at 50 Hz and holds the last frame, check exits 1 naming its missing sources; 89 tests | run the check against real hardware |
| `g1_world_output` `joint_replay` | measured on the live graph: 250 Hz command stream advancing 1/20 of a frame step per tick at `--speed 0.25`; 29 tests | none |
| `starport_wuji_hand` driver | `wuji_sdk` import check passes on the 2026.8.31 pin; `colcon build` and 268 tests pass; shutdown no longer raises on SIGINT | run against a hand |
| Humble `launch_ros` and `list[float]` | accepted: `is_typing_list` checks `__origin__ in (list, List)` | none |
| `replay.launch.py`, `scripts/replay.sh` | one host command brings up the G1 container, the publisher and the viewer, and stops the G1 container on exit; 20 tests | run on the rig |
| model fix | cherry-picked (`2a76a4f`) | none |
| G1 image CRC libraries | present after a `--no-cache` rebuild (`utils/lib/crc_{amd64,aarch64}.so`) | none |
| G1 DDS NIC pin | `g1_robot.yaml` `network_interface` is read and reaches `ChannelFactoryInitialize`; verified live (the node names the NIC and refuses to start when it is absent) | confirm the adapter's name on the rig |
| build context | `.dockerignore` added; the context was 2.9 GB and is now a few hundred MB | none |
| `wuji-sdk` pin | `docker/Dockerfile` moved to 2026.8.31 (2026.5.26 has neither `DeviceType` nor `JointCommand`) and the image is rebuilt: the SDK is on the system path, not a `pip --user` install | none |
| `wujihandros2` (USB driver) | present; teleop launches still use it | remove the submodule and the `wujihandcpp` deb once the Ethernet driver runs on the rig |
| `wujihand_controller` | built, teleop only | none |
| `mujoco_visualizer.py` | mirrors the driver command topics by name and the controller's positional ones; verified live on both forms | none |
| `--check` in sim | cannot pass: with `dry_run` the G1 controller has no `arm_ctrl`, so `/{side}_arm/joint_states` is never published | hardware-only command, by nature |
