# SignAR (RobotSTAR) — Unitree G1 + Wuji Hand 2 Real-Robot Integration Handoff Guide

## 1. Deliverables

Attachment:

```text
SignAR (RobotSTAR)_g1_wuji_hand2_handoff_v1.tar.gz
```

The package contains 15 sign-language motion samples. Each sample provides:

```text
GT motion

Ours generated motion

Source video

Fused SMPL-X/MANO human targets

G1 upper-body reference trajectories

21-point human-hand targets required to retarget Wuji Hand 2

MuJoCo physical-simulation videos

GT/Ours comparison videos

Kinematic / physical audits
```

This package contains only offline data, reference trajectories, videos, and audit files. **It does not connect to or control a real robot.**

Extract the package:

```bash
tar -xzf SignAR (RobotSTAR)_g1_wuji_hand2_handoff_v1.tar.gz

cd SignAR (RobotSTAR)_g1_wuji_hand2_handoff_v1
```

Review these files first:

```text
HANDOFF_README.md

SAMPLES.csv

SAMPLES.json
```

Then inspect the following directories under each sample:

```text
source/

final_videos/

GT/

Ours/
```

---

## 2. Most Important Usage Restrictions

### 2.1 Existing Hand-Joint Trajectories Must Not Be Sent Directly to Wuji Hand 2

The current simulation digital model is:

```text
legacy_wuji

20 DoF per hand

scene_43dof_wuji_y90.xml
```

It is not Wuji Hand 2. The current XML already contains the legacy hand's `y90` mounting relationship. Therefore, trajectories using the old model's joint convention, such as:

```text
hand_targets.csv

left_wuji_finger1_joint1 ...

right_wuji_finger5_joint4
```

must not be mapped directly to the real Hand 2.

If the package contains:

```text
legacy_wuji_sim_only/
```

its contents may be used only to reproduce the old simulation and for visual comparison. **They must not be used as real-robot control commands for Hand 2.**

### 2.2 The Exact Hand 2 Hardware Revision Must Be Confirmed

First confirm whether the physical device is:

```text
Wuji Hand 2 Beta 1

or

Wuji Hand 2 Beta 2
```

Also confirm the left- and right-hand serial numbers, firmware versions, Wuji SDK version, and the actual mounting adapter in use. The official Wuji model repository currently provides separate Beta 1 and Beta 2 models and corresponding with-mount versions. Beta 2 also adds geometry related to fingertip tactile sensors. Therefore, a model must not be selected based only on the generic name “Hand 2.” ([GitHub][1])

### 2.3 The G1 Upper-Body Trajectory Is Not a Complete Free-Standing Controller

The current MuJoCo simulation is:

```text
physics-based fixed-base controller tracking
```

Only the floating root is fixed. The G1 body uses torque PD control, the hands use force-limited servos, and the system is integrated through `mujoco.mj_step`. It is not qpos replay, but it is also not a free-standing balance policy.

Therefore, during free-standing real-robot execution:

```text
Leg, center-of-mass, and attitude stabilization:

Handled by the existing G1 balance / whole-body controller

SignAR (RobotSTAR):

Provides only waist + left-arm + right-arm motion references

Wuji Hand 2:

Uses the official Hand 2 SDK and newly generated 20-DoF commands
```

Do not directly replay all 29 G1 body joints from the package, and do not copy MuJoCo torques or controller gains directly to the real robot.

---

# 3. How Each Type of File Should Be Used

## 3.1 Standard Inputs for Wuji Hand 2

GT and Ours each include a hand-target input. Look first for:

```text
GT/hand2_input/gt_human_targets_v5.npz

Ours/hand2_input/ours_human_targets_v5.npz
```

The most important arrays are:

```python
left_hand_keypoints21

right_hand_keypoints21
```

Contract:

```text
shape: [T, 21, 3]

ordering: MediaPipe Hands 21-point ordering

unit: meters

coordinate source: fused SMPL-X world-space hand
```

The recorded `hand21` contract uses MediaPipe ordering and is derived from the fused SMPL-X joints/tips.

Hand 2 joint targets must be regenerated from these two arrays. Do not convert the legacy `hand_targets.csv`.

The official Wuji `RetargetSession` accepts one `(21, 3)` keypoint array per frame, in meters and in MediaPipe ordering, and returns a `(20,)` Hand 2 joint result. `session.reset()` must be called before each new clip to clear warm-start and filtering state from the preceding clip. ([Wuji Technology Docs][2])

The intended logic is:

```python
left_session.reset()

right_session.reset()

for t in range(T):
    left_q20[t] = left_session.step(left_hand_keypoints21[t])
    right_q20[t] = right_session.step(right_hand_keypoints21[t])
```

The following information must also be saved:

```text
Hand 2 revision

left/right side

actual joint labels

actual joint indices

SDK version or commit

selected SDK user

calibration state

retarget parameters

timestamps
```

The Wuji retargeting documentation states that the default SDK user uses the built-in default hand model, while a named user may use its calibrated model. Therefore, the selected user and whether calibration was completed must be included in the run record. ([Wuji Technology Docs][2])

---

## 3.2 G1 Upper-Body Reference Trajectories

Each GT/Ours method directory should contain a structure similar to:

```text
g1_reference/
├── controller_reference_v7.npz
├── motor_targets.csv
├── target_meta.json
├── target_debug.npz
└── retime_report_v7.json
```

Recommended usage:

```text
controller_reference_v7.npz
→ q / dq / ddq and timing references

motor_targets.csv
→ Human-readable joint reference trajectory

target_meta.json
→ Joint names, ordering, FPS, duration, and terminal behavior

target_debug.npz
→ Task-space / IK debugging information

retime_report_v7.json
→ Timing-processing and waypoint-preservation audit
```

The logical order of the 29 body joints in the current simulation is:

```text
0–11    legs

12–14   waist

15–21   left arm

22–28   right arm
```

The exact names are stored in the metadata.

During real-robot integration, the following requirements are mandatory:

```text
Map joints by joint name

Do not map only by array index

Do not assume that the real-robot firmware order matches the MuJoCo order
```

It is recommended to use only:

```text
waist 3 DoF

left arm 7 DoF

right arm 7 DoF
```

as high-level motion references. Leg commands should continue to be generated by the G1 balance controller.

The official Unitree SDK2 provides G1 communication and control interfaces, as well as G1 7-DoF arm SDK examples and low-state feedback. The actual interface must be selected based on the current G1 firmware and the control stack already validated on site. ([GitHub][3])

---

## 3.3 Purpose of the Videos

The videos for each sample are for visual verification, not for control input:

```text
source_aligned.mp4
→ Original motion content and duration

GT_Source_HumanMesh_G1Wuji_Physical.mp4
→ Baseline for hardware mapping and the control pipeline

Ours_Source_HumanMesh_G1Wuji_Physical.mp4
→ Reference for the final generated motion

GT_Ours_Stacked_*.mp4
→ Synchronized comparison between GT and Ours
```

The real-robot team should compare the recorded physical-robot video against these reference videos frame by frame, focusing on:

```text
Whether the left and right hands are correct

Whether the elbow half-space is correct

Whether the palm orientation is correct

Whether the distance between the two hands is correct

Whether finger opening and closing are correct

Whether the robot contacts its chest, shoulder, head, or opposite hand

Whether the beginning and ending of the motion are consistent
```

---

# 4. Revalidate Arm IK on the Actual Hand 2 Model

Although the G1 shoulder, elbow, and wrist reference trajectories were generated in the previous simulation, the real Hand 2 may differ from the legacy Wuji hand in:

```text
Palm center

Wrist-housing length

Mass and inertia

Collision geometry

Mount transform

Palm frame
```

Therefore, the following process is recommended:

```text
1. Use the correct Beta 1/Beta 2 with-mount model;

2. Use the physically measured G1 flange → Hand 2 wrist transform;

3. Use the existing G1 arm trajectory as the initial guess;

4. Revalidate FK, IK, and collisions in the digital model of the real G1 + Hand 2 assembly;

5. Resolve arm IK again if the palm or wrist error is significant.
```

Do not add an arbitrary fixed 90° rotation, modify the elbow bias, or change the wrist axis without evidence. Every fixed transform must come from real CAD, the official model, or physical measurement.

---

# 5. Hand 2 Real-Robot Control Requirements

The current Wuji SDK real-time control interface requires **exactly 20** `JointCommand` entries in every publication. Each command contains:

```text
position

velocity

effort
```

The controller should also subscribe to:

```text
joint_states

joint_diagnostics
```

to obtain measured position, velocity, force/current, communication status, temperature, and fault codes. ([Wuji Technology Docs][4])

Reference publishing interface:

```python
publisher = hand.joint_command().publish()

publisher.send(joints)  # exactly 20 JointCommand entries
```

Important restrictions:

```text
Do not send position commands without reading feedback

Do not treat a 50 FPS reference as 50 Hz step commands

Do not jump directly to the next q20 vector at every source frame
```

The correct approach is:

```text
50 FPS offline reference

→ Interpolate according to timestamps

→ Enter a fixed-frequency hardware control loop

→ Send position/velocity/effort every control cycle

→ Read actual state/diagnostics every cycle or asynchronously
```

The official Wuji C API recommends publishing at a fixed frequency during continuous control, typically 200 Hz–1 kHz. The final rate must be determined from on-site tests of the SDK, firmware, and communication load. ([Wuji Technology Docs][5])

---

# 6. Hardware Checklist Required Before the First Real-Robot Run

Complete and record the following information first.

## G1

```text
G1 exact model/version

robot serial number

firmware version

unitree_sdk2 version/commit

current control mode

current balance/whole-body controller

arm command interface

joint names and indices

joint signs

zero offsets

position/velocity/acceleration/torque limits

control frequency

watchdog behavior

E-stop behavior
```

## Left and Right Wuji Hand 2

```text
Beta 1 or Beta 2

left/right serial numbers

firmware versions

wuji-sdk version/commit

left/right side assignment

20 joint labels and indices

joint signs

zero/origin settings

position and effort limits

selected SDK user

calibration state

mount model

flange transform

payload and inertia

network/IP configuration

fault and over-temperature behavior
```

Do not substitute the simulation limits in this package for the official real-hardware limits. The following values used previously are:

```text
arm velocity       0.5 rad/s

arm acceleration   3.0 rad/s²

hand velocity      4.0 rad/s

hand acceleration  20.0 rad/s²
```

These are conservative simulation-screening parameters used by the project, not official G1 or Hand 2 hardware specifications.

Wuji requires users to verify power, cabling, mechanical installation, and workspace before operation, and explicitly warns that people must not enter the area around Hand 2 during motion because of pinch and collision hazards. ([Wuji Technology Docs][6])

---

# 7. Recommended Real-Robot Testing Sequence

## Stage A: Read-Only Operation

Do not send any motion command.

Connect only to read:

```text
G1 low state

G1 control mode

G1 balance status

Hand 2 joint states

Hand 2 joint diagnostics

online joint bitmap

fault codes

temperature/current
```

Confirm all of the following first:

```text
The left and right hands are not connected to the wrong sides

All 20 joints are online

The zero pose is reasonable

No fault is present

There is no sustained communication packet loss

The E-stop and watchdog have been physically tested
```

---

## Stage B: Single-Joint Tests Under Safe Support

The robot should be placed in:

```text
Reliable suspension

A protective support frame

Stable external support

or a validated seated/fixed-base configuration
```

Do not start with a free-standing whole-body motion.

Test the following in sequence:

```text
All 20 joints on each Hand 2

Left shoulder, left elbow, and left wrist

Right shoulder, right elbow, and right wrist

Waist joints
```

Move only one joint or one small joint group at a time and use a very small amplitude. Confirm:

```text
The joint index is correct

The positive/negative direction is correct

The zero pose is correct

Measured feedback agrees with the command

There is no abnormal current, temperature, or fault
```

---

## Stage C: Test the Hands and Arms Separately

Use the following order:

```text
1. Keep both arms fixed and operate only the left Hand 2;

2. Keep both arms fixed and operate only the right Hand 2;

3. Keep the hands open and operate only the left arm;

4. Keep the hands open and operate only the right arm;

5. Keep the hands open and operate both arms;

6. Combine both arms and both hands only at the end.
```

This separation helps distinguish:

```text
Hand 2 joint-mapping problems

G1 arm-mapping problems

Mount-transform problems

Combined-collision problems
```

---

## Stage D: Run GT Before Ours

Run GT first for every sample:

```text
GT executes correctly
→ The hardware mapping, mounting, and control pipeline are reasonably trustworthy

GT does not execute correctly
→ Prioritize fixing hardware mapping, control, mounting extrinsics, or the digital model

GT is correct but Ours is incorrect
→ Then inspect the Ours motion, retargeting, or collisions
```

Do not start by running only Ours.

---

## Stage E: Start Slowly, Then Use Normal Speed

For the first complete-motion test, use:

```text
0.25× or 0.5× playback
```

After that succeeds, test:

```text
1.0× normal speed
```

Slower playback must be implemented by redistributing time while preserving the same spatial waypoints. Do not obtain a slower motion by forcibly shrinking joint amplitudes.

Slower playback can improve only:

```text
Tracking lag

Impact

Acceleration

Current/torque peaks
```

Slower playback cannot correct:

```text
Incorrect joint order

Incorrect positive/negative direction

Incorrect zero offset

Incorrect mount transform

Incorrect palm frame

IK branch flip

An incorrect collision path
```

If any of these problems occurs, stop and correct it. Do not merely continue reducing the speed.

---

## Stage F: Test Contact Motions Last

The following motions should be tested only at the end:

```text
Two-hand contact

Crossed arms

Hands close to the head

Hands close to the chest

Palms passing through or across one another

Large-amplitude wrist motion
```

The first real-robot clip should be selected from the audit results and should have:

```text
No two-hand contact

No hand-to-body contact

Small motion amplitude

Large joint-limit margin

Stable physical tracking
```

Important: in a previous simulation diagnostic, `test_x-fZc293MpJk_2-1-rgb_front` produced a peak contact force of approximately `142.6 N` between the right shoulder and the torso, and the deployment audit did not pass. Do not use it as the first real-robot sample unless the latest model and audit explicitly demonstrate that the issue has been eliminated.

---

# 8. Startup and Termination Behavior for Every Run

## Startup

Do not assume that the robot is already at the first frame of the trajectory.

Smoothly transition from the measured current pose:

```text
q_actual
```

to the first target frame:

```text
q_start
```

while limiting:

```text
position increment

velocity

acceleration

effort/current
```

Start the formal clip only after this transition is complete.

## Termination

The current SignAR (RobotSTAR) simulation semantics are:

```text
Hold the final pose at the end

Do not automatically return to neutral
```

The real system should:

```text
1. Hold the final target briefly;

2. Confirm that the measured velocity is close to zero;

3. Use an on-site, validated safe-transition controller to
   return smoothly to a safe pose or continue holding;

4. Never abruptly zero the command, disable control, or jump to neutral.
```

---

# 9. Immediate Stop Conditions

If any of the following occurs, immediately execute the on-site, validated safe-stop/E-stop procedure:

```text
Any joint moves in the direction opposite to the expected direction

The joint index or left/right mapping is uncertain

The measured pose deviates substantially from the target

A discontinuous jump occurs within one frame or a short time window

A hand, wrist, elbow, or shoulder collides with the robot body

The two hands collide unexpectedly or pinch one another

The balance controller raises an alarm or the robot visibly loses stability

Communication exhibits sustained packet loss, timeout, or timestamp anomalies

Any Hand 2 joint goes offline

Current/effort remains saturated

Temperature becomes abnormal

Any hardware fault occurs

The watchdog is not functioning

The E-stop is unavailable

A person or object enters the robot's motion workspace
```

Do not continue testing the remaining 15 samples after an abnormal event.

---

# 10. Required Real-Robot Data Logging

At minimum, log the following for every test:

```text
Unified monotonic timestamps

sample name

GT or Ours

playback speed

commanded G1 q/dq

measured G1 q/dq/tau

commanded Hand 2 q/dq/effort

measured Hand 2 q/dq/effort/current

joint temperature

joint diagnostics

fault/error codes

communication statistics

G1 IMU

G1 balance status

watchdog/E-stop events

real-run video
```

Recommended outputs:

```text
hardware_manifest.json

joint_mapping.json

mount_transform.yaml

hand2_retarget_meta.json

command_vs_actual.npz

tracking_summary.json

fault_log.jsonl

real_run.mp4
```

---

# 11. Results to Return

After the first integration round, return:

```text
1. G1 hardware/firmware/SDK information;

2. Left and right Hand 2 revision, serial number, firmware, and SDK information;

3. The joint-name → hardware-index mapping for G1 and Hand 2;

4. Joint-sign and zero-offset configurations;

5. The measured G1 flange → Hand 2 wrist transform;

6. The left and right q20 trajectories generated after Hand 2 retargeting;

7. The final G1 upper-body reference;

8. GT command-vs-actual logs and real-robot videos;

9. Ours command-vs-actual logs and real-robot videos;

10. Fault, temperature, current, collision, and balance logs;

11. A list of samples that can run safely at 0.5× and 1.0× speed.
```

Before GT completes hardware validation, do not attribute a visual-performance problem in Ours directly to the generative model.

---

## Summary

> This is the SignAR (RobotSTAR) G1 + Wuji Hand 2 real-robot integration handoff package. It contains GT/Ours human targets for 15 samples, G1 upper-body references, 21-point hand targets required by Hand 2, simulation videos, and audit files. Note that the current simulated hand model is legacy Wuji, not Hand 2, so the old hand-joint trajectories in the package must not be used to control the real robot. Use the left- and right-hand 21-point keypoints under `hand2_input` to regenerate 20-DoF trajectories for the actual Hand 2 revision through the current Wuji SDK, and revalidate G1 arm IK using the real with-mount model and flange transform. Treat the G1 trajectory only as a waist/arm reference; free-standing execution must retain the existing balance/whole-body controller. For the first test, use suspension or reliable external support, then proceed through read-only checks, single-joint tests, separate arm and hand tests, GT, and finally Ours. Test at 0.25×/0.5× before 1.0×, and test two-hand contact motions last. Record complete command/state/current/temperature/fault/balance logs and return the hardware manifest, joint mapping, mount transform, real-robot videos, and tracking summary.
