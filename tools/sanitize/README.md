# `tools/sanitize` — drop-in sanitizer for RoboSTAR retargeting output

Implements [docs/spec/RoboSTAR_dropin_fix.md](../../docs/spec/RoboSTAR_dropin_fix.md).
This file is the implementation reference — every measured constant and why it
has the value it has. For how to run the stage, what its report means and how
a sanitized clip gets filed, read [docs/sanitize.md](../../docs/sanitize.md).
npz arrays in, the same npz arrays out, so the result drops straight into the
existing replay pipeline. Offline only: no ROS, no DDS, no hardware.

```bash
# inside the teleop container
cd ~/ros2_ws
python3 tools/sanitize_clip.py IN.npz -o OUT.npz
python3 tools/sanitize_clip.py clips/safe/<clip> -o clips/candidate/<clip>
python3 tools/sanitize_clip.py IN.npz --check          # measure, write nothing
python3 tools/sanitize_clip.py --help
```

Runs in the `teleop` container, not `g1_world_output`: it needs Pinocchio and
coal (`pin==4.0.0`, which bundles coal 3.0.3) and no CasADi, and both are in
the main image. Nothing here imports `mujoco`.

## The three defects, and what is done about each

| Spec defect | What this does |
| --- | --- |
| 1. wrists phase through each other, no collision checking | Checks all 3576 non-adjacent geometry pairs of `g1_29_wuji2.urdf` per frame with coal, and carries every pair it finds into the frame's own IK solve as a separation constraint — so the arm is held out of a contact rather than pushed out of one. It is the URDF and not the MJCF because the two answer different questions: `clip_audit.py` measures the reaction force of a contact (its peak pairs are routinely hand-to-hand), while what a clearance check needs is whether the meshes overlap at all. |
| 2. branch flipping, torque and velocity spikes | Re-solves each frame from the previous one, shoulder to elbow first and then the wrist, and bounds the per-frame step by the URDF velocity limit. A joint jump the wrist pose did not follow is detected as a flip and the source elbow is distrusted for the length of it. |
| 3. wrist rotation drift, hands inverted by the end | Measures each wrist joint's end-to-start rotation and reports it. **Correction is opt-in** (`--max-drift-deg`) — see below. |

## Pipeline

1. **read** either layout (`clipio`), clamp hand angles to the URDF limits, and
   otherwise leave them alone. Hand joints are never re-solved: regenerating
   them is the retargeter's job, and CLAUDE.md keeps that so.
2. **unwrap** 2π jumps out of the arm joints, and measure wrist drift
   (`unwrap`). A jump is only removed if the result still fits the joint's own
   range — every limit on this model is narrower than 2π, and a source that
   runs 3.0 rad and returns as −3.0 rad unwraps to 3.28 rad, past a 3.1 rad
   limit at any whole-turn offset. Those are left alone (and reported) for the
   flip detector rather than handed to the solver as a pose it can only clamp.
   A column written a whole revolution out is recentred, since that is the
   same rotation and the only difference is whether it can be commanded.
3. **extract** the wrist placement and elbow position every source frame
   implies, by forward kinematics (`model.targets`). The poses, not the joint
   angles, are what the sanitized clip preserves.
4. **detect flips**: a joint step over `--flip-step-deg` across which the wrist
   pose moved less than `--flip-pose-pos-mm` / `--flip-pose-ori-deg`. Joints
   jumped, pose did not — that is a solver changing branch, not a human
   moving. The elbow task is then gated off until the source elbow comes back
   within `--elbow-rejoin-mm` of the elbow the output is holding.
5. **re-solve** each frame in three stages (`ik`), seeded with the previous
   frame.
6. **check** the collision pair set and **solve against** what it finds, in
   the same stage 3 (`collision`, `ik`).
7. **write** the same layout, plus `sanitize.json`.

### Why the solve is staged the way it is

The spec asks to "start from shoulder to elbow, then take the target elbow
positions as starting position and solve for wrist pose". That order is what
fixes the flipping, and two of its details had to be measured on this model
rather than assumed:

* **Stage 1 — shoulder pitch and roll → elbow centre.** Shoulder *yaw* is not
  in this stage. It swings the elbow centre by only 0.0158 m/rad against 0.198
  (pitch) and 0.184 (roll), because its axis runs nearly along the upper arm.
  Give all three joints to the elbow task and there is a flat valley — tens of
  degrees of yaw error paid for by a few degrees of pitch and roll at the same
  elbow position — and the solve walks along it: 126° of drift from a clean
  source by the end of one clip. Pitch and roll alone place a point on a
  sphere, which is all the elbow centre is.
* **Stage 2 — shoulder yaw, elbow, and the three wrist joints → wrist pose.**
  Five joints for a six-DoF pose. Yaw belongs here because the wrist pose is
  what actually determines it. Nothing in this stage can disturb stage 1: the
  elbow, wrist roll, pitch and yaw move the elbow centre by exactly 0.000000
  mm, and yaw's effect is what stage 3 mops up.
* **Stage 3 — both tasks, all seven joints.** The staging picks the arm's
  branch; this makes the pose exact. Measured on a real clip: with stage 3 the
  output returns to **0.0001°** of a clean source and 3e-8 m of wrist error;
  without it, 13.5° and 6 mm, because five joints cannot close a six-DoF pose.

Why any of this beats solving the wrist pose alone: one wrist pose admits many
exact arm configurations. Solving only the 6-DoF wrist task from 60 random
seeds on this model found **32 distinct exact solutions** for a single pose —
that is the branch flipping, and the velocity spikes are what jumping between
them costs. Imposing the elbow position leaves exactly one.

### Collision checking

Every unordered geometry pair, minus two exclusions: pairs within two joints
of each other in the kinematic tree (247 pairs — a finger segment and its own
neighbour overlap at the knuckle at every configuration), and pairs where
neither link can move (182 pairs — legs and waist are locked, so their answer
is the same at every frame and is evaluated once). 3576 pairs remain, and the
model is collision-free at its neutral pose, which is the check that validates
the exclusion set; the tool refuses to run if that ever stops being true.

**What the mesh backend can answer.** coal will not sign-distance a triangle
mesh: `enable_signed_distance` is a convex-shape feature, `computeDistance`
returns exactly 0.0 the moment two meshes touch however deep they then go,
and the contact's `penetration_depth` saturates at the security margin. So
there is no depth of overlap to be had, and the tool does not invent one. A
contact is either a **near miss** with a measured gap, or **touching** with no
number — and touching is the defect, so no number is needed to act on it.

Three queries, with very different costs (measured on a real clip frame):

| Query | Cost | Gives |
| --- | --- | --- |
| sweep at `margin = clearance` | 170–220 ms | the 37–92 pairs on this frame worth looking at, out of 3576 |
| `computeDistance` per candidate | 16–26 ms total | the true gap, 0 at touch |
| `computeCollision` at `margin = 0` per candidate | 4–9 ms total | touching or not |

Asking coal for all 3576 distances instead of only the candidates takes
**4.1 s** per frame — 22 minutes for one clip — which is why the sweep goes
first.

Every pair carries a class, because whether a contact can be fixed depends on
it:

| Class | Fixable | |
| --- | --- | --- |
| `cross_side` | yes | left arm or hand against right. Spec defect 1. |
| `arm_body` | yes | an arm or hand against torso, head, pelvis or a leg. |
| `arm_self` | yes | a hand against its own forearm or upper arm. |
| `intra_hand` | **no** | two segments of the same hand. Hand joints pass through this tool untouched, so nothing it can change moves either geometry. |

That last row is not a corner case: a real bundle clip has intra-hand finger
contact on **every frame** (middle and ring finger distal segments, from the
retargeter closing the fingers), which is why it is counted per pair instead
of reported per frame.

### Clearance is part of the solve, not a pass after it

Each contact becomes one linearised separation row — the relative velocity of
the two witness points along the separating normal, over the 14 arm joints —
and the rows go into **stage 3 of the frame's own IK**, weighted 3 against
pose tasks weighted 1, re-measured on every iteration so they follow the arm
instead of pushing along a direction measured once.

Two things make that different from re-solving a finished frame:

* The active set **carries from frame to frame** (`ACTIVE_PAIR_MEMORY_FRAMES`,
  15). Contact in a signing clip lasts tens of frames, so the pair that just
  touched is already a constraint on the next frame's first solve — the arm
  is held out of the contact before it arrives.
* A frame that moves into a pair nobody knew about is solved again with that
  pair added (`--avoid-passes`, 3), not nudged out of it afterwards.

It is still **bounded**, and that bound is not optional. coal reports no
penetration depth (below), so a touching pair asks for the whole clearance on
every iteration however far the arm has already moved: the demand never
shrinks while the pose cost grows quadratically. Unbounded on the
crossed-arms fixture, a separation weight of 10 settles at 48° of joint travel
and 41 mm of wrist error and ends in **53 contacts where it started in 17**,
having driven the arms into the torso. `--avoid-budget-deg` (default 5°) is a
trust region on the clearance solve around the same frame's pose-faithful
answer; `--separation-weight` (default 3) is how hard clearance argues inside
it. Measured on `05_test_G42xKICVj9U_5-5-rgb_front_Ours`, 260 frames with the
hands interlocked through most of them:

| | unresolved touching pair-frames | worst wrist residual | max step out |
| --- | --- | --- | --- |
| weight 3, budget 5° (default) | 757 on 57 frames | 19.1 mm / 3.5° | 15.0°/frame |
| weight 10, budget 2° | 1249 on 67 frames | 12.9 mm / 3.8° | 15.0°/frame |
| bounded post-pass (previous design) | 1435 on 60 frames | 23.3 mm / 10.4° | 18.5°/frame |

A frame that has not cleared is reported: which frame, which timestamp, which
pair, and whether it is touching or merely close. What the arms cannot part,
they are not allowed to wreck the pose chasing.

`--fail-on` decides what counts as a defect — both what the solve is
constrained against and what fails. The default, `touch`, is two meshes
actually touching; `clearance` also treats anything inside the margin as a
defect. Two-handed signing brings the hands within millimetres of each other
on purpose, while phasing through is the defect, so the default separates
them and counts near misses per pair. Constraining every near miss also cost
2 s per frame and moved arms off poses that were fine.

### Drift correction is opt-in

`--max-drift-deg` defaults to infinity: drift is measured and reported, not
removed. On `13_val_..._Ours` the left wrist ends **77.5°** of roll from where
it started, and capping that at 30° rewrote 47° of a real signing motion — 22°
of median deviation across every arm joint, swamping everything else. A sign
sequence has no obligation to end where it began, and no threshold can tell an
intended end pose from the accumulation the spec describes without knowing the
clip. So the number goes in front of a person: every wrist joint's end-to-start
rotation is in `sanitize.json`, anything past `--drift-warn-deg` (default 90°,
half the "inverted" the spec describes) gets a `DRIFT` line on stderr, and
`--max-drift-deg 30` then removes the excess as a ramp linear in frame index.

## Layouts

Auto-detected on read, echoed on write.

* **flat npz** — `arm_q (N, 14)` with an `arm_joint_names (14,)` column index,
  `left_hand_q20 (N, 20)`, `right_hand_q20 (N, 20)`, `target_fps`. Every other
  key is carried through untouched and the arm columns go back in the input's
  own order. The report is written next to it as `OUT_sanitize.json`.
* **clip directory** — `arm_q.npz` with `left`/`right` `(N, 7)`,
  `hand_q20.npz` likewise, `clip.json`. The report is written as
  `sanitize.json` in the directory and also added to `clip.json` under
  `"sanitize"`.

## Exit codes

| | |
| --- | --- |
| 0 | clean |
| 3 | written and playable, but N frames could not be cleared — a person decides |
| 2 | refused: bad arguments, or a model that is not collision-free at rest |
| 1 | unexpected error |

## What it does not do

* **Not a smoother.** No Butterworth, no trim — `prepare_clip.py` owns those.
  It also does not tighten the step below the URDF velocity limit unless asked
  (`--max-step-deg 15` reproduces `prepare_clip.py`'s clamp).
* **Not a judge.** The dynamic verdict — torque, contact force, saturation —
  still comes from `tools/clip_audit.py`, which has to be re-run on the output
  before the clip is filed as safe. This tool never writes a verdict.
* **Never touches** legs, waist, or hand joint angles, and never regenerates
  hand joints from keypoints.
* **Cannot fix** `intra_hand` contact, since it does not move hand joints.

## Runtime

Roughly a quarter of a second per frame on a clip that is mostly clear —
almost all of it the pair-set sweep — and about **1.2 s** per frame on one
that is in contact throughout, where every frame also pays for the clearance
solve and its per-iteration re-measurement: 5m19s for the 260 frames of
`05_..._Ours`. `--no-collision` runs the kinematics alone in about **1.5 s**
for a whole clip, which is the fast way to check pose fidelity — and on a clip
that is already clean the two agree, because a faithful re-solve is the no-op
case.

## Note on the input data

The tool was first calibrated without the bundle on hand, against
`wuji_clips/<sample>/conditioned_clip_v1.npz` and the filed clips under
`clips/safe/` — all already conditioned, all free of branch flips. The
collision numbers quoted above were since measured on clips prepared straight
from the bundle by `prepare_clip.py`, which are not.

The flip path is still covered by the tests
(`tools/tests/test_sanitize_ik.py`) rather than by a clip, because
`prepare_clip.py` refuses a source with a >= 90 deg single-frame step before
it can become one: the three bundle trajectories that flip
(`02_..._GT`, `02_..._Ours`, `03_..._GT`) need `--allow-flips` to produce a
clip at all, and what that clip then contains is a flip a 6 Hz Butterworth has
already smeared into a ramp. The tests build a flipped trajectory out of two
genuinely distinct exact IK branches of this model instead.
