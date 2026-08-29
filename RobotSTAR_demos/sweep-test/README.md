# sweep-test: a joint-sweep sample as a drop-in bundle clip

A synthetic sample in the exact RobotSTAR bundle format, so the joint sweep
funnels through the SAME pipeline as every real clip — conditioning, load
gates, replay stack, MuJoCo — with no runner of its own (this replaces the
retired standalone `sweep_and_visualize.py` flow). One generator produces
the npz; from there on, only existing architecture runs it.

## Contents

- `generate_sweep_sample.py` — the one generator. Writes
  `samples/90_sweep_joints/GT/` (bundle format:
  `g1_reference/controller_reference_v7.npz`, `g1_reference/target_meta.json`,
  `hand2_input/sweep_human_targets_v5.npz`) plus `MANIFEST.sha256` so
  `condition_clip`'s input-hash gate verifies the files (this folder is its
  own bundle root). Options: `--arm-amplitude`, `--donor`, `--arm-limits`.
- `check_collisions.py` — offline kinematic MuJoCo contact audit of any
  `conditioned_clip_v1.npz` (exit 0 clean / 2 collisions). Not a runner —
  it validates the artifact that will replay.
- `samples/90_sweep_joints/GT/` — the generated sample.

## What the clip does (~21 s, arms first, stop, then hands)

- **Arms phase** (hands hold a constant donor pose): the left arm's 7
  joints ramp together 0 → A → 0 as one small joint group (TUITION Stage B
  allows "one joint or one small joint group"), then the right arm's 7.
  Legs and waist stay exactly zero (the waist gate requires it).
- **Stop**: a full-second neutral hold.
- **Hands phase** (arms at zero): the **thumb** keypoints blend toward a
  second donor frame and back — left hand, hold, then right hand — so the
  thumb visibly flexes through the **production retargeter** (bundle hands
  are keypoints by design; there is no joint-space hand track to author in
  this format).

Safety: arm amplitudes are capped (default 0.2 rad) and clipped 10% inside
the URDF position ranges; ramp durations put peak velocity/acceleration at
50% of the deploy screening rows. None of that is trusted: the pass verdict
is **earned** through `condition_clip`'s standard audit (retargeted hands
included), and `check_collisions.py` proves the conditioned artifact is
kinematically self-collision-free.

### Why thumb-only, and why these fractions

Two measured constraints (2026-08-29, sample-05 donor):

1. **Collisions**: whole-hand blends press adjacent fingers into each
   other — the donor's sign pose holds fingers together (the neutral pose
   already has fingertip contacts), and the collision audit flags 17
   finger-on-finger pairs up to 4.3 mm deep. The thumb has lateral
   clearance, so thumb-only motion is contact-free.
2. **Retargeter jumps**: the retargeter emits solver jumps whose FD
   velocity/acceleration do not shrink with slower input (full-frame
   blends: ~10–25 rad/s sustained, 700–1400 rad/s² peak for 12–28 s
   cycles — vs the 4.0 rad/s / 20 rad/s² deploy rows). The chosen blends
   are the largest that stay under the rows with ~2× margin: left thumb →
   max-displacement frame at 0.6 (accel 10.8 rad/s², 1.09 rad excursion);
   right thumb → median-displacement frame at 0.5 (11.9 rad/s², 0.37 rad —
   the max-displacement target crosses a solver boundary at any useful
   fraction).

Larger per-joint hand motion goes through the pipeline's existing
single-joint path instead:

```bash
ros2 run replay condition_clip --single-joint left_hand:thumb_mcp \
    --amplitude 0.4 --out-dir ~/wuji_clips
```

(all 20 joint names: `wujihand_output/config/hand_limits.yaml`; every
joint has ≥ 0.6 rad of room around neutral, so 0.4 rad is safe everywhere).

## Usage

```bash
# 0. (re)generate — HOST shell, repo root (RobotSTAR_demos is mounted
#    read-only in the container). Already checked in; rerun only to change it.
python3 RobotSTAR_demos/sweep-test/generate_sweep_sample.py

# everything below: teleop container, workspace root
# 1. condition it exactly like any bundle sample
ros2 run replay condition_clip \
    --method-dir RobotSTAR_demos/sweep-test/samples/90_sweep_joints/GT \
    --out-dir ~/wuji_clips

# 2. collision audit on the conditioned artifact (the thing that replays)
python3 RobotSTAR_demos/sweep-test/check_collisions.py \
    ~/wuji_clips/90_sweep_joints_GT/conditioned_clip_v1.npz

# 3. replay through the normal gates (T1 g1 node + T2 replay_sim running)
ros2 run replay run_ctl load \
    ~/wuji_clips/90_sweep_joints_GT/conditioned_clip_v1.npz \
    --speed 1.0 --operator <name>
ros2 run replay run_ctl arm && ros2 run replay run_ctl start
# ... clip end -> park -> release, as in docs/spec/spec_1_stage0.md step 3
```

The sample is named `90_*` so it can never collide with the bundle's `01_`
first-clip ban or be mistaken for shipped data.
