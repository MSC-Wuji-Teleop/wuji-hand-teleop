# Sanitized RobotSTAR clips

Bundle trajectories from `RobotSTAR_demos/samples/` are hardware no-gos as
shipped (`real_robot_ready: false`; see
[spec_1_bringup.md](../docs/spec/spec_1_bringup.md), campaign status).
The method dirs here are sanitized copies of the least-bad bundle
trajectories: the first bundle-content candidates for the Stage C/D/E path
that currently runs on the synthetic sweep clip.

Produced by [tools/sanitize_robotstar_clip.py](../tools/sanitize_robotstar_clip.py):
zero-phase low-pass (6 Hz) plus per-frame rate clamp (15 deg) on the 14 arm
columns, optional head trim, dq/ddq recomputed. Legs, waist, and the
`hand2_input/` keypoints pass through untouched; `condition_clip`
regenerates Hand 2 fingers from the keypoints as usual. Exact settings and
before/after stats: `sanitize_report.json` in each clip dir. The tool
refuses clips with a >=90 deg single-frame step (estimator orientation
flips); those need the reference re-solved, not smoothed, and stay no-gos.

## Clips and their standing

Verified by kinematic replay on the composed 29-DoF model *with the
2026-09-01 wrist roll/yaw contact excludes* (fingers at neutral; the
finger-inclusive check happens at conditioning, below):

| clip | frames | max arm step | body-body contact | notes |
|---|---|---|---|---|
| `11_val_..._Ours` | 190 | 15.0 deg | none | cleanest clip in the bundle |
| `13_val_..._Ours` | 290 | 15.0 deg | none | first 30 frames trimmed (all contact lived there) |
| `08_train_..._Ours` | 210 | 15.0 deg | 8% of frames, 16 mm peak | cross-hand wrist press; passed the bundle's own 80 N gate natively (60 N) |
| `04_test_..._Ours` | 150 | 13.3 deg | 24% of frames, 12 mm peak | spare; more contact than 08 |

Run order: 11, 13, then 08 only if its conditioned artifact clears the
collision check below. 04 is a spare, same condition.

Deviation from TUITION 7D (GT before Ours): all four are Ours. No GT
trajectory in the bundle is contact-free after sanitizing (best GT: 06 GT,
21 mm residual). Running Ours-only skips the GT-vs-Ours comparison Stage D
wants; record it as a scoped exception or sanitize 06 GT and accept the
residual.

## Rig procedure

Same path as the sweep clip
([spec_1_bringup.md](../docs/spec/spec_1_bringup.md), "Stage C/E with the
sweep clip"), with these method dirs:

```bash
# T3, teleop container, one per clip
ros2 run replay condition_clip \
    --method-dir sanitized_clips/11_val_a5yNwUSiYpA_9-3-rgb_front_Ours \
    --out-dir ~/wuji_clips

# gate 1: verdict pass (exit 0) and read the k / speed scale it chose
# gate 2: finger-inclusive kinematic collision check on the artifact
python3 RobotSTAR_demos/sweep-test/check_collisions.py \
    ~/wuji_clips/<clip>/conditioned_clip_v1.npz

# then Stage C scoped loads at the artifact's allowed speed scale,
# per the spec's run_ctl sequence (arms left; right; both; then hands)
```

Both gates must pass before `run_ctl arm`. The conditioned artifact's
`max_allowed_speed_scale` binds, as everywhere else: these clips fail
`safe_timing_at_requested_scale` at 1x like the whole bundle, so expect a
large slowdown factor.

Input hashes for files under this directory are recorded by
`condition_clip` as out-of-bundle (not in `MANIFEST.sha256`); the
`sanitize_report.json` in each dir is the provenance link back to the
bundle sample it came from.
