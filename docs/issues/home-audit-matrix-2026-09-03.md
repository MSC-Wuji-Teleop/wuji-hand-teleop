# Rehome audit matrix, 2026-09-03

What the home motion does from realistic starting poses, measured rather than
argued. Design: [spec/spec1_1.md](../spec/spec1_1.md). This is the table that
settles the home pose and the path shape.

## How it was produced

Eight start poses, both home candidates, both assumed hand poses, one speed.
Each row is one `tools/make_home_clip.py` run, which generates the clip and
audits it with `tools/clip_audit.py` on `g1_29_wuji2_fixed.xml` at the G1
node's gains (kp 140 / kd 3, wrists 50 / 2), hands actuated, MuJoCo 3.12.0.

```bash
python3 tools/make_home_clip.py --start-pose <SPEC> --home-pose zeros|stand --out clips
```

Start pose specs: `stand`, `zeros`, `clip:clips/safe/<clip>@last` for each of
the five committed clips, and one synthetic folded pose, arms across the chest
with shoulder roll driven fully inward and the elbows at 2.0 rad:

```
0.3 -1.5882 0.0 2.0 0.0 0.0 0.0   0.3 1.5882 0.0 2.0 0.0 0.0 0.0
```

Columns. `secs` is the realised duration, sized by the largest joint travel at
a 0.2 rad/s peak. `ratio` and `contact` are the worse of the two hand poses.
`rise` is peak contact in the first second minus contact at frame 0: positive
means the motion pressed harder than the pose it started from, which is the
folded-start question. Thresholds are the audit's defaults, torque ratio 0.8
and 80 N.

## The table

| start pose | home | verdict | secs | torque ratio | contact (N) | rise (N) | worst pair |
|---|---|---|---|---|---|---|---|
| stand keyframe | zeros | safe | 10.1 | 0.28 | 2.1 | +0.0 | left_hip_roll / L pinky_distal |
| stand keyframe | stand | safe | 3.0 | 0.07 | 0.4 | +0.0 | R index_finger_proximal_abd / R thumb_middle |
| all-zeros | zeros | safe | 3.0 | 0.28 | 0.5 | +0.0 | R ring_finger_proximal_abd / R thumb_distal |
| all-zeros | stand | safe | 10.1 | 0.27 | 3.4 | +0.0 | left_hip_roll / L pinky_distal |
| folded against torso | zeros | **rejected** | 15.7 | 1.00 | 133.3 | +5.6 | right_shoulder_yaw / torso |
| folded against torso | stand | **rejected** | 14.1 | 1.00 | 54.5 | +5.7 | left_shoulder_yaw / torso |
| 90_sweep_joints_GT | zeros | safe | 3.0 | 0.28 | 0.5 | +0.0 | R ring_finger_proximal_abd / R thumb_distal |
| 90_sweep_joints_GT | stand | safe | 10.1 | 0.27 | 3.4 | +0.0 | left_hip_roll / L pinky_distal |
| 05_test_G42xKICVj9U_5-5-rgb_front_GT | zeros | safe | 15.5 | 0.33 | 5.1 | +0.0 | L thumb_proximal_abd / R mount |
| 05_test_G42xKICVj9U_5-5-rgb_front_GT | stand | safe | 15.5 | 0.32 | 6.4 | +0.0 | right_hip_roll / R pinky_distal |
| 13_val_39FN42e41r0_0-1-rgb_front_Ours | zeros | safe | 15.5 | 0.34 | 0.8 | +0.0 | R index_finger_proximal_abd / R thumb_middle |
| 13_val_39FN42e41r0_0-1-rgb_front_Ours | stand | safe | 15.5 | 0.36 | 13.1 | +0.0 | right_hip_roll / R pinky_middle |
| 15_val_x-f1_kdl050s_10-1-rgb_front_GT | zeros | safe | 15.5 | 0.34 | 0.8 | +0.0 | R index_finger_proximal_abd / R thumb_middle |
| 15_val_x-f1_kdl050s_10-1-rgb_front_GT | stand | safe | 15.5 | 0.29 | 8.5 | +0.0 | left_hip_roll / L pinky_distal |
| 15_val_x-f1_kdl050s_10-1-rgb_front_Ours | zeros | safe | 15.5 | 0.34 | 0.8 | +0.0 | R index_finger_proximal_abd / R thumb_middle |
| 15_val_x-f1_kdl050s_10-1-rgb_front_Ours | stand | safe | 15.5 | 0.29 | 9.0 | +0.0 | right_hip_roll / R pinky_distal |

## What it decides

**1. Home stays all-zeros.** Four of the eight start poses are degenerate for
one candidate or the other, because a home equal to the start is a 3 s hold. Of
the four real comparisons (the sign-language clip end poses), all-zeros gives
the lower peak contact in three and is within 1.3 N in the fourth: 0.8 N against
13.1, 0.8 against 8.5, 0.8 against 9.0, and 5.1 against 6.4. Torque ratios are
indistinguishable, 0.27 to 0.36 everywhere. The stand home is what produces the
hip-to-pinky contacts, not the zeros home, which is the opposite of what was
expected when the pose was chosen: the hands pass the thighs on the way to
either pose, and stand's 1.28 rad elbow holds them there longer.

**2. A straight interpolation does not press harder, unless it starts in
contact.** The rise column is +0.0 N on every safe row. It is +5.6 N on the two
folded rows, and those start at 42.6 N of contact before anything moves, with
the torque ratio already at 1.00 at frame 0.

**3. There is no path shape that rescues a folded start, and none was kept.** A
retract waypoint (move shoulder roll outward first, then descend) was built and
measured. It changes nothing: 133.3 N with it against 133.4 N without, from a
folded pose with roll headroom, and 57.7 against 54.5 from one at the roll
limit, which is slightly worse. The dominant pair is `shoulder_yaw_link` against
`torso_link`, which abduction does not relieve. The waypoint was removed rather
than kept as an unused option.

So the answer for a folded start is the refusal itself: the generator files the
clip under `clips/rejected/`, exits 2, and the publisher never runs. Damp the
robot from the remote and part the arms by hand. That is the documented
behaviour, not a gap.

## What this does not establish

The audit's own limits, unchanged from spec1: fixed base, our gains, no harness
or tether, no knowledge of real contact stiffness or of the unconfirmed Hand 2
mount adapter. The two hand poses are stand-ins, not measurements, and the hands
are limp during a real rehome rather than actuated as modelled. The synthetic
folded pose is one pose, chosen to be bad; it says what happens from a start
already in hard contact, not how likely that start is.

Nothing here has run on the rig.
