# Clip sanitization pipeline

`tools/sanitize_clip.py` re-solves a prepared clip's **arm** joints against
`g1_29_wuji2.urdf` with continuity, joint limits and full-mesh collision
checking, and writes the same npz arrays back. npz in, npz out: the result
drops into the replay path.

Offline only — no ROS, no DDS, no hardware. Runs in the `teleop` container
(Pinocchio + coal); nothing in it imports `mujoco`.

Spec: [spec/RoboSTAR_dropin_fix.md](spec/RoboSTAR_dropin_fix.md). Every
measured constant: [`../tools/sanitize/README.md`](../tools/sanitize/README.md).

## Where it sits

```
RobotSTAR_demos/samples/<sample>/{GT,Ours}
    │  tools/prepare_clip.py     smooth arms, retarget hands, audit in MuJoCo
    ▼
clips/{safe,rejected}/<clip>/    arm_q.npz, hand_q20.npz, clip.json
    │  tools/sanitize_clip.py    THIS STAGE. Mandatory for a bundle clip:
    │                            it re-clocks the wrists (see Wrist clock)
    ▼
<out>/<clip>/                    same layout + sanitize.json
    │  tools/clip_audit.py       re-audit. Mandatory (see below)
    ▼
clips/safe/<clip>/               filed, playable
```

Hand joints pass through untouched (clamped to the URDF limits, nothing else) —
regenerating them is the retargeter's job. Legs and waist are read, never
written.

## The three defects

| Defect | What the tool does |
| --- | --- |
| Wrists phase through each other; nothing on the replay path checks | Sweeps 3576 URDF geometry pairs per frame with coal and constrains the frame's IK with what it finds. MuJoCo sees these contacts too, but reports a reaction force, not an overlap: `clip_audit.py` can say a pair pushed with 55 N, never that it interpenetrated |
| Branch flipping, torque and velocity spikes | Solves each frame from the previous one, shoulder→elbow then wrist, step-bounded by the URDF velocity limit. A joint jump the wrist pose did not follow is a flip: the source elbow is distrusted for its span |
| Wrist drift, hands inverted by the end | Measured and reported. Correction is opt-in (`--max-drift-deg`): a sign sequence need not end where it began |

## Running it

```bash
python3 tools/sanitize_clip.py clips/safe/<clip> -o clips/candidate/<clip>
python3 tools/sanitize_clip.py IN.npz -o OUT.npz     # flat npz, auto-detected
python3 tools/sanitize_clip.py clips/safe/<clip> --check        # measure only
python3 tools/sanitize_clip.py clips/safe/<clip> -o /tmp/x --no-collision

python3 tools/clip_audit.py <out>/<clip>             # then always this
```

A clip directory gets `sanitize.json` beside it and a `"sanitize"` block in
`clip.json`; a flat npz gets `OUT_sanitize.json`. Arm columns come back in the
input's order, and unknown keys pass through.

## Reading the report

| Field | Means | Act on it? |
| --- | --- | --- |
| `unwrap.wraps_removed` | 2π jumps taken out, only where the result still fits the joint's range | No |
| `unwrap.wrist_drift` | Each **source** wrist joint's end-to-start rotation, measured before the re-clock. On a re-clocked clip the output's wrist joints split the same wrist rotation differently (pitch and yaw roughly trade places, roll differs too), so read it as the source's drift | Past 90° it prints `DRIFT`. Your call per clip |
| `flips.detected` / `gated_frames` | Branch flips found, and frames run without the elbow task | A large gated count means the source elbow is untrustworthy |
| `ik.max_wrist_*_residual` | Worst pose error against what the source implied | ~0 on a clean unclocked clip; millimetres then mean limits, the step bound or a contact got in the way. On a re-clocked bundle clip a few mm of wrist position are inherent (median 1 to 9 mm, max 4 to 17 mm on the safe clips with collision off; elbow 18 to 45 mm): the rotated wrist placement and the source elbow position have no common 7-joint solution, and stage 3 splits the difference (wrist 1, elbow 0.5) while holding orientation to about 0.01 deg |
| `ik.frames_not_converged` | Frames whose solve did not reach the 1e-6 m / 1e-6 rad test | Meaningful only on an unclocked clip. On a re-clocked clip it is near the frame count for the reason above; judge the clip on the residual maxima, not this count |
| `ik.frames_at_joint_limit` / `at_step_bound` | Frames each joint spent on a bound | Usually a source outside the URDF range (`source.arm_values_outside_joint_limits`) |
| `wrist_clock.applied` / `deg` / `reason` | Whether the extracted wrist placements were re-clocked, by how much per side, and why | On a bundle clip `applied` must be true; see Wrist clock. Absent: the report was written before 2026-09-11 and the clip is not re-clocked |
| `hand.clamped_values` | Hand angles clamped into range | Fix the retargeter, not this |
| `collision.failures` | Frame, time, pair and state for every defect left | The decision list. Exit 3 |
| `collision.near_miss` | Inside the clearance, never touching | Normally nothing — signing brings the hands close on purpose |
| `collision.unfixable` | `intra_hand`: two segments of one hand | Nothing here can move them |
| `collision.frames_constrained` | Frames whose solve carried separation rows | How much of the clip is in contact |

Classes: `cross_side`, `arm_body` and `arm_self` are fixable by moving arms.
`intra_hand` is not.

A contact is either **touching** or a **near miss** with a gap. There is no
depth: coal returns 0.0 distance the moment meshes meet, however deep they go.

## Clearance is part of the solve

Each contact becomes one separation row inside **stage 3 of the frame's IK**,
weighted 3 against pose tasks weighted 1, re-measured every iteration. The
active pair set carries 15 frames, so a pair that just touched constrains the
next frame's first solve — the arm is held out of contact, not pushed out of
it. A frame that moves into an unknown pair is solved again with it added.

The bound is not optional: a touching pair asks for the full clearance every
iteration, so unbounded the solve walks off the pose for nothing (48° of
travel, 41 mm of wrist error, and 53 contacts where it started with 17).
`--avoid-budget-deg` (5°) is a trust region around the frame's pose-faithful
answer.

`05_test_G42xKICVj9U_5-5-rgb_front_Ours`, 260 frames, re-swept independently:

| | frames with a fixable touch | cross-side touching pair-frames | new arm/body contacts |
| --- | --- | --- | --- |
| source | 70 | 1770 | — |
| sanitized | **57** | **757** | **0** |

Worst wrist residual 19.1 mm / 3.5°, step out unchanged at 15°/frame. What it
does not fix: 1585 intra-hand pair-frames, and so the MuJoCo verdict — the
clip stays rejected at all three speeds. Arm saturation drops (0.104 → 0.027
at 1x). Hands inside each other are not an arm problem.

## Wrist clock

The bundle's arm joints were solved against its authors' model, which mounts
the hand on the G1 wrist 90 deg from this rig's adapter about the forearm
axis. Replayed as shipped they reproduce the bundle's wrist *link* and put the
*hand* 90 deg off, left and right in opposite senses
([wrist-clock-2026-09-11.md](issues/wrist-clock-2026-09-11.md)).

This stage corrects it where the geometry lives: each frame's extracted
`wrist_yaw_link` placement is rotated about its own +x by the clock angle
before the arm is re-solved, so the IK puts the hand where the bundle meant
it and keeps the wrist where the bundle put it. The angles are
`reclock.BUNDLE_WRIST_CLOCK_DEG`, +90 left and -90 right. It is not a
`wrist_roll` offset: the G1 wrist is roll, pitch, yaw along that axis, so the
correction also swaps the roles of pitch and yaw. Adding 90 deg to roll is
off by a median 88 deg over the bundle.

| `--wrist-clock` | Does |
| --- | --- |
| `auto` (default) | Applies the bundle clock when `clip.json` `source.detected_hand_model` is `legacy_wuji` (`prepare_clip.py` writes it from `target_meta.json`); otherwise nothing |
| `bundle` | Forces the bundle clock, for a clip prepared before the key existed |
| `none` | Forces it off |
| `--wrist-clock=LEFT_DEG,RIGHT_DEG` | Any two angles, for a source solved against some other mount. The `=` form is needed when `LEFT_DEG` is negative, or argparse reads the value as a flag |

The report says what happened under `wrist_clock`, and the stderr summary
prints one `wrist clock:` line. A flat npz has no `clip.json`, and a clip
directory whose `source` block lacks `detected_hand_model` or carries null
(the sweep clip, `90_sweep_joints_GT`) passes through unchanged unless the
flag says otherwise.

What the re-clock costs. The rotated wrist placement and the source elbow
position no longer share an exact 7-joint solution, so every re-clocked
bundle clip carries a small wrist position residual with nothing binding:
median 1 to 9 mm and max 4 to 17 mm over the safe clips with collision off,
with the elbow 18 to 45 mm off its source position and orientation held to
about 0.01 deg. Where a wrist range binds (about 7 percent of side-frames
over the bundle, concentrated in six trajectories) the IK gives up
orientation as well, 13 to 27 deg on the right hand of `02_test Ours` and
`10_val Ours`; where several wrist joints pin at once (`02_test GT`,
`03_test GT` and `Ours`, `14_val Ours`) it gives up centimetres of position
too. Read both `ik.max_wrist_pos_residual_mm` and
`ik.max_wrist_ori_residual_deg` at filing time.

## Re-audit before filing. Always

The tool writes no verdict, and copies `verdict`, `safe_speeds` and `audit`
through from the input — where they describe the input's joint data.
`replay/clip.py::load_clip` reads exactly those fields.

So: re-audit, decide, update `clip.json`, then move the clip. Write the output
under `clips/candidate/` while you do, because `load_clip` refuses a clip whose
parent directory is not one it recognises.

## Exit codes

| | |
| --- | --- |
| 0 | clean |
| 3 | written and playable, N frames not cleared — a person decides |
| 2 | refused: bad arguments, or a model not collision-free at rest |
| 1 | unexpected error |

Exit 3 is normal on real bundle clips.

## Tuning

| Flag | Default | Change it when |
| --- | --- | --- |
| `--clearance-mm` | 5.0 | Rarely |
| `--fail-on` | `touch` | `clearance` also treats near misses as defects (~2 s/frame) |
| `--separation-weight` | 3.0 | Higher trades more pose for clearance; 10 measured worse |
| `--avoid-budget-deg` | 5.0 | Lower to protect the pose harder |
| `--avoid-passes` / `--avoid-iters` | 3 / 12 | Runtime; each pass is a 200 ms sweep |
| `--max-step-deg` | none | `15` reproduces `prepare_clip.py`'s clamp |
| `--max-drift-deg` | inf | You have decided the end pose is drift, not intent |
| `--no-collision` | off | Fast pose-fidelity check, ~1.5 s a clip |
| `--wrist-clock` | `auto` | `bundle` for a clip whose `clip.json` predates the provenance key; `none` to inspect a bundle clip unclocked; `--wrist-clock=L,R` for another mount (the `=` form when `L` is negative) |

## Runtime and tests

~0.25 s per frame on a mostly clear clip, ~1.2 s on one in contact throughout
(5m19s for 260 frames). One process per clip, single-threaded; six at a time
on 8 cores keeps the cores busy but stretches per-clip latency, so run it
detached.

```bash
python3 -m pytest tools/tests/test_sanitize_*.py -q     # 88 tests, ~10 s
```

Needs Pinocchio, coal and the URDF; skips cleanly without them.
