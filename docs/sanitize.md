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
    │  tools/sanitize_clip.py    THIS STAGE
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
| `unwrap.wrist_drift` | Each wrist joint's end-to-start rotation | Past 90° it prints `DRIFT`. Your call per clip |
| `flips.detected` / `gated_frames` | Branch flips found, and frames run without the elbow task | A large gated count means the source elbow is untrustworthy |
| `ik.max_wrist_*_residual` | Worst pose error against what the source implied | ~0 on a clean clip. Millimetres mean limits, the step bound or a contact got in the way |
| `ik.frames_at_joint_limit` / `at_step_bound` | Frames each joint spent on a bound | Usually a source outside the URDF range (`source.arm_values_outside_joint_limits`) |
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

## Runtime and tests

~0.25 s per frame on a mostly clear clip, ~1.2 s on one in contact throughout
(5m19s for 260 frames). One process per clip, single-threaded; six at a time
on 8 cores keeps the cores busy but stretches per-clip latency, so run it
detached.

```bash
python3 -m pytest tools/tests/test_sanitize_*.py -q     # 72 tests, ~10 s
```

Needs Pinocchio, coal and the URDF; skips cleanly without them.
