# Replay implementation handoff, 2026-09-02

Where the spec1 build stands when the session ended. Design and clip format:
[spec/spec1.md](../spec/spec1.md). Operator commands: [replay.md](../replay.md).

Four implementers were working in parallel on disjoint file sets and were
stopped mid-run. Nothing in the working tree is committed except the three
commits below. Two milestones are finished and tested, two are partial.

## Committed

| commit | what |
|---|---|
| `2a76a4f` | wrist roll/yaw contact exclude in both `g1_29_wuji2` models (cherry-picked) |
| `b2c18ec` | unitree CRC shared libraries in the G1 image (cherry-picked; the DDS write path dlopens them and the image dropped them) |
| `ba05eb9` | `clips/{safe,rejected,candidate}` layout, gitignore rules, compose mounts for `../clips` (rw) and `../tools` (ro) |

## Working tree, not committed

**Finished and passing.**

| piece | state | test |
|---|---|---|
| `g1_world_output/side_buffer.py` + `tests/test_side_buffer.py` | done. Holds the newest sample until two real samples exist, then interpolates one publish period behind | `pytest tests/test_side_buffer.py` 12 passed |
| `g1_world_output_node.py`, `robot_arm.py`, package README | done. Node uses `SideBuffer`; the stale "G1_29 refuses DDS" comments corrected | `py_compile` clean |
| `src/starport_wuji_hand/` | vendored and pruned (rviz view, `description.py`, Beta 1 URDFs, udev, the three monorepo bench scripts). The two out-of-repo tests re-pointed at `wujihand_urdf` and the composed MJCF | `pytest --noconftest test/test_joint_map.py test/test_safety.py test/test_limits_match_mjcf.py` 84 passed, 0 skipped |
| docs | spec1, replay runbook, usage, architecture, README, CLAUDE.md all rewritten for the clip-directory path | grep clean for stale terms |

**Partial.**

| piece | state | what is missing |
|---|---|---|
| `tools/clip_audit.py` (681 lines) | imports; constants and `audit_clip`, `render_video`, `load_clip_dir`, `main` present | never run end to end. No `tools/tests/` |
| `tools/prepare_clip.py` | **not written** | the whole CLI: sanitize, retarget, judge, filing, `--all`, `summary.md` |
| `replay/clip.py` (273 lines) + `replay_publisher.py` (187 lines) | both import; publisher rewritten for the clip directory, `--loop` gone | `test/` holds only `__init__.py`. No `replay_check.py`, no `setup.py` entry point, no package README |
| `replay.launch.py`, `scripts/replay.sh` | **not written** | both |
| `mujoco_visualizer.py`, `_mujoco_common.py` | **untouched** | subscribe the two driver command topics for `--sim` |

## Decisions taken this session, beyond the spec as written

These are in the docs already; they are listed here because they are the
reason parts of the code look different from the spec's first draft.

1. `--loop` is dropped everywhere.
2. `peak_contact_force_n` is the norm of the 3D force part of
   `mj_contactForce`, not the normal component alone.
3. Fractions are per clip frame: a frame counts when any physics step inside
   its period met the condition.
4. `--auto-trim` audits the full clip, takes the fastest speed with a passing
   window of at least `--min-seconds`, trims, and re-audits at every speed.
   The second audit is what `clip.json` records.
5. The publisher refuses a directory that is not under `clips/safe/`, on top
   of the verdict check.
6. `--check` waits up to 20 s for every selected source, prints the rates, and
   exits 1 on timeout. `--arms` and `--hands` narrow it.
7. The G1 buffer holds the newest sample until two real samples exist, so the
   first frame is a step and not a ramp over however long the node idled.
8. The hand controller is not edited: the URDF-order permutation is
   reimplemented in the offline tool.
9. The USB driver (`wujihandros2`) stays until the Ethernet driver runs on the
   rig.
10. Arm re-gain in the audit is documented in spec1 and lives as named
    constants at the top of `clip_audit.py`.

## Facts established, so they need no rechecking

- `scipy==1.14.1` is already in the teleop image (Dockerfile section 5).
- Humble `launch_ros` accepts `list[float]` as a `ParameterValue` type
  (`is_typing_list` checks `__origin__ in (list, List)`), so the driver's
  launch file needs no change.
- The composed model's arm joints carry `actuatorfrcrange` and MuJoCo clamps
  `qfrc_actuator` with it; `actuator_force` is unclamped and must not be read
  as applied torque.
- Hand actuator `ctrlrange` equals the joint range on all 40 hand actuators,
  and the limits yaml in the driver matches the model's hand joint ranges for
  all 20 joints.
- The driver's hardware joint order equals the movable-joint declaration order
  of `wujihand_urdf` and the hand joint order in the composed MJCF.
- Three of the 30 bundle trajectories have a single-frame arm step of 90 deg or
  more and will be refused without `--allow-flips`: 02 GT (185 deg), 02 Ours
  (186 deg), 03 GT (91 deg).
- The bundle's `body_actuators` names carry no `_joint` suffix; the MJCF's do.
  A raw `mj_name2id` on the bundle names returns -1, which silently indexes the
  last joint.
- Retargeting costs about 8 ms per frame per hand outside the container, so a
  930-frame clip is about 15 s of retargeting; the audit runs at about 18k
  physics steps per second.

## Next steps, in order

1. Write `tools/prepare_clip.py` and `tools/tests/`, then run one clip
   (`11_val_a5yNwUSiYpA_9-3-rgb_front/Ours`) end to end and look at the
   rendered frames before trusting the numbers.
2. Finish the online half: `replay_check.py`, the replay package's tests,
   `setup.py` entry points, README, `replay.launch.py`, `scripts/replay.sh`,
   and the viewer's two new subscriptions.
3. In the container: `colcon build --symlink-install`, then
   `python3 -c "import wuji_sdk; wuji_sdk.DeviceType.WujiHand2; wuji_sdk.JointCommand"`
   (bump the pin if it fails), `colcon test`, `scripts/replay.sh --check`,
   `scripts/replay.sh clips/safe/<clip> --sim`.
4. Rebuild the G1 image so the CRC libraries land.
5. `prepare_clip.py --all` in the container, then commit `clips/safe/` and
   `clips/summary.md`.
