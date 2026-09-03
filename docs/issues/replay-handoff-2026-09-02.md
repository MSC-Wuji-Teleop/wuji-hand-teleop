# Replay implementation handoff, 2026-09-02

Where the spec1 build stands. Design and clip format:
[spec/spec1.md](../spec/spec1.md). Operator commands: [replay.md](../replay.md).

**Updated 2026-09-03.** Every piece is now written and the whole offline and sim
path has been run end to end on the real bundle. What is verified, and the three
findings that change what an operator should expect, are in
[Verified 2026-09-03](#verified-2026-09-03). Nothing has run on the rig.

## Committed

| commit | what | test |
|---|---|---|
| `2a76a4f` | wrist roll/yaw contact exclude in both `g1_29_wuji2` models (cherry-picked) | model loads, exclude present |
| `b2c18ec` | unitree CRC shared libraries in the G1 image (cherry-picked; the DDS write path dlopens them and the image dropped them) | needs an image rebuild |
| `ba05eb9` | `clips/{safe,rejected,candidate}` layout, gitignore rules, compose mounts for `../clips` (rw) and `../tools` (ro) | |
| `f35c600` | spec1, replay runbook, usage, architecture, README, CLAUDE.md rewritten for the clip-directory path, and this note | grep clean for stale terms |
| `5e24e4a` | `side_buffer.py` with tests; the node uses it; stale G1_29 DDS comments corrected | `pytest tests/test_side_buffer.py` 12 passed |
| `ec39b11` | `src/starport_wuji_hand/` vendored and pruned; the two out-of-repo tests re-pointed at `wujihand_urdf` and the composed MJCF | `pytest --noconftest test/{test_joint_map,test_safety,test_limits_match_mjcf}.py` 84 passed, 0 skipped |

Milestones M0, M1, M2, and the documentation pass are done. M3 and M4/M5 are
partial and left in the working tree.

## Working tree, not committed

All five rows below are now written and tested; the table records what each one is.

| piece | state |
|---|---|
| `tools/clip_audit.py`, `tools/tests/` | run end to end; 66 tests |
| `tools/prepare_clip.py` | the full CLI: sanitize, retarget, audit, judge, file, `--all`, `summary.md` |
| `replay/clip.py`, `replay_publisher.py`, `replay_check.py`, `replay/check.py`, `test/` | 88 tests; both entry points installed |
| `replay.launch.py`, `scripts/replay.sh` | one host command; 20 tests |
| `mujoco_visualizer.py`, `_mujoco_common.py` | mirror the driver command topics by name as well as the controller's positional ones; 29 tests |

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

## Verified 2026-09-03

The bundle was restored from `origin/nathan_dev` (997 files; 808 non-video files
match `MANIFEST.sha256`). Everything below ran in the teleop container.

| what ran | result |
|---|---|
| `prepare_clip.py --method-dir .../11_val_.../Ours --out clips` | 190 frames, retarget 2.5 s, three audits, filed rejected, 7 s total |
| `prepare_clip.py --all RobotSTAR_demos/samples --out clips` | 30 trajectories in 6 min: **4 safe, 23 rejected, 3 refused** (the three flips), `clips/summary.md` written |
| `clip_audit.py <clip> --video` | renders under `MUJOCO_GL=egl`; frames inspected and correct |
| `scripts/replay.sh clips/safe/90_sweep_joints_GT --sim` | one command: G1 container (dry run) + publisher + viewer; targets 50 Hz, commands 250 Hz, hand commands 50 Hz; holds the last frame; stops the G1 container on exit |
| the same at `--speed 0.25` | 82.3 s, clean exit |
| `scripts/replay.sh --check --arms none --hands both` | exits 1 after 20 s naming all four missing sources, drivers stop cleanly |
| all suites | 471 passed (tools 66, replay 88, bringup 20, g1 29, starport 268) |

Interpolation was measured on the live graph rather than argued: at `--speed 0.25`
the 250 Hz command stream advances 0.00007 rad per tick against a 0.00135 rad
target frame step, which is the 1/20 the buffer should produce, and only 37% of
consecutive samples repeat where a zero-order hold would repeat 95%.

### Three findings that change what to expect

**1. Wrist pitch and yaw are the binding constraint, and slowing down does not fix
them.** Those two joints per arm carry a 5 Nm `actuatorfrcrange` against 25 Nm
elsewhere, and the G1 node drives them at kp 50 / kd 2. On
`11_val_.../Ours` the peak torque ratio is 1.00 at 1.0x, 0.5x, 0.25x, 0.2x, 0.15x
and 0.1x, and only 0.84 at 0.05x. The load is a hand-to-hand contact reaction of
-5.0 to -5.9 Nm on right wrist pitch that barely changes with speed, so the
actuator sits on its clamp at any speed. A second, speed-dependent mechanism sits
underneath it: at kd 2 the damping term alone reaches 5 Nm at 2.5 rad/s, and the
sanitized arms still peak at 12.9 rad/s. The bundle authors' own
`audits/physical/*_physical_summary_v7_2.json` agree independently: **all 30
trajectories saturate a wrist pitch or yaw actuator** and only `08_..._Ours` and
`15_..._Ours` pass their deployment audit. Re-solving the arm retarget against
this model and nudging contacting frames are the real fixes, and spec1 already
calls them separate work items.

**2. A slower speed is not always a safer speed, and the publisher does not know
that.** `05_test_G42xKICVj9U_5-5-rgb_front_GT` passes at 0.5x (ratio 0.736) and
fails at both 1.0x (1.00) and **0.25x (0.846)**. `check_speed` only refuses a
speed above the fastest entry in `safe_speeds`, so `--speed 0.25` on that clip is
accepted today even though the audit rejected it. The rule should be that the
requested speed must *be* one of `safe_speeds`, not merely below the largest.
Until that lands, pass no `--speed` and take the clip's default.

**3. Four wrist joints are commanded past their model limits.** On
`11_val_.../Ours`: right wrist roll +2.009 against +1.972, right wrist pitch
+1.625 against +1.614, right wrist yaw +1.641 against +1.614, left wrist roll
-1.987 against -1.972, and the right wrist sits at a limit for 134 to 146 of 190
frames. spec1's sanitize step has no position clamp, by decision, because the
dynamic audit is the judge; the audit does reject these clips, so nothing is
silently passed. It is recorded here because it explains the shape of the
rejections.

### Smaller things found and fixed

- `hand_node.py`, `replay_clip.py` and `wave_check.py` called `rclpy.shutdown()`
  unconditionally in `finally`. Launch stops a node with SIGINT and rclpy's own
  handler has already shut the context down, so every clean stop raised
  `RCLError("rcl_shutdown already called")`, exited 1, and printed a traceback
  that made a normal Ctrl-C look like a driver crash. Guarded with `rclpy.ok()`,
  the same way `replay_publisher` already did it.
- The `wuji-sdk` pin moved to 2026.8.31 in `docker/Dockerfile`. 2026.5.26 has
  neither `DeviceType` nor `JointCommand`, which the Ethernet driver calls.
- The teleop image has not been rebuilt for that pin; the container currently
  carries it as a `pip --user` install. The rebuild is pending because the build
  context is 2.9 GB with no `.dockerignore` and the disk is at 98%.

### Fixed after the first pass

- **The publisher now refuses any speed that is not in `safe_speeds`**, slower
  ones included (finding 2). `scripts/replay.sh clips/safe/05_..._GT --speed 0.25`
  exits 2 with the reason instead of playing a speed the audit rejected.
- **`replay.sh` reports the publisher's exit code.** Humble's `ros2 launch`
  returns 0 even when a required process dies non-zero; the script already
  scraped the real code for `--check` and now does it for the publisher too. A
  refused clip used to reach the operator's shell as success.
- **`prepare_clip.py` reads a sample whose meta omits `source_frames`**, falling
  back to the keypoint array's own length, so `RobotSTAR_demos/sweep-test` runs
  straight from the read-only bundle with no staging copy.
- **`clips/summary.md` gained an `at` column** and reports a safe clip's numbers
  at the fastest speed that *passed*. `05_..._GT` used to read `safe` beside a
  torque ratio of 1.00, which was the 1.0x audit it had failed; it now reads 0.74
  at 0.5x.

### Network configuration reconciled against the other branches

Every tracked config was compared by content hash against `origin/main`,
`origin/alex_dev` and `origin/nathan_dev`. `cyclonedds.xml`, `entrypoint.sh`,
`setup_glove_routes.sh` and the four `.yaml.template` files are byte-identical on
all four, and the compose network settings (`network_mode: host`, `ipc: host`,
`cap_add: NET_ADMIN`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, the device cgroup
rules) match as well. One real gap turned up and is now closed:

**The G1's DDS NIC pin was missing.** `nathan_dev` carries
`network_interface: "enx00051bc62afa"` in `g1_robot.yaml` and threads it through
`config_loader.py` -> `g1_controller.py` -> `robot_arm.py` into
`ChannelFactoryInitialize(domain, nic)`. This branch had neither the value nor
the plumbing, so it called `ChannelFactoryInitialize(0)` and let the SDK take
the first interface. `CYCLONEDDS_URI` does not help here: the Unitree SDK builds
its own CycloneDDS config and ignores the environment variable, so on a
multi-NIC host that parameter is the only thing binding the robot link, and
getting it wrong shows up only as the lowstate timeout. The plumbing and the
value are ported, and `docs/spec/hardware_spec.md` again records the robot at
`192.168.123.161` on `192.168.123.0/24`, `mode_machine` 5, lowstate 1000 Hz and
`unitree-sdk2py` 1.0.1. The gains-profile and `limits_file` machinery that sits
beside it on `nathan_dev` was deliberately **not** ported: it is the supervisor
layer this design does without.

That NIC is a USB Ethernet adapter and it is not plugged into this host, so the
name cannot be confirmed here; `ip link` will show it once the G1 is connected.
Nothing in sim touches it, because `dry_run` never constructs the DDS
controller.

Two other subnets show up in a grep and are both benign: `192.168.40.x` appears
only as docstring examples in the vendored `starport_wuji_hand` scripts for
giving a hand a static address, and `192.168.50.127` belongs to the
`wuji-retargeting` submodule's own upstream examples.

### Both images rebuilt, 2026-09-03

`.dockerignore` added first: both images build with the repo root as context and
neither COPYs the source tree, so the daemon was tarring up ~2.9 GB (the bundle,
the run directories, `.git`) to read a few hundred MB of it. A throwaway build
confirmed every COPY path still resolves through the negations before the real
build ran.

- **teleop**: rebuilt; `wuji-sdk` 2026.8.31 now sits at
  `/usr/local/lib/python3.10/dist-packages/wuji_sdk`, not in a `pip --user`
  directory, and the driver's import check passes against it. The container was
  recreated and rebuilt its workspace (16 packages, 26 s). Every entrypoint
  health check reports OK.
- **g1_world_output**: rebuilt `--no-cache`; `crc_amd64.so` and
  `crc_aarch64.so` are in place.
- **The NIC pin is live, not just configured.** Started without `dry_run`, the
  node fails with `enx00051bc62afa: does not match an available interface` from
  inside `ChannelFactoryInitialize(domain, network_interface)`. That is the
  adapter being unplugged on this host, and it is the behaviour we want: a
  missing NIC stops the node by name instead of quietly binding Wi-Fi.

Re-verified on the rebuilt stack: the sweep clip regenerates from the read-only
bundle to the same numbers (safe at 1.0, 0.5 and 0.25; peak torque ratio 0.33;
zero contact force), all 475 tests pass, and
`scripts/replay.sh clips/safe/90_sweep_joints_GT --sim` runs end to end with no
traceback -- targets 50 Hz on both arms, commands 249 Hz, hand commands 50 Hz,
last frame held, G1 container stopped on exit.

### Still open

- `--check` cannot pass in sim: with `dry_run` the G1 controller has no
  `arm_ctrl`, so `/{side}_arm/joint_states` is never published. It is a
  hardware-only command, by nature rather than by defect.
- `network_interface` names a USB Ethernet adapter that is not plugged into this
  host, so the name itself is unverified. Check it with `ip link` once the G1 is
  connected; the node will say so by name if it is wrong.

## Next steps, in order

1. Make the publisher refuse a speed that is not in `safe_speeds` (finding 2), and
   fix the two smaller gaps under [Still open](#still-open).
2. Rebuild the teleop image so the `wuji-sdk` pin lands, after adding a
   `.dockerignore` (the context is 2.9 GB today).
3. On the rig, in this order: `scripts/replay.sh --check`, then
   `scripts/replay.sh clips/safe/90_sweep_joints_GT --arms left --hands none`,
   widening one device at a time as the bundle's own README recommends. The sweep
   clip is the first-motion clip: it passes at all three speeds with peak torque
   ratio 0.33 and zero contact force.
4. Do not expect the sign-language clips to be usable as they stand. Four of 30
   pass and every one of the 30 leans on a wrist joint (finding 1). Re-solving the
   arm retarget against this model is the work that would change that.
5. Commit `clips/safe/` and `clips/summary.md`.
