# Spec 1: unified position-replay command surface

**Status:** open questions, 2026-08-26. Not agreed, no code yet.

One entry point that plays a joint-position trajectory onto the G1_23 arms and
both Wuji Hand 2 units, from a file, on one clock. Not teleop; that is spec_2.
M1 is verified in MuJoCo, M2 on hardware.

## Settled

One process, one timer, all four `joint_commands` topics per tick. Separate arm
and hand captures get aligned at load time, not at publish time.

## M1 questions (sim)

1. Where do trajectories come from: recorded sessions, hand-authored keyposes,
   or an external generator? (Alex + Nathan)
2. Arms in joint space or task space? Nothing forwards
   `/{side}_arm/joint_commands` to DDS today, so a joint-space trajectory has no
   hardware path; task space means IK reinterprets what is replayed. (Nathan)
3. Does the surface own waist yaw, or arms plus hands only? (Alex + Nathan)
4. Keyposes with interpolation, or dense samples played frame by frame? (Alex)
5. Files name-keyed or positional, and what fills the DoF a file does not
   mention? (Alex)
6. Which joint-limit source is authoritative for clamping: MJCF `ctrlrange`, the
   driver's limit parameters, or [hardware_spec.md](hardware_spec.md)? (Alex)
7. Where does the rate limit live? DDS clips at 20 rad/s, the viewer and the
   hand driver clip nothing. (Alex)
8. Start, end, and abort behaviour: ramp to the first pose, hold or release at
   the end, and what stops a run. (Alex + Nathan)
9. Is a recorder in scope, or does `ros2 bag record` / `play` already cover M1?
   (Alex + Nathan)
10. Package name, and where the code lives. (Alex)

## M2 questions (hardware)

Blank on purpose. Alex + Nathan, once the replay procedure on the rig is agreed.

1. How a joint-space trajectory reaches DDS, given question 2.
2. arm_sdk weight: when it is set, how it is ramped, how it is released.
3. Hand `set_enabled` / `reset_error` call order, and a fault mid-trajectory.
4. What the physical abort is, and who holds it.
5. First trajectory to run: which joint, what amplitude, what speed.
6. How success is measured against `rt/lowstate` and `/{side}_hand/joint_states`.
7. What "M2 passed" means, in numbers.
