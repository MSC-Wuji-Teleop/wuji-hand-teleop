# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `410`
- output frames: `410`
- max waypoint geometry error: `1.73749903353837e-14` rad

## Path checks

| Check | Result |
|---|---|
| `simulation_reference_generated` | PASS |
| `no_disallowed_isolated_flip` | PASS |
| `original_waypoints_unmodified` | PASS |
| `no_fixed_rotation` | PASS |
| `no_joint_bias` | PASS |
| `ik_not_rerun` | PASS |
| `waypoint_error` | PASS |

## Rate diagnostics

- source: `{'arm_velocity_rad_s': 43.33629798824357, 'arm_acceleration_rad_s2': 1083.4074497060892, 'left_hand_velocity_rad_s': 8.371031284332275, 'left_hand_acceleration_rad_s2': 228.7742868065834, 'right_hand_velocity_rad_s': 11.418366432189941, 'right_hand_acceleration_rad_s2': 271.3773213326931, 'hand_velocity_rad_s': 11.418366432189941, 'hand_acceleration_rad_s2': 271.3773213326931}`
- output: `{'arm_velocity_rad_s': 43.33629798824357, 'arm_acceleration_rad_s2': 20960.81796538752, 'left_hand_velocity_rad_s': 8.371031284332275, 'left_hand_acceleration_rad_s2': 1107.5410553060328, 'right_hand_velocity_rad_s': 11.418366432189941, 'right_hand_acceleration_rad_s2': 2048.7844868541824, 'hand_velocity_rad_s': 11.418366432189941, 'hand_acceleration_rad_s2': 2048.7844868541824}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
