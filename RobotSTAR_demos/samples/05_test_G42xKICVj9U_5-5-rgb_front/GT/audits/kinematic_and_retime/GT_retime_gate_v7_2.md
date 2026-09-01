# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `260`
- output frames: `260`
- max waypoint geometry error: `7.216449660063518e-15` rad

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

- source: `{'arm_velocity_rad_s': 17.595023248265008, 'arm_acceleration_rad_s2': 439.8755812066252, 'left_hand_velocity_rad_s': 9.56835001707077, 'left_hand_acceleration_rad_s2': 225.679911673069, 'right_hand_velocity_rad_s': 9.994441270828247, 'right_hand_acceleration_rad_s2': 195.3759603202343, 'hand_velocity_rad_s': 9.994441270828247, 'hand_acceleration_rad_s2': 225.679911673069}`
- output: `{'arm_velocity_rad_s': 17.595023248265008, 'arm_acceleration_rad_s2': 3568.3806974464405, 'left_hand_velocity_rad_s': 9.56835001707077, 'left_hand_acceleration_rad_s2': 1174.4454366714285, 'right_hand_velocity_rad_s': 9.994441270828247, 'right_hand_acceleration_rad_s2': 1023.0885202956019, 'hand_velocity_rad_s': 9.994441270828247, 'hand_acceleration_rad_s2': 1174.4454366714285}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
