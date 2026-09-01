# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `390`
- output frames: `390`
- max waypoint geometry error: `8.43769498715119e-15` rad

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

- source: `{'arm_velocity_rad_s': 14.93134945764325, 'arm_acceleration_rad_s2': 292.36452124244533, 'left_hand_velocity_rad_s': 10.856622457504272, 'left_hand_acceleration_rad_s2': 303.2791242003441, 'right_hand_velocity_rad_s': 7.433591783046722, 'right_hand_acceleration_rad_s2': 206.48334175348282, 'hand_velocity_rad_s': 10.856622457504272, 'hand_acceleration_rad_s2': 303.2791242003441}`
- output: `{'arm_velocity_rad_s': 14.93134945764325, 'arm_acceleration_rad_s2': 4260.443795289313, 'left_hand_velocity_rad_s': 10.856622457504185, 'left_hand_acceleration_rad_s2': 1365.2082681894917, 'right_hand_velocity_rad_s': 7.4335917830466505, 'right_hand_acceleration_rad_s2': 994.5987496461773, 'hand_velocity_rad_s': 10.856622457504185, 'hand_acceleration_rad_s2': 1365.2082681894917}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
