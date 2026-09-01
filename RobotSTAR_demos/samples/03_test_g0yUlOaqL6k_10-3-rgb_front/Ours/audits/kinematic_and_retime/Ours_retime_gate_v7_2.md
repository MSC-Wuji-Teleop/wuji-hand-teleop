# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `410`
- output frames: `410`
- max waypoint geometry error: `1.709743457922741e-14` rad

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

- source: `{'arm_velocity_rad_s': 53.57720175324339, 'arm_acceleration_rad_s2': 1294.7247463881765, 'left_hand_velocity_rad_s': 5.388783882551544, 'left_hand_acceleration_rad_s2': 45.654883628315694, 'right_hand_velocity_rad_s': 6.6441413934938875, 'right_hand_acceleration_rad_s2': 57.08812234390764, 'hand_velocity_rad_s': 6.6441413934938875, 'hand_acceleration_rad_s2': 57.08812234390764}`
- output: `{'arm_velocity_rad_s': 53.57720175324339, 'arm_acceleration_rad_s2': 20243.12460646309, 'left_hand_velocity_rad_s': 5.388783882551544, 'left_hand_acceleration_rad_s2': 75.7583364300724, 'right_hand_velocity_rad_s': 6.6441413934938875, 'right_hand_acceleration_rad_s2': 166.6680907510937, 'hand_velocity_rad_s': 6.6441413934938875, 'hand_acceleration_rad_s2': 166.6680907510937}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
