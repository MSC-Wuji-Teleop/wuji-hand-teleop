# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `760`
- output frames: `760`
- max waypoint geometry error: `4.318767565791859e-14` rad

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

- source: `{'arm_velocity_rad_s': 96.20739422957038, 'arm_acceleration_rad_s2': 2800.394464719247, 'left_hand_velocity_rad_s': 13.39472234249115, 'left_hand_acceleration_rad_s2': 412.87433356046677, 'right_hand_velocity_rad_s': 8.620217442512512, 'right_hand_acceleration_rad_s2': 281.60860762000084, 'hand_velocity_rad_s': 13.39472234249115, 'hand_acceleration_rad_s2': 412.87433356046677}`
- output: `{'arm_velocity_rad_s': 96.20739422957038, 'arm_acceleration_rad_s2': 48426.90000000207, 'left_hand_velocity_rad_s': 13.39472234249116, 'left_hand_acceleration_rad_s2': 3147.7632993661814, 'right_hand_velocity_rad_s': 8.620217442512512, 'right_hand_acceleration_rad_s2': 1443.8727686435775, 'hand_velocity_rad_s': 13.39472234249116, 'hand_acceleration_rad_s2': 3147.7632993661814}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
