# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `200`
- output frames: `200`
- max waypoint geometry error: `2.4424906541753444e-15` rad

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

- source: `{'arm_velocity_rad_s': 36.14549342479259, 'arm_acceleration_rad_s2': 1095.611041825503, 'left_hand_velocity_rad_s': 4.058212097372521, 'left_hand_acceleration_rad_s2': 33.048383576826744, 'right_hand_velocity_rad_s': 6.93701285797631, 'right_hand_acceleration_rad_s2': 77.12289545452222, 'hand_velocity_rad_s': 6.93701285797631, 'hand_acceleration_rad_s2': 77.12289545452222}`
- output: `{'arm_velocity_rad_s': 36.14549342479259, 'arm_acceleration_rad_s2': 15459.044688593513, 'left_hand_velocity_rad_s': 4.058212097372521, 'left_hand_acceleration_rad_s2': 67.73468871084788, 'right_hand_velocity_rad_s': 6.93701285797631, 'right_hand_acceleration_rad_s2': 726.4488689874754, 'hand_velocity_rad_s': 6.93701285797631, 'hand_acceleration_rad_s2': 726.4488689874754}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
