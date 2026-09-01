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

- source: `{'arm_velocity_rad_s': 34.03821690967378, 'arm_acceleration_rad_s2': 1049.97728281806, 'left_hand_velocity_rad_s': 5.985930562019348, 'left_hand_acceleration_rad_s2': 122.9383796453476, 'right_hand_velocity_rad_s': 7.704804837703705, 'right_hand_acceleration_rad_s2': 156.29012137651443, 'hand_velocity_rad_s': 7.704804837703705, 'hand_acceleration_rad_s2': 156.29012137651443}`
- output: `{'arm_velocity_rad_s': 34.03821690967378, 'arm_acceleration_rad_s2': 14375.276267761032, 'left_hand_velocity_rad_s': 7.076188921928399, 'left_hand_acceleration_rad_s2': 861.671210660757, 'right_hand_velocity_rad_s': 7.704804837703705, 'right_hand_acceleration_rad_s2': 795.4478516085928, 'hand_velocity_rad_s': 7.704804837703705, 'hand_acceleration_rad_s2': 861.671210660757}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
