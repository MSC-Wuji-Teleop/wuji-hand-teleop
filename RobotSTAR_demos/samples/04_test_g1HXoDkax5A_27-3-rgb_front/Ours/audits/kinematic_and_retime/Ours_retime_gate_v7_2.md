# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `150`
- output frames: `150`
- max waypoint geometry error: `2.1094237467877974e-15` rad

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

- source: `{'arm_velocity_rad_s': 13.762653185881865, 'arm_acceleration_rad_s2': 242.29501118983143, 'left_hand_velocity_rad_s': 7.241228758954504, 'left_hand_acceleration_rad_s2': 66.60502343201686, 'right_hand_velocity_rad_s': 7.289123616293897, 'right_hand_acceleration_rad_s2': 62.99321425376647, 'hand_velocity_rad_s': 7.289123616293897, 'hand_acceleration_rad_s2': 66.60502343201686}`
- output: `{'arm_velocity_rad_s': 13.762653185881865, 'arm_acceleration_rad_s2': 2274.8085931123023, 'left_hand_velocity_rad_s': 7.241228758954504, 'left_hand_acceleration_rad_s2': 101.02126568849113, 'right_hand_velocity_rad_s': 7.289123616293897, 'right_hand_acceleration_rad_s2': 96.68153869983962, 'hand_velocity_rad_s': 7.289123616293897, 'hand_acceleration_rad_s2': 101.02126568849113}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
