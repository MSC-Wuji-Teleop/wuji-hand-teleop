# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `210`
- output frames: `210`
- max waypoint geometry error: `8.770761894538737e-15` rad

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

- source: `{'arm_velocity_rad_s': 20.232897456732008, 'arm_acceleration_rad_s2': 366.040525940858, 'left_hand_velocity_rad_s': 2.9545858502388, 'left_hand_acceleration_rad_s2': 33.14094963881695, 'right_hand_velocity_rad_s': 7.103357037012548, 'right_hand_acceleration_rad_s2': 58.939989168749776, 'hand_velocity_rad_s': 7.103357037012548, 'hand_acceleration_rad_s2': 58.939989168749776}`
- output: `{'arm_velocity_rad_s': 20.232897456732008, 'arm_acceleration_rad_s2': 3701.893579665637, 'left_hand_velocity_rad_s': 2.9545858502388, 'left_hand_acceleration_rad_s2': 90.92997201929136, 'right_hand_velocity_rad_s': 7.103357037012548, 'right_hand_acceleration_rad_s2': 220.4590434279254, 'hand_velocity_rad_s': 7.103357037012548, 'hand_acceleration_rad_s2': 220.4590434279254}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
