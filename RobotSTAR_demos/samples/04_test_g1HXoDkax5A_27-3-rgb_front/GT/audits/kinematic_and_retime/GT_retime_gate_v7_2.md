# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `150`
- output frames: `150`
- max waypoint geometry error: `3.552713678800501e-15` rad

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

- source: `{'arm_velocity_rad_s': 13.15660493523684, 'arm_acceleration_rad_s2': 222.6422083427233, 'left_hand_velocity_rad_s': 13.889439089689404, 'left_hand_acceleration_rad_s2': 350.08168528293027, 'right_hand_velocity_rad_s': 8.806415647268295, 'right_hand_acceleration_rad_s2': 282.2733670473099, 'hand_velocity_rad_s': 13.889439089689404, 'hand_acceleration_rad_s2': 350.08168528293027}`
- output: `{'arm_velocity_rad_s': 13.15660493523684, 'arm_acceleration_rad_s2': 1788.2949568159897, 'left_hand_velocity_rad_s': 13.889439089689404, 'left_hand_acceleration_rad_s2': 3409.167251152149, 'right_hand_velocity_rad_s': 8.806415647268295, 'right_hand_acceleration_rad_s2': 1421.2934640996305, 'hand_velocity_rad_s': 13.889439089689404, 'hand_acceleration_rad_s2': 3409.167251152149}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
