# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `360`
- output frames: `360`
- max waypoint geometry error: `1.9595436384634013e-14` rad

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

- source: `{'arm_velocity_rad_s': 30.862695518510378, 'arm_acceleration_rad_s2': 683.4666998149594, 'left_hand_velocity_rad_s': 7.3085074436411395, 'left_hand_acceleration_rad_s2': 76.02082151010936, 'right_hand_velocity_rad_s': 7.250289172137831, 'right_hand_acceleration_rad_s2': 85.54283529520035, 'hand_velocity_rad_s': 7.3085074436411395, 'hand_acceleration_rad_s2': 85.54283529520035}`
- output: `{'arm_velocity_rad_s': 30.862695518510378, 'arm_acceleration_rad_s2': 7115.766061244831, 'left_hand_velocity_rad_s': 7.3085074436411395, 'left_hand_acceleration_rad_s2': 151.9091949975819, 'right_hand_velocity_rad_s': 7.250289172137831, 'right_hand_acceleration_rad_s2': 579.3518212497127, 'hand_velocity_rad_s': 7.3085074436411395, 'hand_acceleration_rad_s2': 579.3518212497127}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
