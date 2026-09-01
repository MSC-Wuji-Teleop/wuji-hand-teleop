# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `190`
- output frames: `190`
- max waypoint geometry error: `3.3306690738754696e-15` rad

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

- source: `{'arm_velocity_rad_s': 22.35656959626396, 'arm_acceleration_rad_s2': 567.9327177404806, 'left_hand_velocity_rad_s': 7.438725644697919, 'left_hand_acceleration_rad_s2': 76.20448715868199, 'right_hand_velocity_rad_s': 6.861261782501235, 'right_hand_acceleration_rad_s2': 60.37128938248524, 'hand_velocity_rad_s': 7.438725644697919, 'hand_acceleration_rad_s2': 76.20448715868199}`
- output: `{'arm_velocity_rad_s': 22.35656959626391, 'arm_acceleration_rad_s2': 3148.7900709127325, 'left_hand_velocity_rad_s': 7.438725644697919, 'left_hand_acceleration_rad_s2': 507.07204380798373, 'right_hand_velocity_rad_s': 6.861261782501235, 'right_hand_acceleration_rad_s2': 175.145357889266, 'hand_velocity_rad_s': 7.438725644697919, 'hand_acceleration_rad_s2': 507.07204380798373}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
