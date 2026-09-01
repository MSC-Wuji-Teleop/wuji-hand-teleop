# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `320`
- output frames: `320`
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

- source: `{'arm_velocity_rad_s': 17.33113675123916, 'arm_acceleration_rad_s2': 405.0494272790088, 'left_hand_velocity_rad_s': 6.789698634584232, 'left_hand_acceleration_rad_s2': 60.20830344787925, 'right_hand_velocity_rad_s': 6.7026966734483135, 'right_hand_acceleration_rad_s2': 61.45534513052553, 'hand_velocity_rad_s': 6.789698634584232, 'hand_acceleration_rad_s2': 61.45534513052553}`
- output: `{'arm_velocity_rad_s': 17.33113675123916, 'arm_acceleration_rad_s2': 2803.1302204001067, 'left_hand_velocity_rad_s': 6.789698634584232, 'left_hand_acceleration_rad_s2': 94.908690091842, 'right_hand_velocity_rad_s': 6.7026966734483135, 'right_hand_acceleration_rad_s2': 539.6902594779128, 'hand_velocity_rad_s': 6.789698634584232, 'hand_acceleration_rad_s2': 539.6902594779128}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
