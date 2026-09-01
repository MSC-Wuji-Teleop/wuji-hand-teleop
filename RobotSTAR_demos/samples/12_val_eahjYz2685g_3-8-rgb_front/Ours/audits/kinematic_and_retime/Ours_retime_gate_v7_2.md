# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `350`
- output frames: `350`
- max waypoint geometry error: `6.897260540483785e-15` rad

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

- source: `{'arm_velocity_rad_s': 30.817951686768584, 'arm_acceleration_rad_s2': 827.8426622538893, 'left_hand_velocity_rad_s': 8.131056441912493, 'left_hand_acceleration_rad_s2': 73.01146862845692, 'right_hand_velocity_rad_s': 6.607533511747121, 'right_hand_acceleration_rad_s2': 55.514435969974116, 'hand_velocity_rad_s': 8.131056441912493, 'hand_acceleration_rad_s2': 73.01146862845692}`
- output: `{'arm_velocity_rad_s': 30.817951686768584, 'arm_acceleration_rad_s2': 3714.6956828232564, 'left_hand_velocity_rad_s': 8.131056441912493, 'left_hand_acceleration_rad_s2': 141.08344988380424, 'right_hand_velocity_rad_s': 6.607533511747121, 'right_hand_acceleration_rad_s2': 540.5099669743377, 'hand_velocity_rad_s': 8.131056441912493, 'hand_acceleration_rad_s2': 540.5099669743377}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
