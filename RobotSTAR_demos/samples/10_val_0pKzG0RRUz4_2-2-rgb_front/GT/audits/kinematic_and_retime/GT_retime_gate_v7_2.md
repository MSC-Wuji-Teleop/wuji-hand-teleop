# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `590`
- output frames: `590`
- max waypoint geometry error: `2.2093438190040615e-14` rad

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

- source: `{'arm_velocity_rad_s': 21.696834711859715, 'arm_acceleration_rad_s2': 386.16401468464124, 'left_hand_velocity_rad_s': 12.481927871704102, 'left_hand_acceleration_rad_s2': 312.1379017829895, 'right_hand_velocity_rad_s': 12.340957392007113, 'right_hand_acceleration_rad_s2': 315.52016735076904, 'hand_velocity_rad_s': 12.481927871704102, 'hand_acceleration_rad_s2': 315.52016735076904}`
- output: `{'arm_velocity_rad_s': 21.696834711859715, 'arm_acceleration_rad_s2': 3987.730528848313, 'left_hand_velocity_rad_s': 12.481927871704102, 'left_hand_acceleration_rad_s2': 2581.619512682475, 'right_hand_velocity_rad_s': 12.340957392007113, 'right_hand_acceleration_rad_s2': 3151.324469123094, 'hand_velocity_rad_s': 12.481927871704102, 'hand_acceleration_rad_s2': 3151.324469123094}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
