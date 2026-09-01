# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `390`
- output frames: `390`
- max waypoint geometry error: `5.953570969552402e-15` rad

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

- source: `{'arm_velocity_rad_s': 25.14944880519704, 'arm_acceleration_rad_s2': 822.6932030512811, 'left_hand_velocity_rad_s': 8.108021038959704, 'left_hand_acceleration_rad_s2': 73.5628663080673, 'right_hand_velocity_rad_s': 6.615947617286419, 'right_hand_acceleration_rad_s2': 76.91596634685993, 'hand_velocity_rad_s': 8.108021038959704, 'hand_acceleration_rad_s2': 76.91596634685993}`
- output: `{'arm_velocity_rad_s': 25.149448805197043, 'arm_acceleration_rad_s2': 14855.02992330259, 'left_hand_velocity_rad_s': 8.108021038959704, 'left_hand_acceleration_rad_s2': 106.21917147314782, 'right_hand_velocity_rad_s': 6.615947617286419, 'right_hand_acceleration_rad_s2': 610.3183165069966, 'hand_velocity_rad_s': 8.108021038959704, 'hand_acceleration_rad_s2': 610.3183165069966}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
