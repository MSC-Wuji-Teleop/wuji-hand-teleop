# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `760`
- output frames: `760`
- max waypoint geometry error: `1.1657341758564144e-14` rad

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

- source: `{'arm_velocity_rad_s': 84.18331618041293, 'arm_acceleration_rad_s2': 3575.427386267026, 'left_hand_velocity_rad_s': 7.59891356017495, 'left_hand_acceleration_rad_s2': 154.61206436157227, 'right_hand_velocity_rad_s': 6.249442156102175, 'right_hand_acceleration_rad_s2': 54.82206121087074, 'hand_velocity_rad_s': 7.59891356017495, 'hand_acceleration_rad_s2': 154.61206436157227}`
- output: `{'arm_velocity_rad_s': 84.18331618041293, 'arm_acceleration_rad_s2': 47740.726502404636, 'left_hand_velocity_rad_s': 7.59891356017495, 'left_hand_acceleration_rad_s2': 1254.4063961361858, 'right_hand_velocity_rad_s': 6.249442156102175, 'right_hand_acceleration_rad_s2': 447.10409692911526, 'hand_velocity_rad_s': 7.59891356017495, 'hand_acceleration_rad_s2': 1254.4063961361858}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
