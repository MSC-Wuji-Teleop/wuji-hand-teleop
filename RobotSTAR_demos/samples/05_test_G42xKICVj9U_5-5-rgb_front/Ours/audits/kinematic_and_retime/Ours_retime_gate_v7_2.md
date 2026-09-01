# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `260`
- output frames: `260`
- max waypoint geometry error: `8.382183835919932e-15` rad

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

- source: `{'arm_velocity_rad_s': 28.459733890161964, 'arm_acceleration_rad_s2': 558.2663474231658, 'left_hand_velocity_rad_s': 7.4236120527326195, 'left_hand_acceleration_rad_s2': 65.36641940746905, 'right_hand_velocity_rad_s': 6.775176127227486, 'right_hand_acceleration_rad_s2': 68.7010062392801, 'hand_velocity_rad_s': 7.4236120527326195, 'hand_acceleration_rad_s2': 68.7010062392801}`
- output: `{'arm_velocity_rad_s': 28.459733890161964, 'arm_acceleration_rad_s2': 4524.9260690593055, 'left_hand_velocity_rad_s': 7.4236120527326195, 'left_hand_acceleration_rad_s2': 134.41334444775333, 'right_hand_velocity_rad_s': 6.775176127227486, 'right_hand_acceleration_rad_s2': 188.87222851240523, 'hand_velocity_rad_s': 7.4236120527326195, 'hand_acceleration_rad_s2': 188.87222851240523}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
