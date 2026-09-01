# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `210`
- output frames: `210`
- max waypoint geometry error: `8.271161533457416e-15` rad

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

- source: `{'arm_velocity_rad_s': 19.76717629806851, 'arm_acceleration_rad_s2': 494.17940745171273, 'left_hand_velocity_rad_s': 10.004158318042755, 'left_hand_acceleration_rad_s2': 174.04820770025253, 'right_hand_velocity_rad_s': 8.907974511384964, 'right_hand_acceleration_rad_s2': 173.82875084877014, 'hand_velocity_rad_s': 10.004158318042755, 'hand_acceleration_rad_s2': 174.04820770025253}`
- output: `{'arm_velocity_rad_s': 19.76717629806851, 'arm_acceleration_rad_s2': 4605.324070657201, 'left_hand_velocity_rad_s': 10.004158318042778, 'left_hand_acceleration_rad_s2': 1640.3016649327785, 'right_hand_velocity_rad_s': 8.907974511385147, 'right_hand_acceleration_rad_s2': 1104.4541394491305, 'hand_velocity_rad_s': 10.004158318042778, 'hand_acceleration_rad_s2': 1640.3016649327785}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
