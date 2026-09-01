# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `930`
- output frames: `930`
- max waypoint geometry error: `4.3298697960381105e-14` rad

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

- source: `{'arm_velocity_rad_s': 48.656666699255226, 'arm_acceleration_rad_s2': 1700.8655178409938, 'left_hand_velocity_rad_s': 5.085672963565317, 'left_hand_acceleration_rad_s2': 134.74109582602978, 'right_hand_velocity_rad_s': 6.5525759370312215, 'right_hand_acceleration_rad_s2': 56.32324398125997, 'hand_velocity_rad_s': 6.5525759370312215, 'hand_acceleration_rad_s2': 134.74109582602978}`
- output: `{'arm_velocity_rad_s': 48.656666699255226, 'arm_acceleration_rad_s2': 18515.991420689745, 'left_hand_velocity_rad_s': 5.085672963565317, 'left_hand_acceleration_rad_s2': 1074.5195784401387, 'right_hand_velocity_rad_s': 6.5525759370312215, 'right_hand_acceleration_rad_s2': 266.6124024379273, 'hand_velocity_rad_s': 6.5525759370312215, 'hand_acceleration_rad_s2': 1074.5195784401387}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
