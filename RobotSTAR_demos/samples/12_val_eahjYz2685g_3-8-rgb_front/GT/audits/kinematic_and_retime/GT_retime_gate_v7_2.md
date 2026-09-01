# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `350`
- output frames: `350`
- max waypoint geometry error: `8.104628079763643e-15` rad

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

- source: `{'arm_velocity_rad_s': 21.757852263638082, 'arm_acceleration_rad_s2': 791.3543596245341, 'left_hand_velocity_rad_s': 8.89597237110138, 'left_hand_acceleration_rad_s2': 172.4836230278015, 'right_hand_velocity_rad_s': 11.562074720859528, 'right_hand_acceleration_rad_s2': 268.01809668540955, 'hand_velocity_rad_s': 11.562074720859528, 'hand_acceleration_rad_s2': 268.01809668540955}`
- output: `{'arm_velocity_rad_s': 21.757852263638082, 'arm_acceleration_rad_s2': 7510.905313083914, 'left_hand_velocity_rad_s': 8.89597237110134, 'left_hand_acceleration_rad_s2': 971.3093936744847, 'right_hand_velocity_rad_s': 11.562074720859528, 'right_hand_acceleration_rad_s2': 1465.6692543866623, 'hand_velocity_rad_s': 11.562074720859528, 'hand_acceleration_rad_s2': 1465.6692543866623}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
