# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `930`
- output frames: `930`
- max waypoint geometry error: `3.064215547965432e-14` rad

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

- source: `{'arm_velocity_rad_s': 50.59157300213112, 'arm_acceleration_rad_s2': 1491.6748546214558, 'left_hand_velocity_rad_s': 8.846288174390793, 'left_hand_acceleration_rad_s2': 240.40289223194122, 'right_hand_velocity_rad_s': 9.642435610294342, 'right_hand_acceleration_rad_s2': 297.04753309488297, 'hand_velocity_rad_s': 9.642435610294342, 'hand_acceleration_rad_s2': 297.04753309488297}`
- output: `{'arm_velocity_rad_s': 50.59157300213112, 'arm_acceleration_rad_s2': 10925.917213591916, 'left_hand_velocity_rad_s': 8.846288174390793, 'left_hand_acceleration_rad_s2': 1775.9217395069882, 'right_hand_velocity_rad_s': 9.642435610294342, 'right_hand_acceleration_rad_s2': 1477.4706389984085, 'hand_velocity_rad_s': 9.642435610294342, 'hand_acceleration_rad_s2': 1775.9217395069882}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
