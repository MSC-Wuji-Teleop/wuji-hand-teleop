# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `590`
- output frames: `590`
- max waypoint geometry error: `9.2148511043888e-15` rad

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

- source: `{'arm_velocity_rad_s': 24.618956037524043, 'arm_acceleration_rad_s2': 615.4739009381011, 'left_hand_velocity_rad_s': 3.46937714426375, 'left_hand_acceleration_rad_s2': 31.57997624757478, 'right_hand_velocity_rad_s': 6.65572153223748, 'right_hand_acceleration_rad_s2': 58.16801777678307, 'hand_velocity_rad_s': 6.65572153223748, 'hand_acceleration_rad_s2': 58.16801777678307}`
- output: `{'arm_velocity_rad_s': 24.618956037524043, 'arm_acceleration_rad_s2': 10683.808090811417, 'left_hand_velocity_rad_s': 3.46937714426375, 'left_hand_acceleration_rad_s2': 93.48195051255809, 'right_hand_velocity_rad_s': 6.65572153223748, 'right_hand_acceleration_rad_s2': 376.79819418693137, 'hand_velocity_rad_s': 6.65572153223748, 'hand_acceleration_rad_s2': 376.79819418693137}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
