# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `320`
- output frames: `320`
- max waypoint geometry error: `7.771561172376096e-15` rad

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

- source: `{'arm_velocity_rad_s': 19.258191846403122, 'arm_acceleration_rad_s2': 481.45479616007805, 'left_hand_velocity_rad_s': 6.517696380615234, 'left_hand_acceleration_rad_s2': 212.60756999254227, 'right_hand_velocity_rad_s': 10.254564881324768, 'right_hand_acceleration_rad_s2': 254.00154292583466, 'hand_velocity_rad_s': 10.254564881324768, 'hand_acceleration_rad_s2': 254.00154292583466}`
- output: `{'arm_velocity_rad_s': 19.258191846403122, 'arm_acceleration_rad_s2': 7144.496567747213, 'left_hand_velocity_rad_s': 6.517696380615234, 'left_hand_acceleration_rad_s2': 631.3569052924786, 'right_hand_velocity_rad_s': 10.254564881324962, 'right_hand_acceleration_rad_s2': 1281.5759117877046, 'hand_velocity_rad_s': 10.254564881324962, 'hand_acceleration_rad_s2': 1281.5759117877046}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
