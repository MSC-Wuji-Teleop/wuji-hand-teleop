# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `260`
- output frames: `260`
- max waypoint geometry error: `6.050715484207103e-15` rad

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

- source: `{'arm_velocity_rad_s': 32.40352099028625, 'arm_acceleration_rad_s2': 802.0005193444797, 'left_hand_velocity_rad_s': 5.36200264492291, 'left_hand_acceleration_rad_s2': 45.75901134840537, 'right_hand_velocity_rad_s': 6.305942571321263, 'right_hand_acceleration_rad_s2': 54.272388745412094, 'hand_velocity_rad_s': 6.305942571321263, 'hand_acceleration_rad_s2': 54.272388745412094}`
- output: `{'arm_velocity_rad_s': 32.40352099028625, 'arm_acceleration_rad_s2': 6841.398937322443, 'left_hand_velocity_rad_s': 5.36200264492291, 'left_hand_acceleration_rad_s2': 183.81512258201806, 'right_hand_velocity_rad_s': 6.305942571321263, 'right_hand_acceleration_rad_s2': 90.4399609086002, 'hand_velocity_rad_s': 6.305942571321263, 'hand_acceleration_rad_s2': 183.81512258201806}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
