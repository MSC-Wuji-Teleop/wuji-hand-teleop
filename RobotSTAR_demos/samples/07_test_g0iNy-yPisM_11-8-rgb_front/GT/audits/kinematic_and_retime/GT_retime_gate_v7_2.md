# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `260`
- output frames: `260`
- max waypoint geometry error: `2.0095036745715333e-14` rad

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

- source: `{'arm_velocity_rad_s': 23.362741865420116, 'arm_acceleration_rad_s2': 510.9838553011026, 'left_hand_velocity_rad_s': 8.542938844823047, 'left_hand_acceleration_rad_s2': 263.55454698204994, 'right_hand_velocity_rad_s': 11.317386478185654, 'right_hand_acceleration_rad_s2': 274.21802282333374, 'hand_velocity_rad_s': 11.317386478185654, 'hand_acceleration_rad_s2': 274.21802282333374}`
- output: `{'arm_velocity_rad_s': 23.362741865420116, 'arm_acceleration_rad_s2': 3410.2245825548207, 'left_hand_velocity_rad_s': 8.542938844823047, 'left_hand_acceleration_rad_s2': 1205.0293985094022, 'right_hand_velocity_rad_s': 11.317386478185654, 'right_hand_acceleration_rad_s2': 1461.5736504755082, 'hand_velocity_rad_s': 11.317386478185654, 'hand_acceleration_rad_s2': 1461.5736504755082}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
