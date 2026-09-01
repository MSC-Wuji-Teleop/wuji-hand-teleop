# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `190`
- output frames: `190`
- max waypoint geometry error: `6.356026815979021e-15` rad

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

- source: `{'arm_velocity_rad_s': 29.830856592367805, 'arm_acceleration_rad_s2': 541.5109394581343, 'left_hand_velocity_rad_s': 8.48434662258854, 'left_hand_acceleration_rad_s2': 123.05792421102524, 'right_hand_velocity_rad_s': 11.617349460721016, 'right_hand_acceleration_rad_s2': 350.078959017992, 'hand_velocity_rad_s': 11.617349460721016, 'hand_acceleration_rad_s2': 350.078959017992}`
- output: `{'arm_velocity_rad_s': 29.830856592367894, 'arm_acceleration_rad_s2': 4783.389840336306, 'left_hand_velocity_rad_s': 8.48434662258854, 'left_hand_acceleration_rad_s2': 471.30388739976763, 'right_hand_velocity_rad_s': 11.617349460721016, 'right_hand_acceleration_rad_s2': 3005.768335890166, 'hand_velocity_rad_s': 11.617349460721016, 'hand_acceleration_rad_s2': 3005.768335890166}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
