# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `360`
- output frames: `360`
- max waypoint geometry error: `1.8735013540549517e-14` rad

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

- source: `{'arm_velocity_rad_s': 23.86639282800504, 'arm_acceleration_rad_s2': 725.6036423573627, 'left_hand_velocity_rad_s': 8.687721192836761, 'left_hand_acceleration_rad_s2': 166.96736216545105, 'right_hand_velocity_rad_s': 12.124297022819519, 'right_hand_acceleration_rad_s2': 354.33076322078705, 'hand_velocity_rad_s': 12.124297022819519, 'hand_acceleration_rad_s2': 354.33076322078705}`
- output: `{'arm_velocity_rad_s': 23.86639282800504, 'arm_acceleration_rad_s2': 4474.966072973902, 'left_hand_velocity_rad_s': 8.687721192836761, 'left_hand_acceleration_rad_s2': 1100.9341996639532, 'right_hand_velocity_rad_s': 12.124297022819519, 'right_hand_acceleration_rad_s2': 1561.0308721593383, 'hand_velocity_rad_s': 12.124297022819519, 'hand_acceleration_rad_s2': 1561.0308721593383}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
