# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `820`
- output frames: `820`
- max waypoint geometry error: `1.2989609388114332e-14` rad

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

- source: `{'arm_velocity_rad_s': 18.18971368094149, 'arm_acceleration_rad_s2': 383.1158585208036, 'left_hand_velocity_rad_s': 7.518858928000596, 'left_hand_acceleration_rad_s2': 63.28213590060327, 'right_hand_velocity_rad_s': 6.508165044254471, 'right_hand_acceleration_rad_s2': 56.87538192583635, 'hand_velocity_rad_s': 7.518858928000596, 'hand_acceleration_rad_s2': 63.28213590060327}`
- output: `{'arm_velocity_rad_s': 18.18971368094149, 'arm_acceleration_rad_s2': 5249.180509812383, 'left_hand_velocity_rad_s': 7.518858928000596, 'left_hand_acceleration_rad_s2': 104.30455922908568, 'right_hand_velocity_rad_s': 6.508165044254471, 'right_hand_acceleration_rad_s2': 110.82261979265124, 'hand_velocity_rad_s': 7.518858928000596, 'hand_acceleration_rad_s2': 110.82261979265124}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
