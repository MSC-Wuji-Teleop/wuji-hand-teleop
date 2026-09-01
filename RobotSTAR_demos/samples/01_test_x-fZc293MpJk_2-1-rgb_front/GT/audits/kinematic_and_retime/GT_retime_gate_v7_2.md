# SignAR V7.1 post-IK retiming gate

- overall: **PASS**
- path preservation: **PASS**
- conservative rate screen: **WARN**
- rate screen hard-required: `False`
- time scale: `1`
- source waypoints: `820`
- output frames: `820`
- max waypoint geometry error: `1.6653345369377348e-14` rad

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

- source: `{'arm_velocity_rad_s': 46.47406421700692, 'arm_acceleration_rad_s2': 1102.2772815807225, 'left_hand_velocity_rad_s': 14.175808944855817, 'left_hand_acceleration_rad_s2': 354.59130915114656, 'right_hand_velocity_rad_s': 8.46456174278934, 'right_hand_acceleration_rad_s2': 224.61112588644028, 'hand_velocity_rad_s': 14.175808944855817, 'hand_acceleration_rad_s2': 354.59130915114656}`
- output: `{'arm_velocity_rad_s': 46.47406421700692, 'arm_acceleration_rad_s2': 19285.201907034432, 'left_hand_velocity_rad_s': 14.175808944855817, 'left_hand_acceleration_rad_s2': 3325.189953392732, 'right_hand_velocity_rad_s': 8.46456174278934, 'right_hand_acceleration_rad_s2': 1550.9754021332624, 'hand_velocity_rad_s': 14.175808944855817, 'hand_acceleration_rad_s2': 3325.189953392732}`
- limits: `{'arm_velocity_rad_s': 0.5, 'arm_acceleration_rad_s2': 3.0, 'hand_velocity_rad_s': 4.0, 'hand_acceleration_rad_s2': 20.0}`

> Normal and 0.5× simulation profiles may be intentionally run without treating the conservative 0.5 rad/s / 3 rad/s² arm screen as a hard gate.
> This does not approve the result for real hardware. Watch both kinematic previews before physical rollout and inspect force/collision audits afterward.
