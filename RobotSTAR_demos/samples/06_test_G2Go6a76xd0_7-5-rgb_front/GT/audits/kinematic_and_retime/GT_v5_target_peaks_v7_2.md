# SignAR V7.1 target peak diagnosis

- frames: `390`
- target FPS: `50.0`
- duration: `7.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 14.9313 rad/s | 370 / `28_right_wrist_yaw` | 292.365 rad/s² | 212 / `28_right_wrist_yaw` |
| `left_hand20` | 10.8566 rad/s | 356 / `Lq_03` | 303.279 rad/s² | 339 / `Lq_03` |
| `right_hand20` | 7.43359 rad/s | 113 / `Rq_06` | 206.483 rad/s² | 43 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 7.46567 rad/s | 73.0911 rad/s² |
| `left_hand20` | 5.42831 rad/s | 75.8198 rad/s² |
| `right_hand20` | 3.7168 rad/s | 51.6208 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
