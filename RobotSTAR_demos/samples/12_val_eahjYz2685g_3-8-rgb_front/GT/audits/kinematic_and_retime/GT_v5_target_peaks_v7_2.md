# SignAR V7.1 target peak diagnosis

- frames: `350`
- target FPS: `50.0`
- duration: `6.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 21.7579 rad/s | 324 / `28_right_wrist_yaw` | 791.354 rad/s² | 88 / `19_left_wrist_roll` |
| `left_hand20` | 8.89597 rad/s | 81 / `Lq_06` | 172.484 rad/s² | 79 / `Lq_06` |
| `right_hand20` | 11.5621 rad/s | 97 / `Rq_16` | 268.018 rad/s² | 292 / `Rq_16` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 10.8789 rad/s | 197.839 rad/s² |
| `left_hand20` | 4.44799 rad/s | 43.1209 rad/s² |
| `right_hand20` | 5.78104 rad/s | 67.0045 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
