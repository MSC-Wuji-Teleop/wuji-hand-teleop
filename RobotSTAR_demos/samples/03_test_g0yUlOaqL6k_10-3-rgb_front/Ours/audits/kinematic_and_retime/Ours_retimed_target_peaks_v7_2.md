# SignAR V7.1 target peak diagnosis

- frames: `410`
- target FPS: `50.0`
- duration: `8.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 53.5772 rad/s | 219 / `19_left_wrist_roll` | 1294.72 rad/s² | 25 / `28_right_wrist_yaw` |
| `left_hand20` | 5.38878 rad/s | 9 / `Lq_12` | 45.6549 rad/s² | 4 / `Lq_12` |
| `right_hand20` | 6.64414 rad/s | 9 / `Rq_18` | 57.0881 rad/s² | 13 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 26.7886 rad/s | 323.681 rad/s² |
| `left_hand20` | 2.69439 rad/s | 11.4137 rad/s² |
| `right_hand20` | 3.32207 rad/s | 14.272 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
