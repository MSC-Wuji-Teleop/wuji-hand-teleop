# SignAR V7.1 target peak diagnosis

- frames: `260`
- target FPS: `50.0`
- duration: `5.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 32.4035 rad/s | 218 / `21_left_wrist_yaw` | 802.001 rad/s² | 217 / `19_left_wrist_roll` |
| `left_hand20` | 5.362 rad/s | 9 / `Lq_12` | 45.759 rad/s² | 4 / `Lq_12` |
| `right_hand20` | 6.30594 rad/s | 9 / `Rq_12` | 54.2724 rad/s² | 4 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 16.2018 rad/s | 200.5 rad/s² |
| `left_hand20` | 2.681 rad/s | 11.4398 rad/s² |
| `right_hand20` | 3.15297 rad/s | 13.5681 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
