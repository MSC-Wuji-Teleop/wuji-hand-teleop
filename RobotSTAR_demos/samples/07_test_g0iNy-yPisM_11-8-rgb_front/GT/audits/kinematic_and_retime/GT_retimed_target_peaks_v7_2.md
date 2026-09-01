# SignAR V7.1 target peak diagnosis

- frames: `260`
- target FPS: `50.0`
- duration: `5.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 23.3627 rad/s | 226 / `27_right_wrist_pitch` | 510.984 rad/s² | 43 / `21_left_wrist_yaw` |
| `left_hand20` | 8.54294 rad/s | 9 / `Lq_18` | 263.555 rad/s² | 230 / `Lq_16` |
| `right_hand20` | 11.3174 rad/s | 135 / `Rq_06` | 274.218 rad/s² | 252 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 11.6814 rad/s | 127.746 rad/s² |
| `left_hand20` | 4.27147 rad/s | 65.8886 rad/s² |
| `right_hand20` | 5.65869 rad/s | 68.5545 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
