# SignAR V7.1 target peak diagnosis

- frames: `390`
- target FPS: `50.0`
- duration: `7.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 25.1494 rad/s | 113 / `28_right_wrist_yaw` | 822.693 rad/s² | 114 / `28_right_wrist_yaw` |
| `left_hand20` | 8.10802 rad/s | 9 / `Lq_18` | 73.5629 rad/s² | 14 / `Lq_18` |
| `right_hand20` | 6.61595 rad/s | 9 / `Rq_12` | 76.916 rad/s² | 107 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 12.5747 rad/s | 205.673 rad/s² |
| `left_hand20` | 4.05401 rad/s | 18.3907 rad/s² |
| `right_hand20` | 3.30797 rad/s | 19.229 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
