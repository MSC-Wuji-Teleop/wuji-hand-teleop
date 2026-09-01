# SignAR V7.1 target peak diagnosis

- frames: `320`
- target FPS: `50.0`
- duration: `6.380` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 19.2582 rad/s | 9 / `19_left_wrist_roll` | 481.455 rad/s² | 10 / `19_left_wrist_roll` |
| `left_hand20` | 6.5177 rad/s | 129 / `Lq_03` | 212.608 rad/s² | 134 / `Lq_06` |
| `right_hand20` | 10.2546 rad/s | 281 / `Rq_16` | 254.002 rad/s² | 279 / `Rq_16` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 9.6291 rad/s | 120.364 rad/s² |
| `left_hand20` | 3.25885 rad/s | 53.1519 rad/s² |
| `right_hand20` | 5.12728 rad/s | 63.5004 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
