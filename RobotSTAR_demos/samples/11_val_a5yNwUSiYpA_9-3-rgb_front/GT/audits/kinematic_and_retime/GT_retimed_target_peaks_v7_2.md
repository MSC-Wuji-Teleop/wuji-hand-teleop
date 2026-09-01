# SignAR V7.1 target peak diagnosis

- frames: `190`
- target FPS: `50.0`
- duration: `3.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 29.8309 rad/s | 70 / `27_right_wrist_pitch` | 541.511 rad/s² | 71 / `28_right_wrist_yaw` |
| `left_hand20` | 8.48435 rad/s | 9 / `Lq_18` | 123.058 rad/s² | 69 / `Lq_18` |
| `right_hand20` | 11.6173 rad/s | 80 / `Rq_18` | 350.079 rad/s² | 79 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 14.9154 rad/s | 135.378 rad/s² |
| `left_hand20` | 4.24217 rad/s | 30.7645 rad/s² |
| `right_hand20` | 5.80867 rad/s | 87.5197 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
