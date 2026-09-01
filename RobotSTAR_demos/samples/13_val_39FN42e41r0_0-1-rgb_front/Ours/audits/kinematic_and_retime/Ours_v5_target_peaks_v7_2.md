# SignAR V7.1 target peak diagnosis

- frames: `320`
- target FPS: `50.0`
- duration: `6.380` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 17.3311 rad/s | 106 / `27_right_wrist_pitch` | 405.049 rad/s² | 14 / `27_right_wrist_pitch` |
| `left_hand20` | 6.7897 rad/s | 9 / `Lq_12` | 60.2083 rad/s² | 13 / `Lq_12` |
| `right_hand20` | 6.7027 rad/s | 9 / `Rq_18` | 61.4553 rad/s² | 307 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 8.66557 rad/s | 101.262 rad/s² |
| `left_hand20` | 3.39485 rad/s | 15.0521 rad/s² |
| `right_hand20` | 3.35135 rad/s | 15.3638 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
