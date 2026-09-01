# SignAR V7.1 target peak diagnosis

- frames: `760`
- target FPS: `50.0`
- duration: `15.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 84.1833 rad/s | 205 / `19_left_wrist_roll` | 3575.43 rad/s² | 121 / `20_left_wrist_pitch` |
| `left_hand20` | 7.59891 rad/s | 9 / `Lq_18` | 154.612 rad/s² | 122 / `Lq_07` |
| `right_hand20` | 6.24944 rad/s | 9 / `Rq_12` | 54.8221 rad/s² | 84 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 42.0917 rad/s | 893.857 rad/s² |
| `left_hand20` | 3.79946 rad/s | 38.653 rad/s² |
| `right_hand20` | 3.12472 rad/s | 13.7055 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
