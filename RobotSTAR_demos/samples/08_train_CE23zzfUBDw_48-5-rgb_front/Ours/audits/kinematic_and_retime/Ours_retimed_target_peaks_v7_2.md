# SignAR V7.1 target peak diagnosis

- frames: `210`
- target FPS: `50.0`
- duration: `4.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 20.2329 rad/s | 183 / `27_right_wrist_pitch` | 366.041 rad/s² | 179 / `28_right_wrist_yaw` |
| `left_hand20` | 2.95459 rad/s | 32 / `Lq_18` | 33.1409 rad/s² | 10 / `Lq_18` |
| `right_hand20` | 7.10336 rad/s | 9 / `Rq_18` | 58.94 rad/s² | 14 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 10.1164 rad/s | 91.5101 rad/s² |
| `left_hand20` | 1.47729 rad/s | 8.28524 rad/s² |
| `right_hand20` | 3.55168 rad/s | 14.735 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
