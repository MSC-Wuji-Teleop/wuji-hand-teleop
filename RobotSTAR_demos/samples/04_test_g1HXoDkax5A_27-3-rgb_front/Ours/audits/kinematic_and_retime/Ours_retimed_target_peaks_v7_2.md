# SignAR V7.1 target peak diagnosis

- frames: `150`
- target FPS: `50.0`
- duration: `2.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 13.7627 rad/s | 7 / `26_right_wrist_roll` | 242.295 rad/s² | 8 / `27_right_wrist_pitch` |
| `left_hand20` | 7.24123 rad/s | 9 / `Lq_18` | 66.605 rad/s² | 14 / `Lq_18` |
| `right_hand20` | 7.28912 rad/s | 9 / `Rq_18` | 62.9932 rad/s² | 14 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 6.88133 rad/s | 60.5738 rad/s² |
| `left_hand20` | 3.62061 rad/s | 16.6513 rad/s² |
| `right_hand20` | 3.64456 rad/s | 15.7483 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
