# SignAR V7.1 target peak diagnosis

- frames: `360`
- target FPS: `50.0`
- duration: `7.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 30.8627 rad/s | 312 / `27_right_wrist_pitch` | 683.467 rad/s² | 224 / `28_right_wrist_yaw` |
| `left_hand20` | 7.30851 rad/s | 8 / `Lq_18` | 76.0208 rad/s² | 13 / `Lq_18` |
| `right_hand20` | 7.25029 rad/s | 9 / `Rq_18` | 85.5428 rad/s² | 223 / `Rq_06` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 15.4313 rad/s | 170.867 rad/s² |
| `left_hand20` | 3.65425 rad/s | 19.0052 rad/s² |
| `right_hand20` | 3.62514 rad/s | 21.3857 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
