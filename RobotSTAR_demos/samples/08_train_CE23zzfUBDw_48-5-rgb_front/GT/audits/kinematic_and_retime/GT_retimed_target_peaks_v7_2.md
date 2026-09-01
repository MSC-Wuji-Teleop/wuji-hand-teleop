# SignAR V7.1 target peak diagnosis

- frames: `210`
- target FPS: `50.0`
- duration: `4.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 19.7672 rad/s | 83 / `20_left_wrist_pitch` | 494.179 rad/s² | 82 / `20_left_wrist_pitch` |
| `left_hand20` | 10.0042 rad/s | 202 / `Lq_18` | 174.048 rad/s² | 129 / `Lq_06` |
| `right_hand20` | 8.90797 rad/s | 204 / `Rq_18` | 173.829 rad/s² | 173 / `Rq_00` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 9.88359 rad/s | 123.545 rad/s² |
| `left_hand20` | 5.00208 rad/s | 43.5121 rad/s² |
| `right_hand20` | 4.45399 rad/s | 43.4572 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
