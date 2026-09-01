# SignAR V7.1 target peak diagnosis

- frames: `760`
- target FPS: `50.0`
- duration: `15.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 96.2074 rad/s | 205 / `19_left_wrist_roll` | 2800.39 rad/s² | 120 / `19_left_wrist_roll` |
| `left_hand20` | 13.3947 rad/s | 231 / `Lq_10` | 412.874 rad/s² | 150 / `Lq_18` |
| `right_hand20` | 8.62022 rad/s | 370 / `Rq_18` | 281.609 rad/s² | 139 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 48.1037 rad/s | 700.099 rad/s² |
| `left_hand20` | 6.69736 rad/s | 103.219 rad/s² |
| `right_hand20` | 4.31011 rad/s | 70.4022 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
