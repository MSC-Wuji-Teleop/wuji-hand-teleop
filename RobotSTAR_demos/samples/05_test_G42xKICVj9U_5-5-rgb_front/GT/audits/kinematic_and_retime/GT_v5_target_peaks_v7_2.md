# SignAR V7.1 target peak diagnosis

- frames: `260`
- target FPS: `50.0`
- duration: `5.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 17.595 rad/s | 11 / `19_left_wrist_roll` | 439.876 rad/s² | 12 / `19_left_wrist_roll` |
| `left_hand20` | 9.56835 rad/s | 181 / `Lq_18` | 225.68 rad/s² | 28 / `Lq_16` |
| `right_hand20` | 9.99444 rad/s | 214 / `Rq_16` | 195.376 rad/s² | 131 / `Rq_04` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 8.79751 rad/s | 109.969 rad/s² |
| `left_hand20` | 4.78418 rad/s | 56.42 rad/s² |
| `right_hand20` | 4.99722 rad/s | 48.844 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
