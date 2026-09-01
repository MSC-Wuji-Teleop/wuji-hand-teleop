# SignAR V7.1 target peak diagnosis

- frames: `930`
- target FPS: `50.0`
- duration: `18.580` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 48.6567 rad/s | 404 / `20_left_wrist_pitch` | 1700.87 rad/s² | 403 / `19_left_wrist_roll` |
| `left_hand20` | 5.08567 rad/s | 9 / `Lq_12` | 134.741 rad/s² | 405 / `Lq_17` |
| `right_hand20` | 6.55258 rad/s | 9 / `Rq_18` | 56.3232 rad/s² | 4 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 24.3283 rad/s | 425.216 rad/s² |
| `left_hand20` | 2.54284 rad/s | 33.6853 rad/s² |
| `right_hand20` | 3.27629 rad/s | 14.0808 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
