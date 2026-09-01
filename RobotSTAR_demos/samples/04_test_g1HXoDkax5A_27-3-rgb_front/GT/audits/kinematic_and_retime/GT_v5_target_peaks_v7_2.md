# SignAR V7.1 target peak diagnosis

- frames: `150`
- target FPS: `50.0`
- duration: `2.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 13.1566 rad/s | 89 / `26_right_wrist_roll` | 222.642 rad/s² | 91 / `26_right_wrist_roll` |
| `left_hand20` | 13.8894 rad/s | 108 / `Lq_06` | 350.082 rad/s² | 107 / `Lq_06` |
| `right_hand20` | 8.80642 rad/s | 87 / `Rq_16` | 282.273 rad/s² | 99 / `Rq_06` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 6.5783 rad/s | 55.6606 rad/s² |
| `left_hand20` | 6.94472 rad/s | 87.5204 rad/s² |
| `right_hand20` | 4.40321 rad/s | 70.5683 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
