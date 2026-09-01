# SignAR V7.1 target peak diagnosis

- frames: `260`
- target FPS: `50.0`
- duration: `5.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 28.4597 rad/s | 145 / `27_right_wrist_pitch` | 558.266 rad/s² | 146 / `27_right_wrist_pitch` |
| `left_hand20` | 7.42361 rad/s | 9 / `Lq_18` | 65.3664 rad/s² | 14 / `Lq_18` |
| `right_hand20` | 6.77518 rad/s | 9 / `Rq_18` | 68.701 rad/s² | 201 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 14.2299 rad/s | 139.567 rad/s² |
| `left_hand20` | 3.71181 rad/s | 16.3416 rad/s² |
| `right_hand20` | 3.38759 rad/s | 17.1753 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
