# SignAR V7.1 target peak diagnosis

- frames: `820`
- target FPS: `50.0`
- duration: `16.380` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 46.4741 rad/s | 359 / `27_right_wrist_pitch` | 1102.28 rad/s² | 360 / `27_right_wrist_pitch` |
| `left_hand20` | 14.1758 rad/s | 468 / `Lq_19` | 354.591 rad/s² | 467 / `Lq_19` |
| `right_hand20` | 8.46456 rad/s | 9 / `Rq_16` | 224.611 rad/s² | 78 / `Rq_16` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 23.237 rad/s | 275.569 rad/s² |
| `left_hand20` | 7.0879 rad/s | 88.6478 rad/s² |
| `right_hand20` | 4.23228 rad/s | 56.1528 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
