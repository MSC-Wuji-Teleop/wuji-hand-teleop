# SignAR V7.1 target peak diagnosis

- frames: `590`
- target FPS: `50.0`
- duration: `11.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 24.619 rad/s | 369 / `27_right_wrist_pitch` | 615.474 rad/s² | 370 / `27_right_wrist_pitch` |
| `left_hand20` | 3.46938 rad/s | 8 / `Lq_12` | 31.58 rad/s² | 3 / `Lq_12` |
| `right_hand20` | 6.65572 rad/s | 9 / `Rq_18` | 58.168 rad/s² | 13 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 12.3095 rad/s | 153.868 rad/s² |
| `left_hand20` | 1.73469 rad/s | 7.89499 rad/s² |
| `right_hand20` | 3.32786 rad/s | 14.542 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
