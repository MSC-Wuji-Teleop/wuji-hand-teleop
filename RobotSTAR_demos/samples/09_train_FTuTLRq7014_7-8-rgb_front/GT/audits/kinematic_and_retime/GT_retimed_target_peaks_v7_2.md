# SignAR V7.1 target peak diagnosis

- frames: `360`
- target FPS: `50.0`
- duration: `7.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 23.8664 rad/s | 310 / `27_right_wrist_pitch` | 725.604 rad/s² | 307 / `27_right_wrist_pitch` |
| `left_hand20` | 8.68772 rad/s | 155 / `Lq_03` | 166.967 rad/s² | 151 / `Lq_03` |
| `right_hand20` | 12.1243 rad/s | 223 / `Rq_12` | 354.331 rad/s² | 224 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 11.9332 rad/s | 181.401 rad/s² |
| `left_hand20` | 4.34386 rad/s | 41.7418 rad/s² |
| `right_hand20` | 6.06215 rad/s | 88.5827 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
