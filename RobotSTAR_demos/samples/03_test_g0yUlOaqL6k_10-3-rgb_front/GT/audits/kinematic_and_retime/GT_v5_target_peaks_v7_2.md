# SignAR V7.1 target peak diagnosis

- frames: `410`
- target FPS: `50.0`
- duration: `8.180` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 43.3363 rad/s | 37 / `28_right_wrist_yaw` | 1083.41 rad/s² | 38 / `28_right_wrist_yaw` |
| `left_hand20` | 8.37103 rad/s | 87 / `Lq_16` | 228.774 rad/s² | 86 / `Lq_06` |
| `right_hand20` | 11.4184 rad/s | 183 / `Rq_06` | 271.377 rad/s² | 182 / `Rq_06` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 21.6681 rad/s | 270.852 rad/s² |
| `left_hand20` | 4.18552 rad/s | 57.1936 rad/s² |
| `right_hand20` | 5.70918 rad/s | 67.8443 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
