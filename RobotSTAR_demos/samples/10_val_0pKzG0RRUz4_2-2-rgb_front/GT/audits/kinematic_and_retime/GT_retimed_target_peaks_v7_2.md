# SignAR V7.1 target peak diagnosis

- frames: `590`
- target FPS: `50.0`
- duration: `11.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 21.6968 rad/s | 392 / `28_right_wrist_yaw` | 386.164 rad/s² | 572 / `28_right_wrist_yaw` |
| `left_hand20` | 12.4819 rad/s | 298 / `Lq_18` | 312.138 rad/s² | 297 / `Lq_18` |
| `right_hand20` | 12.341 rad/s | 310 / `Rq_19` | 315.52 rad/s² | 309 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 10.8484 rad/s | 96.541 rad/s² |
| `left_hand20` | 6.24096 rad/s | 78.0345 rad/s² |
| `right_hand20` | 6.17048 rad/s | 78.88 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
