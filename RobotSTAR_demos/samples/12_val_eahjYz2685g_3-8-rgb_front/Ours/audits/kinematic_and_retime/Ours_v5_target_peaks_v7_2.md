# SignAR V7.1 target peak diagnosis

- frames: `350`
- target FPS: `50.0`
- duration: `6.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 30.818 rad/s | 116 / `28_right_wrist_yaw` | 827.843 rad/s² | 115 / `28_right_wrist_yaw` |
| `left_hand20` | 8.13106 rad/s | 9 / `Lq_18` | 73.0115 rad/s² | 14 / `Lq_18` |
| `right_hand20` | 6.60753 rad/s | 9 / `Rq_18` | 55.5144 rad/s² | 4 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 15.409 rad/s | 206.961 rad/s² |
| `left_hand20` | 4.06553 rad/s | 18.2529 rad/s² |
| `right_hand20` | 3.30377 rad/s | 13.8786 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
