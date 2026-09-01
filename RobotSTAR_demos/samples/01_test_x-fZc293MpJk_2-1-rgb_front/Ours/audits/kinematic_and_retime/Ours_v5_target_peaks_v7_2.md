# SignAR V7.1 target peak diagnosis

- frames: `820`
- target FPS: `50.0`
- duration: `16.380` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 18.1897 rad/s | 387 / `28_right_wrist_yaw` | 383.116 rad/s² | 10 / `26_right_wrist_roll` |
| `left_hand20` | 7.51886 rad/s | 9 / `Lq_18` | 63.2821 rad/s² | 4 / `Lq_18` |
| `right_hand20` | 6.50817 rad/s | 9 / `Rq_12` | 56.8754 rad/s² | 13 / `Rq_12` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 9.09486 rad/s | 95.779 rad/s² |
| `left_hand20` | 3.75943 rad/s | 15.8205 rad/s² |
| `right_hand20` | 3.25408 rad/s | 14.2188 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
