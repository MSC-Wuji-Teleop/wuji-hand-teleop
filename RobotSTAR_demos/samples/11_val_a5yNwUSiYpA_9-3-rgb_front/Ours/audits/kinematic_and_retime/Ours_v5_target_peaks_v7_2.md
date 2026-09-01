# SignAR V7.1 target peak diagnosis

- frames: `190`
- target FPS: `50.0`
- duration: `3.780` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 22.3566 rad/s | 40 / `26_right_wrist_roll` | 567.933 rad/s² | 39 / `26_right_wrist_roll` |
| `left_hand20` | 7.43873 rad/s | 9 / `Lq_18` | 76.2045 rad/s² | 13 / `Lq_18` |
| `right_hand20` | 6.86126 rad/s | 9 / `Rq_18` | 60.3713 rad/s² | 13 / `Rq_18` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 11.1783 rad/s | 141.983 rad/s² |
| `left_hand20` | 3.71936 rad/s | 19.0511 rad/s² |
| `right_hand20` | 3.43063 rad/s | 15.0928 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
