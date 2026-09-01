# SignAR V7.1 target peak diagnosis

- frames: `200`
- target FPS: `50.0`
- duration: `3.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 34.0382 rad/s | 6 / `19_left_wrist_roll` | 1049.98 rad/s² | 5 / `19_left_wrist_roll` |
| `left_hand20` | 5.98593 rad/s | 199 / `Lq_10` | 122.938 rad/s² | 37 / `Lq_03` |
| `right_hand20` | 7.7048 rad/s | 179 / `Rq_10` | 156.29 rad/s² | 20 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 17.0191 rad/s | 262.494 rad/s² |
| `left_hand20` | 2.99297 rad/s | 30.7346 rad/s² |
| `right_hand20` | 3.8524 rad/s | 39.0725 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
