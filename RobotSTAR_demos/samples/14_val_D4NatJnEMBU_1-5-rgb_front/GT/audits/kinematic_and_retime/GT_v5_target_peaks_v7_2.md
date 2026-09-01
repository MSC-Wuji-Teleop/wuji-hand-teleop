# SignAR V7.1 target peak diagnosis

- frames: `930`
- target FPS: `50.0`
- duration: `18.580` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 50.5916 rad/s | 404 / `19_left_wrist_roll` | 1491.67 rad/s² | 403 / `19_left_wrist_roll` |
| `left_hand20` | 8.84629 rad/s | 270 / `Lq_03` | 240.403 rad/s² | 405 / `Lq_00` |
| `right_hand20` | 9.64244 rad/s | 410 / `Rq_14` | 297.048 rad/s² | 201 / `Rq_06` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 25.2958 rad/s | 372.919 rad/s² |
| `left_hand20` | 4.42314 rad/s | 60.1007 rad/s² |
| `right_hand20` | 4.82122 rad/s | 74.2619 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
