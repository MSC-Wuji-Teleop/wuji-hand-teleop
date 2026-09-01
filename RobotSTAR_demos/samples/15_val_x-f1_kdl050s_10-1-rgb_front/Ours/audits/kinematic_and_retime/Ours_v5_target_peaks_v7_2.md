# SignAR V7.1 target peak diagnosis

- frames: `200`
- target FPS: `50.0`
- duration: `3.980` s

| Group | Velocity peak | Frame / joint | Acceleration peak | Frame / joint |
|---|---:|---|---:|---|
| `arm14` | 36.1455 rad/s | 6 / `19_left_wrist_roll` | 1095.61 rad/s² | 5 / `19_left_wrist_roll` |
| `left_hand20` | 4.05821 rad/s | 9 / `Lq_12` | 33.0484 rad/s² | 4 / `Lq_12` |
| `right_hand20` | 6.93701 rad/s | 9 / `Rq_18` | 77.1229 rad/s² | 105 / `Rq_03` |

## Projected 0.5× playback (2× duration)

| Group | Projected velocity peak | Projected acceleration peak |
|---|---:|---:|
| `arm14` | 18.0727 rad/s | 273.903 rad/s² |
| `left_hand20` | 2.02911 rad/s | 8.2621 rad/s² |
| `right_hand20` | 3.46851 rad/s | 19.2807 rad/s² |

> A sparse peak suggests a local discontinuity or IK branch event; global slowing is then the wrong primary fix.
> A broad peak can benefit from slower timing, but the kinematic geometry must still be inspected independently.
