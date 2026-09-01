# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **FAIL**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0983998 m | 0.08 | FAIL |
| `right_palm_m_mean` | 0.0999284 m | 0.08 | FAIL |
| `left_distal_deg_mean` | 45.6641 deg | 30 | FAIL |
| `right_distal_deg_mean` | 35.9652 deg | 30 | FAIL |
| `left_normal_deg_mean` | 56.0543 deg | 35 | FAIL |
| `right_normal_deg_mean` | 62.681 deg | 35 | FAIL |
| `left_elbow_plane_deg_mean` | 12.5985 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 8.07323 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0652977 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.141583 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.481228 | 0.2 | WARN |
| `solve_rmse_max` | 0.777736 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 3.25506 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.152947 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.126428 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
