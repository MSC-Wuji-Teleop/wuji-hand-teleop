# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **PASS**
- temporal continuity: **PASS**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **PASS**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0672897 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.061611 m | 0.08 | PASS |
| `left_distal_deg_mean` | 24.9689 deg | 30 | PASS |
| `right_distal_deg_mean` | 19.676 deg | 30 | PASS |
| `left_normal_deg_mean` | 34.3826 deg | 35 | PASS |
| `right_normal_deg_mean` | 33.2211 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 8.61261 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 7.51854 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0483671 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.156409 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.311777 | 0.2 | WARN |
| `solve_rmse_max` | 0.605201 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.517921 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.21654 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.18499 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
