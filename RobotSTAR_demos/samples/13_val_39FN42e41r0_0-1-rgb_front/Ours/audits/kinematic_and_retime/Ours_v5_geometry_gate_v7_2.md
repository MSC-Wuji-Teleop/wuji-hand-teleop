# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **PASS**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0658075 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0670785 m | 0.08 | PASS |
| `left_distal_deg_mean` | 12.7029 deg | 30 | PASS |
| `right_distal_deg_mean` | 17.7434 deg | 30 | PASS |
| `left_normal_deg_mean` | 31.134 deg | 35 | PASS |
| `right_normal_deg_mean` | 50.4975 deg | 35 | FAIL |
| `left_elbow_plane_deg_mean` | 3.68081 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 12.086 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0498567 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.136939 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.313016 | 0.2 | WARN |
| `solve_rmse_max` | 0.460605 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.410335 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.136608 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.134227 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
