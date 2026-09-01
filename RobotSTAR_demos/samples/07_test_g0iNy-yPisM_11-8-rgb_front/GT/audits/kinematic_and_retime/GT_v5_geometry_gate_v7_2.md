# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **PASS**
- temporal continuity: **PASS**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **PASS**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0520729 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0467778 m | 0.08 | PASS |
| `left_distal_deg_mean` | 11.128 deg | 30 | PASS |
| `right_distal_deg_mean` | 11.2522 deg | 30 | PASS |
| `left_normal_deg_mean` | 13.7259 deg | 35 | PASS |
| `right_normal_deg_mean` | 23.9383 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 5.29637 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 8.34093 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0439511 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.133808 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.211236 | 0.2 | WARN |
| `solve_rmse_max` | 0.365098 | 0.4 | PASS |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.562125 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.198548 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.250929 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
