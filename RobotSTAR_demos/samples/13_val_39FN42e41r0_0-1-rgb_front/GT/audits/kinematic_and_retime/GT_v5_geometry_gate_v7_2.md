# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **PASS**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0413542 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0406371 m | 0.08 | PASS |
| `left_distal_deg_mean` | 6.72224 deg | 30 | PASS |
| `right_distal_deg_mean` | 7.47984 deg | 30 | PASS |
| `left_normal_deg_mean` | 11.9748 deg | 35 | PASS |
| `right_normal_deg_mean` | 37.7511 deg | 35 | FAIL |
| `left_elbow_plane_deg_mean` | 2.42793 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 9.64827 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0448969 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.178432 m | 0.16 | FAIL |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.21891 | 0.2 | WARN |
| `solve_rmse_max` | 0.340418 | 0.4 | PASS |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.490652 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.142101 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.22859 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
