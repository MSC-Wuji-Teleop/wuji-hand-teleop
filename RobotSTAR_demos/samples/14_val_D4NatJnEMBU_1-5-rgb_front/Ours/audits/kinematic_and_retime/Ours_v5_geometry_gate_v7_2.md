# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **FAIL**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0320454 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0759546 m | 0.08 | PASS |
| `left_distal_deg_mean` | 6.77169 deg | 30 | PASS |
| `right_distal_deg_mean` | 42.0904 deg | 30 | FAIL |
| `left_normal_deg_mean` | 12.3676 deg | 35 | PASS |
| `right_normal_deg_mean` | 24.7581 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 2.18329 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 9.78307 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0525405 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.114236 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.287687 | 0.2 | WARN |
| `solve_rmse_max` | 0.501777 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 1.33114 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.102958 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.132342 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
