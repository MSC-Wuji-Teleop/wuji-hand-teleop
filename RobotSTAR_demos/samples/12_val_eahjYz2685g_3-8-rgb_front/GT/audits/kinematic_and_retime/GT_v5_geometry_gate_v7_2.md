# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **PASS**
- temporal continuity: **FAIL**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.024542 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0477718 m | 0.08 | PASS |
| `left_distal_deg_mean` | 7.51096 deg | 30 | PASS |
| `right_distal_deg_mean` | 10.7026 deg | 30 | PASS |
| `left_normal_deg_mean` | 16.4709 deg | 35 | PASS |
| `right_normal_deg_mean` | 24.0826 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 3.16083 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 7.61098 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0425912 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.140335 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.202063 | 0.2 | WARN |
| `solve_rmse_max` | 0.372433 | 0.4 | PASS |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.724681 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.197046 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.243472 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
