# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **FAIL**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0929591 m | 0.08 | FAIL |
| `right_palm_m_mean` | 0.0425405 m | 0.08 | PASS |
| `left_distal_deg_mean` | 35.2381 deg | 30 | FAIL |
| `right_distal_deg_mean` | 9.98545 deg | 30 | PASS |
| `left_normal_deg_mean` | 71.6185 deg | 35 | FAIL |
| `right_normal_deg_mean` | 25.5544 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 12.5214 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 7.21168 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0696197 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.124101 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.381617 | 0.2 | WARN |
| `solve_rmse_max` | 0.562716 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 3.22846 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.297673 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.195447 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
