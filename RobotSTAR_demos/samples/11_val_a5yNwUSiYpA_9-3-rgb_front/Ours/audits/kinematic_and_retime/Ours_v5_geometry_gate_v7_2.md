# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **PASS**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0214578 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0751659 m | 0.08 | PASS |
| `left_distal_deg_mean` | 4.43904 deg | 30 | PASS |
| `right_distal_deg_mean` | 28.5134 deg | 30 | PASS |
| `left_normal_deg_mean` | 6.71555 deg | 35 | PASS |
| `right_normal_deg_mean` | 94.1108 deg | 35 | FAIL |
| `left_elbow_plane_deg_mean` | 2.43144 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 6.35994 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0505053 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.132975 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.35791 | 0.2 | WARN |
| `solve_rmse_max` | 0.495047 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.45297 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.150547 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.138277 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
