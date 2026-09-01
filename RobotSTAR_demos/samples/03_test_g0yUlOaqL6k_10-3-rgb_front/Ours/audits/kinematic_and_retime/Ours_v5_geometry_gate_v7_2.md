# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **FAIL**
- temporal continuity: **FAIL**
- mixed solve residual: **WARN** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0845463 m | 0.08 | FAIL |
| `right_palm_m_mean` | 0.0749277 m | 0.08 | PASS |
| `left_distal_deg_mean` | 36.2932 deg | 30 | FAIL |
| `right_distal_deg_mean` | 18.7323 deg | 30 | PASS |
| `left_normal_deg_mean` | 46.978 deg | 35 | FAIL |
| `right_normal_deg_mean` | 33.3787 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 6.50505 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 7.89108 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0556613 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.0764318 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.359781 | 0.2 | WARN |
| `solve_rmse_max` | 0.550428 | 0.4 | WARN |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 1.50456 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.108208 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.133667 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
