# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **PASS**
- temporal continuity: **FAIL**
- mixed solve residual: **PASS** (`hard_gated=False`)
- overall: **FAIL**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0341883 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0512218 m | 0.08 | PASS |
| `left_distal_deg_mean` | 5.02304 deg | 30 | PASS |
| `right_distal_deg_mean` | 10.6714 deg | 30 | PASS |
| `left_normal_deg_mean` | 5.06929 deg | 35 | PASS |
| `right_normal_deg_mean` | 14.9854 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 5.53014 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 7.54493 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0276137 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.137109 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.168184 | 0.2 | PASS |
| `solve_rmse_max` | 0.280734 | 0.4 | PASS |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.687006 rad/frame | 0.65 | FAIL |
| `max_left_hand_frame_step_rad` | 0.148882 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.136079 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
