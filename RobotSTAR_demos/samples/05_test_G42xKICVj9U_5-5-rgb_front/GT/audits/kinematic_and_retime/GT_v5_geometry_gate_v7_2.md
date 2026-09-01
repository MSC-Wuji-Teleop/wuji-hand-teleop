# SignAR V7.1 — unchanged V5/V5.1 geometry gate

- contract: **PASS**
- task-space geometry: **PASS**
- temporal continuity: **PASS**
- mixed solve residual: **PASS** (`hard_gated=False`)
- overall: **PASS**

## Task-space geometry (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `left_palm_m_mean` | 0.0341286 m | 0.08 | PASS |
| `right_palm_m_mean` | 0.0478262 m | 0.08 | PASS |
| `left_distal_deg_mean` | 3.58156 deg | 30 | PASS |
| `right_distal_deg_mean` | 9.18418 deg | 30 | PASS |
| `left_normal_deg_mean` | 3.82764 deg | 35 | PASS |
| `right_normal_deg_mean` | 16.2741 deg | 35 | PASS |
| `left_elbow_plane_deg_mean` | 3.60072 deg | 35 | PASS |
| `right_elbow_plane_deg_mean` | 6.15866 deg | 35 | PASS |
| `inter_palm_vector_error_mean_m` | 0.0307087 m | 0.08 | PASS |
| `contact_pair_vector_error_mean_m` | 0.120473 m | 0.16 | PASS |

## Solver residual (diagnostic by default)

| Metric | Value | Reference max | Status |
|---|---:|---:|---|
| `solve_rmse_mean` | 0.156543 | 0.2 | PASS |
| `solve_rmse_max` | 0.275567 | 0.4 | PASS |

## Continuity (hard gate)

| Metric | Value | Max | Result |
|---|---:|---:|---|
| `max_arm_frame_step_rad` | 0.359575 rad/frame | 0.65 | PASS |
| `max_left_hand_frame_step_rad` | 0.198248 rad/frame | 1.2 | PASS |
| `max_right_hand_frame_step_rad` | 0.218767 rad/frame | 1.2 | PASS |

> `solve_rmse` combines differently weighted meters, directions, radians, regularizers and contact terms. It should not stop an otherwise acceptable target unless explicitly requested.
> Numeric checks do not replace watching the V5/V5.1 kinematic preview. A controller cannot repair a wrong target.
