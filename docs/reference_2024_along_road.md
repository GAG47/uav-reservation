# Reference Scenario: Li et al. (2024)

This is an environment adapted from Li et al. (2024), not a full reproduction of the
original simulation. It implements the two-road, double-layer along-road intersection
needed by this project while retaining the existing EarliestEntry FCFS scheduler.

Reference: Shan Li, Honghai Zhang, Zhuolun Li, and Hao Liu, “Air Route Design of
Multi-Rotor UAVs for Urban Air Mobility,” *Drones*, 2024, 8, 601.

## Direct from Li et al. (2024)

| Parameter | Value |
|---|---:|
| Flight levels | 60 m / 90 m |
| Level spacing / elevator height | 30 m |
| Route width | 4 m |
| Crossing-area radius | 50 m |
| Horizontal speed | 6 m/s |
| Ascending speed | 4 m/s |
| Descending speed | 3 m/s |
| Maximum roll angle | 45° |
| Arrival-rate experiment range | 1–6 UAV/min/route |
| Layer-changing proportion | 20% |
| Reference UAV separation | Inspire 2–Inspire 2 |
| Horizontal separation | 15 m |
| Vertical separation | 1 m |

## Derived parameters

| Parameter | Formula / value |
|---|---:|
| Minimum turning radius | `6² / (9.81 tan 45°) = 3.669725 m` |
| Maximum turning radius | `4 / (1 - cos 90°) = 4.000000 m` |
| Fixed turning radius | `(r_min + r_max) / 2 = 3.834862 m` |
| Quarter-circle arc time | `r_turn × (π/2) / 6 = 1.003965 s` |
| 30 m ascent time | `30 / 4 = 7.5 s` |
| 30 m descent time | `30 / 3 = 10 s` |
| XY padding | `ceil(7.5 / 1) × 1 + 1 = 9 m` |
| Z padding per side | `(ceil(0.5 / 1) + 1) × 1 = 2 m` |
| Grid range | `x,y ∈ [-59,59) m`, `z ∈ [58,92) m` |
| Grid dimensions | `118 × 118 × 34` |

## Adapted parameters and rules

| Parameter / rule | Project implementation |
|---|---|
| UAV population | Inspire 2 only |
| Arrival process | Independent Poisson process at each active inlet |
| Movement probabilities | Straight 0.4, same-level turn 0.4, cross-level turn 0.2 |
| Ascent elevator | `(-8, 0)`, only 60 m → 90 m |
| Descent elevator | `(8, 0)`, only 90 m → 60 m |
| Cross-level connector | Orthogonal projections onto source/target centerlines |
| Connector heading change | Zero-time multirotor yaw at `P_in`, elevator, and `P_out` |
| Safety mapping | Horizontal cylinder radius 7.5 m, vertical half-height 0.5 m |
| Cube size | 1 m |
| Occupancy sampling interval | 0.05 s |
| Conflict resource | Cube-Time Reservation |

The cylindrical safety envelope is an adapted engineering mapping of the paper’s operational
horizontal and vertical separation values into this simulator’s Cube-Time resources. It is
not a direct reproduction of the paper’s spherical protected-zone formula.

The Poisson arrival distribution is inherited from this simulator. The paper provides mean
arrival rates but does not directly specify this random process.

The same-level turn uses a continuous quarter-circle. A cross-level turn instead follows the
specified projection connector and permits zero-time yaw at connector vertices; it does not
combine the connector with a same-level turning arc.
