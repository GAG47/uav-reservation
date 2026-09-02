# 3D UAV Intersection Reservation Simulator

This project simulates centralized FCFS reservations for UAVs traversing one discretized
3D intersection. Each accepted flight reserves a sequence of cube-time intervals, so no
two UAVs can occupy the same cube at overlapping times.

This is a simplified first-stage reproduction of the reservation-based 3D intersection idea,
extended with a benchmark for comparing 2D and 3D FCFS reservations. It is not a complete
reproduction of any paper.

## Current scope

- A `10 x 10 x 3` intersection grid with configurable cube size
- Straight movements from North, South, East, and West to the opposite side
- Three predefined candidate paths per movement: middle, upper, and lower
- Horizontal and vertical trajectory segments with separate fixed speeds
- Spherical UAV safety volumes sampled along continuous 3D motion
- Swept-volume Cube occupancy with merged time intervals
- Half-open cube reservations (`[start, end)`)
- Deterministic and seeded Poisson traffic generation
- FCFS ordering by arrival time, then UAV ID
- CSV results and summary statistics
- Configurable 2D-FCFS and 3D-FCFS experiment modes
- C++ batch benchmark with raw and aggregated CSV output

GA, A*, Modified A*, ROS, Gazebo, AirSim, GUI, communication effects, localization
error, weather, obstacles, and detailed vehicle dynamics are intentionally not implemented.

## Build

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

## Test

```sh
ctest --test-dir build --output-on-failure
```

The tests cover reservation correctness, horizontal/vertical travel time, safety-volume
occupancy, vertical-transition conflicts, height separation, sampling validation, unchanged
EarliestEntry FCFS behavior, layer-mode selection, statistics, and reproducibility.

## Run

```sh
./build/uav_reservation \
    --arrival-rate 0.5 \
    --duration 100 \
    --seed 42 \
    --mode three_layers \
    --horizontal-speed 2.0 \
    --vertical-speed 1.0 \
    --uav-radius 0.15 \
    --safety-margin 0.10 \
    --occupancy-dt 0.10
```

Run `./build/uav_reservation --help` for all options. Without arguments, the program uses the
defaults in `Config`. `arrival_rate` is the total rate for the whole intersection in UAV/s;
it is not a per-direction rate. Each generated UAV chooses one of the four sources uniformly.

The middle path is one horizontal segment. Upper and lower paths consist of a vertical
transition, a horizontal crossing, and a final vertical transition. Segment time is distance
divided by the matching horizontal or vertical speed. At each occupancy sampling interval,
the simulator reserves every grid Cube intersected by the UAV safety sphere, whose radius is
`uav_radius + safety_margin`. Consecutive or overlapping intervals for the same Cube are
merged before reservation. Configuration is rejected when the maximum center movement per
sample exceeds `0.5 * cube_size`.

The run writes `results/results.csv` and prints generated, completed, and unfinished counts;
completed/all-scheduled average delays; maximum and P95 completed-UAV delays; average system
time; and departure throughput. A UAV is completed only when
`exit_time <= simulation_duration`. Therefore:

```text
throughput = completed_uav_count / simulation_duration
```

The CSV fields are:

```text
uav_id,arrival_time,source,destination,scheduled_entry_time,exit_time,delay,selected_path_id,completed
```

## Benchmark: 2D-FCFS vs 3D-FCFS

- `MiddleOnly` (`middle_only`) is the 2D-FCFS baseline and permits only path 0 at `z = 1`.
- `ThreeLayers` (`three_layers`) is 3D-FCFS and permits middle, upper, and lower paths.
- Both modes use the same FCFS scheduler; only the candidate-path set changes.

Run the default benchmark:

```sh
./build/uav_reservation_benchmark
```

It scans total arrival rates `0.2, 0.4, ..., 2.0, 2.5, 3.0` UAV/s. Each mode/rate pair
uses 10 seeds (`1000` through `1009`) and a 300-second observation window, for 240 runs.
The C++ executable performs all simulation and aggregation, using sample standard deviation,
and writes:

- `results/benchmark_raw.csv`: one row per simulation run
- `results/benchmark_summary.csv`: mean and standard deviation by rate and layer mode

Create the comparison plots after running the benchmark:

```sh
python3 scripts/plot_benchmark.py
```

This requires Matplotlib and produces average-delay, P95-delay, and departure-throughput
plots in `results/`. Inside an activated virtual environment, `python` may be used instead.

## Reference Scenario: Li et al. (2024)

The project also includes a two-road, double-layer along-road environment selected with:

```sh
./build/uav_reservation \
    --scenario reference-2024-along-road \
    --arrival-rate-per-route 3 \
    --seed 42
```

The four active inlets each use an independent Poisson arrival process. The CLI rate is in
UAV/min/route, so a value of 3 means an expected total of 12 UAV/min over four routes. Poisson
arrival is inherited from this simulator and is not directly specified by the 2024 paper.

Reference paths use continuous world coordinates, 60/90 m flight levels, continuous
quarter-circle same-level turns, and separate one-way ascent/descent elevators for cross-level
turns. The reference safety model is a deterministic cylindrical envelope that adapts the
paper’s 15 m horizontal and 1 m vertical operational separation values to Cube-Time
Reservation. It is not a direct reproduction of the paper’s spherical protected-zone formula.

Direct, derived, and adapted parameters—including the actual `118 × 118 × 34` grid covering
`[-59,59) × [-59,59) × [58,92)` m—are documented in
[docs/reference_2024_along_road.md](docs/reference_2024_along_road.md).

For a small deterministic geometry check:

```sh
./build/uav_reservation --scenario reference-2024-along-road --deterministic
python3 scripts/plot_reference_trajectory.py
```

This writes `results/trajectory_debug.csv` and `results/reference_trajectory_debug.png`.
