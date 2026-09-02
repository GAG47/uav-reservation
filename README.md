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
- Fixed-speed timed trajectories
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

The tests cover reservation correctness, layer-mode selection, observation-window completion,
departure throughput, P95 delay, random-seed reproducibility, and benchmark aggregation.

## Run

```sh
./build/uav_reservation \
    --arrival-rate 0.5 \
    --duration 100 \
    --seed 42 \
    --mode three_layers
```

Run `./build/uav_reservation --help` for all options. Without arguments, the program uses the
defaults in `Config`. `arrival_rate` is the total rate for the whole intersection in UAV/s;
it is not a per-direction rate. Each generated UAV chooses one of the four sources uniformly.

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
