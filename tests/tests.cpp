#include "benchmark.hpp"
#include "config.hpp"
#include "intersection.hpp"
#include "reservation_table.hpp"
#include "scheduler.hpp"
#include "simulator.hpp"
#include "statistics.hpp"
#include "trajectory.hpp"
#include "uav.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool nearlyEqual(double left, double right) {
    return std::abs(left - right) < 1e-9;
}

Config testConfig() {
    Config config;
    config.dt = 1.0;
    config.speed = 1.0;
    config.max_search_time = 100.0;
    return config;
}

UAV northboundUav(int id, double arrival) {
    return {id,
            arrival,
            Direction::North,
            Direction::South,
            Movement::Straight,
            1.0};
}

void testSingleUavHasNoDelay() {
    const Config config = testConfig();
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    UAV uav = northboundUav(1, 2.5);

    scheduler.schedule(uav);

    require(nearlyEqual(uav.scheduled_entry_time, uav.arrival_time), "single UAV entry time");
    require(nearlyEqual(uav.delay, 0.0), "single UAV delay");
}

void testSameMiddlePathRequiresDelay() {
    const Config config = testConfig();
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    const CandidatePath middle =
        intersection.candidatePaths(Direction::North, Direction::South, Movement::Straight).front();
    const std::vector<CandidatePath> middle_only{middle};
    UAV first = northboundUav(1, 0.0);
    UAV second = northboundUav(2, 0.0);

    scheduler.schedule(first, middle_only);
    scheduler.schedule(second, middle_only);

    require(second.delay > 0.0, "second UAV must be delayed on the same path");
    require(table.hasNoOverlaps(), "same-path reservations must not overlap");
}

void testDifferentLayersCanEnterTogether() {
    const Config config = testConfig();
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    const std::vector<CandidatePath> upper{
        intersection.candidatePaths(Direction::North, Direction::South, Movement::Straight).at(1)};
    const std::vector<CandidatePath> lower{
        intersection.candidatePaths(Direction::East, Direction::West, Movement::Straight).at(2)};
    UAV first = northboundUav(1, 0.0);
    UAV second{
        2, 0.0, Direction::East, Direction::West, Movement::Straight, 1.0};

    scheduler.schedule(first, upper);
    scheduler.schedule(second, lower);

    require(nearlyEqual(first.delay, 0.0) && nearlyEqual(second.delay, 0.0),
            "spatially disjoint layers should allow simultaneous entry");
    require(table.hasNoOverlaps(), "different-layer reservations must remain valid");
}

void testOneCubeOverlapRejectsTrajectory() {
    ReservationTable table(10, 10, 3);
    const TimedTrajectory existing{
        0, {{{1, 1, 1}, 0.0, 1.0}, {{2, 1, 1}, 1.0, 2.0}}, 2.0};
    const TimedTrajectory candidate{
        1, {{{9, 9, 1}, 0.0, 1.0}, {{2, 1, 1}, 1.5, 2.5}}, 2.5};
    table.reserveTrajectory(existing, 1);

    require(!table.isTrajectoryAvailable(candidate), "one overlapping cube must reject trajectory");
}

void testTouchingHalfOpenIntervalsDoNotConflict() {
    ReservationTable table(10, 10, 3);
    const Cube cube{3, 3, 1};
    table.reserveTrajectory({0, {{cube, 1.0, 2.0}}, 2.0}, 1);

    require(table.isAvailable(cube, 2.0, 3.0), "[1,2) and [2,3) must not conflict");
}

void testSeededSimulationGlobalInvariant() {
    Config config = testConfig();
    config.seed = 2026;
    config.arrival_rate = 2.0;
    config.simulation_duration = 15.0;
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    Simulator simulator(config, scheduler);

    simulator.run(simulator.poissonTraffic());

    require(!simulator.results().empty(), "seeded simulation should create UAVs");
    require(table.hasNoOverlaps(), "all cube reservation intervals must be disjoint");
}

void testMiddleOnlySelectsOnlyMiddlePath() {
    Config config = testConfig();
    config.layer_mode = LayerMode::MiddleOnly;
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    Simulator simulator(config, scheduler);

    simulator.run(simulator.deterministicTraffic());

    for (const UAV& uav : simulator.results()) {
        require(uav.selected_path_id == 0, "MiddleOnly must never select upper/lower paths");
    }
}

void testThreeLayersCanSelectUpperPath() {
    Config config = testConfig();
    config.layer_mode = LayerMode::ThreeLayers;
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);

    table.reserveTrajectory({99, {{{4, 8, 1}, 1.0, 2.0}}, 2.0}, 99);
    UAV uav = northboundUav(1, 0.0);
    scheduler.schedule(uav);

    require(uav.selected_path_id == 1,
            "ThreeLayers should select upper path when middle conflicts at the same entry time");
    require(nearlyEqual(uav.delay, 0.0), "upper path should avoid an entry delay");
}

UAV completedFixture(double exit_time, double delay) {
    UAV uav = northboundUav(1, 0.0);
    uav.scheduled_entry_time = delay;
    uav.exit_time = exit_time;
    uav.delay = delay;
    uav.selected_path_id = 0;
    return uav;
}

void testExitAtWindowEndIsCompleted() {
    const UAV uav = completedFixture(10.0, 1.0);
    const SimulationSummary summary = computeSimulationSummary({uav}, 10.0);

    require(summary.completed_uav_count == 1U, "exit at duration must be completed");
    require(summary.unfinished_uav_count == 0U, "exit at duration must not be unfinished");
}

void testExitAfterWindowIsUnfinished() {
    const UAV uav = completedFixture(10.001, 1.0);
    const SimulationSummary summary = computeSimulationSummary({uav}, 10.0);

    require(summary.completed_uav_count == 0U, "exit after duration must not be completed");
    require(summary.unfinished_uav_count == 1U, "exit after duration must be unfinished");
}

void testThroughputUsesCompletedCount() {
    const std::vector<UAV> uavs{
        completedFixture(9.0, 0.0),
        completedFixture(11.0, 1.0),
        completedFixture(12.0, 2.0),
    };
    const SimulationSummary summary = computeSimulationSummary(uavs, 10.0);

    require(summary.generated_uav_count == 3U && summary.completed_uav_count == 1U,
            "throughput fixture counts");
    require(nearlyEqual(summary.throughput, 0.1),
            "throughput must be completed count divided by duration");
}

void testP95NearestRank() {
    std::vector<double> values;
    for (int value = 1; value <= 20; ++value) {
        values.push_back(static_cast<double>(value));
    }
    require(nearlyEqual(percentile95(values), 19.0),
            "P95 must use the sorted nearest-rank value");
}

std::vector<UAV> runSeededSimulation(const Config& config) {
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    Simulator simulator(config, scheduler);
    simulator.run(simulator.poissonTraffic());
    return simulator.results();
}

void testFixedSeedIsExactlyReproducible() {
    Config config = testConfig();
    config.seed = 314159;
    config.arrival_rate = 1.7;
    config.simulation_duration = 30.0;
    config.layer_mode = LayerMode::ThreeLayers;

    const std::vector<UAV> first = runSeededSimulation(config);
    const std::vector<UAV> second = runSeededSimulation(config);
    require(first.size() == second.size(), "fixed seed generated count");
    for (std::size_t index = 0; index < first.size(); ++index) {
        const UAV& left = first[index];
        const UAV& right = second[index];
        require(left.id == right.id && left.arrival_time == right.arrival_time &&
                    left.source == right.source && left.destination == right.destination &&
                    left.scheduled_entry_time == right.scheduled_entry_time &&
                    left.exit_time == right.exit_time && left.delay == right.delay &&
                    left.selected_path_id == right.selected_path_id,
                "fixed seed run must be exactly reproducible");
    }
}

void testBenchmarkAggregationMeanAndStd() {
    SimulationSummary first;
    first.average_delay_completed = 2.0;
    first.p95_delay = 4.0;
    first.throughput = 1.0;
    first.unfinished_uav_count = 3U;
    SimulationSummary second;
    second.average_delay_completed = 4.0;
    second.p95_delay = 8.0;
    second.throughput = 3.0;
    second.unfinished_uav_count = 5U;
    const std::vector<BenchmarkRun> runs{
        {1.0, LayerMode::MiddleOnly, 1000, first},
        {1.0, LayerMode::MiddleOnly, 1001, second},
    };

    const auto aggregates = aggregateBenchmarkRuns(runs);

    require(aggregates.size() == 1U && aggregates.front().runs == 2U,
            "benchmark aggregation group size");
    const BenchmarkAggregate& aggregate = aggregates.front();
    require(nearlyEqual(aggregate.mean_average_delay, 3.0), "aggregate delay mean");
    require(nearlyEqual(aggregate.std_average_delay, std::sqrt(2.0)),
            "aggregate sample delay std");
    require(nearlyEqual(aggregate.mean_p95_delay, 6.0), "aggregate P95 mean");
    require(nearlyEqual(aggregate.mean_throughput, 2.0), "aggregate throughput mean");
    require(nearlyEqual(aggregate.mean_unfinished_uav_count, 4.0),
            "aggregate unfinished mean");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, void (*)()>> tests{
        {"single UAV has no delay", testSingleUavHasNoDelay},
        {"same middle path requires delay", testSameMiddlePathRequiresDelay},
        {"different layers can enter together", testDifferentLayersCanEnterTogether},
        {"one cube overlap rejects trajectory", testOneCubeOverlapRejectsTrajectory},
        {"touching half-open intervals do not conflict", testTouchingHalfOpenIntervalsDoNotConflict},
        {"seeded simulation global invariant", testSeededSimulationGlobalInvariant},
        {"MiddleOnly selects only middle path", testMiddleOnlySelectsOnlyMiddlePath},
        {"ThreeLayers can select upper path", testThreeLayersCanSelectUpperPath},
        {"exit at window end is completed", testExitAtWindowEndIsCompleted},
        {"exit after window is unfinished", testExitAfterWindowIsUnfinished},
        {"throughput uses completed count", testThroughputUsesCompletedCount},
        {"P95 uses nearest rank", testP95NearestRank},
        {"fixed seed is exactly reproducible", testFixedSeedIsExactlyReproducible},
        {"benchmark aggregation mean and std", testBenchmarkAggregationMeanAndStd},
    };

    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
        }
    }

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return 1;
    }
    std::cout << tests.size() << " test(s) passed\n";
    return 0;
}
