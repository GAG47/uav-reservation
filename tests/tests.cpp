#include "benchmark.hpp"
#include "config.hpp"
#include "intersection.hpp"
#include "reference_scenario.hpp"
#include "reservation_table.hpp"
#include "scheduler.hpp"
#include "simulator.hpp"
#include "statistics.hpp"
#include "trajectory.hpp"
#include "uav.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_set>
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
    config.horizontal_speed = 1.0;
    config.ascending_speed = 0.5;
    config.descending_speed = 0.5;
    config.uav_radius = 0.10;
    config.safety_margin = 0.05;
    config.occupancy_dt = 0.10;
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

void testHorizontalMovementTime() {
    const TrajectorySegment segment{
        {0.0, 0.0, 1.5}, {10.0, 0.0, 1.5}, SegmentType::HorizontalLine};
    require(nearlyEqual(segmentTravelTime(segment, 5.0, 2.5, 2.5), 2.0),
            "10 m horizontal movement at 5 m/s must take 2 s");
}

void testVerticalMovementTime() {
    const TrajectorySegment segment{
        {2.0, 3.0, 0.0}, {2.0, 3.0, 5.0}, SegmentType::Ascending};
    require(nearlyEqual(segmentTravelTime(segment, 5.0, 2.5, 2.5), 2.0),
            "5 m vertical movement at 2.5 m/s must take 2 s");
}

void testSlowVerticalSpeedIncreasesLayeredPathTime() {
    Config config = testConfig();
    config.horizontal_speed = 2.0;
    config.ascending_speed = 0.25;
    config.descending_speed = 0.25;
    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    const auto& paths =
        intersection.candidatePaths(Direction::North, Direction::South, Movement::Straight);

    const TimedTrajectory middle = makeTimedTrajectory(paths.at(0), 0.0, config);
    const TimedTrajectory upper = makeTimedTrajectory(paths.at(1), 0.0, config);
    const TimedTrajectory lower = makeTimedTrajectory(paths.at(2), 0.0, config);

    require(upper.exit_time > middle.exit_time && lower.exit_time > middle.exit_time,
            "slow vertical speed must add real travel time to upper/lower paths");
    require(nearlyEqual(upper.exit_time, lower.exit_time),
            "symmetric upper/lower paths must have equal travel times");
}

CandidatePath horizontalPath(int id, double z) {
    return {id,
            {{{2.5, 4.5, z}, {6.5, 4.5, z}, SegmentType::HorizontalLine}}};
}

std::unordered_set<Cube, CubeHash> occupiedCubes(const TimedTrajectory& trajectory) {
    std::unordered_set<Cube, CubeHash> cubes;
    for (const Occupancy& occupancy : trajectory.occupancies) {
        cubes.insert(occupancy.cube);
    }
    return cubes;
}

void testSafetySphereOccupiesMultipleCubes() {
    Config config = testConfig();
    config.uav_radius = 0.60;
    config.safety_margin = 0.0;
    const TimedTrajectory trajectory =
        makeTimedTrajectory(horizontalPath(10, 1.5), 0.0, config);

    std::size_t simultaneous_cube_count = 0U;
    constexpr double observation_time = 0.05;
    for (const Occupancy& occupancy : trajectory.occupancies) {
        if (occupancy.start_time <= observation_time &&
            observation_time < occupancy.end_time) {
            ++simultaneous_cube_count;
        }
    }
    require(simultaneous_cube_count > 1U,
            "a sufficiently large safety sphere must occupy adjacent cubes simultaneously");
}

void testLargerSafetyMarginDoesNotReduceOccupiedCubes() {
    Config small = testConfig();
    small.uav_radius = 0.10;
    small.safety_margin = 0.0;
    Config large = small;
    large.safety_margin = 0.55;
    const CandidatePath path = horizontalPath(11, 1.5);
    const auto small_cubes = occupiedCubes(makeTimedTrajectory(path, 0.0, small));
    const auto large_cubes = occupiedCubes(makeTimedTrajectory(path, 0.0, large));

    require(large_cubes.size() >= small_cubes.size(),
            "larger margin must not reduce occupied cube count");
    for (const Cube& cube : small_cubes) {
        require(large_cubes.contains(cube),
                "large-margin occupancy must include every small-margin cube");
    }
}

void testVerticalTransitionVolumeConflicts() {
    Config config = testConfig();
    config.horizontal_speed = 2.0;
    config.ascending_speed = 1.0;
    config.descending_speed = 1.0;
    const CandidatePath vertical{
        20, {{{4.5, 4.5, 1.5}, {4.5, 4.5, 2.5}, SegmentType::Ascending}}};
    const CandidatePath crossing{
        21, {{{3.5, 4.5, 2.0}, {5.5, 4.5, 2.0}, SegmentType::HorizontalLine}}};
    const TimedTrajectory vertical_trajectory =
        makeTimedTrajectory(vertical, 0.0, config);
    const TimedTrajectory crossing_trajectory =
        makeTimedTrajectory(crossing, 0.0, config);
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(vertical_trajectory, 1);

    require(!table.isTrajectoryAvailable(crossing_trajectory),
            "a horizontal path through an active vertical transition must conflict");
}

void testSufficientHeightSeparationIsAvailable() {
    Config config = testConfig();
    const CandidatePath low{
        30, {{{2.5, 4.5, 0.5}, {6.5, 4.5, 0.5}, SegmentType::HorizontalLine}}};
    const CandidatePath high{
        31, {{{4.5, 2.5, 2.5}, {4.5, 6.5, 2.5}, SegmentType::HorizontalLine}}};
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(low, 0.0, config), 1);

    require(table.isTrajectoryAvailable(makeTimedTrajectory(high, 0.0, config)),
            "XY crossings with sufficient height separation must remain available");
}

void testInsufficientHeightSeparationConflicts() {
    Config config = testConfig();
    config.uav_radius = 0.20;
    config.safety_margin = 0.0;
    const CandidatePath lower{
        40, {{{2.5, 4.5, 0.9}, {6.5, 4.5, 0.9}, SegmentType::HorizontalLine}}};
    const CandidatePath higher{
        41, {{{4.5, 2.5, 1.1}, {4.5, 6.5, 1.1}, SegmentType::HorizontalLine}}};
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(lower, 0.0, config), 1);

    require(!table.isTrajectoryAvailable(makeTimedTrajectory(higher, 0.0, config)),
            "XY crossings with overlapping safety volumes must conflict");
}

void testUnsafeOccupancyDtIsRejected() {
    Config config = testConfig();
    config.horizontal_speed = 2.0;
    config.occupancy_dt = 1.0;
    bool rejected = false;
    try {
        static_cast<void>(makeTimedTrajectory(horizontalPath(50, 1.5), 0.0, config));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "occupancy_dt allowing movement over half a cube must be rejected");
}

void testOriginalEarliestEntryFcfsBehavior() {
    Config config = testConfig();
    config.layer_mode = LayerMode::MiddleOnly;
    const Intersection order_intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable order_table(config.nx, config.ny, config.nz);
    FCFSScheduler order_scheduler(config, order_intersection, order_table);
    Simulator simulator(config, order_scheduler);
    simulator.run({northboundUav(2, 0.0), northboundUav(1, 0.0)});
    require(simulator.results().at(0).id == 1 && simulator.results().at(1).id == 2,
            "FCFS ties must still be ordered by UAV id");

    const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    UAV first = northboundUav(1, 0.0);
    UAV second = northboundUav(2, 0.0);
    scheduler.schedule(first);

    const CandidatePath& middle =
        intersection.candidatePaths(Direction::North, Direction::South, Movement::Straight).front();
    double first_feasible_entry = -1.0;
    for (int step = 0; step <= 10; ++step) {
        const double entry_time = static_cast<double>(step) * config.dt;
        if (table.isTrajectoryAvailable(makeTimedTrajectory(middle, entry_time, config))) {
            first_feasible_entry = entry_time;
            break;
        }
    }
    scheduler.schedule(second);

    require(nearlyEqual(first.scheduled_entry_time, 0.0),
            "first FCFS UAV must enter at its arrival time");
    require(first_feasible_entry >= 0.0 &&
                nearlyEqual(second.scheduled_entry_time, first_feasible_entry),
            "second UAV must use the first feasible dt entry slot");
}

Config referenceConfig() {
    Config config = makeReference2024Config();
    config.simulation_duration = 120.0;
    config.max_search_time = 300.0;
    return config;
}

const TrajectorySegment& verticalSegment(const CandidatePath& path) {
    for (const TrajectorySegment& segment : path.segments) {
        if (segment.type == SegmentType::Ascending ||
            segment.type == SegmentType::Descending) {
            return segment;
        }
    }
    throw std::runtime_error("expected a vertical segment");
}

const TrajectorySegment& arcSegment(const CandidatePath& path) {
    for (const TrajectorySegment& segment : path.segments) {
        if (segment.type == SegmentType::HorizontalArc) {
            return segment;
        }
    }
    throw std::runtime_error("expected an arc segment");
}

void testReferenceLevelsAndGrid() {
    const Config config = referenceConfig();
    require(nearlyEqual(config.reference.lower_level, 60.0) &&
                nearlyEqual(config.reference.upper_level, 90.0),
            "reference levels must be 60/90 m");
    require(config.nx == 118 && config.ny == 118 && config.nz == 34,
            "reference grid dimensions must follow the padding formula");
    require(nearlyEqual(config.grid_origin_x, -59.0) &&
                nearlyEqual(config.grid_origin_y, -59.0) &&
                nearlyEqual(config.grid_origin_z, 58.0),
            "reference grid origins must follow the padding formula");
}

void testReferenceHorizontalTravelTime() {
    const Config config = referenceConfig();
    const TrajectorySegment segment{
        {0.0, 0.0, 60.0}, {60.0, 0.0, 60.0}, SegmentType::HorizontalLine};
    require(nearlyEqual(
                segmentTravelTime(
                    segment,
                    config.horizontal_speed,
                    config.ascending_speed,
                    config.descending_speed),
                10.0),
            "reference 60 m horizontal travel must take 10 s");
}

void testReferenceAscendingTravelTime() {
    const Config config = referenceConfig();
    const TrajectorySegment segment{
        {-8.0, 0.0, 60.0}, {-8.0, 0.0, 90.0}, SegmentType::Ascending};
    require(nearlyEqual(
                segmentTravelTime(
                    segment,
                    config.horizontal_speed,
                    config.ascending_speed,
                    config.descending_speed),
                7.5),
            "reference ascent must take 7.5 s");
}

void testReferenceDescendingTravelTime() {
    const Config config = referenceConfig();
    const TrajectorySegment segment{
        {8.0, 0.0, 90.0}, {8.0, 0.0, 60.0}, SegmentType::Descending};
    require(nearlyEqual(
                segmentTravelTime(
                    segment,
                    config.horizontal_speed,
                    config.ascending_speed,
                    config.descending_speed),
                10.0),
            "reference descent must take 10 s");
}

void testReferenceMinimumTurningRadiusFormula() {
    const Config config = referenceConfig();
    const double expected = 36.0 / (9.81 * std::tan(std::numbers::pi / 4.0));
    require(nearlyEqual(referenceMinimumTurningRadius(config), expected),
            "reference minimum turning radius formula");
}

void testReferenceMaximumTurningRadiusFormula() {
    const Config config = referenceConfig();
    require(nearlyEqual(referenceMaximumTurningRadius(config), 4.0),
            "reference maximum turning radius must be 4 m");
}

void testReferenceTurningRadiusWithinBounds() {
    const Config config = referenceConfig();
    const double minimum = referenceMinimumTurningRadius(config);
    const double maximum = referenceMaximumTurningRadius(config);
    const double turn = referenceTurningRadius(config);
    require(minimum < turn && turn < maximum &&
                nearlyEqual(turn, (minimum + maximum) / 2.0),
            "reference turn radius must be the midpoint within strict bounds");
}

void testReferenceLeftTurnQuarterCircle() {
    const Config config = referenceConfig();
    const CandidatePath path =
        makeReferenceCandidatePath(config, FlightDirection::Eastbound, Movement::LeftTurn);
    const TrajectorySegment& arc = arcSegment(path);
    require(nearlyEqual(arc.radius, referenceTurningRadius(config)) &&
                nearlyEqual(arc.sweep_angle, std::numbers::pi / 2.0),
            "same-level left turn must be a continuous positive quarter circle");
}

void testReferenceRightTurnQuarterCircle() {
    const Config config = referenceConfig();
    const CandidatePath path =
        makeReferenceCandidatePath(config, FlightDirection::Northbound, Movement::RightTurn);
    const TrajectorySegment& arc = arcSegment(path);
    require(nearlyEqual(arc.radius, referenceTurningRadius(config)) &&
                nearlyEqual(arc.sweep_angle, -std::numbers::pi / 2.0),
            "same-level right turn must be a continuous negative quarter circle");
}

void testReferenceArcTangents() {
    const Config config = referenceConfig();
    for (const auto& [source, movement] : {
             std::pair{FlightDirection::Eastbound, Movement::LeftTurn},
             std::pair{FlightDirection::Northbound, Movement::RightTurn}}) {
        const CandidatePath path = makeReferenceCandidatePath(config, source, movement);
        const TrajectorySegment& arc = arcSegment(path);
        const FlightDirection target = referenceTargetDirection(source, movement);
        const Vector2D incoming = referenceDirectionVector(source);
        const Vector2D outgoing = referenceDirectionVector(target);
        const double sign = arc.sweep_angle > 0.0 ? 1.0 : -1.0;
        const Vector2D start_tangent{
            sign * -std::sin(arc.start_angle), sign * std::cos(arc.start_angle)};
        const double end_angle = arc.start_angle + arc.sweep_angle;
        const Vector2D end_tangent{
            sign * -std::sin(end_angle), sign * std::cos(end_angle)};
        require(nearlyEqual(start_tangent.x, incoming.x) &&
                    nearlyEqual(start_tangent.y, incoming.y) &&
                    nearlyEqual(end_tangent.x, outgoing.x) &&
                    nearlyEqual(end_tangent.y, outgoing.y),
                "arc start/end tangents must match route headings");
    }
}

void testReferenceStraightCrossesCenter() {
    const Config config = referenceConfig();
    const CandidatePath path =
        makeReferenceCandidatePath(config, FlightDirection::Northbound, Movement::Straight);
    require(path.segments.size() == 1U &&
                path.segments.front().type == SegmentType::HorizontalLine &&
                nearlyEqual(path.segments.front().start.x, 0.0) &&
                nearlyEqual(path.segments.front().end.x, 0.0) &&
                path.segments.front().start.y < 0.0 && path.segments.front().end.y > 0.0,
            "reference straight path must continuously cross the origin");
}

void requireDirectionMapping(
    FlightDirection source,
    Movement same_level_turn,
    Movement cross_level_turn,
    FlightDirection same_target,
    FlightDirection cross_target) {
    require(referenceTargetDirection(source, Movement::Straight) == source,
            "straight target mapping");
    require(referenceTargetDirection(source, same_level_turn) == same_target &&
                !referenceMovementChangesLevel(source, same_level_turn),
            "same-level target mapping");
    require(referenceTargetDirection(source, cross_level_turn) == cross_target &&
                referenceMovementChangesLevel(source, cross_level_turn),
            "cross-level target mapping");
}

void testReferenceNorthboundMapping() {
    requireDirectionMapping(
        FlightDirection::Northbound,
        Movement::RightTurn,
        Movement::LeftTurn,
        FlightDirection::Eastbound,
        FlightDirection::Westbound);
}

void testReferenceEastboundMapping() {
    requireDirectionMapping(
        FlightDirection::Eastbound,
        Movement::LeftTurn,
        Movement::RightTurn,
        FlightDirection::Northbound,
        FlightDirection::Southbound);
}

void testReferenceSouthboundMapping() {
    requireDirectionMapping(
        FlightDirection::Southbound,
        Movement::RightTurn,
        Movement::LeftTurn,
        FlightDirection::Westbound,
        FlightDirection::Eastbound);
}

void testReferenceWestboundMapping() {
    requireDirectionMapping(
        FlightDirection::Westbound,
        Movement::LeftTurn,
        Movement::RightTurn,
        FlightDirection::Southbound,
        FlightDirection::Northbound);
}

void testCrossLevelUsesCorrectElevator() {
    const Config config = referenceConfig();
    for (const auto& [source, movement] : {
             std::pair{FlightDirection::Southbound, Movement::LeftTurn},
             std::pair{FlightDirection::Westbound, Movement::RightTurn}}) {
        const CandidatePath path = makeReferenceCandidatePath(config, source, movement);
        const TrajectorySegment& vertical = verticalSegment(path);
        require(vertical.type == SegmentType::Ascending &&
                    nearlyEqual(vertical.start.x, -8.0),
                "ascending movement must use ascent elevator");
    }
    for (const auto& [source, movement] : {
             std::pair{FlightDirection::Northbound, Movement::LeftTurn},
             std::pair{FlightDirection::Eastbound, Movement::RightTurn}}) {
        const CandidatePath path = makeReferenceCandidatePath(config, source, movement);
        const TrajectorySegment& vertical = verticalSegment(path);
        require(vertical.type == SegmentType::Descending &&
                    nearlyEqual(vertical.start.x, 8.0),
                "descending movement must use descent elevator");
    }
}

void testSameLevelMovementAvoidsElevators() {
    const Config config = referenceConfig();
    for (const auto& [source, movement] : {
             std::pair{FlightDirection::Northbound, Movement::Straight},
             std::pair{FlightDirection::Northbound, Movement::RightTurn},
             std::pair{FlightDirection::Eastbound, Movement::LeftTurn},
             std::pair{FlightDirection::Southbound, Movement::RightTurn},
             std::pair{FlightDirection::Westbound, Movement::LeftTurn}}) {
        const CandidatePath path = makeReferenceCandidatePath(config, source, movement);
        for (const TrajectorySegment& segment : path.segments) {
            require(segment.type != SegmentType::Ascending &&
                        segment.type != SegmentType::Descending,
                    "same-level movement must not enter an elevator");
        }
    }
}

void testAscendingNeverUsesDescentElevator() {
    const Config config = referenceConfig();
    const CandidatePath path = makeReferenceCandidatePath(
        config, FlightDirection::Southbound, Movement::LeftTurn);
    const TrajectorySegment& segment = verticalSegment(path);
    require(segment.type == SegmentType::Ascending && nearlyEqual(segment.start.x, -8.0) &&
                !nearlyEqual(segment.start.x, 8.0),
            "ascending movement must not use descent elevator");
}

void testDescendingNeverUsesAscentElevator() {
    const Config config = referenceConfig();
    const CandidatePath path = makeReferenceCandidatePath(
        config, FlightDirection::Northbound, Movement::LeftTurn);
    const TrajectorySegment& segment = verticalSegment(path);
    require(segment.type == SegmentType::Descending && nearlyEqual(segment.start.x, 8.0) &&
                !nearlyEqual(segment.start.x, -8.0),
            "descending movement must not use ascent elevator");
}

void testCrossLevelProjectionConnectors() {
    const Config config = referenceConfig();
    const CandidatePath path = makeReferenceCandidatePath(
        config, FlightDirection::Northbound, Movement::LeftTurn);
    const TrajectorySegment& vertical = verticalSegment(path);
    require(path.segments.size() == 4U &&
                nearlyEqual(path.segments.at(0).end.x, 0.0) &&
                nearlyEqual(path.segments.at(0).end.y, 0.0) &&
                nearlyEqual(path.segments.at(1).start.x, 0.0) &&
                nearlyEqual(path.segments.at(1).end.x, 8.0) &&
                nearlyEqual(vertical.start.x, 8.0) &&
                nearlyEqual(path.segments.back().start.y, 0.0),
            "cross-level connectors must use orthogonal route projections");
}

CandidatePath referenceShortLine(int id, double x, double z) {
    return {id,
            {{{x, -0.3, z}, {x, 0.3, z}, SegmentType::HorizontalLine}}};
}

void testReferenceHorizontalSeparationConflict() {
    const Config config = referenceConfig();
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(referenceShortLine(1, 0.0, 60.0), 0.0, config), 1);
    require(!table.isTrajectoryAvailable(
                makeTimedTrajectory(referenceShortLine(2, 14.0, 60.0), 0.0, config)),
            "same-level centers less than 15 m apart must conflict");
}

void testReferenceHorizontalSeparationAvailable() {
    const Config config = referenceConfig();
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(referenceShortLine(1, -8.5, 60.0), 0.0, config), 1);
    require(table.isTrajectoryAvailable(
                makeTimedTrajectory(referenceShortLine(2, 8.5, 60.0), 0.0, config)),
            "same-level centers more than 15 m apart must not conflict solely by envelope");
}

void testReferenceFlightLevelsDoNotConflict() {
    const Config config = referenceConfig();
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(referenceShortLine(1, 0.0, 60.0), 0.0, config), 1);
    require(table.isTrajectoryAvailable(
                makeTimedTrajectory(referenceShortLine(2, 0.0, 90.0), 0.0, config)),
            "60 m and 90 m flights must not conflict by safety envelope");
}

void testSameElevatorVerticalConflict() {
    const Config config = referenceConfig();
    const CandidatePath ascent{
        1, {{{-8.0, 0.0, 60.0}, {-8.0, 0.0, 90.0}, SegmentType::Ascending}}};
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(ascent, 0.0, config), 1);
    require(!table.isTrajectoryAvailable(makeTimedTrajectory(ascent, 0.1, config)),
            "same-elevator UAVs violating vertical separation must conflict");
}

void testSeparateElevatorsCanOperateTogether() {
    const Config config = referenceConfig();
    const CandidatePath ascent{
        1, {{{-8.0, 0.0, 60.0}, {-8.0, 0.0, 90.0}, SegmentType::Ascending}}};
    const CandidatePath descent{
        2, {{{8.0, 0.0, 90.0}, {8.0, 0.0, 60.0}, SegmentType::Descending}}};
    ReservationTable table(config.nx, config.ny, config.nz);
    table.reserveTrajectory(makeTimedTrajectory(ascent, 0.0, config), 1);
    require(table.isTrajectoryAvailable(makeTimedTrajectory(descent, 0.0, config)),
            "spatially separated elevators must allow simultaneous operation");
}

std::vector<UAV> generatedReferenceTraffic(Config config) {
    const Intersection intersection(config);
    ReservationTable table(config.nx, config.ny, config.nz);
    FCFSScheduler scheduler(config, intersection, table);
    Simulator simulator(config, scheduler);
    return simulator.referencePoissonTraffic();
}

void testReferenceArrivalSeedReproducible() {
    Config config = referenceConfig();
    config.seed = 2468;
    const auto first = generatedReferenceTraffic(config);
    const auto second = generatedReferenceTraffic(config);
    require(first.size() == second.size(), "reference seeded arrival count");
    for (std::size_t index = 0; index < first.size(); ++index) {
        require(first[index].arrival_time == second[index].arrival_time &&
                    first[index].source_direction == second[index].source_direction,
                "reference arrivals must be exactly reproducible");
    }
}

void testReferenceMovementSeedReproducible() {
    Config config = referenceConfig();
    config.seed = 1357;
    const auto first = generatedReferenceTraffic(config);
    const auto second = generatedReferenceTraffic(config);
    require(first.size() == second.size(), "reference seeded movement count");
    for (std::size_t index = 0; index < first.size(); ++index) {
        require(first[index].movement == second[index].movement &&
                    first[index].target_direction == second[index].target_direction &&
                    first[index].target_level == second[index].target_level,
                "reference movements must be exactly reproducible");
    }
}

void testReferenceLayerChangeRatio() {
    Config config = referenceConfig();
    config.seed = 9876;
    config.simulation_duration = 60000.0;
    config.reference.arrival_rate_per_route = 6.0;
    const auto traffic = generatedReferenceTraffic(config);
    std::size_t changes = 0U;
    for (const UAV& uav : traffic) {
        if (uav.source_level != uav.target_level) {
            ++changes;
        }
    }
    const double ratio = static_cast<double>(changes) / static_cast<double>(traffic.size());
    require(std::abs(ratio - 0.20) <= 0.02,
            "large-sample reference layer-change ratio must be near 20%");
}

void testReferencePerInletMovementRatios() {
    Config config = referenceConfig();
    config.seed = 8642;
    config.simulation_duration = 60000.0;
    config.reference.arrival_rate_per_route = 6.0;
    const auto traffic = generatedReferenceTraffic(config);
    struct Counts {
        std::size_t total{};
        std::size_t straight{};
        std::size_t same_level{};
        std::size_t cross_level{};
    };
    std::array<Counts, 4> counts;
    for (const UAV& uav : traffic) {
        Counts& count = counts[static_cast<std::size_t>(uav.source_direction)];
        ++count.total;
        if (uav.movement == Movement::Straight) {
            ++count.straight;
        } else if (uav.source_level != uav.target_level) {
            ++count.cross_level;
        } else {
            ++count.same_level;
        }
    }
    for (const Counts& count : counts) {
        const double total = static_cast<double>(count.total);
        require(std::abs(static_cast<double>(count.straight) / total - 0.40) <= 0.02 &&
                    std::abs(static_cast<double>(count.same_level) / total - 0.40) <= 0.02 &&
                    std::abs(static_cast<double>(count.cross_level) / total - 0.20) <= 0.02,
                "each inlet movement ratios must be near 40/40/20%");
    }
}

void testReferenceOccupancyDtSafety() {
    const Config config = referenceConfig();
    require(nearlyEqual(config.horizontal_speed * config.occupancy_dt, 0.30) &&
                config.horizontal_speed * config.occupancy_dt <= 0.5 * config.cube_size,
            "reference occupancy_dt must satisfy the sampling safety constraint");
    validateTrajectoryConfig(config);
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
        {"horizontal movement time", testHorizontalMovementTime},
        {"vertical movement time", testVerticalMovementTime},
        {"slow vertical speed increases layered path time", testSlowVerticalSpeedIncreasesLayeredPathTime},
        {"safety sphere occupies multiple cubes", testSafetySphereOccupiesMultipleCubes},
        {"larger safety margin does not reduce occupied cubes", testLargerSafetyMarginDoesNotReduceOccupiedCubes},
        {"vertical transition volume conflicts", testVerticalTransitionVolumeConflicts},
        {"sufficient height separation is available", testSufficientHeightSeparationIsAvailable},
        {"insufficient height separation conflicts", testInsufficientHeightSeparationConflicts},
        {"unsafe occupancy dt is rejected", testUnsafeOccupancyDtIsRejected},
        {"original EarliestEntry FCFS behavior", testOriginalEarliestEntryFcfsBehavior},
        {"reference levels and grid", testReferenceLevelsAndGrid},
        {"reference horizontal travel time", testReferenceHorizontalTravelTime},
        {"reference ascending travel time", testReferenceAscendingTravelTime},
        {"reference descending travel time", testReferenceDescendingTravelTime},
        {"reference minimum turning radius formula", testReferenceMinimumTurningRadiusFormula},
        {"reference maximum turning radius formula", testReferenceMaximumTurningRadiusFormula},
        {"reference turning radius within bounds", testReferenceTurningRadiusWithinBounds},
        {"reference left turn quarter circle", testReferenceLeftTurnQuarterCircle},
        {"reference right turn quarter circle", testReferenceRightTurnQuarterCircle},
        {"reference arc tangents", testReferenceArcTangents},
        {"reference straight crosses center", testReferenceStraightCrossesCenter},
        {"reference Northbound mapping", testReferenceNorthboundMapping},
        {"reference Eastbound mapping", testReferenceEastboundMapping},
        {"reference Southbound mapping", testReferenceSouthboundMapping},
        {"reference Westbound mapping", testReferenceWestboundMapping},
        {"cross-level movement uses correct elevator", testCrossLevelUsesCorrectElevator},
        {"same-level movement avoids elevators", testSameLevelMovementAvoidsElevators},
        {"ascending never uses descent elevator", testAscendingNeverUsesDescentElevator},
        {"descending never uses ascent elevator", testDescendingNeverUsesAscentElevator},
        {"cross-level projection connectors", testCrossLevelProjectionConnectors},
        {"reference horizontal separation conflict", testReferenceHorizontalSeparationConflict},
        {"reference horizontal separation available", testReferenceHorizontalSeparationAvailable},
        {"reference flight levels do not conflict", testReferenceFlightLevelsDoNotConflict},
        {"same elevator vertical conflict", testSameElevatorVerticalConflict},
        {"separate elevators operate together", testSeparateElevatorsCanOperateTogether},
        {"reference arrival seed reproducible", testReferenceArrivalSeedReproducible},
        {"reference movement seed reproducible", testReferenceMovementSeedReproducible},
        {"reference layer-change ratio", testReferenceLayerChangeRatio},
        {"reference per-inlet movement ratios", testReferencePerInletMovementRatios},
        {"reference occupancy dt safety", testReferenceOccupancyDtSafety},
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
