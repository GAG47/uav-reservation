#include "simulator.hpp"

#include "reference_scenario.hpp"
#include "types.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

Direction inletSide(FlightDirection direction) {
    switch (direction) {
        case FlightDirection::Northbound:
            return Direction::South;
        case FlightDirection::Eastbound:
            return Direction::West;
        case FlightDirection::Southbound:
            return Direction::North;
        case FlightDirection::Westbound:
            return Direction::East;
    }
    throw std::invalid_argument("Unknown reference inlet direction");
}

Direction exitSide(FlightDirection direction) {
    switch (direction) {
        case FlightDirection::Northbound:
            return Direction::North;
        case FlightDirection::Eastbound:
            return Direction::East;
        case FlightDirection::Southbound:
            return Direction::South;
        case FlightDirection::Westbound:
            return Direction::West;
    }
    throw std::invalid_argument("Unknown reference target direction");
}

UAV makeReferenceUav(
    const Config& config,
    int id,
    double arrival_time,
    FlightDirection source,
    Movement movement) {
    const FlightDirection target = referenceTargetDirection(source, movement);
    UAV uav{
        id,
        arrival_time,
        inletSide(source),
        exitSide(target),
        movement,
        config.horizontal_speed,
    };
    uav.source_direction = source;
    uav.target_direction = target;
    uav.source_level = referenceFlightLevel(config, source);
    uav.target_level = referenceFlightLevel(config, target);
    return uav;
}

}  // namespace

Simulator::Simulator(const Config& config, FCFSScheduler& scheduler)
    : config_(config), scheduler_(scheduler) {
    if (config_.simulation_duration <= 0.0 || config_.horizontal_speed <= 0.0 ||
        config_.ascending_speed <= 0.0 || config_.descending_speed <= 0.0) {
        throw std::invalid_argument("Simulation duration, arrival rate, and speed must be positive");
    }
    if (config_.scenario == ScenarioType::Toy && config_.arrival_rate <= 0.0) {
        throw std::invalid_argument("Toy arrival rate must be positive");
    }
    if (config_.scenario == ScenarioType::Reference2024AlongRoad &&
        (config_.reference.arrival_rate_per_route < 1.0 ||
         config_.reference.arrival_rate_per_route > 6.0)) {
        throw std::invalid_argument("Reference arrival rate must be in [1,6] UAV/min/route");
    }
}

std::vector<UAV> Simulator::deterministicTraffic() const {
    if (config_.scenario == ScenarioType::Reference2024AlongRoad) {
        return referenceDeterministicTraffic();
    }
    return {
        {1, 0.0, Direction::North, Direction::South, Movement::Straight, config_.horizontal_speed},
        {2, 0.0, Direction::East, Direction::West, Movement::Straight, config_.horizontal_speed},
        {3, 1.0, Direction::South, Direction::North, Movement::Straight, config_.horizontal_speed},
        {4, 1.0, Direction::West, Direction::East, Movement::Straight, config_.horizontal_speed},
        {5, 2.0, Direction::North, Direction::South, Movement::Straight, config_.horizontal_speed},
        {6, 2.0, Direction::East, Direction::West, Movement::Straight, config_.horizontal_speed},
    };
}

std::vector<UAV> Simulator::poissonTraffic() const {
    if (config_.scenario == ScenarioType::Reference2024AlongRoad) {
        return referencePoissonTraffic();
    }
    std::mt19937 generator(config_.seed);
    std::exponential_distribution<double> inter_arrival(config_.arrival_rate);
    std::uniform_int_distribution<int> source_distribution(0, 3);
    constexpr std::array<Direction, 4> directions{
        Direction::North, Direction::South, Direction::East, Direction::West};

    std::vector<UAV> traffic;
    double arrival_time = 0.0;
    int id = 1;
    while (true) {
        arrival_time += inter_arrival(generator);
        if (arrival_time > config_.simulation_duration) {
            break;
        }
        const Direction source = directions[static_cast<std::size_t>(source_distribution(generator))];
        traffic.push_back(
            {id++,
             arrival_time,
             source,
             opposite(source),
             Movement::Straight,
             config_.horizontal_speed});
    }
    return traffic;
}

std::vector<UAV> Simulator::referenceDeterministicTraffic() const {
    return {
        makeReferenceUav(
            config_, 1, 0.0, FlightDirection::Northbound, Movement::Straight),
        makeReferenceUav(
            config_, 2, 0.5, FlightDirection::Northbound, Movement::RightTurn),
        makeReferenceUav(
            config_, 3, 1.0, FlightDirection::Eastbound, Movement::LeftTurn),
        makeReferenceUav(
            config_, 4, 1.5, FlightDirection::Southbound, Movement::LeftTurn),
        makeReferenceUav(
            config_, 5, 2.0, FlightDirection::Westbound, Movement::RightTurn),
        makeReferenceUav(
            config_, 6, 2.5, FlightDirection::Northbound, Movement::LeftTurn),
    };
}

std::vector<UAV> Simulator::referencePoissonTraffic() const {
    std::mt19937 generator(config_.seed);
    const double rate_per_second = config_.reference.arrival_rate_per_route / 60.0;
    std::exponential_distribution<double> inter_arrival(rate_per_second);
    std::uniform_real_distribution<double> movement_draw(0.0, 1.0);
    constexpr std::array<FlightDirection, 4> directions{
        FlightDirection::Northbound,
        FlightDirection::Eastbound,
        FlightDirection::Southbound,
        FlightDirection::Westbound,
    };

    std::vector<UAV> traffic;
    int id = 1;
    for (const FlightDirection source : directions) {
        double arrival_time = 0.0;
        while (true) {
            arrival_time += inter_arrival(generator);
            if (arrival_time > config_.simulation_duration) {
                break;
            }
            const Movement movement =
                referenceMovementFromDraw(source, movement_draw(generator));
            traffic.push_back(
                makeReferenceUav(config_, id++, arrival_time, source, movement));
        }
    }
    return traffic;
}

void Simulator::run(std::vector<UAV> traffic) {
    std::sort(traffic.begin(), traffic.end(), [](const UAV& left, const UAV& right) {
        if (left.arrival_time != right.arrival_time) {
            return left.arrival_time < right.arrival_time;
        }
        return left.id < right.id;
    });

    for (UAV& uav : traffic) {
        scheduler_.schedule(uav);
    }
    results_ = std::move(traffic);
}

void Simulator::saveResults(const std::filesystem::path& output_path) const {
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Unable to open results file: " + output_path.string());
    }

    output << "uav_id,arrival_time,source,destination,scheduled_entry_time,exit_time,delay,"
              "selected_path_id,completed\n";
    output << std::fixed << std::setprecision(6);
    for (const UAV& uav : results_) {
        output << uav.id << ',' << uav.arrival_time << ',' << toString(uav.source) << ','
               << toString(uav.destination) << ',' << uav.scheduled_entry_time << ','
               << uav.exit_time << ',' << uav.delay << ',' << uav.selected_path_id << ','
               << (isCompleted(uav, config_.simulation_duration) ? 1 : 0) << '\n';
    }
}

void Simulator::saveTrajectoryDebug(const std::filesystem::path& output_path) const {
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Unable to open trajectory debug file: " + output_path.string());
    }
    output << "uav_id,time,x,y,z,movement,source_direction,target_direction\n";
    output << std::fixed << std::setprecision(6);
    for (const UAV& uav : results_) {
        const auto& paths = scheduler_.candidatePathsFor(uav);
        const auto selected = std::find_if(
            paths.begin(), paths.end(), [&uav](const CandidatePath& path) {
                return path.id == uav.selected_path_id;
            });
        if (selected == paths.end()) {
            throw std::logic_error("Scheduled path is unavailable for trajectory export");
        }
        for (const TrajectorySample& sample :
             sampleTrajectory(*selected, uav.scheduled_entry_time, config_)) {
            output << uav.id << ',' << sample.time << ',' << sample.position.x << ','
                   << sample.position.y << ',' << sample.position.z << ','
                   << toString(uav.movement) << ',' << toString(uav.source_direction) << ','
                   << toString(uav.target_direction) << '\n';
        }
    }
}

SimulationSummary Simulator::summary() const {
    return computeSimulationSummary(results_, config_.simulation_duration);
}

void Simulator::printSummary(std::ostream& output) const {
    const SimulationSummary result = summary();
    output << std::fixed << std::setprecision(3);
    output << "Generated UAV count: " << result.generated_uav_count << '\n';
    output << "Completed UAV count: " << result.completed_uav_count << '\n';
    output << "Unfinished UAV count: " << result.unfinished_uav_count << '\n';
    output << "Average delay (completed): " << result.average_delay_completed << " s\n";
    output << "Average delay (all scheduled): " << result.average_delay_all_scheduled << " s\n";
    output << "Max delay: " << result.max_delay << " s\n";
    output << "P95 delay: " << result.p95_delay << " s\n";
    output << "Average system time: " << result.average_system_time << " s\n";
    output << "Throughput: " << result.throughput << " UAV/s\n";
}
