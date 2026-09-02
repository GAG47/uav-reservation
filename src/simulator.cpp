#include "simulator.hpp"

#include "types.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <utility>

Simulator::Simulator(const Config& config, FCFSScheduler& scheduler)
    : config_(config), scheduler_(scheduler) {
    if (config_.simulation_duration <= 0.0 || config_.arrival_rate <= 0.0 ||
        config_.horizontal_speed <= 0.0 || config_.vertical_speed <= 0.0) {
        throw std::invalid_argument("Simulation duration, arrival rate, and speed must be positive");
    }
}

std::vector<UAV> Simulator::deterministicTraffic() const {
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
