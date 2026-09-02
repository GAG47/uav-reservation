#include "statistics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

bool isCompleted(const UAV& uav, double simulation_duration) noexcept {
    return uav.exit_time <= simulation_duration;
}

double mean(const std::vector<double>& values) noexcept {
    if (values.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (const double value : values) {
        total += value;
    }
    return total / static_cast<double>(values.size());
}

double sampleStandardDeviation(const std::vector<double>& values) noexcept {
    if (values.size() < 2U) {
        return 0.0;
    }
    const double average = mean(values);
    double squared_difference_sum = 0.0;
    for (const double value : values) {
        const double difference = value - average;
        squared_difference_sum += difference * difference;
    }
    return std::sqrt(squared_difference_sum / static_cast<double>(values.size() - 1U));
}

double percentile95(std::vector<double> values) noexcept {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const auto rank = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(values.size())));
    return values[rank - 1U];
}

SimulationSummary computeSimulationSummary(
    const std::vector<UAV>& uavs,
    double simulation_duration) {
    if (!std::isfinite(simulation_duration) || simulation_duration <= 0.0) {
        throw std::invalid_argument("Simulation duration must be finite and positive");
    }

    SimulationSummary summary;
    summary.generated_uav_count = uavs.size();
    std::vector<double> completed_delays;
    std::vector<double> all_delays;
    std::vector<double> completed_system_times;
    completed_delays.reserve(uavs.size());
    all_delays.reserve(uavs.size());
    completed_system_times.reserve(uavs.size());

    for (const UAV& uav : uavs) {
        all_delays.push_back(uav.delay);
        if (isCompleted(uav, simulation_duration)) {
            ++summary.completed_uav_count;
            completed_delays.push_back(uav.delay);
            completed_system_times.push_back(uav.exit_time - uav.arrival_time);
        }
    }

    summary.unfinished_uav_count =
        summary.generated_uav_count - summary.completed_uav_count;
    summary.average_delay_completed = mean(completed_delays);
    summary.average_delay_all_scheduled = mean(all_delays);
    summary.max_delay = completed_delays.empty()
                            ? 0.0
                            : *std::max_element(completed_delays.begin(), completed_delays.end());
    summary.p95_delay = percentile95(completed_delays);
    summary.throughput =
        static_cast<double>(summary.completed_uav_count) / simulation_duration;
    summary.average_system_time = mean(completed_system_times);
    return summary;
}
