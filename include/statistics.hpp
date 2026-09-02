#pragma once

#include "uav.hpp"

#include <cstddef>
#include <vector>

struct SimulationSummary {
    std::size_t generated_uav_count{};
    std::size_t completed_uav_count{};
    std::size_t unfinished_uav_count{};
    double average_delay_completed{};
    double average_delay_all_scheduled{};
    double max_delay{};
    double p95_delay{};
    double throughput{};
    double average_system_time{};
};

[[nodiscard]] bool isCompleted(const UAV& uav, double simulation_duration) noexcept;
[[nodiscard]] double mean(const std::vector<double>& values) noexcept;
[[nodiscard]] double sampleStandardDeviation(const std::vector<double>& values) noexcept;
[[nodiscard]] double percentile95(std::vector<double> values) noexcept;
[[nodiscard]] SimulationSummary computeSimulationSummary(
    const std::vector<UAV>& uavs,
    double simulation_duration);
