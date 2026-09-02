#pragma once

#include "config.hpp"
#include "statistics.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

struct BenchmarkConfig {
    std::vector<double> arrival_rates{
        0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0};
    std::size_t repetitions{10};
    std::uint32_t seed_base{1000};
    double simulation_duration{300.0};
};

struct BenchmarkRun {
    double arrival_rate{};
    LayerMode layer_mode{LayerMode::MiddleOnly};
    std::uint32_t seed{};
    SimulationSummary summary;
};

struct BenchmarkAggregate {
    double arrival_rate{};
    LayerMode layer_mode{LayerMode::MiddleOnly};
    double mean_average_delay{};
    double std_average_delay{};
    double mean_p95_delay{};
    double std_p95_delay{};
    double mean_throughput{};
    double std_throughput{};
    double mean_unfinished_uav_count{};
    double std_unfinished_uav_count{};
    std::size_t runs{};
};

[[nodiscard]] std::vector<BenchmarkRun> runBenchmark(
    const Config& base_config,
    const BenchmarkConfig& benchmark_config);
[[nodiscard]] std::vector<BenchmarkAggregate> aggregateBenchmarkRuns(
    const std::vector<BenchmarkRun>& runs);
void saveBenchmarkRaw(
    const std::filesystem::path& output_path,
    const std::vector<BenchmarkRun>& runs);
void saveBenchmarkSummary(
    const std::filesystem::path& output_path,
    const std::vector<BenchmarkAggregate>& aggregates);
