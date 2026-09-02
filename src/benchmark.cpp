#include "benchmark.hpp"

#include "intersection.hpp"
#include "reservation_table.hpp"
#include "scheduler.hpp"
#include "simulator.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <utility>

namespace {

void prepareOutput(const std::filesystem::path& output_path) {
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
}

}  // namespace

std::vector<BenchmarkRun> runBenchmark(
    const Config& base_config,
    const BenchmarkConfig& benchmark_config) {
    if (benchmark_config.arrival_rates.empty() || benchmark_config.repetitions == 0U ||
        benchmark_config.simulation_duration <= 0.0) {
        throw std::invalid_argument("Benchmark rates, repetitions, and duration must be valid");
    }

    constexpr std::array<LayerMode, 2> modes{
        LayerMode::MiddleOnly, LayerMode::ThreeLayers};
    std::vector<BenchmarkRun> runs;
    runs.reserve(
        benchmark_config.arrival_rates.size() * modes.size() * benchmark_config.repetitions);

    for (const double arrival_rate : benchmark_config.arrival_rates) {
        if (arrival_rate <= 0.0) {
            throw std::invalid_argument("Benchmark arrival rates must be positive");
        }
        for (const LayerMode mode : modes) {
            for (std::size_t repetition = 0; repetition < benchmark_config.repetitions; ++repetition) {
                Config config = base_config;
                config.arrival_rate = arrival_rate;
                config.simulation_duration = benchmark_config.simulation_duration;
                config.seed = benchmark_config.seed_base + static_cast<std::uint32_t>(repetition);
                config.layer_mode = mode;

                const Intersection intersection(
                    config.nx, config.ny, config.nz, config.cube_size);
                ReservationTable reservations(config.nx, config.ny, config.nz);
                FCFSScheduler scheduler(config, intersection, reservations);
                Simulator simulator(config, scheduler);
                simulator.run(simulator.poissonTraffic());
                runs.push_back({arrival_rate, mode, config.seed, simulator.summary()});
            }
        }
    }
    return runs;
}

std::vector<BenchmarkAggregate> aggregateBenchmarkRuns(
    const std::vector<BenchmarkRun>& runs) {
    using Key = std::pair<double, LayerMode>;
    std::map<Key, std::vector<const BenchmarkRun*>> groups;
    for (const BenchmarkRun& run : runs) {
        groups[{run.arrival_rate, run.layer_mode}].push_back(&run);
    }

    std::vector<BenchmarkAggregate> aggregates;
    aggregates.reserve(groups.size());
    for (const auto& [key, group] : groups) {
        std::vector<double> average_delays;
        std::vector<double> p95_delays;
        std::vector<double> throughputs;
        std::vector<double> unfinished_counts;
        average_delays.reserve(group.size());
        p95_delays.reserve(group.size());
        throughputs.reserve(group.size());
        unfinished_counts.reserve(group.size());

        for (const BenchmarkRun* run : group) {
            average_delays.push_back(run->summary.average_delay_completed);
            p95_delays.push_back(run->summary.p95_delay);
            throughputs.push_back(run->summary.throughput);
            unfinished_counts.push_back(
                static_cast<double>(run->summary.unfinished_uav_count));
        }

        aggregates.push_back({
            key.first,
            key.second,
            mean(average_delays),
            sampleStandardDeviation(average_delays),
            mean(p95_delays),
            sampleStandardDeviation(p95_delays),
            mean(throughputs),
            sampleStandardDeviation(throughputs),
            mean(unfinished_counts),
            sampleStandardDeviation(unfinished_counts),
            group.size(),
        });
    }
    return aggregates;
}

void saveBenchmarkRaw(
    const std::filesystem::path& output_path,
    const std::vector<BenchmarkRun>& runs) {
    prepareOutput(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Unable to open benchmark raw file: " + output_path.string());
    }
    output << "arrival_rate,layer_mode,seed,generated_uav_count,completed_uav_count,"
              "unfinished_uav_count,average_delay_completed,p95_delay,max_delay,"
              "average_system_time,throughput\n";
    output << std::fixed << std::setprecision(6);
    for (const BenchmarkRun& run : runs) {
        const SimulationSummary& summary = run.summary;
        output << run.arrival_rate << ',' << toString(run.layer_mode) << ',' << run.seed << ','
               << summary.generated_uav_count << ',' << summary.completed_uav_count << ','
               << summary.unfinished_uav_count << ',' << summary.average_delay_completed << ','
               << summary.p95_delay << ',' << summary.max_delay << ','
               << summary.average_system_time << ',' << summary.throughput << '\n';
    }
}

void saveBenchmarkSummary(
    const std::filesystem::path& output_path,
    const std::vector<BenchmarkAggregate>& aggregates) {
    prepareOutput(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Unable to open benchmark summary file: " + output_path.string());
    }
    output << "arrival_rate,layer_mode,mean_average_delay,std_average_delay,mean_p95_delay,"
              "std_p95_delay,mean_throughput,std_throughput,mean_unfinished_uav_count,"
              "std_unfinished_uav_count,runs\n";
    output << std::fixed << std::setprecision(6);
    for (const BenchmarkAggregate& aggregate : aggregates) {
        output << aggregate.arrival_rate << ',' << toString(aggregate.layer_mode) << ','
               << aggregate.mean_average_delay << ',' << aggregate.std_average_delay << ','
               << aggregate.mean_p95_delay << ',' << aggregate.std_p95_delay << ','
               << aggregate.mean_throughput << ',' << aggregate.std_throughput << ','
               << aggregate.mean_unfinished_uav_count << ','
               << aggregate.std_unfinished_uav_count << ',' << aggregate.runs << '\n';
    }
}
