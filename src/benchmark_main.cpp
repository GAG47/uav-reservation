#include "benchmark.hpp"
#include "config.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        const Config config;
        const BenchmarkConfig benchmark_config;
        std::cout << "Running "
                  << benchmark_config.arrival_rates.size() * 2U * benchmark_config.repetitions
                  << " benchmark simulations...\n";
        const auto runs = runBenchmark(config, benchmark_config);
        const auto aggregates = aggregateBenchmarkRuns(runs);
        saveBenchmarkRaw("results/benchmark_raw.csv", runs);
        saveBenchmarkSummary("results/benchmark_summary.csv", aggregates);
        std::cout << "Completed " << runs.size()
                  << " runs; wrote results/benchmark_raw.csv and results/benchmark_summary.csv\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
