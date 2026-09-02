#pragma once

#include "config.hpp"
#include "scheduler.hpp"
#include "statistics.hpp"
#include "uav.hpp"

#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <vector>

class Simulator {
public:
    Simulator(const Config& config, FCFSScheduler& scheduler);

    [[nodiscard]] std::vector<UAV> deterministicTraffic() const;
    [[nodiscard]] std::vector<UAV> poissonTraffic() const;
    void run(std::vector<UAV> traffic);

    void saveResults(const std::filesystem::path& output_path) const;
    [[nodiscard]] SimulationSummary summary() const;
    void printSummary(std::ostream& output) const;
    [[nodiscard]] const std::vector<UAV>& results() const noexcept { return results_; }

private:
    const Config& config_;
    FCFSScheduler& scheduler_;
    std::vector<UAV> results_;
};
