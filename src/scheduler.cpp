#include "scheduler.hpp"

#include "trajectory.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

FCFSScheduler::FCFSScheduler(
    const Config& config,
    const Intersection& intersection,
    ReservationTable& reservations)
    : config_(config), intersection_(intersection), reservations_(reservations) {
    if (config_.dt <= 0.0 || config_.max_search_time < 0.0) {
        throw std::invalid_argument("Scheduler dt must be positive and max search time non-negative");
    }
}

void FCFSScheduler::schedule(UAV& uav) {
    const auto& paths =
        intersection_.candidatePaths(uav.source, uav.destination, uav.movement);
    if (config_.layer_mode == LayerMode::MiddleOnly) {
        schedule(uav, std::vector<CandidatePath>{paths.front()});
        return;
    }
    schedule(uav, paths);
}

void FCFSScheduler::schedule(UAV& uav, const std::vector<CandidatePath>& candidate_paths) {
    if (candidate_paths.empty()) {
        throw std::invalid_argument("At least one candidate path is required");
    }

    const auto maximum_steps = static_cast<std::size_t>(
        std::floor(config_.max_search_time / config_.dt + 1e-12));
    for (std::size_t step = 0; step <= maximum_steps; ++step) {
        const double entry_time = uav.arrival_time + static_cast<double>(step) * config_.dt;
        std::optional<TimedTrajectory> best;

        for (const CandidatePath& path : candidate_paths) {
            if (config_.layer_mode == LayerMode::MiddleOnly && path.id != 0) {
                continue;
            }
            TimedTrajectory trajectory =
                makeTimedTrajectory(path, entry_time, uav.speed, intersection_.cubeSize());
            if (!reservations_.isTrajectoryAvailable(trajectory)) {
                continue;
            }
            if (!best || trajectory.exit_time < best->exit_time ||
                (trajectory.exit_time == best->exit_time && trajectory.path_id < best->path_id)) {
                best = std::move(trajectory);
            }
        }

        if (best) {
            reservations_.reserveTrajectory(*best, uav.id);
            uav.scheduled_entry_time = entry_time;
            uav.exit_time = best->exit_time;
            uav.delay = entry_time - uav.arrival_time;
            uav.selected_path_id = best->path_id;
            return;
        }
    }

    throw std::runtime_error(
        "No conflict-free trajectory found for UAV " + std::to_string(uav.id) +
        " within max_search_time");
}
