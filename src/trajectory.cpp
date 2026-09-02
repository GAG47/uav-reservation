#include "trajectory.hpp"

#include <cmath>
#include <stdexcept>

TimedTrajectory makeTimedTrajectory(
    const CandidatePath& path,
    double entry_time,
    double speed,
    double cube_size) {
    if (path.cubes.empty()) {
        throw std::invalid_argument("A candidate path must contain at least one cube");
    }
    if (!std::isfinite(entry_time) || entry_time < 0.0) {
        throw std::invalid_argument("Entry time must be finite and non-negative");
    }
    if (!std::isfinite(speed) || speed <= 0.0 || !std::isfinite(cube_size) || cube_size <= 0.0) {
        throw std::invalid_argument("Speed and cube size must be finite and positive");
    }

    const double traversal_time = cube_size / speed;
    TimedTrajectory trajectory{path.id, {}, entry_time};
    trajectory.occupancies.reserve(path.cubes.size());

    double start = entry_time;
    for (const Cube& cube : path.cubes) {
        const double end = start + traversal_time;
        trajectory.occupancies.push_back({cube, start, end});
        start = end;
    }
    trajectory.exit_time = start;
    return trajectory;
}
