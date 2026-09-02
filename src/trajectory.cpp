#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {

constexpr double kCoordinateTolerance = 1e-9;
constexpr double kMaximumStepFraction = 0.5;

struct TimeInterval {
    double start{};
    double end{};
};

double distance(const Point3D& start, const Point3D& end) {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Point3D interpolate(const TrajectorySegment& segment, double fraction) {
    if (segment.type == SegmentType::HorizontalArc) {
        const double angle = segment.start_angle + segment.sweep_angle * fraction;
        return {
            segment.center.x + segment.radius * std::cos(angle),
            segment.center.y + segment.radius * std::sin(angle),
            segment.start.z,
        };
    }
    return {
        segment.start.x + (segment.end.x - segment.start.x) * fraction,
        segment.start.y + (segment.end.y - segment.start.y) * fraction,
        segment.start.z + (segment.end.z - segment.start.z) * fraction,
    };
}

bool nearlyEqual(double left, double right) {
    return std::abs(left - right) <= kCoordinateTolerance;
}

bool samePoint(const Point3D& left, const Point3D& right) {
    return nearlyEqual(left.x, right.x) && nearlyEqual(left.y, right.y) &&
           nearlyEqual(left.z, right.z);
}

bool sphereIntersectsCube(
    const Point3D& center,
    double radius,
    const Cube& cube,
    const Config& config) {
    const double cube_size = config.cube_size;
    const double minimum_x =
        config.grid_origin_x + static_cast<double>(cube.x) * cube_size;
    const double minimum_y =
        config.grid_origin_y + static_cast<double>(cube.y) * cube_size;
    const double minimum_z =
        config.grid_origin_z + static_cast<double>(cube.z) * cube_size;
    const double maximum_x = minimum_x + cube_size;
    const double maximum_y = minimum_y + cube_size;
    const double maximum_z = minimum_z + cube_size;

    const double closest_x = std::clamp(center.x, minimum_x, maximum_x);
    const double closest_y = std::clamp(center.y, minimum_y, maximum_y);
    const double closest_z = std::clamp(center.z, minimum_z, maximum_z);
    const double dx = center.x - closest_x;
    const double dy = center.y - closest_y;
    const double dz = center.z - closest_z;
    return dx * dx + dy * dy + dz * dz <= radius * radius;
}

bool cylinderIntersectsCube(
    const Point3D& center,
    double horizontal_radius,
    double vertical_half_height,
    const Cube& cube,
    const Config& config) {
    const double minimum_x =
        config.grid_origin_x + static_cast<double>(cube.x) * config.cube_size;
    const double minimum_y =
        config.grid_origin_y + static_cast<double>(cube.y) * config.cube_size;
    const double minimum_z =
        config.grid_origin_z + static_cast<double>(cube.z) * config.cube_size;
    const double maximum_x = minimum_x + config.cube_size;
    const double maximum_y = minimum_y + config.cube_size;
    const double maximum_z = minimum_z + config.cube_size;
    if (maximum_z < center.z - vertical_half_height ||
        minimum_z > center.z + vertical_half_height) {
        return false;
    }
    const double closest_x = std::clamp(center.x, minimum_x, maximum_x);
    const double closest_y = std::clamp(center.y, minimum_y, maximum_y);
    const double dx = center.x - closest_x;
    const double dy = center.y - closest_y;
    return dx * dx + dy * dy <= horizontal_radius * horizontal_radius;
}

void addEnvelopeCubes(
    const Point3D& center,
    const Config& config,
    std::unordered_set<Cube, CubeHash>& cubes) {
    const bool reference = config.scenario == ScenarioType::Reference2024AlongRoad;
    const double horizontal_radius =
        reference ? config.reference.horizontal_radius
                  : config.uav_radius + config.safety_margin;
    const double vertical_radius =
        reference ? config.reference.vertical_half_height : horizontal_radius;
    const int minimum_x = std::max(
        0,
        static_cast<int>(std::floor(
            (center.x - horizontal_radius - config.grid_origin_x) / config.cube_size)));
    const int minimum_y = std::max(
        0,
        static_cast<int>(std::floor(
            (center.y - horizontal_radius - config.grid_origin_y) / config.cube_size)));
    const int minimum_z = std::max(
        0,
        static_cast<int>(std::floor(
            (center.z - vertical_radius - config.grid_origin_z) / config.cube_size)));
    const int maximum_x = std::min(
        config.nx - 1,
        static_cast<int>(std::floor(
            (center.x + horizontal_radius - config.grid_origin_x) / config.cube_size)));
    const int maximum_y = std::min(
        config.ny - 1,
        static_cast<int>(std::floor(
            (center.y + horizontal_radius - config.grid_origin_y) / config.cube_size)));
    const int maximum_z = std::min(
        config.nz - 1,
        static_cast<int>(std::floor(
            (center.z + vertical_radius - config.grid_origin_z) / config.cube_size)));

    for (int z = minimum_z; z <= maximum_z; ++z) {
        for (int y = minimum_y; y <= maximum_y; ++y) {
            for (int x = minimum_x; x <= maximum_x; ++x) {
                const Cube cube{x, y, z};
                const bool intersects =
                    reference
                        ? cylinderIntersectsCube(
                              center,
                              horizontal_radius,
                              vertical_radius,
                              cube,
                              config)
                        : sphereIntersectsCube(center, horizontal_radius, cube, config);
                if (intersects) {
                    cubes.insert(cube);
                }
            }
        }
    }
}

}  // namespace

double segmentTravelTime(
    const TrajectorySegment& segment,
    double horizontal_speed,
    double ascending_speed,
    double descending_speed) {
    if (!std::isfinite(horizontal_speed) || horizontal_speed <= 0.0 ||
        !std::isfinite(ascending_speed) || ascending_speed <= 0.0 ||
        !std::isfinite(descending_speed) || descending_speed <= 0.0) {
        throw std::invalid_argument("Trajectory speeds must be finite and positive");
    }

    if (segment.type == SegmentType::HorizontalLine) {
        if (!nearlyEqual(segment.start.z, segment.end.z)) {
            throw std::invalid_argument("A horizontal line must have constant z");
        }
        return distance(segment.start, segment.end) / horizontal_speed;
    }
    if (segment.type == SegmentType::HorizontalArc) {
        if (!nearlyEqual(segment.start.z, segment.end.z) || segment.radius <= 0.0 ||
            segment.sweep_angle == 0.0) {
            throw std::invalid_argument("A horizontal arc must have valid planar geometry");
        }
        return segment.radius * std::abs(segment.sweep_angle) / horizontal_speed;
    }
    if (!nearlyEqual(segment.start.x, segment.end.x) ||
        !nearlyEqual(segment.start.y, segment.end.y)) {
        throw std::invalid_argument("A vertical segment must have constant x and y");
    }
    if (segment.type == SegmentType::Ascending) {
        if (segment.end.z <= segment.start.z) {
            throw std::invalid_argument("An ascending segment must increase z");
        }
        return distance(segment.start, segment.end) / ascending_speed;
    }
    if (segment.end.z >= segment.start.z) {
        throw std::invalid_argument("A descending segment must decrease z");
    }
    return distance(segment.start, segment.end) / descending_speed;
}

void validateTrajectoryConfig(const Config& config) {
    if (!std::isfinite(config.horizontal_speed) || config.horizontal_speed <= 0.0 ||
        !std::isfinite(config.ascending_speed) || config.ascending_speed <= 0.0 ||
        !std::isfinite(config.descending_speed) || config.descending_speed <= 0.0 ||
        !std::isfinite(config.cube_size) || config.cube_size <= 0.0 ||
        !std::isfinite(config.occupancy_dt) || config.occupancy_dt <= 0.0 ||
        !std::isfinite(config.uav_radius) || config.uav_radius <= 0.0 ||
        !std::isfinite(config.safety_margin) || config.safety_margin < 0.0 ||
        config.nx <= 0 || config.ny <= 0 || config.nz <= 0) {
        throw std::invalid_argument(
            "Trajectory speeds, cube size, UAV radius, and occupancy_dt must be positive; "
            "safety margin must be non-negative");
    }
    const double maximum_step =
        std::max(
            {config.horizontal_speed, config.ascending_speed, config.descending_speed}) *
        config.occupancy_dt;
    if (maximum_step > kMaximumStepFraction * config.cube_size) {
        throw std::invalid_argument(
            "occupancy_dt is too large: center movement per sample must not exceed "
            "0.5 * cube_size");
    }
}

TimedTrajectory makeTimedTrajectory(
    const CandidatePath& path,
    double entry_time,
    const Config& config) {
    if (path.segments.empty()) {
        throw std::invalid_argument("A candidate path must contain at least one segment");
    }
    if (!std::isfinite(entry_time) || entry_time < 0.0) {
        throw std::invalid_argument("Entry time must be finite and non-negative");
    }
    validateTrajectoryConfig(config);
    for (std::size_t index = 1; index < path.segments.size(); ++index) {
        if (!samePoint(path.segments[index - 1U].end, path.segments[index].start)) {
            throw std::invalid_argument("Candidate path segments must be continuous");
        }
    }

    TimedTrajectory trajectory{path.id, {}, entry_time};
    std::unordered_map<Cube, std::vector<TimeInterval>, CubeHash> intervals_by_cube;
    double segment_start_time = entry_time;

    for (const TrajectorySegment& segment : path.segments) {
        const double duration =
            segmentTravelTime(
                segment,
                config.horizontal_speed,
                config.ascending_speed,
                config.descending_speed);
        if (duration <= 0.0) {
            throw std::invalid_argument("Trajectory segments must have positive length");
        }
        double elapsed = 0.0;
        while (elapsed < duration) {
            const double next_elapsed = std::min(duration, elapsed + config.occupancy_dt);
            const double interval_start = segment_start_time + elapsed;
            const double interval_end = segment_start_time + next_elapsed;
            std::unordered_set<Cube, CubeHash> occupied_cubes;
            addEnvelopeCubes(
                interpolate(segment, elapsed / duration),
                config,
                occupied_cubes);
            addEnvelopeCubes(
                interpolate(segment, next_elapsed / duration),
                config,
                occupied_cubes);
            for (const Cube& cube : occupied_cubes) {
                intervals_by_cube[cube].push_back({interval_start, interval_end});
            }
            elapsed = next_elapsed;
        }
        segment_start_time += duration;
    }

    for (auto& [cube, intervals] : intervals_by_cube) {
        std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& left, const TimeInterval& right) {
            return left.start < right.start;
        });
        TimeInterval merged = intervals.front();
        for (std::size_t index = 1; index < intervals.size(); ++index) {
            if (intervals[index].start <= merged.end) {
                merged.end = std::max(merged.end, intervals[index].end);
            } else {
                trajectory.occupancies.push_back({cube, merged.start, merged.end});
                merged = intervals[index];
            }
        }
        trajectory.occupancies.push_back({cube, merged.start, merged.end});
    }
    std::sort(
        trajectory.occupancies.begin(),
        trajectory.occupancies.end(),
        [](const Occupancy& left, const Occupancy& right) {
            if (left.start_time != right.start_time) {
                return left.start_time < right.start_time;
            }
            if (left.cube.z != right.cube.z) {
                return left.cube.z < right.cube.z;
            }
            if (left.cube.y != right.cube.y) {
                return left.cube.y < right.cube.y;
            }
            return left.cube.x < right.cube.x;
        });
    trajectory.exit_time = segment_start_time;
    return trajectory;
}

std::vector<TrajectorySample> sampleTrajectory(
    const CandidatePath& path,
    double entry_time,
    const Config& config) {
    if (path.segments.empty()) {
        throw std::invalid_argument("A candidate path must contain at least one segment");
    }
    validateTrajectoryConfig(config);
    std::vector<TrajectorySample> samples{{entry_time, path.segments.front().start}};
    double segment_start_time = entry_time;
    for (const TrajectorySegment& segment : path.segments) {
        const double duration = segmentTravelTime(
            segment,
            config.horizontal_speed,
            config.ascending_speed,
            config.descending_speed);
        double elapsed = 0.0;
        while (elapsed < duration) {
            elapsed = std::min(duration, elapsed + config.occupancy_dt);
            samples.push_back({
                segment_start_time + elapsed,
                interpolate(segment, elapsed / duration),
            });
        }
        segment_start_time += duration;
    }
    return samples;
}
