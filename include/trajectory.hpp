#pragma once

#include "config.hpp"
#include "types.hpp"

#include <vector>

struct Point3D {
    double x{};
    double y{};
    double z{};

    bool operator==(const Point3D&) const = default;
};

enum class SegmentType { Horizontal, Vertical };

struct TrajectorySegment {
    Point3D start;
    Point3D end;
    SegmentType type{SegmentType::Horizontal};
};

struct CandidatePath {
    int id{};
    std::vector<TrajectorySegment> segments;
};

struct Occupancy {
    Cube cube;
    double start_time{};
    double end_time{};
};

struct TimedTrajectory {
    int path_id{-1};
    std::vector<Occupancy> occupancies;
    double exit_time{};
};

[[nodiscard]] TimedTrajectory makeTimedTrajectory(
    const CandidatePath& path,
    double entry_time,
    const Config& config);

[[nodiscard]] double segmentTravelTime(
    const TrajectorySegment& segment,
    double horizontal_speed,
    double vertical_speed);

void validateTrajectoryConfig(const Config& config);
