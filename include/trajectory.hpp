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

enum class SegmentType { HorizontalLine, HorizontalArc, Ascending, Descending };

struct TrajectorySegment {
    Point3D start;
    Point3D end;
    SegmentType type{SegmentType::HorizontalLine};
    Point3D center;
    double radius{};
    double start_angle{};
    double sweep_angle{};

    TrajectorySegment() = default;
    TrajectorySegment(Point3D segment_start, Point3D segment_end, SegmentType segment_type)
        : start(segment_start), end(segment_end), type(segment_type) {}
    TrajectorySegment(
        Point3D segment_start,
        Point3D segment_end,
        SegmentType segment_type,
        Point3D arc_center,
        double arc_radius,
        double arc_start_angle,
        double arc_sweep_angle)
        : start(segment_start),
          end(segment_end),
          type(segment_type),
          center(arc_center),
          radius(arc_radius),
          start_angle(arc_start_angle),
          sweep_angle(arc_sweep_angle) {}
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

struct TrajectorySample {
    double time{};
    Point3D position;
};

[[nodiscard]] TimedTrajectory makeTimedTrajectory(
    const CandidatePath& path,
    double entry_time,
    const Config& config);

[[nodiscard]] double segmentTravelTime(
    const TrajectorySegment& segment,
    double horizontal_speed,
    double ascending_speed,
    double descending_speed);

[[nodiscard]] std::vector<TrajectorySample> sampleTrajectory(
    const CandidatePath& path,
    double entry_time,
    const Config& config);

void validateTrajectoryConfig(const Config& config);
