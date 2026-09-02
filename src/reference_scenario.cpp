#include "reference_scenario.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1e-9;

Point3D point(double x, double y, double z) {
    return {x, y, z};
}

double cross(const Vector2D& left, const Vector2D& right) {
    return left.x * right.y - left.y * right.x;
}

bool samePoint(const Point3D& left, const Point3D& right) {
    return std::abs(left.x - right.x) <= kTolerance &&
           std::abs(left.y - right.y) <= kTolerance &&
           std::abs(left.z - right.z) <= kTolerance;
}

bool isUpperDirection(FlightDirection direction) {
    return direction == FlightDirection::Northbound ||
           direction == FlightDirection::Eastbound;
}

void appendLine(std::vector<TrajectorySegment>& segments, Point3D start, Point3D end) {
    if (!samePoint(start, end)) {
        segments.push_back({start, end, SegmentType::HorizontalLine});
    }
}

Point3D projectToRoute(
    const Point3D& elevator,
    FlightDirection route_direction,
    double level) {
    if (route_direction == FlightDirection::Northbound ||
        route_direction == FlightDirection::Southbound) {
        return {0.0, elevator.y, level};
    }
    return {elevator.x, 0.0, level};
}

CandidatePath makeSameLevelTurn(
    const Config& config,
    FlightDirection source,
    FlightDirection target) {
    const double level = referenceFlightLevel(config, source);
    const double radius = referenceTurningRadius(config);
    const Vector2D incoming = referenceDirectionVector(source);
    const Vector2D outgoing = referenceDirectionVector(target);
    const Point3D waiting = referenceWaitingPoint(config, source);
    const Point3D exit = referenceExitPoint(config, target, level);
    const Point3D arc_start{-radius * incoming.x, -radius * incoming.y, level};
    const Point3D arc_end{radius * outgoing.x, radius * outgoing.y, level};
    const Point3D center{
        arc_start.x + arc_end.x, arc_start.y + arc_end.y, level};
    const double start_angle =
        std::atan2(arc_start.y - center.y, arc_start.x - center.x);
    const double sweep_angle =
        cross(incoming, outgoing) > 0.0 ? std::numbers::pi / 2.0
                                        : -std::numbers::pi / 2.0;

    std::vector<TrajectorySegment> segments;
    appendLine(segments, waiting, arc_start);
    segments.push_back({
        arc_start,
        arc_end,
        SegmentType::HorizontalArc,
        center,
        radius,
        start_angle,
        sweep_angle,
    });
    appendLine(segments, arc_end, exit);
    return {0, std::move(segments)};
}

CandidatePath makeCrossLevelTurn(
    const Config& config,
    FlightDirection source,
    FlightDirection target) {
    const double source_level = referenceFlightLevel(config, source);
    const double target_level = referenceFlightLevel(config, target);
    const bool ascending = target_level > source_level;
    const Point3D elevator{
        ascending ? config.reference.ascent_elevator_x
                  : config.reference.descent_elevator_x,
        config.reference.elevator_y,
        source_level,
    };
    const Point3D elevator_target{elevator.x, elevator.y, target_level};
    const Point3D projection_in = projectToRoute(elevator, source, source_level);
    const Point3D projection_out = projectToRoute(elevator_target, target, target_level);

    std::vector<TrajectorySegment> segments;
    appendLine(segments, referenceWaitingPoint(config, source), projection_in);
    appendLine(segments, projection_in, elevator);
    segments.push_back({
        elevator,
        elevator_target,
        ascending ? SegmentType::Ascending : SegmentType::Descending,
    });
    appendLine(segments, elevator_target, projection_out);
    appendLine(segments, projection_out, referenceExitPoint(config, target, target_level));
    return {0, std::move(segments)};
}

}  // namespace

Config makeReference2024Config() {
    Config config;
    config.scenario = ScenarioType::Reference2024AlongRoad;
    config.horizontal_speed = 6.0;
    config.ascending_speed = 4.0;
    config.descending_speed = 3.0;
    config.cube_size = config.reference.reference_cube_size;
    config.occupancy_dt = 0.05;
    config.layer_mode = LayerMode::ThreeLayers;

    const double xy_padding =
        std::ceil(config.reference.horizontal_radius / config.cube_size) * config.cube_size +
        config.cube_size;
    const double z_padding =
        (std::ceil(config.reference.vertical_half_height / config.cube_size) + 1.0) *
        config.cube_size;
    const double xy_min = -config.reference.intersection_radius - xy_padding;
    const double xy_max = config.reference.intersection_radius + xy_padding;
    const double z_min = config.reference.lower_level - z_padding;
    const double z_max = config.reference.upper_level + z_padding;
    config.grid_origin_x = xy_min;
    config.grid_origin_y = xy_min;
    config.grid_origin_z = z_min;
    config.nx = static_cast<int>(std::ceil((xy_max - xy_min) / config.cube_size));
    config.ny = config.nx;
    config.nz = static_cast<int>(std::ceil((z_max - z_min) / config.cube_size));
    return config;
}

double referenceMinimumTurningRadius(const Config& config) {
    const double roll_radians =
        config.reference.maximum_roll_degrees * std::numbers::pi / 180.0;
    return config.horizontal_speed * config.horizontal_speed /
           (config.reference.gravity * std::tan(roll_radians));
}

double referenceMaximumTurningRadius(const Config& config) {
    constexpr double crossing_angle = std::numbers::pi / 2.0;
    return config.reference.route_width / (1.0 - std::cos(crossing_angle));
}

double referenceTurningRadius(const Config& config) {
    const double minimum = referenceMinimumTurningRadius(config);
    const double maximum = referenceMaximumTurningRadius(config);
    const double radius = (minimum + maximum) / 2.0;
    assert(minimum < radius);
    assert(radius < maximum);
    if (!(minimum < radius && radius < maximum)) {
        throw std::logic_error("Derived reference turning radius violates paper constraints");
    }
    return radius;
}

double referenceFlightLevel(const Config& config, FlightDirection direction) {
    if (direction == FlightDirection::Northbound ||
        direction == FlightDirection::Eastbound) {
        return config.reference.upper_level;
    }
    return config.reference.lower_level;
}

FlightDirection referenceTargetDirection(FlightDirection source, Movement movement) {
    if (movement == Movement::Straight) {
        return source;
    }
    switch (source) {
        case FlightDirection::Northbound:
            return movement == Movement::LeftTurn ? FlightDirection::Westbound
                                                   : FlightDirection::Eastbound;
        case FlightDirection::Eastbound:
            return movement == Movement::LeftTurn ? FlightDirection::Northbound
                                                   : FlightDirection::Southbound;
        case FlightDirection::Southbound:
            return movement == Movement::LeftTurn ? FlightDirection::Eastbound
                                                   : FlightDirection::Westbound;
        case FlightDirection::Westbound:
            return movement == Movement::LeftTurn ? FlightDirection::Southbound
                                                   : FlightDirection::Northbound;
    }
    throw std::invalid_argument("Unknown reference source direction");
}

bool referenceMovementChangesLevel(FlightDirection source, Movement movement) {
    return isUpperDirection(source) !=
           isUpperDirection(referenceTargetDirection(source, movement));
}

Movement referenceMovementFromDraw(FlightDirection source, double draw) {
    if (draw < 0.0 || draw >= 1.0) {
        throw std::invalid_argument("Movement probability draw must be in [0,1)");
    }
    if (draw < 0.4) {
        return Movement::Straight;
    }
    const Movement same_level =
        (source == FlightDirection::Northbound || source == FlightDirection::Southbound)
            ? Movement::RightTurn
            : Movement::LeftTurn;
    if (draw < 0.8) {
        return same_level;
    }
    return same_level == Movement::LeftTurn ? Movement::RightTurn : Movement::LeftTurn;
}

Vector2D referenceDirectionVector(FlightDirection direction) {
    switch (direction) {
        case FlightDirection::Northbound:
            return {0.0, 1.0};
        case FlightDirection::Eastbound:
            return {1.0, 0.0};
        case FlightDirection::Southbound:
            return {0.0, -1.0};
        case FlightDirection::Westbound:
            return {-1.0, 0.0};
    }
    throw std::invalid_argument("Unknown flight direction");
}

Point3D referenceWaitingPoint(const Config& config, FlightDirection direction) {
    const Vector2D vector = referenceDirectionVector(direction);
    const double level = referenceFlightLevel(config, direction);
    return point(
        -config.reference.intersection_radius * vector.x,
        -config.reference.intersection_radius * vector.y,
        level);
}

Point3D referenceExitPoint(
    const Config& config,
    FlightDirection direction,
    double level) {
    const Vector2D vector = referenceDirectionVector(direction);
    return point(
        config.reference.intersection_radius * vector.x,
        config.reference.intersection_radius * vector.y,
        level);
}

CandidatePath makeReferenceCandidatePath(
    const Config& config,
    FlightDirection source,
    Movement movement) {
    const FlightDirection target = referenceTargetDirection(source, movement);
    if (movement == Movement::Straight) {
        const double level = referenceFlightLevel(config, source);
        return {0,
                {{referenceWaitingPoint(config, source),
                  referenceExitPoint(config, target, level),
                  SegmentType::HorizontalLine}}};
    }
    if (referenceMovementChangesLevel(source, movement)) {
        return makeCrossLevelTurn(config, source, target);
    }
    return makeSameLevelTurn(config, source, target);
}
