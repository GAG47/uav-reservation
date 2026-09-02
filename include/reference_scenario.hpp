#pragma once

#include "config.hpp"
#include "trajectory.hpp"
#include "types.hpp"

struct Vector2D {
    double x{};
    double y{};
};

[[nodiscard]] Config makeReference2024Config();
[[nodiscard]] double referenceMinimumTurningRadius(const Config& config);
[[nodiscard]] double referenceMaximumTurningRadius(const Config& config);
[[nodiscard]] double referenceTurningRadius(const Config& config);
[[nodiscard]] double referenceFlightLevel(
    const Config& config,
    FlightDirection direction);
[[nodiscard]] FlightDirection referenceTargetDirection(
    FlightDirection source,
    Movement movement);
[[nodiscard]] bool referenceMovementChangesLevel(
    FlightDirection source,
    Movement movement);
[[nodiscard]] Movement referenceMovementFromDraw(
    FlightDirection source,
    double draw);
[[nodiscard]] Vector2D referenceDirectionVector(FlightDirection direction);
[[nodiscard]] Point3D referenceWaitingPoint(
    const Config& config,
    FlightDirection direction);
[[nodiscard]] Point3D referenceExitPoint(
    const Config& config,
    FlightDirection direction,
    double level);
[[nodiscard]] CandidatePath makeReferenceCandidatePath(
    const Config& config,
    FlightDirection source,
    Movement movement);
