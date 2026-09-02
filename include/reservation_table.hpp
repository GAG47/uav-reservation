#pragma once

#include "trajectory.hpp"

#include <map>
#include <vector>

struct Reservation {
    double start_time{};
    double end_time{};
    int uav_id{};
};

class ReservationTable {
public:
    ReservationTable(int nx, int ny, int nz);

    [[nodiscard]] bool isAvailable(const Cube& cube, double start, double end) const;
    [[nodiscard]] bool isTrajectoryAvailable(const TimedTrajectory& trajectory) const;
    void reserveTrajectory(const TimedTrajectory& trajectory, int uav_id);

    [[nodiscard]] const std::map<double, Reservation>& reservationsFor(const Cube& cube) const;
    [[nodiscard]] bool hasNoOverlaps() const noexcept;

private:
    [[nodiscard]] bool isValid(const Cube& cube) const noexcept;
    [[nodiscard]] std::size_t index(const Cube& cube) const;

    int nx_;
    int ny_;
    int nz_;
    std::vector<std::map<double, Reservation>> reservations_;
};
