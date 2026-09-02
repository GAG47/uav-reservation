#include "reservation_table.hpp"

#include <cmath>
#include <iterator>
#include <stdexcept>
#include <unordered_map>

ReservationTable::ReservationTable(int nx, int ny, int nz) : nx_(nx), ny_(ny), nz_(nz) {
    if (nx_ <= 0 || ny_ <= 0 || nz_ <= 0) {
        throw std::invalid_argument("Reservation table dimensions must be positive");
    }
    reservations_.resize(
        static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_));
}

bool ReservationTable::isAvailable(const Cube& cube, double start, double end) const {
    if (!isValid(cube) || !std::isfinite(start) || !std::isfinite(end) || start >= end) {
        return false;
    }

    const auto& cube_reservations = reservations_[index(cube)];
    const auto next = cube_reservations.lower_bound(start);
    if (next != cube_reservations.end() && next->second.start_time < end) {
        return false;
    }
    if (next != cube_reservations.begin()) {
        const auto previous = std::prev(next);
        if (previous->second.end_time > start) {
            return false;
        }
    }
    return true;
}

bool ReservationTable::isTrajectoryAvailable(const TimedTrajectory& trajectory) const {
    std::unordered_map<Cube, std::vector<Occupancy>, CubeHash> proposed;
    for (const Occupancy& occupancy : trajectory.occupancies) {
        if (!isAvailable(occupancy.cube, occupancy.start_time, occupancy.end_time)) {
            return false;
        }
        auto& same_cube = proposed[occupancy.cube];
        for (const Occupancy& earlier : same_cube) {
            if (earlier.end_time > occupancy.start_time && occupancy.end_time > earlier.start_time) {
                return false;
            }
        }
        same_cube.push_back(occupancy);
    }
    return true;
}

void ReservationTable::reserveTrajectory(const TimedTrajectory& trajectory, int uav_id) {
    if (!isTrajectoryAvailable(trajectory)) {
        throw std::logic_error("Cannot reserve a conflicting or invalid trajectory");
    }
    for (const Occupancy& occupancy : trajectory.occupancies) {
        reservations_[index(occupancy.cube)].emplace(
            occupancy.start_time,
            Reservation{occupancy.start_time, occupancy.end_time, uav_id});
    }
}

const std::map<double, Reservation>& ReservationTable::reservationsFor(const Cube& cube) const {
    if (!isValid(cube)) {
        throw std::out_of_range("Cube is outside the reservation table");
    }
    return reservations_[index(cube)];
}

bool ReservationTable::hasNoOverlaps() const noexcept {
    for (const auto& cube_reservations : reservations_) {
        if (cube_reservations.empty()) {
            continue;
        }
        auto previous = cube_reservations.begin();
        for (auto current = std::next(previous); current != cube_reservations.end(); ++current) {
            if (previous->second.end_time > current->second.start_time) {
                return false;
            }
            previous = current;
        }
    }
    return true;
}

bool ReservationTable::isValid(const Cube& cube) const noexcept {
    return cube.x >= 0 && cube.x < nx_ && cube.y >= 0 && cube.y < ny_ && cube.z >= 0 &&
           cube.z < nz_;
}

std::size_t ReservationTable::index(const Cube& cube) const {
    return (static_cast<std::size_t>(cube.z) * static_cast<std::size_t>(ny_) +
            static_cast<std::size_t>(cube.y)) *
               static_cast<std::size_t>(nx_) +
           static_cast<std::size_t>(cube.x);
}
