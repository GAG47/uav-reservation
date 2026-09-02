#pragma once

#include "config.hpp"
#include "intersection.hpp"
#include "reservation_table.hpp"
#include "uav.hpp"

#include <vector>

class FCFSScheduler {
public:
    FCFSScheduler(
        const Config& config,
        const Intersection& intersection,
        ReservationTable& reservations);

    void schedule(UAV& uav);
    void schedule(UAV& uav, const std::vector<CandidatePath>& candidate_paths);
    [[nodiscard]] const std::vector<CandidatePath>& candidatePathsFor(
        const UAV& uav) const;

private:
    const Config& config_;
    const Intersection& intersection_;
    ReservationTable& reservations_;
};
