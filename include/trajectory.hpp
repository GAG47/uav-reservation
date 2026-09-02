#pragma once

#include "types.hpp"

#include <vector>

struct CandidatePath {
    int id{};
    std::vector<Cube> cubes;
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
    double speed,
    double cube_size);
