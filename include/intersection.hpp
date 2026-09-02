#pragma once

#include "trajectory.hpp"
#include "types.hpp"

#include <array>
#include <vector>

class Intersection {
public:
    Intersection(int nx, int ny, int nz, double cube_size);

    [[nodiscard]] int nx() const noexcept { return nx_; }
    [[nodiscard]] int ny() const noexcept { return ny_; }
    [[nodiscard]] int nz() const noexcept { return nz_; }
    [[nodiscard]] double cubeSize() const noexcept { return cube_size_; }
    [[nodiscard]] bool isValid(const Cube& cube) const noexcept;

    [[nodiscard]] const std::vector<CandidatePath>& candidatePaths(
        Direction source,
        Direction destination,
        Movement movement) const;

private:
    [[nodiscard]] std::vector<Cube> straightLine(Direction source) const;
    [[nodiscard]] std::vector<CandidatePath> buildStraightPaths(Direction source) const;
    [[nodiscard]] static std::size_t directionIndex(Direction direction);

    int nx_;
    int ny_;
    int nz_;
    double cube_size_;
    std::array<std::vector<CandidatePath>, 4> straight_paths_;
};
