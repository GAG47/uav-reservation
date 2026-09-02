#pragma once

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string_view>

enum class Direction { North, South, East, West };

enum class Movement { Straight };

struct Cube {
    int x{};
    int y{};
    int z{};

    bool operator==(const Cube&) const = default;
};

struct CubeHash {
    [[nodiscard]] std::size_t operator()(const Cube& cube) const noexcept {
        std::size_t seed = std::hash<int>{}(cube.x);
        seed ^= std::hash<int>{}(cube.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<int>{}(cube.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

[[nodiscard]] constexpr Direction opposite(Direction direction) {
    switch (direction) {
        case Direction::North:
            return Direction::South;
        case Direction::South:
            return Direction::North;
        case Direction::East:
            return Direction::West;
        case Direction::West:
            return Direction::East;
    }
    throw std::invalid_argument("Unknown direction");
}

[[nodiscard]] constexpr std::string_view toString(Direction direction) {
    switch (direction) {
        case Direction::North:
            return "North";
        case Direction::South:
            return "South";
        case Direction::East:
            return "East";
        case Direction::West:
            return "West";
    }
    return "Unknown";
}
