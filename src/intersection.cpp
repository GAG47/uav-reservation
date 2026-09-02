#include "intersection.hpp"

#include "reference_scenario.hpp"

#include <stdexcept>
#include <utility>

Intersection::Intersection(int nx, int ny, int nz, double cube_size)
    : nx_(nx), ny_(ny), nz_(nz), cube_size_(cube_size) {
    if (nx_ < 2 || ny_ < 2 || nz_ < 3 || cube_size_ <= 0.0) {
        throw std::invalid_argument("Intersection dimensions and cube size are invalid");
    }

    for (Direction source : {Direction::North, Direction::South, Direction::East, Direction::West}) {
        straight_paths_[directionIndex(source)] = buildStraightPaths(source);
    }
}

Intersection::Intersection(const Config& config)
    : Intersection(config.nx, config.ny, config.nz, config.cube_size) {
    scenario_ = config.scenario;
    if (scenario_ == ScenarioType::Reference2024AlongRoad) {
        for (const FlightDirection source : {
                 FlightDirection::Northbound,
                 FlightDirection::Eastbound,
                 FlightDirection::Southbound,
                 FlightDirection::Westbound}) {
            for (const Movement movement : {
                     Movement::Straight, Movement::LeftTurn, Movement::RightTurn}) {
                const FlightDirection target = referenceTargetDirection(source, movement);
                reference_paths_[{source, target, movement}] = {
                    makeReferenceCandidatePath(config, source, movement)};
            }
        }
    }
}

bool Intersection::isValid(const Cube& cube) const noexcept {
    return cube.x >= 0 && cube.x < nx_ && cube.y >= 0 && cube.y < ny_ && cube.z >= 0 &&
           cube.z < nz_;
}

const std::vector<CandidatePath>& Intersection::candidatePaths(
    Direction source,
    Direction destination,
    Movement movement) const {
    if (movement != Movement::Straight || destination != opposite(source)) {
        throw std::invalid_argument("Only straight movement to the opposite side is supported");
    }
    return straight_paths_[directionIndex(source)];
}

const std::vector<CandidatePath>& Intersection::candidatePaths(
    FlightDirection source,
    FlightDirection target,
    Movement movement) const {
    if (scenario_ != ScenarioType::Reference2024AlongRoad) {
        throw std::invalid_argument("FlightDirection paths require the reference scenario");
    }
    const auto found = reference_paths_.find({source, target, movement});
    if (found == reference_paths_.end()) {
        throw std::invalid_argument("Invalid reference source, target, and movement mapping");
    }
    return found->second;
}

std::vector<Cube> Intersection::straightLine(Direction source) const {
    std::vector<Cube> line;
    if (source == Direction::North || source == Direction::South) {
        const int x = source == Direction::North ? (nx_ - 1) / 2 : nx_ / 2;
        line.reserve(static_cast<std::size_t>(ny_));
        if (source == Direction::North) {
            for (int y = ny_ - 1; y >= 0; --y) {
                line.push_back({x, y, 1});
            }
        } else {
            for (int y = 0; y < ny_; ++y) {
                line.push_back({x, y, 1});
            }
        }
    } else {
        const int y = source == Direction::East ? ny_ / 2 : (ny_ - 1) / 2;
        line.reserve(static_cast<std::size_t>(nx_));
        if (source == Direction::East) {
            for (int x = nx_ - 1; x >= 0; --x) {
                line.push_back({x, y, 1});
            }
        } else {
            for (int x = 0; x < nx_; ++x) {
                line.push_back({x, y, 1});
            }
        }
    }
    return line;
}

std::vector<CandidatePath> Intersection::buildStraightPaths(Direction source) const {
    const std::vector<Cube> middle = straightLine(source);
    const auto cubeCenter = [this](const Cube& cube) {
        return Point3D{
            (static_cast<double>(cube.x) + 0.5) * cube_size_,
            (static_cast<double>(cube.y) + 0.5) * cube_size_,
            (static_cast<double>(cube.z) + 0.5) * cube_size_,
        };
    };
    const Point3D middle_start = cubeCenter(middle.front());
    const Point3D middle_end = cubeCenter(middle.back());
    std::vector<CandidatePath> paths;
    paths.push_back({0, {{middle_start, middle_end, SegmentType::HorizontalLine}}});

    for (const auto& [path_id, height] :
         {std::pair{1, 2}, std::pair{2, 0}}) {
        Cube layered_start_cube = middle.front();
        Cube layered_end_cube = middle.back();
        layered_start_cube.z = height;
        layered_end_cube.z = height;
        const Point3D layered_start = cubeCenter(layered_start_cube);
        const Point3D layered_end = cubeCenter(layered_end_cube);
        paths.push_back({
            path_id,
            {
                {middle_start,
                 layered_start,
                 height == 2 ? SegmentType::Ascending : SegmentType::Descending},
                {layered_start, layered_end, SegmentType::HorizontalLine},
                {layered_end,
                 middle_end,
                 height == 2 ? SegmentType::Descending : SegmentType::Ascending},
            },
        });
    }
    return paths;
}

std::size_t Intersection::directionIndex(Direction direction) {
    return static_cast<std::size_t>(direction);
}
