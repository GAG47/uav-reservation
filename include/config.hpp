#pragma once

#include <cstdint>
#include <string_view>

enum class LayerMode { MiddleOnly, ThreeLayers };
enum class ScenarioType { Toy, Reference2024AlongRoad };

[[nodiscard]] constexpr std::string_view toString(LayerMode mode) {
    switch (mode) {
        case LayerMode::MiddleOnly:
            return "middle_only";
        case LayerMode::ThreeLayers:
            return "three_layers";
    }
    return "unknown";
}

[[nodiscard]] constexpr std::string_view toString(ScenarioType scenario) {
    switch (scenario) {
        case ScenarioType::Toy:
            return "toy";
        case ScenarioType::Reference2024AlongRoad:
            return "reference-2024-along-road";
    }
    return "unknown";
}

struct ReferenceScenarioConfig {
    double lower_level{60.0};
    double upper_level{90.0};
    double route_width{4.0};
    double intersection_radius{50.0};
    double maximum_roll_degrees{45.0};
    double gravity{9.81};
    double horizontal_separation{15.0};
    double vertical_separation{1.0};
    double horizontal_radius{7.5};
    double vertical_half_height{0.5};
    double arrival_rate_per_route{3.0};
    double ascent_elevator_x{-8.0};
    double descent_elevator_x{8.0};
    double elevator_y{0.0};
    double reference_cube_size{1.0};
};

struct Config {
    ScenarioType scenario{ScenarioType::Toy};
    int nx{10};
    int ny{10};
    int nz{3};
    double cube_size{1.0};
    double grid_origin_x{0.0};
    double grid_origin_y{0.0};
    double grid_origin_z{0.0};

    double dt{0.5};
    double horizontal_speed{2.0};
    double ascending_speed{1.0};
    double descending_speed{1.0};
    double uav_radius{0.15};
    double safety_margin{0.10};
    double occupancy_dt{0.10};
    double max_search_time{300.0};

    std::uint32_t seed{42};
    double arrival_rate{1.0};
    double simulation_duration{20.0};
    LayerMode layer_mode{LayerMode::ThreeLayers};
    ReferenceScenarioConfig reference;
};
