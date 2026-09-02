#pragma once

#include <cstdint>
#include <string_view>

enum class LayerMode { MiddleOnly, ThreeLayers };

[[nodiscard]] constexpr std::string_view toString(LayerMode mode) {
    switch (mode) {
        case LayerMode::MiddleOnly:
            return "middle_only";
        case LayerMode::ThreeLayers:
            return "three_layers";
    }
    return "unknown";
}

struct Config {
    int nx{10};
    int ny{10};
    int nz{3};
    double cube_size{1.0};

    double dt{0.5};
    double horizontal_speed{2.0};
    double vertical_speed{1.0};
    double uav_radius{0.15};
    double safety_margin{0.10};
    double occupancy_dt{0.10};
    double max_search_time{300.0};

    std::uint32_t seed{42};
    double arrival_rate{1.0};
    double simulation_duration{20.0};
    LayerMode layer_mode{LayerMode::ThreeLayers};
};
