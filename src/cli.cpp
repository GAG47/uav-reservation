#include "cli.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

double parseDouble(std::string_view option, const char* text) {
    std::size_t parsed = 0;
    const std::string value(text);
    double result = 0.0;
    try {
        result = std::stod(value, &parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid value for " + std::string(option) + ": " + value);
    }
    if (parsed != value.size() || !std::isfinite(result)) {
        throw std::invalid_argument("Invalid value for " + std::string(option) + ": " + value);
    }
    return result;
}

int parseInt(std::string_view option, const char* text) {
    std::size_t parsed = 0;
    const std::string value(text);
    long result = 0;
    try {
        result = std::stol(value, &parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid value for " + std::string(option) + ": " + value);
    }
    if (parsed != value.size() || result < std::numeric_limits<int>::min() ||
        result > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("Invalid value for " + std::string(option) + ": " + value);
    }
    return static_cast<int>(result);
}

std::uint32_t parseSeed(const char* text) {
    std::size_t parsed = 0;
    const std::string value(text);
    unsigned long result = 0;
    try {
        result = std::stoul(value, &parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid value for --seed: " + value);
    }
    if (parsed != value.size() || result > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("Invalid value for --seed: " + value);
    }
    return static_cast<std::uint32_t>(result);
}

LayerMode parseMode(const char* text) {
    const std::string_view value(text);
    if (value == "middle_only") {
        return LayerMode::MiddleOnly;
    }
    if (value == "three_layers") {
        return LayerMode::ThreeLayers;
    }
    throw std::invalid_argument(
        "Invalid value for --mode: " + std::string(value) +
        " (expected middle_only or three_layers)");
}

const char* requireValue(int argc, char* argv[], int& index) {
    if (index + 1 >= argc) {
        throw std::invalid_argument("Missing value for " + std::string(argv[index]));
    }
    ++index;
    return argv[index];
}

void validate(const Config& config) {
    if (config.arrival_rate <= 0.0 || config.simulation_duration <= 0.0 ||
        config.dt <= 0.0 || config.horizontal_speed <= 0.0 ||
        config.vertical_speed <= 0.0 || config.cube_size <= 0.0 ||
        config.uav_radius <= 0.0 || config.safety_margin < 0.0 ||
        config.occupancy_dt <= 0.0 || config.max_search_time < 0.0) {
        throw std::invalid_argument(
            "Arrival rate, duration, dt, speeds, cube size, UAV radius, and occupancy_dt "
            "must be positive; safety margin must be non-negative");
    }
    if (config.nx < 2 || config.ny < 2 || config.nz < 3) {
        throw std::invalid_argument("Grid dimensions must satisfy nx >= 2, ny >= 2, nz >= 3");
    }
}

}  // namespace

CommandLineOptions parseCommandLine(int argc, char* argv[]) {
    CommandLineOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string_view option(argv[index]);
        if (option == "--help" || option == "-h") {
            options.show_help = true;
        } else if (option == "--arrival-rate") {
            options.config.arrival_rate = parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--duration") {
            options.config.simulation_duration = parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--seed") {
            options.config.seed = parseSeed(requireValue(argc, argv, index));
        } else if (option == "--mode") {
            options.config.layer_mode = parseMode(requireValue(argc, argv, index));
        } else if (option == "--dt") {
            options.config.dt = parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--horizontal-speed") {
            options.config.horizontal_speed =
                parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--vertical-speed") {
            options.config.vertical_speed =
                parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--uav-radius") {
            options.config.uav_radius =
                parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--safety-margin") {
            options.config.safety_margin =
                parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--occupancy-dt") {
            options.config.occupancy_dt =
                parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--cube-size") {
            options.config.cube_size = parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--max-search-time") {
            options.config.max_search_time = parseDouble(option, requireValue(argc, argv, index));
        } else if (option == "--nx") {
            options.config.nx = parseInt(option, requireValue(argc, argv, index));
        } else if (option == "--ny") {
            options.config.ny = parseInt(option, requireValue(argc, argv, index));
        } else if (option == "--nz") {
            options.config.nz = parseInt(option, requireValue(argc, argv, index));
        } else {
            throw std::invalid_argument("Unknown option: " + std::string(option));
        }
    }
    validate(options.config);
    return options;
}

void printUsage(std::ostream& output, const char* program_name) {
    output << "Usage: " << program_name << " [options]\n"
           << "  --arrival-rate RATE    Total intersection arrival rate in UAV/s\n"
           << "  --duration SECONDS     Observation duration\n"
           << "  --seed INTEGER         Random seed\n"
           << "  --mode MODE            middle_only or three_layers\n"
           << "  --dt SECONDS           Scheduling time step\n"
           << "  --horizontal-speed V   Fixed horizontal speed\n"
           << "  --vertical-speed V     Fixed ascent/descent speed\n"
           << "  --uav-radius VALUE     UAV sphere radius\n"
           << "  --safety-margin VALUE  Additional safety distance\n"
           << "  --occupancy-dt SEC     Swept-volume sampling interval\n"
           << "  --cube-size VALUE      Cube side length\n"
           << "  --max-search-time SEC  Scheduling search horizon\n"
           << "  --nx N --ny N --nz N   Grid dimensions\n"
           << "  --help                  Show this help\n";
}
