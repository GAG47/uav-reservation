#pragma once

#include "config.hpp"

#include <iosfwd>

struct CommandLineOptions {
    Config config;
    bool show_help{};
    bool deterministic_traffic{};
};

[[nodiscard]] CommandLineOptions parseCommandLine(int argc, char* argv[]);
void printUsage(std::ostream& output, const char* program_name);
