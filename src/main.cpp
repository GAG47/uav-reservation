#include "cli.hpp"
#include "config.hpp"
#include "intersection.hpp"
#include "reservation_table.hpp"
#include "scheduler.hpp"
#include "simulator.hpp"

#include <exception>
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        const CommandLineOptions options = parseCommandLine(argc, argv);
        if (options.show_help) {
            printUsage(std::cout, argv[0]);
            return 0;
        }
        const Config& config = options.config;
        const Intersection intersection(config.nx, config.ny, config.nz, config.cube_size);
        ReservationTable reservations(config.nx, config.ny, config.nz);
        FCFSScheduler scheduler(config, intersection, reservations);
        Simulator simulator(config, scheduler);

        simulator.run(simulator.poissonTraffic());
        simulator.saveResults("results/results.csv");
        std::cout << "Layer mode: " << toString(config.layer_mode) << '\n';
        std::cout << "Total arrival rate: " << config.arrival_rate << " UAV/s\n";
        simulator.printSummary(std::cout);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
