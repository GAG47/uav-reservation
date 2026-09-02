#include "cli.hpp"
#include "config.hpp"
#include "intersection.hpp"
#include "reference_scenario.hpp"
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
        const Intersection intersection(config);
        ReservationTable reservations(config.nx, config.ny, config.nz);
        FCFSScheduler scheduler(config, intersection, reservations);
        Simulator simulator(config, scheduler);

        simulator.run(
            options.deterministic_traffic ? simulator.deterministicTraffic()
                                          : simulator.poissonTraffic());
        simulator.saveResults("results/results.csv");
        if (config.scenario == ScenarioType::Reference2024AlongRoad) {
            simulator.saveTrajectoryDebug("results/trajectory_debug.csv");
            std::cout << "Scenario: Reference2024AlongRoad\n"
                      << "Source: Li et al., Drones 2024\n"
                      << "Intersection: Two-road Along-road\n"
                      << "Crossing radius: 50 m\n"
                      << "Flight levels: 60 / 90 m\n"
                      << "Horizontal speed: 6 m/s\n"
                      << "Ascending: 4 m/s\n"
                      << "Descending: 3 m/s\n"
                      << "Reference UAV: Inspire 2\n"
                      << "Horizontal separation: 15 m\n"
                      << "Vertical separation: 1 m\n"
                      << "Turn radius: " << referenceTurningRadius(config) << " m\n"
                      << "Ascent elevator: (-8,0)\n"
                      << "Descent elevator: (8,0)\n"
                      << "Arrival rate: " << config.reference.arrival_rate_per_route
                      << " UAV/min/route\n";
        } else {
            std::cout << "Scenario: ToyScenario\n";
            std::cout << "Layer mode: " << toString(config.layer_mode) << '\n';
            std::cout << "Total arrival rate: " << config.arrival_rate << " UAV/s\n";
        }
        simulator.printSummary(std::cout);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
