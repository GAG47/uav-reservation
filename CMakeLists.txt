cmake_minimum_required(VERSION 3.20)

project(uav_reservation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(uav_reservation_core
    src/benchmark.cpp
    src/cli.cpp
    src/intersection.cpp
    src/reference_scenario.cpp
    src/reservation_table.cpp
    src/scheduler.cpp
    src/simulator.cpp
    src/statistics.cpp
    src/trajectory.cpp
)

target_include_directories(uav_reservation_core PUBLIC include)
target_compile_options(uav_reservation_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(uav_reservation
    src/main.cpp
)

target_link_libraries(uav_reservation PRIVATE uav_reservation_core)
target_compile_options(uav_reservation PRIVATE -Wall -Wextra -Wpedantic)

add_executable(uav_reservation_benchmark src/benchmark_main.cpp)
target_link_libraries(uav_reservation_benchmark PRIVATE uav_reservation_core)
target_compile_options(uav_reservation_benchmark PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()

add_executable(uav_reservation_tests tests/tests.cpp)
target_link_libraries(uav_reservation_tests PRIVATE uav_reservation_core)
target_compile_options(uav_reservation_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME uav_reservation_tests COMMAND uav_reservation_tests)
