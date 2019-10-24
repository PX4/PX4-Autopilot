cmake_minimum_required(VERSION 3.5.1)

find_package(MAVSDK REQUIRED)

add_executable(test_mission_multicopter
    test_main.cpp
    autopilot_tester.cpp
    test_mission_multicopter.cpp
)

target_link_libraries(test_mission_multicopter
    MAVSDK::mavsdk
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_mission
    MAVSDK::mavsdk_telemetry
)

target_compile_options(test_mission_multicopter
    PRIVATE -std=c++17 -Wall -Wextra -Werror)
