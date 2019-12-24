cmake_minimum_required(VERSION 3.5.1)

if(${PX4_BOARD_MODEL} MATCHES "sitl")
    find_package(MAVSDK QUIET)

    if (MAVSDK_FOUND)
        add_executable(mavsdk_tests
            test_main.cpp
            autopilot_tester.cpp
            test_mission_multicopter.cpp
        )

        target_link_libraries(mavsdk_tests
            MAVSDK::mavsdk
            MAVSDK::mavsdk_action
            MAVSDK::mavsdk_mission
            MAVSDK::mavsdk_telemetry
        )

        target_compile_options(mavsdk_tests
            PRIVATE -std=c++17 -Wall -Wextra -Werror)
    else()
        message("MAVSDK C++ not found, skipping mavsdk_tests build..")
    endif()
endif()
