cmake_minimum_required(VERSION 3.5.1)

project(mavsdk_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(READ MAVSDK_VERSION MAVSDK_VERSION)
string(STRIP ${MAVSDK_VERSION} MAVSDK_VERSION)

find_package(MAVSDK "${MAVSDK_VERSION}" REQUIRED)
find_package(Threads REQUIRED)

if(MAVSDK_FOUND)
    add_executable(mavsdk_tests
        test_main.cpp
        autopilot_tester.cpp
        autopilot_tester_failure.cpp
        test_multicopter_control_allocation.cpp
        test_multicopter_failure_injection.cpp
        test_multicopter_failsafe.cpp
        test_multicopter_mission.cpp
        test_multicopter_offboard.cpp
        test_multicopter_manual.cpp
        test_vtol_mission.cpp
    )

    target_link_libraries(mavsdk_tests
        MAVSDK::mavsdk
        ${CMAKE_THREAD_LIBS_INIT}
    )

    target_compile_options(mavsdk_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
            -Wno-error=deprecated-declarations
    )
else()
    message("MAVSDK C++ not found, skipping mavsdk_tests build..")
endif()
