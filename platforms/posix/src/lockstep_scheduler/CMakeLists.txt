cmake_minimum_required(VERSION 2.8.12)

if(NOT PROJECT_NAME STREQUAL "px4")

    project(lockstep_scheduler)

    set (CMAKE_CXX_STANDARD 11)

    add_library(lockstep_scheduler
        src/lockstep_scheduler.cpp
    )

    target_include_directories(lockstep_scheduler
        PUBLIC
            $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(lockstep_scheduler
        pthread
    )

    target_compile_options(lockstep_scheduler PRIVATE -Wall -Wextra -Werror -O2)

    add_subdirectory(test)

else()

    add_library(lockstep_scheduler
        src/lockstep_scheduler.cpp
    )
    include_directories(
        include
    )

endif()
