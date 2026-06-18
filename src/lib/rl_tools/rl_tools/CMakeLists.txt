#cmake_minimum_required(VERSION 3.14)
#cmake_minimum_required(VERSION 3.18) # for BLAS::BLAS
cmake_minimum_required(VERSION 3.19) # for HDF5::HDF5
set(CMAKE_POLICY_VERSION_MINIMUM 3.5) # to prevent warnings about "Compatibility with CMake < 3.5 has been removed from CMake."

project(rl_tools_core)

set(CMAKE_CXX_STANDARD 17)

option(RL_TOOLS_ENABLE_TARGETS "Enable building the main targets" ON)
option(RL_TOOLS_ENABLE_TESTS "Enable building the tests" OFF)
option(RL_TOOLS_OFFLINE_BUILD "Use the dependencies in .dependencies without checking for updates (useful after nuking the build directory when offline)" OFF)
option(RL_TOOLS_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)

option(RL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO "Enable MuJoCo" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif()

add_library(rl_tools_core INTERFACE)
add_library(RLtools::Core ALIAS rl_tools_core)
target_compile_features(rl_tools_core INTERFACE cxx_std_17)
target_include_directories(rl_tools_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(rl_tools_full INTERFACE)
add_library(RLtools::RLtools ALIAS rl_tools_full)
target_link_libraries(rl_tools_full INTERFACE rl_tools_core)

# Compiler optimizations
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(rl_tools_full INTERFACE -O3 -ffast-math -march=native)
    endif()
    if(RL_TOOLS_WARNINGS_AS_ERRORS)
        # Safe (just unused warnings)
        target_compile_options(rl_tools_full INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-Werror -Wall -Wextra>) # the $<$<COMPILE_LANGUAGE:CXX>: is required for NVCC to not complain about -Wall
        target_compile_options(rl_tools_full INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter -Wno-unused-local-typedefs -Wno-unused-variable -Wno-unused-but-set-variable>)
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            target_compile_options(rl_tools_full INTERFACE -Wno-unused-lambda-capture)
        endif()

        # todo (put exclusions here)
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            include(CheckCXXCompilerFlag)
            check_cxx_compiler_flag("-Wnan-infinity-disabled" HAS_NAN_INF_WARN)
            if (HAS_NAN_INF_WARN)
                target_compile_options(rl_tools_full INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=nan-infinity-disabled>) # clang does not like some NaN handling in nlohmann_json when -ffast-math is on
            endif()
        endif()
    endif()
endif()


# Configure FetchContent
get_filename_component(CMAKE_BINARY_DIR_BASENAME ${CMAKE_BINARY_DIR} NAME)
if(NOT DEFINED FETCHCONTENT_BASE_DIR)
    set(_test_file "${CMAKE_CURRENT_SOURCE_DIR}/.cmake_write_test")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E touch "${_test_file}"
        RESULT_VARIABLE _write_test_result
        ERROR_QUIET
        OUTPUT_QUIET
    )
    if(_write_test_result EQUAL 0)
        file(REMOVE "${_test_file}")
        set(_fetchcontent_base_dir_root "${CMAKE_CURRENT_SOURCE_DIR}")
    else()
        set(_fetchcontent_base_dir_root "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    set(FETCHCONTENT_BASE_DIR "${_fetchcontent_base_dir_root}/.dependencies/${CMAKE_BINARY_DIR_BASENAME}" CACHE PATH "Base dir for FetchContent downloads" FORCE)
else()
    set(FETCHCONTENT_BASE_DIR "${FETCHCONTENT_BASE_DIR}/${CMAKE_BINARY_DIR_BASENAME}" CACHE PATH "Base dir for FetchContent downloads")
endif()
if(RL_TOOLS_OFFLINE_BUILD)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif()
include(FetchContent)


include(cmake/autodetect/all.cmake)

if(RL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO)
    include(cmake/optional/mujoco.cmake)
endif()

if(RL_TOOLS_ENABLE_TARGETS)
    add_subdirectory(src)
endif()

if (RL_TOOLS_ENABLE_TESTS)
    include(cmake/optional/googletest.cmake)
    if(RL_TOOLS_ENABLE_GTEST)
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)
            set(RL_TOOLS_TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data" CACHE PATH "Path to test data")
            target_compile_definitions(rl_tools_full INTERFACE RL_TOOLS_TEST_DATA_PATH=${RL_TOOLS_TEST_DATA_PATH})
        endif()
        include(CTest)
        add_subdirectory(tests)
    endif()
endif()
