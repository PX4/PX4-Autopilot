cmake_minimum_required(VERSION 2.8.12)
project(micrortps_receiver)

# Find requirements
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

# Set C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
    CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(--std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

file(GLOB MICRORTPS_AGENT_SOURCES *.cxx)
add_executable(micrortps_agent ${MICRORTPS_AGENT_SOURCES})
target_link_libraries(micrortps_agent fastrtps fastcdr)