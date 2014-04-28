#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

cmake_minimum_required(VERSION 2.8)

project(libuavcan_linux)

#
# Library (header only)
#
install(DIRECTORY include/uavcan_linux DESTINATION include)

#
# Finding libuavcan - it will be a target if we're running from the top-level CMakeLists.txt,
# otherwise try to find it in the system directories.
#
if (TARGET uavcan)
    message(STATUS "Using uavcan target; source dir: ${libuavcan_SOURCE_DIR}")
    set(UAVCAN_LIB uavcan)
    include_directories(${libuavcan_SOURCE_DIR}/include
                        ${libuavcan_SOURCE_DIR}/include/dsdlc_generated)
else ()
    message(STATUS "Using installed uavcan library")
    find_library(UAVCAN_LIB uavcan REQUIRED)
endif ()

#
# Test/demo executables
#
include_directories(include)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -pedantic -std=c++0x -g3")  # GCC or Clang

add_executable(test_clock test/test_clock.cpp)
target_link_libraries(test_clock ${UAVCAN_LIB} rt)

add_executable(test_socket test/test_socket.cpp)
target_link_libraries(test_socket ${UAVCAN_LIB} rt)

add_executable(test_node test/test_node.cpp)
target_link_libraries(test_node ${UAVCAN_LIB} rt)

add_executable(test_node_status_monitor test/test_node_status_monitor.cpp)
target_link_libraries(test_node_status_monitor ${UAVCAN_LIB} rt)

add_executable(test_time_sync test/test_time_sync.cpp)
target_link_libraries(test_time_sync ${UAVCAN_LIB} rt)

add_executable(test_nodetool test/test_nodetool.cpp)
target_link_libraries(test_nodetool ${UAVCAN_LIB} rt)
