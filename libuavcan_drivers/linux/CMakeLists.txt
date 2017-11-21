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
# Scripts
#
install(DIRECTORY scripts/
        USE_SOURCE_PERMISSIONS
        DESTINATION bin)

#
# System dependecies 
#
find_package(Threads REQUIRED)

#
# Finding libuavcan - it will be a target if we're running from the top-level CMakeLists.txt,
# otherwise try to find it in the system directories.
#
if (TARGET uavcan)
    message(STATUS "Using uavcan target; source dir: ${libuavcan_SOURCE_DIR}")
    set(UAVCAN_LIB uavcan)
    include_directories(${libuavcan_SOURCE_DIR}/include
                        ${libuavcan_SOURCE_DIR}/include/dsdlc_generated)
    message(STATUS "POSIX source dir: ${libuavcan_posix_SOURCE_DIR}")
    include_directories(${libuavcan_posix_SOURCE_DIR}/include)
else ()
    message(STATUS "Using installed uavcan library")
    find_library(UAVCAN_LIB uavcan REQUIRED)
endif ()

#
# Applications - tests, tools.
#
include_directories(include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -std=c++11")  # GCC or Clang

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DUAVCAN_DEBUG=1)
endif()

#
# Tests
# These aren't installed, an average library user should not care about them.
#
add_executable(test_clock apps/test_clock.cpp)
target_link_libraries(test_clock ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_socket apps/test_socket.cpp)
target_link_libraries(test_socket ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_node apps/test_node.cpp)
target_link_libraries(test_node ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_time_sync apps/test_time_sync.cpp)
target_link_libraries(test_time_sync ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_system_utils apps/test_system_utils.cpp)
target_link_libraries(test_system_utils ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_posix apps/test_posix.cpp)
target_link_libraries(test_posix ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_dynamic_node_id_client apps/test_dynamic_node_id_client.cpp)
target_link_libraries(test_dynamic_node_id_client ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_file_server apps/test_file_server.cpp)
target_link_libraries(test_file_server ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_multithreading apps/test_multithreading.cpp)
target_link_libraries(test_multithreading ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

#
# Tools
#
add_executable(uavcan_monitor apps/uavcan_monitor.cpp)
target_link_libraries(uavcan_monitor ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(uavcan_nodetool apps/uavcan_nodetool.cpp)
target_link_libraries(uavcan_nodetool ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

add_executable(uavcan_dynamic_node_id_server apps/uavcan_dynamic_node_id_server.cpp)
target_link_libraries(uavcan_dynamic_node_id_server ${UAVCAN_LIB} rt ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS uavcan_monitor
                uavcan_nodetool
                uavcan_dynamic_node_id_server
        RUNTIME DESTINATION bin)
        