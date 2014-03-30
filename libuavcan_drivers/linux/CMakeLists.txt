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
# Test/demo executables
#
find_library(UAVCAN_LIB uavcan REQUIRED)
include_directories(include)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -pedantic -std=c++0x -g3")  # GCC or Clang

add_executable(test_clock test/test_clock.cpp)
target_link_libraries(test_clock ${UAVCAN_LIB} rt)

add_executable(test_socket test/test_socket.cpp)
target_link_libraries(test_socket ${UAVCAN_LIB} rt)
