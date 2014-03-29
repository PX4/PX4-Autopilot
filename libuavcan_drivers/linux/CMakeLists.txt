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
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -pedantic -std=c++0x -g3 -O1")  # GCC or Clang

add_executable(log_subscriber test/log_subscriber.cpp)
target_link_libraries(log_subscriber ${UAVCAN_LIB})
target_link_libraries(log_subscriber rt)
