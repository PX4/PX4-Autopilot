cmake_minimum_required(VERSION 2.8)

project(matrix CXX)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Weffc++")

enable_testing()

include_directories(src)

add_subdirectory(test)
