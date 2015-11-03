cmake_minimum_required(VERSION 2.8)

project(matrix CXX)

list(APPEND CMAKE_CXX_FLAGS
	-Wall
	-Weffc++
	-Wfatal-errors
	)

string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

enable_testing()

include_directories(matrix)

add_subdirectory(test)
