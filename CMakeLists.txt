cmake_minimum_required(VERSION 2.8)

project(matrix CXX)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
	message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()

list(APPEND CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	-Wall
	-Weffc++
	-Werror
	-Wfatal-errors
	)

string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

enable_testing()

include_directories(matrix)

add_subdirectory(test)

add_custom_target(format
	COMMAND astyle --recursive
		${CMAKE_SOURCE_DIR}/matrix/*.*pp
		${CMAKE_SOURCE_DIR}/test/*.*pp
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	VERBATIM
	)
