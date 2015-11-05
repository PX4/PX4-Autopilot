cmake_minimum_required(VERSION 2.8)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "1")
set(VERSION_PATCH "0")

project(matrix CXX)

option(COVERALLS "Turn on coveralls support" OFF)
option(COVERALLS_UPLOAD "Upload the generated coveralls json" OFF)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
	message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
	${CMAKE_SOURCE_DIR}/cmake/coveralls-cmake/cmake)

set(CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	-Wall
	-Weffc++
	-Werror
	#-Wfatal-errors
	)

if (COVERALLS)
	list(APPEND CMAKE_CXX_FLAGS
		-fno-inline
		-fno-inline-small-functions
		-fno-default-inline
		--coverage
		)
	include(Coveralls)
	coveralls_turn_on_coverage()
	add_custom_target(coverage
		COMMAND lcov --capture --directory . --output-file coverage.info
		COMMAND genhtml coverage.info --output-directory out
		COMMAND firefox out/index.html
		WORKING_DIRECTORY ${CMAKE_BUILD_DIR}
		DEPENDS coveralls
		)
endif()

string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

enable_testing()

include_directories(matrix)

file(GLOB_RECURSE COV_SRCS matrix/*.hpp matrix/*.cpp)

# Setup the coveralls target and tell it to gather
# coverage data for all the lib sources.
if (COVERALLS)
	coveralls_setup(
		"${COV_SRCS}"
        ${COVERALLS_UPLOAD}                 
		"${CMAKE_SOURCE_DIR}/cmake/coveralls-cmake/cmake"
		)
endif()

add_subdirectory(test)

add_custom_target(format
	COMMAND astyle --recursive
		${CMAKE_SOURCE_DIR}/matrix/*.*pp
		${CMAKE_SOURCE_DIR}/test/*.*pp
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	VERBATIM
	)

set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)
