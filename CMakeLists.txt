cmake_minimum_required(VERSION 2.8)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "1")
set(VERSION_PATCH "0")

project(matrix CXX)

option(COVERAGE "Enable code coverage" OFF)
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
	-Wno-sign-compare
	-Wextra
	-Wshadow
	-Wfloat-equal
	-Wpointer-arith
	-Wmissing-declarations
	-Wno-unused-parameter
	-Werror=format-security
	-Werror=array-bounds
	#-Wfatal-errors
	-Werror=unused-variable
	-Werror=reorder
	-Werror=uninitialized
	-Werror=init-self
	-Wcast-qual
	-Wconversion
	-Wcast-align
	#-Werror
	)

if (COVERAGE)
	set(COVERALLS ON)
	list(APPEND CMAKE_CXX_FLAGS
		-fno-inline
		-fno-inline-small-functions
		-fno-default-inline
		--coverage
		)
	include(Coveralls)
	coveralls_turn_on_coverage()
endif()

string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

enable_testing()

include_directories(${CMAKE_SOURCE_DIR})

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

set(astyle_exe ${CMAKE_BINARY_DIR}/astyle/src/bin/astyle)
add_custom_command(OUTPUT ${astyle_exe}
	COMMAND wget http://sourceforge.net/projects/astyle/files/astyle/astyle%202.05.1/astyle_2.05.1_linux.tar.gz -O /tmp/astyle.tar.gz
	COMMAND tar -xvf /tmp/astyle.tar.gz
	COMMAND cd astyle/src && make -f ../build/gcc/Makefile
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

add_custom_target(check_format
	COMMAND scripts/format.sh ${astyle_exe} 0
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	DEPENDS ${astyle_exe}
	VERBATIM
)

add_custom_target(format
	COMMAND scripts/format.sh ${astyle_exe} 1
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	VERBATIM
	DEPENDS ${astyle_exe}
)

set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)
