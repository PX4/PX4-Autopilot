############################################################################
#
#   Copyright (c) 2015-2018 ECL Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ECL nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

cmake_minimum_required(VERSION 3.0)

project(ECL CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
    message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage")

execute_process(
	COMMAND git describe --always --tags
	OUTPUT_VARIABLE git_tag
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)

message(STATUS "PX4 ECL: Very lightweight Estimation & Control Library ${git_tag}")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# code coverage support
option(COV_HTML "Display html for coverage" OFF)
option(ECL_ASAN "Enable ECL address sanitizer" OFF)

set(CMAKE_CXX_FLAGS_COVERAGE
    "--coverage -fprofile-arcs -ftest-coverage -fno-default-inline -fno-inline -fno-inline-small-functions -fno-elide-constructors"
    CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
    "--coverage -ftest-coverage -lgcov"
    CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)
mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE)


set(ECL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE string "ECL source location" FORCE)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
	# ECL standalone build
	add_definitions(-DECL_STANDALONE)
	set(ECL_STANDALONE 1)

	add_custom_target(prebuild_targets)

	if(MSVC)
		add_compile_options(
			/W4
			/WX

			/D_USE_MATH_DEFINES

			/wd4100 # warning C4100: unreferenced formal parameter
			/wd4244 # warning C4244: '=': conversion from 'double' to 'int32_t', possible loss of data
			)
	elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
		add_compile_options(
			-pedantic
		
			-Wall
			-Wextra
			-Werror

			-Wno-missing-field-initializers # ignore for GCC 4.8 support
		)
	endif()

	# testing
	include(CTest)
	enable_testing()
	
	if(BUILD_TESTING)
		option(ECL_TESTS "Build ECL tests" ON)

		add_custom_target(check
			COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -C Debug
			DEPENDS ecl_EKF
			USES_TERMINAL
		)

		option(EKF_PYTHON_TESTS "Build the EKF python tests" OFF)

		if (EKF_PYTHON_TESTS)
			# swig requires -fPIC
			set(CMAKE_POSITION_INDEPENDENT_CODE ON)
		endif()
	endif()

	# fetch latest matrix from github
	include(ExternalProject)
	ExternalProject_Add(matrix
			GIT_REPOSITORY "https://github.com/PX4/Matrix.git"
			GIT_TAG 6b0777d815cd64902eb0575d56ec52f53aebb4a0
			UPDATE_COMMAND ""
			PATCH_COMMAND ""
			CONFIGURE_COMMAND ""
			BUILD_COMMAND ""
			INSTALL_COMMAND ""
			)
	add_dependencies(prebuild_targets matrix)
	include_directories(${CMAKE_BINARY_DIR}/matrix-prefix/src/matrix)

	# mathlib only needed in standalone build
	add_subdirectory(mathlib)

endif()

# santiziers (ASAN)
if(ECL_ASAN)
	message(STATUS "ecl address sanitizer enabled ")

	# environment variables
	#  ASAN_OPTIONS=detect_stack_use_after_return=1
	#  ASAN_OPTIONS=check_initialization_order=1

	add_compile_options(
		-fsanitize=address
		-g3
		-O1
		-fno-omit-frame-pointer
	)

	set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address)
endif()

add_subdirectory(airdata)
add_subdirectory(attitude_fw)
add_subdirectory(EKF)
add_subdirectory(geo)
add_subdirectory(geo_lookup)
add_subdirectory(l1)
add_subdirectory(tecs)
add_subdirectory(validation)

#=============================================================================
# Coverage
#
if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")

    add_custom_target(coverage
        COMMAND ${CMAKE_CTEST_COMMAND}
        COMMAND lcov --capture --directory . --output-file coverage.info
        COMMAND lcov --remove coverage.info '/usr/*' --output-file coverage.info # filter out system
        COMMAND lcov --remove coverage.info 'build/coverage_build/EKF/swig/*' --output-file coverage.info
        COMMAND lcov --summary coverage.info
        WORKING_DIRECTORY ${CMAKE_BUILD_DIR}
        DEPENDS check
        )

    add_custom_target(coverage_html
        COMMAND genhtml coverage.info --output-directory out
        WORKING_DIRECTORY ${CMAKE_BUILD_DIR}
        DEPENDS coverage
        )

endif()

#=============================================================================
# Doxygen
#
# Only in standalone build
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)

	option(BUILD_DOXYGEN "Build doxygen documentation" OFF)
	
	if (BUILD_DOXYGEN)
		find_package(Doxygen)
		if (DOXYGEN_FOUND)
			# set input and output files
			set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
			set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	
			# request to configure the file
			configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
	
			# note the option ALL which allows to build the docs together with the application
			add_custom_target(doxygen ALL
				COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
				WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
				COMMENT "Generating documentation with Doxygen"
				DEPENDS
				VERBATIM
				USES_TERMINAL
			)
	
		else()
			message(FATAL_ERROR "Doxygen needs to be installed to generate documentation")
		endif()
	endif()
endif()
