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

set(ECL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

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

			-Wno-unused-parameter
		)
	endif()

	# fetch latest matrix from github
	include(ExternalProject)
	ExternalProject_Add(matrix
			GIT_REPOSITORY "https://github.com/PX4/Matrix.git"
			UPDATE_COMMAND ""
			PATCH_COMMAND ""
			CONFIGURE_COMMAND ""
			BUILD_COMMAND ""
			INSTALL_COMMAND ""
			)
	add_dependencies(prebuild_targets matrix)
	include_directories(${CMAKE_BINARY_DIR}/matrix-prefix/src/matrix)

	add_subdirectory(mathlib)

	include(CTest)
	enable_testing()
	
	if(BUILD_TESTING)

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

endif()

add_subdirectory(airdata)
add_subdirectory(attitude_fw)
add_subdirectory(EKF)
add_subdirectory(geo)
add_subdirectory(geo_lookup)
add_subdirectory(l1)
add_subdirectory(tecs)
add_subdirectory(validation)
