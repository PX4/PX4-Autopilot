############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
# 3. Neither the name PX4 nor the names of its contributors may be
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

#=============================================================================
# FILE: posix/px4_target_impl.cmake
#
# Each PX4 target OS must implement the cmake/${OS}/px4_target_impl.cmake
# rules for their target that implement the following macros:
#
#	px4_target_set_flags
#	px4_target_validate_config
#	px4_target_firmware
#	px4_target_rules
#	px4_target_testing
#
# The macros are called from the top level CMakeLists.txt
#
set(POSIX_APPS_HEADER ${CMAKE_BINARY_DIR}/apps.h)

add_git_submodule(eigen src/lib/eigen)

macro(px4_target_set_flags)
	include_directories(
		src/platforms/posix/include
		)
	add_definitions(
		-D__PX4_POSIX
		-D__PX4_LINUX
		"-Dnoreturn_function=__attribute__\(\(noreturn\)\)"
		-DCLOCK_MONOTONIC=1
		)
	list(APPEND EXE_LINK_LIBS
		pthread
		)
endmacro()

macro(px4_target_validate_config)
	if(NOT EXISTS ${CMAKE_SOURCE_DIR}/cmake/${OS}/${TARGET_NAME}.cmake)
		message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
	endif()
endmacro()

macro(px4_target_set_modules)
	# Include the target config file
	include(${TARGET_NAME})
endmacro()

macro(px4_target_firmware)
	set(installed_targets)
	add_executable(main ./src/platforms/posix/main.cpp)
	target_link_libraries(main ${module_list} ${EXE_LINK_LIBS} ${module_list} ${EXE_LINK_LIBS})
	list(APPEND installed_targets main)
endmacro()

macro(px4_target_rules)
	#=============================================================================
	#		apps
	#
	add_custom_command(OUTPUT ${POSIX_APPS_HEADER}
		COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
			${CMAKE_SOURCE_DIR}/Tools/posix_apps.py > ${POSIX_APPS_HEADER}
		COMMENT "Generating posix apps"
		VERBATIM
		)

	add_custom_target(posix_apps ALL DEPENDS ${POSIX_APPS_HEADER})
endmacro()

macro(px4_target_testing)
	if (${BOARD} STREQUAL "sitl")

		add_test(test_px4_simple_app ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/cmake/test_compare.py
			--command ${CMAKE_BINARY_DIR}/main
			--stdout ${CMAKE_BINARY_DIR}/test/px4_simple_app_output.txt
			--stdin ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_input.txt
			--check ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_correct.txt
			--start 4 --stop -1
			)

	endif()
endmacro()
