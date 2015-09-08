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
set(QURT_APPS_HEADER ${CMAKE_BINARY_DIR}/apps.h)

px4_add_git_submodule(TARGET git_dspal PATH "src/lib/dspal")
px4_add_git_submodule(TARGET git_eigen32 PATH "src/lib/eigen-3.2")

function(px4_target_set_flags)
	set(inout_vars
		C_FLAGS CXX_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_target_set_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

        set(DSPAL_ROOT src/lib/dspal)
        set(added_include_dirs
                ${DSPAL_ROOT}/include 
                ${DSPAL_ROOT}/sys 
                ${DSPAL_ROOT}/sys/sys 
                ${DSPAL_ROOT}/mpu_spi/inc
                ${DSPAL_ROOT}/uart_esc/inc
                src/platforms/qurt/include
                src/platforms/posix/include
		src/lib/eigen-3.2
                )

        set(added_definitions
                -D__PX4_QURT 
		-D__PX4_POSIX
		-include ${PX4_INCLUDE_DIR}visibility.h
                )

	# Add the toolchain specific flags
        set(added_cflags ${QURT_CMAKE_C_FLAGS})
        set(added_cxx_flags ${QURT_CMAKE_CXX_FLAGS})

	# FIXME @jgoppert - how to work around issues like this?
	# Without changing global variables?
	# Clear -rdynamic flag which fails for hexagon
	set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
	set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
	endforeach()
endfunction()

function(px4_target_add_modules out_module_directories)
	list(APPEND ${out_module_directories}
		./src/platforms/qurt/px4_layer
		./src/platforms/posix/work_queue
		PARENT_SCOPE
	)
endfunction()

function(px4_target_validate_config)
	# FIXME - this can be done in Firmware/CMakeLists.txt
	list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/qurt)

	if (NOT EXISTS("${CMAKE_SOURCE_DIR}/cmake/qurt/${TARGET_NAME}.cmake")
		message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
	endif()
endfunction()

function(px4_target_firmware)
	set(installed_targets)
	add_library(dspal_main SHARED ./src/platforms/qurt/px4_layer/main.cpp)
	target_link_libraries(dspal_main ${module_list})
	list(APPEND installed_targets dspal_main)
endfunction()

function(px4_target_rules)
	#=============================================================================
	#		apps
	#
	add_custom_command(OUTPUT ${QURT_APPS_HEADER}
		COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
			${CMAKE_SOURCE_DIR}/Tools/qurt_apps.py > ${QURT_APPS_HEADER}
		COMMENT "Generating qurt apps"
		VERBATIM
		)

	add_custom_target(qurt_apps DEPENDS ${QURT_APPS_HEADER})
endfunction()

function(px4_target_testing)
endfunction()
