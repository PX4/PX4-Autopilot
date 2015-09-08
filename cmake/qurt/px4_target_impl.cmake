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

add_git_submodule(dspal src/lib/dspal)
add_git_submodule(eigen src/lib/eigen-3.2)

macro(px4_target_set_flags)
	list(APPEND EXE_LINK_LIBS
		pthread
		)
        set(DSPAL_ROOT src/lib/dspal)
        include_directories(
                ${DSPAL_ROOT}/include 
                ${DSPAL_ROOT}/sys 
                ${DSPAL_ROOT}/sys/sys 
                ${DSPAL_ROOT}/mpu_spi/inc
                ${DSPAL_ROOT}/uart_esc/inc
                src/platforms/qurt/include
                src/platforms/posix/include
		src/lib/eigen-3.2
                )
        add_definitions(
                -D__PX4_QURT 
		-D__PX4_POSIX
		-include ${PX4_INCLUDE_DIR}visibility.h
                )

	# Add the toolchain specific flags
        set(CMAKE_C_FLAGS ${QURT_CMAKE_C_FLAGS})
        set(CMAKE_CXX_FLAGS ${QURT_CMAKE_CXX_FLAGS})
        set(CMAKE_SHARED_LINKER_FLAGS "")

        message(STATUS "CMAKE_C_FLAGS: -${CMAKE_C_FLAGS}-")
        message(STATUS "CMAKE_CXX_FLAGS: -${CMAKE_CXX_FLAGS}-")

	# Clear -rdynamic flag which fails for hexagon
	set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
	set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

endmacro()

macro(px4_target_set_modules)
	list(APPEND module_directories
		./src/platforms/qurt/px4_layer
		./src/platforms/posix/work_queue
	)
endmacro()

macro(px4_target_validate_config)
	if (${TARGET_NAME} STREQUAL "qurt-hil-simple")
	else()
		message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
	endif()
endmacro()

macro(px4_target_firmware)
	set(installed_targets)
	add_library(dspal_main SHARED ./src/platforms/qurt/px4_layer/main.cpp)
	target_link_libraries(dspal_main ${module_list})
	list(APPEND installed_targets dspal_main)
endmacro()

macro(px4_target_rules)
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
endmacro()

macro(px4_target_testing)
endmacro()
