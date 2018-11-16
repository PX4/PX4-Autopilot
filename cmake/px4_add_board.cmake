############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_add_board
#

include(px4_base)

#=============================================================================
#
#	px4_add_board
#
#	This function creates a PX4 board.
#
#	Usage:
#		px4_add_module(
#			BOARD <string>
#			OS <string>
#			[ TOOLCHAIN ] <string>
#			)
#
#	Input:
#		BOARD		: name of board
#		OS			: posix, nuttx, qurt
#
#
#	Example:
#		px4_add_board(
#			BOARD px4fmu-v2_default
#			OS nuttx
#			)
#
function(px4_add_board)

	px4_parse_function_args(
		NAME px4_add_board
		ONE_VALUE
			VENDOR
			MODEL
			LABEL
			BOARD_OVERRIDE
			PLATFORM
			TOOLCHAIN
			ARCH
			ROMFSROOT
			IO
			BOOTLOADER
			UAVCAN_INTERFACES
		MULTI_VALUE
			DRIVERS
			MODULES
			SYSTEMCMDS
			EXAMPLES
			SERIAL_PORTS
			DF_DRIVERS # DriverFramework drivers
		OPTIONS
			CONSTRAINED_FLASH
			ROMFS
			TESTING
		REQUIRED
			PLATFORM
			VENDOR
			MODEL
		ARGN ${ARGN})

	set(PX4_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE STRING "PX4 board directory" FORCE)

	set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
	set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

	if(BOARD_OVERRIDE)
		set(PX4_BOARD ${BOARD_OVERRIDE} CACHE STRING "PX4 board" FORCE)
	else()
		set(PX4_BOARD ${VENDOR}${MODEL} CACHE STRING "PX4 board" FORCE)
	endif()

	if(LABEL)
		set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)
	else()
		set(PX4_BOARD_LABEL "default" CACHE STRING "PX4 board label" FORCE)
	endif()

	# set OS, and append specific platform module path
	set(PX4_PLATFORM ${PLATFORM} CACHE STRING "PX4 board OS" FORCE)

	list(APPEND CMAKE_MODULE_PATH
		${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake
		)

	if(ARCH)
		set(CMAKE_SYSTEM_PROCESSOR ${ARCH} CACHE INTERNAL "system processor" FORCE)
	endif()

	if(TOOLCHAIN)
		set(CMAKE_TOOLCHAIN_FILE ${TOOLCHAIN} CACHE INTERNAL "toolchain file" FORCE)
	endif()

	if(BOOTLOADER)
		set(config_bl_file ${BOOTLOADER} CACHE INTERNAL "bootloader" FORCE)
	endif()

	if(SERIAL_PORTS)
		set(board_serial_ports ${SERIAL_PORTS} PARENT_SCOPE)
	endif()

	include(px4_add_board_os)
	px4_add_board_os(${ARGV})


	# Modules (includes drivers, examples, modules, systemcmds)
	set(config_module_list)

	if(DRIVERS)
		foreach(driver ${DRIVERS})
			list(APPEND config_module_list drivers/${driver})
		endforeach()
	endif()

	if(MODULES)
		foreach(module ${MODULES})
			list(APPEND config_module_list modules/${module})
		endforeach()
	endif()

	if(SYSTEMCMDS)
		foreach(systemcmd ${SYSTEMCMDS})
			list(APPEND config_module_list systemcmds/${systemcmd})
		endforeach()
	endif()

	if(EXAMPLES)
		foreach(example ${EXAMPLES})
			list(APPEND config_module_list examples/${example})
		endforeach()
	endif()

	# DriverFramework drivers
	if(DF_DRIVERS)
		set(config_df_driver_list)
		foreach(driver ${DF_DRIVERS})
			list(APPEND config_df_driver_list ${driver})
			list(APPEND config_module_list platforms/posix/drivers/df_${driver}_wrapper)
		endforeach()
		set(config_df_driver_list ${config_df_driver_list} PARENT_SCOPE)
	endif()

	# add board config directory src to build modules
	file(RELATIVE_PATH board_support_src_rel ${PX4_SOURCE_DIR}/src ${PX4_BOARD_DIR})
	list(APPEND config_module_list ${board_support_src_rel}/src)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/src)

	set(config_module_list ${config_module_list} PARENT_SCOPE)

	# OPTIONS
	
	if(CONSTRAINED_FLASH)
		set(px4_constrained_flash_build "1" CACHE INTERNAL "constrained flash build" FORCE)
	endif()

	if(TESTING)
		set(PX4_TESTING "1" CACHE INTERNAL "testing enabled" FORCE)
	endif()

endfunction()
