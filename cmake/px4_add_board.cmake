############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
#	px4_add_board
#
#	This function creates a PX4 board.
#
#	Usage:
#		px4_add_board(
#			PLATFORM <string>
#			[ TOOLCHAIN <string> ]
#			[ ARCHITECTURE <string> ]
#			[ ROMFSROOT <string> ]
#			[ BUILD_BOOTLOADER ]
#			[ IO <string> ]
#			[ UAVCAN_INTERFACES <string> ]
#			[ UAVCAN_PERIPHERALS <list> ]
#			[ DRIVERS <list> ]
#			[ MODULES <list> ]
#			[ SYSTEMCMDS <list> ]
#			[ EXAMPLES <list> ]
#			[ SERIAL_PORTS <list> ]
#			[ CONSTRAINED_FLASH ]
#			[   NO_HELP ]
#			[ CONSTRAINED_MEMORY ]
#			[ EXTERNAL_METADATA ]
#			[ TESTING ]
#			[ LINKER_PREFIX <string> ]
#			[ ETHERNET ]
#			[ CRYPTO <string> ]
#			[ KEYSTORE <string> ]
#			)
#
#	Input:
#		PLATFORM		: PX4 platform name (posix, nuttx, qurt)
#		TOOLCHAIN		: cmake toolchain
#		ARCHITECTURE		: name of the CPU CMake is building for (used by the toolchain)
#		ROMFSROOT		: relative path to the ROMFS root directory
#		BUILD_BOOTLOADER	: flag to enable building and including the bootloader config
#		IO			: name of IO board to be built and included in the ROMFS (requires a valid ROMFSROOT)
#		UAVCAN_INTERFACES	: number of interfaces for UAVCAN
#		UAVCAN_PERIPHERALS      : list of UAVCAN peripheral firmware to build and embed
#		DRIVERS			: list of drivers to build for this board (relative to src/drivers)
#		MODULES			: list of modules to build for this board (relative to src/modules)
#		SYSTEMCMDS		: list of system commands to build for this board (relative to src/systemcmds)
#		EXAMPLES		: list of example modules to build for this board (relative to src/examples)
#		SERIAL_PORTS		: mapping of user configurable serial ports and param facing name
#		CONSTRAINED_FLASH 	: flag to enable constrained flash options (eg limit init script status text)
#		  NO_HELP 	 	: optional condition flag to disable help text on constrained flash systems
#		CONSTRAINED_MEMORY	: flag to enable constrained memory options (eg limit maximum number of uORB publications)
#		EXTERNAL_METADATA	: flag to exclude metadata to reduce flash
#		TESTING			: flag to enable automatic inclusion of PX4 testing modules
#		LINKER_PREFIX	: optional to prefix on the Linker script.
#		ETHERNET		: flag to indicate that ethernet is enabled
#		CRYPTO			: Crypto implementation selection
#		KEYSTORE		: Keystore implememntation selection
#
#
#	Example:
#		px4_add_board(
#			PLATFORM nuttx
#			TOOLCHAIN arm-none-eabi
#			ARCHITECTURE cortex-m7
#			ROMFSROOT px4fmu_common
#			IO px4_io-v2_default
#			SERIAL_PORTS
#				GPS1:/dev/ttyS0
#				TEL1:/dev/ttyS1
#				TEL2:/dev/ttyS2
#				TEL4:/dev/ttyS3
#			DRIVERS
#				barometer/ms5611
#				gps
#				imu/bosch/bmi055
#				imu/invensense/mpu6000
#				magnetometer/isentek/ist8310
#				pwm_out
#				px4io
#				rgbled
#			MODULES
#				commander
#				ekf2
#				land_detector
#				mavlink
#				mc_att_control
#				mc_pos_control
#				navigator
#				sensors
#			MODULES
#				mixer
#				mtd
#				param
#				perf
#				pwm
#				reboot
#				shutdown
#				top
#				topic_listener
#				tune_control
#			)
#
function(px4_add_board)

	px4_parse_function_args(
		NAME px4_add_board
		ONE_VALUE
			PLATFORM
			TOOLCHAIN
			ARCHITECTURE
			ROMFSROOT
			IO
			UAVCAN_INTERFACES
			UAVCAN_TIMER_OVERRIDE
			LINKER_PREFIX
			CRYPTO
			KEYSTORE
		MULTI_VALUE
			DRIVERS
			MODULES
			SYSTEMCMDS
			EXAMPLES
			SERIAL_PORTS
			UAVCAN_PERIPHERALS
		OPTIONS
			BUILD_BOOTLOADER
			CONSTRAINED_FLASH
			NO_HELP
			CONSTRAINED_MEMORY
			EXTERNAL_METADATA
			TESTING
			ETHERNET
		REQUIRED
			PLATFORM
		ARGN ${ARGN})

	set(PX4_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE STRING "PX4 board directory" FORCE)
	include_directories(${PX4_BOARD_DIR}/src)

	# get the VENDOR & MODEL from the caller's directory names
	get_filename_component(base_dir "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
	get_filename_component(MODEL "${base_dir}" NAME)
	get_filename_component(base_dir "${base_dir}" DIRECTORY)
	get_filename_component(VENDOR "${base_dir}" NAME)

	set(PX4_BOARD ${VENDOR}_${MODEL} CACHE STRING "PX4 board" FORCE)

	# board name is uppercase with no underscores when used as a define
	string(TOUPPER ${PX4_BOARD} PX4_BOARD_NAME)
	string(REPLACE "-" "_" PX4_BOARD_NAME ${PX4_BOARD_NAME})
	set(PX4_BOARD_NAME ${PX4_BOARD_NAME} CACHE STRING "PX4 board define" FORCE)

	set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
	set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

	if(NOT LABEL)
		get_filename_component(LABEL "${CMAKE_CURRENT_LIST_FILE}" NAME_WE)
	endif()
	set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)

	set(PX4_CONFIG "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}" CACHE STRING "PX4 config" FORCE)


endfunction()
