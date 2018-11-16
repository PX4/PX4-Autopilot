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
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_add_board_os
#

include(px4_base)

#=============================================================================
#
#	px4_add_board_os
#
#	This function creates a PX4 board.
#
#	Usage:
#		px4_add_board_os(
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
#		px4_add_board_os(
#			BOARD px4fmu-v2_default
#			OS nuttx
#			)
#
function(px4_add_board_os)

	px4_parse_function_args(
		NAME px4_add_board_os
		ONE_VALUE
			VENDOR
			MODEL
			LABEL
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
		ARGN ${ARGN})

	if(NOT CMAKE_TOOLCHAIN_FILE)
		# default to native toolchain
		set(CMAKE_TOOLCHAIN_FILE Toolchain-native CACHE INTERNAL "toolchain file" FORCE)
	endif()

	include(px4_impl_os)
	px4_os_prebuild_targets(OUT prebuild_targets BOARD ${PX4_BOARD})

endfunction()
