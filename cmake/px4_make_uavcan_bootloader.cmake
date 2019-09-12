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
#
#	px4_make_uavcan_bootloadable
#
#	This function adds a uavcan boot loadable target.
#
#	Usage:
#	  px4_make_uavcan_bootloadable(
#	   BOARD	<board>
#	   BIN <input bin file>)
#	   HWNAME <uavcan name>
#	   HW_MAJOR <number>
#	   HW_MINOR <number>
#	   SW_MAJOR <number>
#	   SW_MINOR <number>)
#
#	Input:
#	  BOARD	     : the board
#	  BIN	     : the bin file to generate the bootloadable image from
#	  HWNAME     : the uavcan name
#	  HW_MAJOR   : the major hardware revision
#	  HW_MINOR   : the minor hardware revision
#	  SW_MAJOR   : the major software revision
#	  SW_MINOR   : the minor software revision
#
#	Output:
#		OUT			: None
#
#	Example:
#	px4_make_uavcan_bootloadable(
#	  BOARD ${PX4_BOARD}
#		BIN ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		HWNAME ${uavcanblid_name}
#		HW_MAJOR ${uavcanblid_hw_version_major}
#		HW_MINOR ${uavcanblid_hw_version_minor}
#		SW_MAJOR ${uavcanblid_sw_version_major}
#		SW_MINOR ${uavcanblid_sw_version_minor}
#	 )
#
function(px4_make_uavcan_bootloadable)
	px4_parse_function_args(
		NAME px4_make_uavcan_bootloadable
		ONE_VALUE BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		REQUIRED BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		ARGN ${ARGN})

	string(REPLACE "\"" "" HWNAME ${HWNAME})

	execute_process(
		COMMAND git rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit
		OUTPUT_VARIABLE uavcanbl_git_desc
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

	if ("${uavcanbl_git_desc}" STREQUAL "")
		set(uavcanbl_git_desc ffffffff)
	endif()
	set(uavcan_bl_imange_name ${HWNAME}-${HW_MAJOR}.${HW_MINOR}-${SW_MAJOR}.${SW_MINOR}.${uavcanbl_git_desc}.uavcan.bin)
	message(STATUS "Generating UAVCAN Bootable as ${uavcan_bl_imange_name}")
	add_custom_command(OUTPUT ${uavcan_bl_imange_name}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/make_can_boot_descriptor.py
			-v --use-git-hash ${BIN} ${uavcan_bl_imange_name}
		DEPENDS ${BIN})
	add_custom_target(build_uavcan_bl_${PX4_BOARD} ALL DEPENDS ${uavcan_bl_imange_name})
endfunction()
