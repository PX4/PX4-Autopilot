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
#		* px4_add_upload
#		* px4_add_adb_push
#		* px4_add_adb_push_to_bebop
#		* px4_add_scp_push
#		* px4_add_upload_aero
#


#=============================================================================
#
#	px4_add_upload
#
#	This function generates source code from ROS msg definitions.
#
#	Usage:
#		px4_add_upload(OUT <target> BUNDLE <file.px4>)
#
#	Input:
#		BUNDLE		: the firmware.px4 file
#		OS			: the operating system
#		BOARD		: the board
#
#	Output:
#		OUT			: the firmware target
#
#	Example:
#		px4_add_upload(OUT upload
#			BUNDLE main.px4
#			)
#
function(px4_add_upload)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT BUNDLE
		REQUIRED OS BOARD OUT BUNDLE
		ARGN ${ARGN})
	set(serial_ports)
	if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
		list(APPEND serial_ports
			/dev/serial/by-id/*_PX4_*
			/dev/serial/by-id/usb-3D_Robotics*
			/dev/serial/by-id/usb-The_Autopilot*
			/dev/serial/by-id/usb-Bitcraze*
			/dev/serial/by-id/pci-3D_Robotics*
			/dev/serial/by-id/pci-Bitcraze*
			/dev/serial/by-id/usb-Gumstix*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
		list(APPEND serial_ports
			/dev/tty.usbmodemPX*,/dev/tty.usbmodem*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
		foreach(port RANGE 32 0)
			list(APPEND serial_ports
				"COM${port}")
		endforeach()
	endif()
	px4_join(OUT serial_ports LIST "${serial_ports}" GLUE ",")
	add_custom_target(${OUT}
		COMMAND ${PYTHON_EXECUTABLE}
			${PX4_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${BUNDLE}
		DEPENDS ${BUNDLE}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


function(px4_add_adb_push)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


function(px4_add_adb_push_to_bebop)
	px4_parse_function_args(
		NAME px4_add_upload_to_bebop
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload_to_bebop.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


function(px4_add_scp_push)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT DEST
		MULTI_VALUE FILES DEPENDS
		REQUIRED OS BOARD OUT FILES DEPENDS DEST
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${PX4_SOURCE_DIR}/Tools/scp_upload.sh ${FILES} ${DEST}
		DEPENDS ${DEPENDS}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


function(px4_add_upload_aero)
	px4_parse_function_args(
		NAME px4_add_upload_aero
		ONE_VALUE OS BOARD OUT BUNDLE
		REQUIRED OS BOARD OUT BUNDLE
		ARGN ${ARGN})

	add_custom_target(${OUT}
		COMMAND ${PX4_SOURCE_DIR}/Tools/aero_upload.sh ${BUNDLE}
		DEPENDS ${BUNDLE}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()
