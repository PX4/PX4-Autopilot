############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

file(GLOB_RECURSE black_magic_probe_path
     FOLLOW_SYMLINKS
     /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_*-if00
     )
file(GLOB_RECURSE black_magic_probe_console_path
     FOLLOW_SYMLINKS
     /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_*-if02
     )

if(black_magic_probe_path)

	add_custom_target(blackmagic_debug
		COMMAND ${CMAKE_GDB} --nh
			-iex 'set auto-load safe-path ${PX4_BINARY_DIR}'
			-ex 'target extended-remote ${black_magic_probe_path}'
			-ex 'monitor version'
			-ex 'monitor connect_srst enable'
			-ex 'monitor swdp_scan'
			-ex 'attach 1'
			-ex 'load'
			-ex 'run'
			$<TARGET_FILE:px4>
		DEPENDS px4 ${PX4_BINARY_DIR}/.gdbinit
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		)

	add_custom_target(blackmagic_upload
		COMMAND ${CMAKE_GDB} --nx --batch
			-ex 'target extended-remote ${black_magic_probe_path}'
			-ex 'monitor version'
			-ex 'monitor connect_srst enable'
			-ex 'monitor swdp_scan'
			-ex 'attach 1'
			-ex 'load'
			-ex 'kill'
			$<TARGET_FILE:px4>
		DEPENDS px4
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		COMMENT "Uploading with Black Magic Probe"
		)

	add_custom_target(blackmagic_console
		COMMAND screen -t "${PX4_BOARD} console" ${black_magic_probe_console_path} 57600 8N1
		USES_TERMINAL
		)

endif()
