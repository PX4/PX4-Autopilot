############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
px4_add_module(
	MODULE drivers__px4io
	MAIN px4io
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
	SRCS
		px4io.cpp
		px4io_serial.cpp
		px4io_uploader.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		arch_px4io_serial
		button_publisher
		circuit_breaker
		mixer_module
	)

# include the px4io binary in ROMFS
if(CONFIG_BOARD_IO)
	message(STATUS "drivers/px4io: ROMFS including ${CONFIG_BOARD_IO}")

	include(ExternalProject)

	ExternalProject_Add(px4io_firmware_build
		DOWNLOAD_COMMAND rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
		UPDATE_COMMAND   rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
		DEPENDS git_nuttx git_nuttx_apps
		SOURCE_DIR ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
		CMAKE_ARGS -DCONFIG=${CONFIG_BOARD_IO}
		INSTALL_COMMAND ""
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS true
	)

	ExternalProject_Get_Property(px4io_firmware_build BINARY_DIR)

	add_custom_target(px4io_firmware_update
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DIR}/${CONFIG_BOARD_IO}.bin ${PX4_BOARD_DIR}/extras/${CONFIG_BOARD_IO}.bin
		DEPENDS px4io_firmware_build
		COMMENT "Copying ${CONFIG_BOARD_IO} to board extras"
	)
endif()
