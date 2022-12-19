############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

add_library(drivers_board
	can.c
	i2c.cpp
	init.cpp
	led.c
	manifest.c
	mtd.cpp
	sdio.c
	spi.cpp
	timer_config.cpp
	usb.c
	toc.c
)

add_dependencies(drivers_board platform_gpio_mcp23009)

target_link_libraries(drivers_board
	PRIVATE
		arch_io_pins
		arch_spi
		arch_board_hw_info
		drivers__led # drv_led_start
		nuttx_arch # sdio
		nuttx_drivers # sdio
		px4_layer
		platform_gpio_mcp23009
)

set(manifest_bin_name "mft_v0_0x0010.bin" CACHE INTERNAL "MFT" FORCE)
set(manifest_bin "${PX4_BINARY_DIR}/romfs_extras/${manifest_bin_name}" CACHE FILEPATH "manifest binary path")

if(EXISTS ${PX4_BOARD_DIR}/manifest/${manifest_bin_name})
	make_directory(${PX4_BINARY_DIR}/romfs_extras/)
	file(COPY ${PX4_BOARD_DIR}/manifest/${manifest_bin_name} DESTINATION ${PX4_BINARY_DIR}/romfs_extras/)
	file(RELATIVE_PATH manifest_bin_relative ${PX4_SOURCE_DIR} ${manifest_bin})
	add_custom_command(OUTPUT ${manifest_bin}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/etc/extras/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${manifest_bin} ${PX4_BINARY_DIR}/etc/extras/
		)
	add_custom_target(copy_manifest_bin DEPENDS ${manifest_bin})
endif()
