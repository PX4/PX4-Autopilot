############################################################################
#
#   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

if("${PX4_BOARD_LABEL}" STREQUAL "bootloader")
	add_library(drivers_board
		bootloader_main.c
		mpfs_emmcsd.c
		mpfs_domain.c
		mpfs_composite.c
	)
	target_link_libraries(drivers_board
		PRIVATE
			nuttx_arch
			nuttx_drivers
			bootloader
	)

	target_include_directories(drivers_board
		 PRIVATE
			${PX4_SOURCE_DIR}/platforms/nuttx/src/bootloader/common
			${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/opensbi/opensbi-3rdparty/include
	)

else()
	add_library(board_bus_info
		i2c.cpp
		spi.cpp
	)
	add_dependencies(board_bus_info nuttx_context)

	add_library(drivers_board
		init.c
		led.c
		mpfs_emmcsd.c
		mpfs_ihc.c
		mpfs_spinor.c
		mpfs_pwm.c
		manifest.c
		mtd.cpp
		spi_drv.cpp
	)
	add_dependencies(drivers_board nuttx_context)
	if (CONFIG_OPENAMP)
		target_link_libraries(drivers_board
			PRIVATE
				drivers__led # drv_led_start
				nuttx_arch
				nuttx_drivers
				nuttx_openamp)
	else()
		target_link_libraries(drivers_board
			PRIVATE
				drivers__led # drv_led_start
				nuttx_arch
				nuttx_drivers)
	endif()

	if (NOT DEFINED CONFIG_BUILD_FLAT)
		target_link_libraries(drivers_board PRIVATE px4_kernel_layer)
		target_compile_options(drivers_board PRIVATE -D__KERNEL__)
	else()
		target_link_libraries(drivers_board PRIVATE px4_layer board_bus_info)
	endif()

	add_library(drivers_userspace
		init_entrypt.c
		mpfs_userspace.c
	)
	add_dependencies(drivers_userspace nuttx_context)

endif()
