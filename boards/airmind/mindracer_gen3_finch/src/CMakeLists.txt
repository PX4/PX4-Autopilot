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
	init.c
	led.c
	spi.cpp
	timer_config.cpp
	usb.c
)
add_dependencies(drivers_board arch_board_hw_info)

# apply patches to Nuttx ETH RMII and PX4 ADC
message(STATUS "[MindRacer Gen.3] Add patch trigger ... ... ")
set_source_files_properties(nuttx_patch_event PROPERTIES SYMBOLIC "True")
add_custom_command(
    OUTPUT nuttx_patch_event
    COMMAND ${CMAKE_COMMAND}
      -Dtarget_dir:FILEPATH=${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32f7/
      -Dpatch:FILEPATH=${CMAKE_CURRENT_SOURCE_DIR}/patch/stm32_ethernet.c.patch
      -Dmsg:STRING='[MindRacer Gen.3] Patching Nuttx ethernet ...'
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch.cmake
    COMMAND ${CMAKE_COMMAND}
      -Dtarget_dir:FILEPATH=${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32f7/
      -Dpatch:FILEPATH=${CMAKE_CURRENT_SOURCE_DIR}/patch/Kconfig.patch
      -Dmsg:STRING='[MindRacer Gen.3] Patching Nuttx Kconfig ...'
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch.cmake
    COMMAND ${CMAKE_COMMAND}
      -Dtarget_dir:FILEPATH=${PX4_SOURCE_DIR}
      -Dpatch:FILEPATH=${CMAKE_CURRENT_SOURCE_DIR}/patch/adc.cpp.patch
      -Dmsg:STRING='[MindRacer Gen.3] Patching PX4 adc ...'
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch.cmake
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
add_custom_target(patch_trigs DEPENDS nuttx_patch_event)
add_dependencies(px4_layer patch_trigs)

target_link_libraries(drivers_board
	PRIVATE
		arch_board_hw_info
		arch_io_pins
		arch_spi
		drivers__led # drv_led_start
		nuttx_arch # sdio
		nuttx_drivers # sdio
		px4_layer
)

# add airmind board specific drivers
set(config_module_list
    drivers/adj_bec
    PARENT_SCOPE
    )
add_subdirectory("drivers/adj_bec")

set(config_module_list
    drivers/rgbled_ktd202x
    PARENT_SCOPE
    )
add_subdirectory("drivers/rgbled_ktd202x")
