############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_library(drivers_board
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_adc.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_appinitialize.c
	stm32_autoleds.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_boot.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_bringup.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_buttons.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_dma_alloc.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_sdmmc.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_spi.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_uid.c
	../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/stm32h747i-disco/src/stm32_usb.c
	stm32_userleds.c
	foo.cpp
)

target_link_libraries(drivers_board
	PRIVATE
		px4_layer
)
