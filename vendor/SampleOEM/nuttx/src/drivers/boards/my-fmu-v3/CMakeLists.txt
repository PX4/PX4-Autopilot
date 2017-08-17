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
	MODULE drivers__boards__my-fmu-v3
	COMPILE_FLAGS
	SRCS
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/common/board_crashdump.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/common/board_dma_alloc.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/common/stm32/board_reset.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/common/stm32/board_identity.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/common/stm32/board_mcu_version.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/px4fmu-v2/px4fmu_can.c
		px4fmu2_init.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/px4fmu-v2/px4fmu_timer_config.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/px4fmu-v2/px4fmu_spi.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/px4fmu-v2/px4fmu_usb.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/drivers/boards/px4fmu-v2/px4fmu2_led.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
