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


add_definitions(
		-DHW_UAVCAN_NAME=${uavcanblid_name}
		-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
		-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

include_directories(${px4_bootloader_base}include)
include_directories(${px4_module_base}systemlib)
include_directories(.)

px4_add_module(
	MODULE drivers__boards__px4esc-v1__bootloader
	COMPILE_FLAGS
		-Os
	SRCS
		boot.c
		led.c
		../../common/stm32/board_identity.c
		${px4_bootloader_base}src/uavcan/main.c
		${px4_bootloader_base}src/common/boot_app_shared.c
		${px4_bootloader_base}src/sched/timer.c
		${px4_bootloader_base}src/fs/flash.c
		${px4_bootloader_base}src/util/crc.c
		${px4_bootloader_base}src/util/random.c
		${px4_bootloader_base}src/arch/stm32/drivers/can/driver.c
		${px4_bootloader_base}src/protocol/uavcan.c
	DEPENDS
		nuttx_export_${BOARD}
)


# vim: set noet ft=cmake fenc=utf-8 ff=unix :
