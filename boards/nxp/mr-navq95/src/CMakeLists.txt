############################################################################
#
#   Copyright (c) 2016, 2019, 2023 PX4 Development Team. All rights reserved.
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

px4_add_library(drivers_board
	i2c.cpp
	led.c
	spi.cpp
	timer_config.cpp
	init.c
	imx95_boardinit.c
	imx95_bringup.c
	mr-navq95.h
	imx95_i2c.c
	imx95_pwm.c
	imx95_spi.c
	iomux_table.c

	${SRCS}
)

# Force compiler not to use builtin functions (like memcpy) to optimize the
# for-loops in init.c (imx9_init_isram_functions), and forbid FPU/NEON
# instructions: that code runs before arm_fpuconfig() enables the FPU, so the
# 64-bit ITCM stores must stay integer (strd) rather than vstr.
set_source_files_properties(init.c PROPERTIES COMPILE_FLAGS "-fno-builtin -mgeneral-regs-only")

target_link_libraries(drivers_board
	PRIVATE
		arch_spi
		arch_io_pins
		drivers__led # drv_led_start
		nuttx_arch
		nuttx_drivers
		px4_layer
)
