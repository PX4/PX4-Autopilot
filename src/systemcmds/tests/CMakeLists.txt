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

set(srcs
	test_adc.c
	test_bson.c
	test_float.c
	test_gpio.c
	test_hott_telemetry.c
	test_hrt.c
	test_int.c
	test_jig_voltages.c
	test_led.c
	test_sensors.c
	test_servo.c
	test_sleep.c
	test_uart_baudchange.c
	test_uart_console.c
	test_uart_loopback.c
	test_uart_send.c
	test_mixer.cpp
	test_mathlib.cpp
	test_file.c
	test_file2.c
	tests_main.c
	test_params.c
	test_ppm_loopback.c
	test_rc.c
	test_conv.cpp
	test_mount.c
	)

if(${OS} STREQUAL "nuttx")
	list(APPEND srcs
		test_time.c
		)
endif()

px4_add_module(
	MODULE systemcmds__tests
	MAIN tests
	STACK 8000
	COMPILE_FLAGS
		-Wframe-larger-than=6000
		-Wno-float-equal
		-O0
	SRCS ${srcs}
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
