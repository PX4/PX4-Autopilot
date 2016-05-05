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

# for generated files
include_directories(${CMAKE_BINARY_DIR}/src/modules/param)

set(SRCS
	perf_counter.c
	conversions.c
	cpuload.c
	pid/pid.c
	airspeed.c
	mavlink_log.c
	rc_check.c
	otp.c
	board_serial.c
	pwm_limit/pwm_limit.c
	mcu_version.c
	bson/tinybson.c
	circuit_breaker.cpp
	battery.cpp
	)

if(${OS} STREQUAL "nuttx")
	list(APPEND SRCS
		err.c
		printload.c
		param/param.c
		up_cxxinitialize.c
		)
elseif ("${CONFIG_SHMEM}" STREQUAL "1")
	list(APPEND SRCS
		param/param_shmem.c
		print_load_posix.c
		)
else()
	list(APPEND SRCS
		param/param.c
		print_load_posix.c
		)
endif()

if(NOT ${OS} STREQUAL "qurt")
	list(APPEND SRCS
		hx_stream.c
		)
endif()

px4_add_module(
	MODULE modules__systemlib
	COMPILE_FLAGS
		-Wno-sign-compare
		-Os
	SRCS ${SRCS}
	DEPENDS
		platforms__common
		modules__param
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
