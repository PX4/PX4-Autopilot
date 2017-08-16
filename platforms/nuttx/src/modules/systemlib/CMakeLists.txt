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

include_directories(${PX4_SOURCE_DIR}/src/modules/systemlib)

set(SRCS
	${PX4_SOURCE_DIR}/src/modules/systemlib/perf_counter.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/conversions.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/cpuload.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/pid/pid.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/airspeed.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/mavlink_log.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/rc_check.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/otp.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/board_serial.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/pwm_limit/pwm_limit.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/bson/tinybson.c
	${PX4_SOURCE_DIR}/src/modules/systemlib/circuit_breaker.cpp
	${PX4_SOURCE_DIR}/src/modules/systemlib/battery.cpp
	${PX4_SOURCE_DIR}/src/modules/systemlib/hysteresis/hysteresis.cpp
	${PX4_SOURCE_DIR}/src/modules/systemlib/hx_stream.c
	err.c
	printload.c
	flashparams/flashparams.c
	flashparams/flashfs.c
	up_cxxinitialize.c
	)

px4_add_module(
	MODULE modules__systemlib
	COMPILE_FLAGS
		-Wno-sign-compare
	SRCS ${SRCS}
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
