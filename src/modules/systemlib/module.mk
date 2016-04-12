############################################################################
#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#
# System utility library
#

SRCS		 = \
		   perf_counter.c \
		   conversions.c \
		   cpuload.c \
		   getopt_long.c \
		   pid/pid.c \
		   airspeed.c \
		   mavlink_log.c \
		   rc_check.c \
		   otp.c \
		   board_serial.c \
		   pwm_limit/pwm_limit.c \
		   mcu_version.c \
		   bson/tinybson.c \
		   circuit_breaker.cpp \
		   printload.c \
		   $(BUILD_DIR)git_version.c

ifneq ($(ARDUPILOT_BUILD),1)
# ArduPilot uses its own parameter system
SRCS		+= param/param.c \
		   system_params.c \
		   circuit_breaker_params.c
endif

ifeq ($(PX4_TARGET_OS),nuttx)
SRCS		+= err.c \
		   up_cxxinitialize.c 
endif

ifneq ($(PX4_TARGET_OS),qurt)
SRCS 		+= hx_stream.c 
endif

MAXOPTIMIZATION	 = -Os

EXTRACFLAGS	= -Wno-sign-compare
