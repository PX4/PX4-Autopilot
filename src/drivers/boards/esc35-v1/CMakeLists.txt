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

message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"DO NOT RUN THIS ON HW")
message(WARNING-"IT IS NOT PINED OUT TO HW")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")
message(WARNING-"Configuraton is incomplete")

include_directories(../../bootloaders/include)

set(SRCS)
list(APPEND SRCS
    esc35_init.c
    esc35_led.c
    esc35_usb.c
   ../../../drivers/bootloaders/src/common/boot_app_shared.c
   ../../../drivers/bootloaders/src/util/crc.c
)

if(${OS} STREQUAL "nuttx")
	if (NOT ${BOARD} STREQUAL "sim")
		list(APPEND SRCS
			../../../drivers/device/device_nuttx.cpp
			../../../drivers/device/cdev.cpp
		)
	endif()
else()
	list(APPEND SRCS
		../../../drivers/device/device_posix.cpp
		../../../drivers/device/vdev.cpp
	)
endif()

px4_add_module(
	MODULE drivers__boards__esc35-v1
	COMPILE_FLAGS
		-Os
	SRCS ${SRCS}
	DEPENDS
		platforms__common
	)
