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

set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")
set(UAVCAN_PLATFORM stm32 CACHE STRING "uavcan platform")

string(TOUPPER "${OS}" OS_UPPER)
add_definitions(
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_MAX_NETWORK_SIZE_HINT=16
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-DUAVCAN_PLATFORM=stm32
		-DUAVCAN_STM32_${OS_UPPER}=1
		-DUAVCAN_STM32_NUM_IFACES=1
		-DUAVCAN_STM32_TIMER_NUMBER=2
		-DUAVCAN_USE_CPP03=ON
		-DUAVCAN_USE_EXTERNAL_SNPRINT
		-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
		-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
		-DHW_UAVCAN_NAME=${uavcanblid_name}
		-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
		-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
		)

px4_share_subdirectory(RELDIR ../uavcan/libuavcan ARGS EXCLUDE_FROM_ALL)
add_dependencies(uavcan platforms__nuttx)

include_directories(../../drivers/bootloaders/include)
include_directories(../uavcan/libuavcan/libuavcan/include)
include_directories(../uavcan/libuavcan/libuavcan/include/dsdlc_generated)
include_directories(../uavcan/libuavcan/libuavcan_drivers/posix/include)
include_directories(../uavcan/libuavcan/libuavcan_drivers/stm32/driver/include)

px4_add_module(
	MODULE modules__uavcannode
	MAIN uavcannode
	STACK_MAIN 1048
	COMPILE_FLAGS
		-Wframe-larger-than=1500
		-Wno-deprecated-declarations
		-O3
	SRCS
		uavcannode_main.cpp
    indication_controller.cpp
    sim_controller.cpp
    led.cpp
    resources.cpp
    uavcannode_params.c

	DEPENDS
		platforms__common
		uavcan
	)
