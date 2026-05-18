############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

set(LIBDRONECAN_DIR         ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan)
set(LIBDRONECAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)

# Mirror src/drivers/uavcannode/CMakeLists.txt's platform-driver detection
# so libuavcan headers compile with the same flags here as in uavcannode.
if(CONFIG_ARCH_CHIP)
	if(${CONFIG_NET_CAN} MATCHES "y")
		set(UAVCAN_DRIVER "socketcan")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "kinetis")
		set(UAVCAN_DRIVER "kinetis")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32h7")
		set(UAVCAN_DRIVER "stm32h7")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32")
		set(UAVCAN_DRIVER "stm32")
	endif()
endif()

string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
string(TOUPPER "${UAVCAN_DRIVER}" UAVCAN_DRIVER_UPPER)

px4_add_module(
	MODULE drivers__teseo_fw_proxy
	MAIN teseo_fw_proxy
	COMPILE_FLAGS
		-DUAVCAN_${UAVCAN_DRIVER_UPPER}_${OS_UPPER}=1
		-DUAVCAN_${UAVCAN_DRIVER_UPPER}_NUM_IFACES=${config_uavcan_num_ifaces}
		-DUAVCAN_DRIVER=uavcan_${UAVCAN_DRIVER}
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-Wno-cast-align
		-Wno-deprecated-copy
	SRCS
		TeseoFwProxy.cpp
	INCLUDES
		${LIBDRONECAN_DIR}/libuavcan/include
		${LIBDRONECAN_DIR}/libuavcan/include/dsdlc_generated
		${LIBDRONECAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver/include
		${PX4_SOURCE_DIR}/src/drivers
	)
