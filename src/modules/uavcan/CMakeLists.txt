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

# uavcan project
set(uavcan_c_flags ${c_flags})
list(REMOVE_ITEM uavcan_c_flags -std=gnu++0x -D__CUSTOM_FILE_IO__)
set(uavcan_cxx_flags ${cxx_flags})
list(REMOVE_ITEM uavcan_cxx_flags -std=gnu++0x -std=c++11 -Wundef -Werror -D__CUSTOM_FILE_IO__)
set(uavcan_deps git_uavcan)
set(uavcan_platform generic)

set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)

string(TOUPPER "${OS}" OS_UPPER)

set(uavcan_definitions
	-DUAVCAN_NO_ASSERTIONS
	-DUAVCAN_STM32_NUM_IFACES=2
	-DUAVCAN_USE_EXTERNAL_SNPRINT
	-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
	-DUAVCAN_MAX_NETWORK_SIZE_HINT=16
	-DUAVCAN_STM32_TIMER_NUMBER=5
	-DUAVCAN_STM32_${OS_UPPER}=1
	-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
	)
set(uavcan_extra_flags ${uavcan_definitions})

if (${OS} STREQUAL "nuttx")
	set(uavcan_platform stm32)
	list(APPEND uavcan_extra_flags
		-I${nuttx_export_dir}/include
		-I${nuttx_export_dir}/include/cxx
		-I${nuttx_export_dir}/arch/chip
		-I${nuttx_export_dir}/arch/common
		)
	list(APPEND uavcan_deps nuttx_export_${BOARD})
endif()

list(APPEND uavcan_c_flags ${uavcan_extra_flags})
list(APPEND uavcan_cxx_flags ${uavcan_extra_flags})

px4_join(OUT uavcan_c_flags LIST "${uavcan_c_flags}" GLUE " ")
px4_join(OUT uavcan_cxx_flags LIST "${uavcan_cxx_flags}" GLUE " ")

externalproject_add(libuavcan
	DEPENDS ${uavcan_deps}
	DOWNLOAD_COMMAND ""
	UPDATE_COMMAND git submodule update --init
	LOG_INSTALL 1
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/lib/uavcan
	CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
		-DUAVCAN_USE_CPP03=ON
		-DUAVCAN_PLATFORM=${uavcan_platform}
		-DUAVCAN_OS=${OS}
		-DCMAKE_CXX_FLAGS=${uavcan_cxx_flags}
		-DCMAKE_C_FLAGS=${uavcan_c_flags}
		-DCMAKE_INSTALL_PREFIX=${ep_base}/Install
)

string(TOUPPER ${OS} OS_UPPER)

add_definitions(${uavcan_definitions})

px4_add_module(
	MODULE modules__uavcan
	MAIN uavcan
	STACK 3200
	COMPILE_FLAGS
		-Wframe-larger-than=1500
		-Wno-deprecated-declarations
		-O3
	SRCS

		# Main
		uavcan_main.cpp
		uavcan_servers.cpp
		uavcan_params.c

		# Actuators
		actuators/esc.cpp

		# Sensors
		sensors/sensor_bridge.cpp
		sensors/gnss.cpp
		sensors/mag.cpp
		sensors/baro.cpp

	DEPENDS
		platforms__common
		libuavcan	
	)

## vim: set noet ft=cmake fenc=utf-8 ff=unix : 
