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

set(LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libuavcan)
set(LIBUAVCAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)

px4_add_git_submodule(TARGET git_uavcan PATH ${LIBUAVCAN_DIR})

set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")
set(UAVCAN_PLATFORM generic CACHE STRING "uavcan platform")

string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
add_definitions(
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_MAX_NETWORK_SIZE_HINT=16
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-DUAVCAN_PLATFORM=generic
		-DUAVCAN_STM32_${OS_UPPER}=1
		-DUAVCAN_STM32_NUM_IFACES=1
		-DUAVCAN_STM32_TIMER_NUMBER=2
		-DUAVCAN_USE_CPP03=ON
		-DUAVCAN_USE_EXTERNAL_SNPRINT
		)

add_compile_options(-Wno-cast-align) # TODO: fix and enable
add_subdirectory(${LIBUAVCAN_DIR} uavcannode_libuavcan)
add_subdirectory(${LIBUAVCAN_DIR_DRIVERS}/stm32/driver uavcanesc_uavcan_drivers)
target_include_directories(uavcan_stm32_driver PUBLIC
	${LIBUAVCAN_DIR}/libuavcan/include
	${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
	)
add_dependencies(uavcan prebuild_targets)

px4_add_module(
	MODULE modules__uavcannode
	MAIN uavcannode
	STACK_MAIN 1048
	INCLUDES
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
		${LIBUAVCAN_DIR}/libuavcan_drivers/posix/include
		${LIBUAVCAN_DIR_DRIVERS}/stm32/driver/include
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
	DEPENDS
		drivers_bootloaders
		git_uavcan
		version

		uavcan_stm32_driver

		# within libuavcan
		libuavcan_dsdlc
		uavcan
	)

set_property(GLOBAL APPEND PROPERTY PX4_MODULE_LIBRARIES uavcan_stm32_driver)
