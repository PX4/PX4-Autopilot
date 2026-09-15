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
#
# Standalone test build for UAVCAN sensor bridges.
#
# The production uavcan driver only builds when CONFIG_DRIVERS_UAVCAN is set,
# which is gated on PLATFORM_NUTTX. SITL test target (px4_sitl_test) is Linux,
# so the driver dir is otherwise skipped. This file re-creates the minimum
# pieces (libuavcan core + DSDL gen + bridge sources) needed to compile and
# link host-side gtest binaries that exercise the UAVCAN -> uORB bridge logic
# through libuavcan's in-tree loopback transport.
#

if(NOT BUILD_TESTING)
	return()
endif()

set(UAVCAN_DRIVER_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan)
set(LIBDRONECAN_DIR ${UAVCAN_DRIVER_DIR}/libdronecan)
set(DSDLC_DIR ${LIBDRONECAN_DIR}/dsdl)

px4_add_git_submodule(TARGET git_uavcan_test_dsdl PATH ${DSDLC_DIR})
px4_add_git_submodule(TARGET git_uavcan_test_pydronecan PATH ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/pydronecan)

set(DSDLC_INPUTS
	"${DSDLC_DIR}/ardupilot"
	"${DSDLC_DIR}/com"
	"${DSDLC_DIR}/cuav"
	"${DSDLC_DIR}/dronecan"
	"${DSDLC_DIR}/uavcan"
)
set(DSDLC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/dsdlc_generated)
set(DSDLC_STAMP ${CMAKE_CURRENT_BINARY_DIR}/uavcan_test_dsdlc.stamp)
set(DSDLC_INPUT_FILES)
foreach(_in ${DSDLC_INPUTS})
	file(GLOB_RECURSE _files CONFIGURE_DEPENDS "${_in}/*.uavcan")
	list(APPEND DSDLC_INPUT_FILES ${_files})
endforeach()

add_custom_command(OUTPUT ${DSDLC_STAMP}
	COMMAND ${PYTHON_EXECUTABLE} ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/libuavcan_dsdlc
		--outdir ${DSDLC_OUTPUT} ${DSDLC_INPUTS}
	COMMAND ${CMAKE_COMMAND} -E touch ${DSDLC_STAMP}
	DEPENDS ${DSDLC_INPUT_FILES}
	COMMENT "UAVCAN bridge test: DSDL compiler"
)
add_custom_target(uavcan_test_dsdlc DEPENDS ${DSDLC_STAMP})
add_dependencies(uavcan_test_dsdlc git_uavcan_test_dsdl git_uavcan_test_pydronecan)

# libuavcan core (test-only static lib).
file(GLOB_RECURSE LIBUAVCAN_SRCS CONFIGURE_DEPENDS "${LIBDRONECAN_DIR}/libuavcan/src/*.cpp")
add_library(uavcan_test_core STATIC ${LIBUAVCAN_SRCS})
add_dependencies(uavcan_test_core uavcan_test_dsdlc)
target_include_directories(uavcan_test_core PUBLIC
	${LIBDRONECAN_DIR}/libuavcan/include
	${DSDLC_OUTPUT}
)
target_compile_definitions(uavcan_test_core PUBLIC
	UAVCAN_CPP_VERSION=UAVCAN_CPP03
	UAVCAN_MEM_POOL_BLOCK_SIZE=64
	UAVCAN_PLATFORM=generic
	UAVCAN_NUM_IFACES=1
	UAVCAN_NO_ASSERTIONS
	UAVCAN_IMPLEMENT_PLACEMENT_NEW=0
	UAVCAN_TOSTRING=1
)
target_compile_options(uavcan_test_core PRIVATE
	-Wno-cast-align
	-Wno-deprecated-copy
	-Wno-address-of-packed-member
	-Wno-zero-as-null-pointer-constant
)

# Warning-suppression flags repeat on the gtest binary because libuavcan
# headers (included transitively from <sensors/battery.hpp>) trip
# -Wcast-align / -Wzero-as-null-pointer-constant at the point of inclusion,
# not only inside libuavcan's own .cpp files.
function(add_uavcan_bridge_gtest)
	set(options)
	set(oneValueArgs SRC)
	set(multiValueArgs EXTRA_SRCS)
	cmake_parse_arguments(ABG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

	px4_add_functional_gtest(
		SRC ${ABG_SRC}
		EXTRA_SRCS
			${UAVCAN_DRIVER_DIR}/sensors/sensor_bridge.cpp
			${UAVCAN_DRIVER_DIR}/sensors/battery.cpp
			# node_info is pulled in because UavcanBatteryBridge's serial-number
			# path consults the NodeInfoPublisher abstraction at link time.
			${UAVCAN_DRIVER_DIR}/node_info.cpp
			${ABG_EXTRA_SRCS}
		INCLUDES
			${UAVCAN_DRIVER_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}
			${PX4_SOURCE_DIR}/src
			${PX4_SOURCE_DIR}/src/include
			${PX4_SOURCE_DIR}/src/lib
			${PX4_SOURCE_DIR}/src/lib/matrix
			${PX4_SOURCE_DIR}/platforms/common
			${PX4_SOURCE_DIR}/platforms/common/include
			${PX4_SOURCE_DIR}/platforms/posix/include
			${PX4_SOURCE_DIR}/platforms/posix/src/px4/generic/generic/include
			${PX4_BINARY_DIR}
			${PX4_BINARY_DIR}/uORB
			${PX4_BINARY_DIR}/src/lib
		LINKLIBS
			uavcan_test_core
			battery
			drivers__device
		COMPILE_FLAGS
			-D__PX4_POSIX
			-D__PX4_LINUX
			-include visibility.h
			-DCONFIG_UAVCAN_SENSOR_BATTERY=1
			-Wno-cast-align
			-Wno-deprecated-copy
			-Wno-address-of-packed-member
			-Wno-zero-as-null-pointer-constant
	)
endfunction()

add_uavcan_bridge_gtest(SRC BatteryBridgeTest.cpp)
add_uavcan_bridge_gtest(SRC BatteryBridgeMultiInstanceTest.cpp)
