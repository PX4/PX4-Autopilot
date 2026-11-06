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

px4_add_library(can_forward
	CanForwardFilter.cpp
	CanForwardSession.cpp
)

target_include_directories(can_forward PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC CanForwardFilterTest.cpp LINKLIBS can_forward)
px4_add_unit_gtest(SRC CanForwardSessionTest.cpp LINKLIBS can_forward)
px4_add_unit_gtest(SRC CanFrameCodecTest.cpp LINKLIBS can_forward)

# ForwardingCanDriver is tested against the real libuavcan CanIOManager and transport layer.
# The uavcan driver itself is NuttX only, so the parts of libuavcan the tests need are built here.
if(BUILD_TESTING)
	set(LIBDRONECAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan)
	set(CAN_FORWARD_DSDLC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated)

	px4_add_git_submodule(TARGET git_can_forward_dsdl PATH ${LIBDRONECAN_DIR}/dsdl)
	px4_add_git_submodule(TARGET git_can_forward_pydronecan PATH ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/pydronecan)

	file(GLOB_RECURSE CAN_FORWARD_DSDL_FILES ${LIBDRONECAN_DIR}/dsdl/uavcan/*.uavcan)

	add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/can_forward_dsdlc.stamp
		COMMAND ${PYTHON_EXECUTABLE} ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/libuavcan_dsdlc
			--outdir ${CAN_FORWARD_DSDLC_OUTPUT} ${LIBDRONECAN_DIR}/dsdl/uavcan
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/can_forward_dsdlc.stamp
		DEPENDS ${CAN_FORWARD_DSDL_FILES} git_can_forward_dsdl git_can_forward_pydronecan
		COMMENT "can_forward test: DSDL compiler"
	)
	add_custom_target(can_forward_dsdlc DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/can_forward_dsdlc.stamp)

	set(LIBUAVCAN_SRC ${LIBDRONECAN_DIR}/libuavcan/src)

	add_library(can_forward_libuavcan STATIC EXCLUDE_FROM_ALL
		${LIBUAVCAN_SRC}/driver/uc_can.cpp
		${LIBUAVCAN_SRC}/node/uc_global_data_type_registry.cpp
		${LIBUAVCAN_SRC}/transport/uc_can_io.cpp
		${LIBUAVCAN_SRC}/transport/uc_crc.cpp
		${LIBUAVCAN_SRC}/transport/uc_dispatcher.cpp
		${LIBUAVCAN_SRC}/transport/uc_frame.cpp
		${LIBUAVCAN_SRC}/transport/uc_outgoing_transfer_registry.cpp
		${LIBUAVCAN_SRC}/transport/uc_transfer.cpp
		${LIBUAVCAN_SRC}/transport/uc_transfer_buffer.cpp
		${LIBUAVCAN_SRC}/transport/uc_transfer_listener.cpp
		${LIBUAVCAN_SRC}/transport/uc_transfer_receiver.cpp
		${LIBUAVCAN_SRC}/transport/uc_transfer_sender.cpp
		${LIBUAVCAN_SRC}/uc_data_type.cpp
		${LIBUAVCAN_SRC}/uc_dynamic_memory.cpp
		${LIBUAVCAN_SRC}/uc_error.cpp
	)
	add_dependencies(can_forward_libuavcan can_forward_dsdlc)
	target_compile_definitions(can_forward_libuavcan PUBLIC UAVCAN_CPP_VERSION=UAVCAN_CPP11)
	target_include_directories(can_forward_libuavcan SYSTEM PUBLIC
		${LIBDRONECAN_DIR}/libuavcan/include
		${CAN_FORWARD_DSDLC_OUTPUT}
	)
	target_compile_options(can_forward_libuavcan PRIVATE -w)

	px4_add_unit_gtest(SRC ForwardingCanDriverTest.cpp LINKLIBS can_forward can_forward_libuavcan)
	px4_add_unit_gtest(SRC CanForwardTransferTest.cpp LINKLIBS can_forward can_forward_libuavcan)
endif()
