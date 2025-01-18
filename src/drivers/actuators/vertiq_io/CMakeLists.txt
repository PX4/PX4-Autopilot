############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#build our submodule as its own library so that we can avoid changing it at all
include_directories(
    iq-module-communication-cpp/inc
    )

#add all of our submodule sources
set(SOURCES
	iq-module-communication-cpp/src/generic_interface.cpp
	iq-module-communication-cpp/inc/client_communication.cpp
	iq-module-communication-cpp/src/packet_finder.c
	iq-module-communication-cpp/src/crc_helper.c
	iq-module-communication-cpp/src/byte_queue.c
	)

add_library(iq_communication ${SOURCES})
add_dependencies(iq_communication prebuild_targets)

if("${PX4_PLATFORM}" MATCHES "nuttx")
    target_compile_definitions(iq_communication PUBLIC __NUTTX__)
endif()

px4_add_git_submodule(TARGET git_iq-module-communication-cpp PATH "iq-module-communication-cpp")

px4_add_module(
	MODULE drivers__actuators__vertiq_io
	MAIN vertiq_io
	INCLUDES
	COMPILE_FLAGS
	SRCS
		vertiq_io.cpp
		vertiq_io.hpp
		vertiq_serial_interface.cpp
		vertiq_serial_interface.hpp
		vertiq_telemetry_manager.cpp
		vertiq_telemetry_manager.hpp
		vertiq_client_manager.cpp
		vertiq_client_manager.hpp
		vertiq_configuration_handler.cpp
		vertiq_configuration_handler.hpp
		entry_wrapper.hpp
	DEPENDS
		px4_work_queue
		mixer_module
		iq_communication
	MODULE_CONFIG
		module.yaml
	)
