############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
	iq-module-communication-cpp/inc/bipbuffer.h
	iq-module-communication-cpp/inc/byte_queue.h
	iq-module-communication-cpp/inc/communication_interface.h
	iq-module-communication-cpp/inc/crc_helper.h
	iq-module-communication-cpp/inc/packet_finder.h

	iq-module-communication-cpp/inc/adc_interface_client.hpp
	iq-module-communication-cpp/inc/anticogging_client.hpp
	iq-module-communication-cpp/inc/anticogging_pro_client.hpp
	iq-module-communication-cpp/inc/arming_handler_client.hpp
	iq-module-communication-cpp/inc/brushless_drive_client.hpp
	iq-module-communication-cpp/inc/buzzer_control_client.hpp
	iq-module-communication-cpp/inc/client_communication.hpp
	iq-module-communication-cpp/inc/coil_temperature_estimator_client.hpp
	iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp
	iq-module-communication-cpp/inc/generic_interface.hpp
	iq-module-communication-cpp/inc/gpio_controller_client.hpp
	iq-module-communication-cpp/inc/hobby_input_client.hpp
	iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp
	iq-module-communication-cpp/inc/mag_alpha_client.hpp
	iq-module-communication-cpp/inc/multi_turn_angle_control_client.hpp
	iq-module-communication-cpp/inc/persistent_memory_client.hpp
	iq-module-communication-cpp/inc/power_monitor_client.hpp
	iq-module-communication-cpp/inc/power_safety_client.hpp
	iq-module-communication-cpp/inc/propeller_motor_control_client.hpp
	iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp
	iq-module-communication-cpp/inc/pwm_interface_client.hpp
	iq-module-communication-cpp/inc/serial_interface_client.hpp
	iq-module-communication-cpp/inc/servo_input_parser_client.hpp
	iq-module-communication-cpp/inc/step_direction_input_client.hpp
	iq-module-communication-cpp/inc/stopping_handler_client.hpp
	iq-module-communication-cpp/inc/stow_user_interface_client.hpp
	iq-module-communication-cpp/inc/system_control_client.hpp
	iq-module-communication-cpp/inc/temperature_estimator_client.hpp
	iq-module-communication-cpp/inc/temperature_monitor_uc_client.hpp
	iq-module-communication-cpp/inc/uavcan_node_client.hpp
	iq-module-communication-cpp/inc/voltage_superposition_client.hpp

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

px4_add_module(
	MODULE drivers__actuators__vertiq_io
	MAIN vertiq_io
	INCLUDES
		iq-module-communication-cpp/inc
	COMPILE_FLAGS
	SRCS
		vertiq_io.cpp
		vertiq_io.hpp
		vertiq_serial_interface.cpp
		vertiq_serial_interface.hpp
		ifci_telemetry.h
		ifci.hpp
		ifci.cpp
	DEPENDS
		px4_work_queue
		mixer_module
		iq_communication
	MODULE_CONFIG
		module.yaml
	)
