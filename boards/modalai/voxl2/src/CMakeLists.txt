############################################################################
#
#   Copyright (c) 2022-2026 ModalAI, Inc. All rights reserved.
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

# Both processors need to know about all parameters since modules are
# split across processors and may reference parameters from the other side.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE PARENT_SCOPE)

set(SRCS
	board_config.h
	init.c
)

if("${PX4_PLATFORM}" STREQUAL "qurt")
	list(APPEND SRCS
		i2c_qurt.cpp
		spi_qurt.cpp
	)
elseif("${PX4_PLATFORM}" STREQUAL "posix")
	list(APPEND SRCS
		boardctl.c
		i2c_posix.cpp
		spi_posix.cpp
	)
endif()

add_library(drivers_board ${SRCS})

if("${PX4_PLATFORM}" STREQUAL "qurt")
	# Generate MAVLink common headers for SLPI drivers (dsp_hitl, mavlink_rc_in)
	# Replicates the generation from src/modules/mavlink/CMakeLists.txt so the
	# SLPI build is self-contained and does not depend on voxl2-default.
	set(MAVLINK_GIT_DIR "${PX4_SOURCE_DIR}/src/modules/mavlink/mavlink")
	set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")

	px4_add_git_submodule(TARGET git_mavlink_v2_slpi PATH "${MAVLINK_GIT_DIR}")

	add_custom_command(
		OUTPUT ${MAVLINK_LIBRARY_DIR}/common/common.h
		COMMAND ${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/common.xml
			> ${CMAKE_BINARY_DIR}/mavgen_common.log
		DEPENDS
			git_mavlink_v2_slpi
			${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/common.xml
		COMMENT "Generating MAVLink common headers for SLPI"
	)
	add_custom_target(mavlink_common_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/common/common.h)

	add_library(mavlink_common_headers INTERFACE)
	add_dependencies(mavlink_common_headers mavlink_common_generate)
	target_compile_options(mavlink_common_headers INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
	target_include_directories(mavlink_common_headers INTERFACE
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/common
	)

	# Add custom drivers for SLPI
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/rc_controller rc_controller)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/mavlink_rc_in mavlink_rc_in)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/spektrum_rc spektrum_rc)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/ghst_rc ghst_rc)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/dsp_hitl dsp_hitl)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/dsp_sbus dsp_sbus)
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/qurt/elrs_led elrs_led)

elseif("${PX4_PLATFORM}" STREQUAL "posix")
	# Add custom drivers
	add_subdirectory(${PX4_BOARD_DIR}/src/drivers/posix/apps_sbus apps_sbus)

	# Add custom libraries
	add_subdirectory(${PX4_BOARD_DIR}/src/lib/mpa mpa)

	# Add custom modules
	add_subdirectory(${PX4_BOARD_DIR}/src/modules/voxl_save_cal_params voxl_save_cal_params)
	add_subdirectory(${PX4_BOARD_DIR}/src/modules/vehicle_air_data_bridge vehicle_air_data_bridge)
	add_subdirectory(${PX4_BOARD_DIR}/src/modules/sensor_baro_bridge sensor_baro_bridge)
	add_subdirectory(${PX4_BOARD_DIR}/src/modules/vehicle_local_position_bridge vehicle_local_position_bridge)
endif()
