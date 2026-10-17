############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
file(RELATIVE_PATH MAVLINK_GIT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_GIT_DIR})

px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")

# always generate uAvionix dialect
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")
add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
	COMMAND
		${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${MAVLINK_DIALECT_UAVIONIX}.log
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml

	COMMENT "Generating Mavlink ${MAVLINK_DIALECT_UAVIONIX}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml"
)
add_custom_target(mavlink_c_generate_uavionix DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h PROPERTIES GENERATED true)

# generate the primary dialect; an external dialect is staged into the build tree by px4_add_external_mavlink_dialect()
# and needs the upstream definitions copied next to it once the submodule is checked out
get_property(MAVLINK_DIALECT_DIR GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT_DIR)
set(MAVLINK_DIALECT_STAGE_TARGET)
if(MAVLINK_DIALECT_DIR)
	get_property(MAVLINK_EXTERNAL_DIALECT GLOBAL PROPERTY PX4_EXTERNAL_MAVLINK_DIALECT)
	add_custom_target(mavlink_stage_external_dialect
		COMMAND ${CMAKE_COMMAND}
			-DUPSTREAM_DIR=${MAVLINK_GIT_DIR}/message_definitions/v1.0
			-DSTAGING_DIR=${MAVLINK_DIALECT_DIR}
			-DDIALECT=${MAVLINK_EXTERNAL_DIALECT}
			-P ${PX4_SOURCE_DIR}/cmake/px4_stage_external_mavlink_dialect.cmake
		DEPENDS git_mavlink_v2
		COMMENT "Staging upstream MAVLink definitions for external dialect ${MAVLINK_EXTERNAL_DIALECT}"
	)
	set(MAVLINK_DIALECT_STAGE_TARGET mavlink_stage_external_dialect)
else()
	set(MAVLINK_DIALECT_DIR ${MAVLINK_GIT_DIR}/message_definitions/v1.0)
endif()
file(RELATIVE_PATH MAVLINK_DIALECT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_DIALECT_DIR})

add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
	COMMAND
		${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_DIALECT_DIR}/${CONFIG_MAVLINK_DIALECT}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${CONFIG_MAVLINK_DIALECT}.log
	DEPENDS
		git_mavlink_v2
		${MAVLINK_DIALECT_STAGE_TARGET}
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_DIALECT_DIR}/${CONFIG_MAVLINK_DIALECT}.xml
		${BOARD_DEFCONFIG}

		# generate after uAvionix
		mavlink_c_generate_uavionix
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h

	COMMENT "Generating Mavlink ${CONFIG_MAVLINK_DIALECT}: ${MAVLINK_DIALECT_DIR_RELATIVE}/${CONFIG_MAVLINK_DIALECT}.xml"
)
add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h PROPERTIES GENERATED true)



# mavlink header only library
add_library(mavlink_c INTERFACE)
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
target_sources(mavlink_c
	INTERFACE
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
)
target_include_directories(mavlink_c
	INTERFACE
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/../
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
)

# Out-of-tree module extension points, compiled only when external modules are built
# so firmware without them is byte-identical
set(MAVLINK_EXT_SRCS)
set(MAVLINK_EXT_FLAGS)
if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	set(MAVLINK_EXT_SRCS mavlink_ext_handler.cpp mavlink_ext_stream.cpp)
	set(MAVLINK_EXT_FLAGS -DMAVLINK_EXTERNAL_MODULES)
endif()

px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
		${MAVLINK_EXT_FLAGS}
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_events.cpp
		mavlink_ftp.cpp
		mavlink_ftp_param.cpp
		mavlink_log_handler.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_parameters.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_timesync.cpp
		mavlink_ulog.cpp
		mavlink_sign_control.cpp
		MavlinkStatustextHandler.cpp
		open_drone_id_translations.cpp
		tune_publisher.cpp
		${MAVLINK_EXT_SRCS}
	MODULE_CONFIG
		module.yaml
		mavlink_params.yaml
	DEPENDS
		airspeed
		component_general_json # for checksums.h
		dataman_client
		drivers_accelerometer
		drivers_gyroscope
		drivers_magnetometer
		conversion
		gnss
		sensor_calibration
		geo
		mavlink_c
		timesync
		tunes
		variable_length_ringbuffer
		version
	UNITY_BUILD
	)

if(CONFIG_DRIVERS_SERIALPASSTHROUGH)
	target_link_libraries(modules__mavlink PRIVATE drivers__serialpassthrough)
endif()

target_link_libraries(modules__mavlink PRIVATE failure_injection)

px4_add_unit_gtest(SRC MavlinkStatustextHandlerTest.cpp
	INCLUDES
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
	COMPILE_FLAGS
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
		-Wno-cast-align # TODO: fix
	LINKLIBS
		modules__mavlink
	)

px4_add_unit_gtest(SRC MavlinkFtpParamTest.cpp EXTRA_SRCS mavlink_ftp_param.cpp)

if(CONFIG_NET AND "${PX4_PLATFORM}" MATCHES "nuttx")
	target_link_libraries(modules__mavlink PRIVATE nuttx_apps) # netlib_get_ipv4netmask
endif()
