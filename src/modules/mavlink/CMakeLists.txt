############################################################################
#
#   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

set(MAVLINK_DIALECT "development") # standard, development, etc

# force mavlink dialect to standard if flash constrained
if(px4_constrained_flash_build)
	set(MAVLINK_DIALECT "standard")
endif()

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
file(RELATIVE_PATH MAVLINK_GIT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_GIT_DIR})

px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")

add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h
	COMMAND
		${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			#--no-validate
			#--strict-units
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
	COMMENT "Generating Mavlink ${MAVLINK_DIALECT}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml"
	USES_TERMINAL
)
add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)

# mavlink header only library
add_library(mavlink_c INTERFACE)
target_sources(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)
target_include_directories(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR} ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT})
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)


px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_events.cpp
		mavlink_ftp.cpp
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
		tune_publisher.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		airspeed
		component_general_json # for checksums.h
		drivers_accelerometer
		drivers_barometer
		drivers_gyroscope
		drivers_magnetometer
		conversion
		sensor_calibration
		geo
		mavlink_c
		tunes
		version
	UNITY_BUILD
	)

if(PX4_TESTING)
	add_subdirectory(mavlink_tests)
endif()
