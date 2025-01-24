############################################################################
#
#   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

# GZ Harmonic only
# https://github.com/gazebo-tooling/gazebodistro/blob/master/collection-harmonic.yaml
find_package(gz-cmake3 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13)

find_package(Protobuf REQUIRED)

# Find all proto files in proto directory
file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/msgs/*.proto")

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
	${MSGS_PROTOS}
)

px4_add_library(gz_px4_messages
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_include_directories(gz_px4_messages
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gz_px4_messages
    PUBLIC ${PROTOBUF_LIBRARIES}
)

# TODO: glob every file in proto/
# set(MSGS_PROTOS
#      ${CMAKE_CURRENT_SOURCE_DIR}/proto/px4/msgs/opticalflow.proto
# )

# gz_msgs_generate_messages(
#   TARGET custom_msgs
#   PROTO_PACKAGE "px4.msgs"
#   MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
#   MSGS_PROTOS ${MSGS_PROTOS}
#   DEPENDENCIES gz-msgs10::gz-msgs10
# )

add_subdirectory(plugins)

add_compile_options(-frtti -fexceptions)

# set(GZ_TRANSPORT_VER ${gz-transport_VERSION_MAJOR})

# if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
# 	set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
# else()
# 	set(GZ_TRANSPORT_LIB ignition-transport${GZ_TRANSPORT_VER}::core)
# endif()

px4_add_module(
	MODULE modules__simulation__gz_bridge
	MAIN gz_bridge
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		GZBridge.cpp
		GZBridge.hpp
		GZMixingInterfaceESC.cpp
		GZMixingInterfaceESC.hpp
		GZMixingInterfaceServo.cpp
		GZMixingInterfaceServo.hpp
		GZMixingInterfaceWheel.cpp
		GZMixingInterfaceWheel.hpp
		GZGimbal.cpp
		GZGimbal.hpp
	DEPENDS
		mixer_module
		px4_work_queue
		gz-transport13::core
		gz_px4_messages
	MODULE_CONFIG
		module.yaml
)

target_include_directories(modules__simulation__gz_bridge PUBLIC gz_px4_messages)

# TODO: this doesn't feel right
target_link_libraries(modules__simulation__gz_bridge PUBLIC gz_px4_messages)

px4_add_git_submodule(TARGET git_gz PATH "${PX4_SOURCE_DIR}/Tools/simulation/gz")
include(ExternalProject)
ExternalProject_Add(gz
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/gz
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_gz
	INSTALL_COMMAND ""
	DEPENDS git_gz
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

set(gz_worlds
	default
	windy
	baylands
	lawn
	aruco
	rover
	walls
)

# find corresponding airframes
file(GLOB gz_airframes
     RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
     ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*
)

# remove any .post files
foreach(gz_airframe IN LISTS gz_airframes)
	if(gz_airframe MATCHES ".post")
		list(REMOVE_ITEM gz_airframes ${gz_airframe})
	endif()
endforeach()
list(REMOVE_DUPLICATES gz_airframes)

foreach(gz_airframe IN LISTS gz_airframes)
	set(model_only)
	string(REGEX REPLACE ".*_gz_" "" model_only ${gz_airframe})

	foreach(world ${gz_worlds})

		get_filename_component("world_name" ${world} NAME_WE)

		if(world_name STREQUAL "default")
			add_custom_target(gz_${model_only}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_only} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 OpticalFlowSystem
			)
		else()
			add_custom_target(gz_${model_only}_${world_name}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_only} PX4_GZ_WORLD=${world_name} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 OpticalFlowSystem
			)
		endif()
	endforeach()
endforeach()
# PX4_GZ_MODELS, PX4_GZ_WORLDS, GZ_SIM_RESOURCE_PATH
configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)

