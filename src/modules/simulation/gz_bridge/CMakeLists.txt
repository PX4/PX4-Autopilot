############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

if(NOT DEFINED ENV{GZ_DISTRO})
    set(GZ_DISTRO "harmonic" CACHE STRING "Gazebo distribution to use")
else()
    set(GZ_DISTRO $ENV{GZ_DISTRO})
endif()

# Define library version combinations for different Gazebo distributions
# https://github.com/gazebo-tooling/gazebodistro/blob/master/collection-harmonic.yaml
if(GZ_DISTRO STREQUAL "harmonic")
    set(GZ_CMAKE_VERSION "3")
    set(GZ_MSGS_VERSION "10")
    set(GZ_TRANSPORT_VERSION "13")
    message(STATUS "Using Gazebo Harmonic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
elseif(GZ_DISTRO STREQUAL "ionic")
    set(GZ_CMAKE_VERSION "4")
    set(GZ_MSGS_VERSION "11")
    set(GZ_TRANSPORT_VERSION "14")
    message(STATUS "Using Gazebo Ionic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
else()
    message(FATAL_ERROR "Unknown Gazebo distribution: ${GZ_DISTRO}. Valid options are: harmonic or ionic")
endif()

find_package(gz-cmake${GZ_CMAKE_VERSION} REQUIRED)
find_package(gz-msgs${GZ_MSGS_VERSION} REQUIRED)
find_package(gz-transport${GZ_TRANSPORT_VERSION} REQUIRED)
find_package(Protobuf REQUIRED)

# Custom message generation
file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/msgs/*.proto")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})
px4_add_library(px4_msgs ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(px4_msgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(px4_msgs PUBLIC ${PROTOBUF_LIBRARIES})

# Build the plugins
add_subdirectory(plugins)

# Add the gz_bridge module
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
		gz-transport${GZ_TRANSPORT_VERSION}::core
		px4_msgs
	MODULE_CONFIG
		module.yaml
)

# Link custom messages with module
target_include_directories(modules__simulation__gz_bridge PUBLIC px4_msgs)
target_link_libraries(modules__simulation__gz_bridge PUBLIC px4_msgs)

# What are we doing here?
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

# Below we setup the build targets for our worlds and models
# e.g: gz_x500_flow_aruco --> gz_<model>_<world>

file(GLOB gz_worlds ${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds/*.sdf)
file(GLOB gz_airframes ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*)

foreach(gz_airframe IN LISTS gz_airframes)
	set(model_name)
	string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})

	foreach(world ${gz_worlds})

		get_filename_component("world_name" ${world} NAME_WE)

		if(world_name STREQUAL "default")
			add_custom_target(gz_${model_name}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 OpticalFlowSystem
			)
		else()
			add_custom_target(gz_${model_name}_${world_name}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} PX4_GZ_WORLD=${world_name} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 OpticalFlowSystem
			)
		endif()
	endforeach()
endforeach()

# Setup the environment variables: PX4_GZ_MODELS, PX4_GZ_WORLDS, GZ_SIM_RESOURCE_PATH
configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)

