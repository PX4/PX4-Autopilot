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

# Find the gz_Transport library
# Look for GZ Ionic or Harmonic
find_package(gz-transport NAMES gz-transport14 gz-transport13)

if(gz-transport_FOUND)

	add_compile_options(-frtti -fexceptions)

	set(GZ_TRANSPORT_VER ${gz-transport_VERSION_MAJOR})

	if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
		set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
	else()
		set(GZ_TRANSPORT_LIB ignition-transport${GZ_TRANSPORT_VER}::core)
	endif()

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
		DEPENDS
			mixer_module
			px4_work_queue
			${GZ_TRANSPORT_LIB}
		MODULE_CONFIG
			module.yaml
	)

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
					DEPENDS px4
				)
			else()
				add_custom_target(gz_${model_only}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_only} PX4_GZ_WORLD=${world_name} $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4
				)
			endif()
		endforeach()
	endforeach()
	# PX4_GZ_MODELS, PX4_GZ_WORLDS, GZ_SIM_RESOURCE_PATH
	configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)

endif()
