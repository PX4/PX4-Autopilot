############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
# First look for GZ Harmonic libraries
find_package(gz-transport NAMES gz-transport13)

# If Harmonic not found, look for GZ Garden libraries
if(NOT gz-transport_FOUND)
	find_package(gz-transport NAMES gz-transport12)
endif()

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
		DEPENDS
			mixer_module
			px4_work_queue
			${GZ_TRANSPORT_LIB}
		MODULE_CONFIG
			module.yaml
	)

	file(GLOB gz_models
		LIST_DIRECTORIES true
		RELATIVE ${PX4_SOURCE_DIR}/Tools/simulation/gz/models
		${PX4_SOURCE_DIR}/Tools/simulation/gz/models/*
	)

	file(GLOB gz_worlds
		${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds/*.sdf
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

		if(EXISTS "${PX4_SOURCE_DIR}/Tools/simulation/gz/models/${model_only}")

			if((EXISTS "${PX4_SOURCE_DIR}/Tools/simulation/gz/models/${model_only}/model.sdf"))
				#message(STATUS "Ignition SDF file found for ${model_only}")
			else()
				message(WARNING "Ignition no SDF file found for ${model_only}")
			endif()

		else()
			message(WARNING "model directory ${PX4_SOURCE_DIR}/Tools/simulation/gz/models/${model_only} not found")
		endif()
	endforeach()

	foreach(model ${gz_models})

		# match model to airframe
		set(airframe_model_only)
		set(airframe_sys_autostart)
		set(gz_airframe_found)
		foreach(gz_airframe IN LISTS gz_airframes)

			string(REGEX REPLACE ".*_gz_" "" airframe_model_only ${gz_airframe})
			string(REGEX REPLACE "_gz_.*" "" airframe_sys_autostart ${gz_airframe})

			if(model STREQUAL ${airframe_model_only})
				set(gz_airframe_found ${gz_airframe})
				break()
			endif()
		endforeach()

		if(gz_airframe_found)
			#message(STATUS "gz model: ${model} (${airframe_model_only}), airframe: ${gz_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		else()
			message(WARNING "gz missing model: ${model} (${airframe_model_only}), airframe: ${gz_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		endif()

		foreach(world ${gz_worlds})

			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name STREQUAL "default")
				add_custom_target(gz_${model}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model} $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4
				)
			else()
				add_custom_target(gz_${model}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model} PX4_GZ_WORLD=${world_name} $<TARGET_FILE:px4>
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
