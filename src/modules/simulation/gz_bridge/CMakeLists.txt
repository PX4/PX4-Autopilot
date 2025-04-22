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

if(NOT DEFINED ENV{GZ_DISTRO} OR NOT "$ENV{GZ_DISTRO}" STREQUAL "harmonic")
    find_package(gz-transport NAMES gz-transport14 gz-transport13)
else()
    find_package(gz-transport NAMES gz-transport13)
endif()

file(GLOB gz_worlds ${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds/*.sdf)
file(GLOB gz_airframes ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*)

if (gz-transport_FOUND)
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
			gz-transport${gz-transport_VERSION_MAJOR}::core
		MODULE_CONFIG
			module.yaml
	)

    target_include_directories(modules__simulation__gz_bridge
        PUBLIC
            ${PX4_GZ_MSGS_BINARY_DIR}
    )

	target_include_directories(modules__simulation__gz_bridge PUBLIC px4_gz_msgs)
	target_link_libraries(modules__simulation__gz_bridge PUBLIC px4_gz_msgs)

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
	)

	# Below we setup the build targets for our worlds and models
	# Syntax: gz_<model_name>_<world_name>
	# Example: gz_x500_flow_forest
	foreach(gz_airframe IN LISTS gz_airframes)
		set(model_name)
		string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})

		foreach(world ${gz_worlds})
			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name STREQUAL "default")
				add_custom_target(gz_${model_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} GZ_IP=127.0.0.1 $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 px4_gz_plugins
				)
			else()
				add_custom_target(gz_${model_name}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} PX4_GZ_WORLD=${world_name} GZ_IP=127.0.0.1 $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 px4_gz_plugins
				)
			endif()
		endforeach()
	endforeach()

	# Setup the environment variables: PX4_GZ_MODELS, PX4_GZ_WORLDS, GZ_SIM_RESOURCE_PATH
	configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)

else()
	# Create fallback targets that provide helpful error messages when Gazebo dependencies are missing
	foreach(gz_airframe IN LISTS gz_airframes)
		set(model_name)
		string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})

		foreach(world ${gz_worlds})
			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name STREQUAL "default")
				add_custom_target(gz_${model_name}
					COMMAND ${CMAKE_COMMAND} -E echo "ERROR: Gazebo simulation dependencies not found!"
					COMMAND ${CMAKE_COMMAND} -E echo "  - For installation instructions, see: https://gazebosim.org/docs/harmonic/install_ubuntu/"
					COMMAND ${CMAKE_COMMAND} -E false
					VERBATIM
				)
			else()
				add_custom_target(gz_${model_name}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E echo "ERROR: Gazebo simulation dependencies not found!"
					COMMAND ${CMAKE_COMMAND} -E echo "  - For installation instructions, see: https://gazebosim.org/docs/harmonic/install_ubuntu/"
					COMMAND ${CMAKE_COMMAND} -E false
					VERBATIM
				)
			endif()
		endforeach()
	endforeach()

	message(STATUS "Gazebo simulation bridge module disabled: missing dependencies")
endif()
