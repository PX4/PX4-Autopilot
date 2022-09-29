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

# Find the Ignition_Transport library
find_package(ignition-transport
	#REQUIRED COMPONENTS core
	NAMES
		ignition-transport8
		ignition-transport10
		ignition-transport11
	#QUIET
)

if(ignition-transport_FOUND)

	add_compile_options(-frtti -fexceptions)

	set(IGN_TRANSPORT_VER ${ignition-transport_VERSION_MAJOR})

	px4_add_module(
		MODULE modules__simulation__gz_bridge
		MAIN gz_bridge
		COMPILE_FLAGS
			${MAX_CUSTOM_OPT_LEVEL}
		SRCS
			GZBridge.cpp
			GZBridge.hpp
		DEPENDS
			mixer_module
			px4_work_queue
			ignition-transport${IGN_TRANSPORT_VER}::core
		MODULE_CONFIG
			module.yaml
	)

	file(GLOB gz_models
		LIST_DIRECTORIES true
		RELATIVE ${PX4_SOURCE_DIR}/Tools/simulation/gz/models
		CONFIGURE_DEPENDS
		${PX4_SOURCE_DIR}/Tools/simulation/gz/models/*
	)

	file(GLOB gz_worlds
		CONFIGURE_DEPENDS
		${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds/*.sdf
	)

	foreach(model ${gz_models})
		foreach(world ${gz_worlds})

			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name MATCHES "default")
				add_custom_target(gz_${model}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=${model} $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4
				)
			else()
				add_custom_target(gz_${model}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_GZ_MODEL=${model} PX4_GZ_WORLD=${world_name} $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4
				)
			endif()
		endforeach()
	endforeach()


	# TODO: PX4_GZ_MODELS_PATH
	# PX4_GZ_WORLDS_PATH
	configure_file(gazebo_env.sh.in ${PX4_BINARY_DIR}/rootfs/gazebo_env.sh)

endif()
