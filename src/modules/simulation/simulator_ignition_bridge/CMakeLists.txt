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

add_compile_options(-frtti -fexceptions)

# Find the Ignition_Transport library
find_package(ignition-transport
	REQUIRED COMPONENTS core
	NAMES
		ignition-transport8
		ignition-transport10
		ignition-transport11
	#QUIET
)
set(IGN_TRANSPORT_VER ${ignition-transport_VERSION_MAJOR})

px4_add_module(
	MODULE modules__simulation__ignition_bridge
	MAIN simulator_ignition_bridge
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		SimulatorIgnitionBridge.cpp
		SimulatorIgnitionBridge.hpp
	DEPENDS
		mixer_module
		px4_work_queue
		ignition-transport${IGN_TRANSPORT_VER}::core
	MODULE_CONFIG
		module.yaml
)

file(GLOB ign_models
	LIST_DIRECTORIES true
	RELATIVE ${PX4_SOURCE_DIR}/Tools/simulation/ignition/models
	CONFIGURE_DEPENDS
	${PX4_SOURCE_DIR}/Tools/simulation/ignition/models/*
)

file(GLOB ign_worlds
	CONFIGURE_DEPENDS
	${PX4_SOURCE_DIR}/Tools/simulation/ignition/worlds/*.sdf
)

foreach(model ${ign_models})
	foreach(world ${ign_worlds})

		get_filename_component("world_name" ${world} NAME_WE)

		if(world_name MATCHES "default")
			add_custom_target(ign_${model}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=${model} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4
			)
		else()
			add_custom_target(ign_${model}_${world_name}
				COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=${model} PX4_SIM_WORLD=${world_name} $<TARGET_FILE:px4>
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4
			)
		endif()
	endforeach()
endforeach()


# TODO: PX4_IGN_MODELS_PATH
# PX4_IGN_WORLDS_PATH
configure_file(gazebo_env.sh.in ${PX4_BINARY_DIR}/rootfs/gazebo_env.sh)
