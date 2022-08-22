############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#add_compile_options(-fexceptions)

# Find the Ignition_Transport library
find_package(ignition-transport
	REQUIRED COMPONENTS core
	NAMES
		ignition-transport8
		ignition-transport10
		ignition-transport11

	#QUIET
)

message(STATUS "PACKAGE_FIND_NAME: ${ignition-transport_NAME}")
message(STATUS "PACKAGE_FIND_VERSION_MAJOR: ${ignition-transport_VERSION_MAJOR}")
message(STATUS "PACKAGE_FIND_VERSION_MINOR: ${ignition-transport_VERSION_MINOR}")
message(STATUS "PACKAGE_FIND_VERSION_COUNT: ${ignition-transport_VERSION_COUNT}")

set(IGN_TRANSPORT_VER ${ignition-transport_VERSION_MAJOR})

px4_add_module(
	MODULE modules__ignition_simulator
	MAIN ignition_simulator
	COMPILE_FLAGS
		#${MAX_CUSTOM_OPT_LEVEL}
		-O0
	SRCS
		IgnitionSimulator.cpp
		IgnitionSimulator.hpp
	DEPENDS
		drivers_accelerometer
		drivers_gyroscope
		mixer_module
		px4_work_queue
	MODULE_CONFIG
		module.yaml
)

target_link_libraries(modules__ignition_simulator PRIVATE ignition-transport${IGN_TRANSPORT_VER}::core)
