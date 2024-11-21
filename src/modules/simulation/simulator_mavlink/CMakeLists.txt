############################################################################
#
#   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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

px4_add_module(
	MODULE modules__simulation__simulator_mavlink
	MAIN simulator_mavlink
	COMPILE_FLAGS
		-Wno-double-promotion
		-Wno-cast-align
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
	INCLUDES
		${CMAKE_BINARY_DIR}/mavlink
		${CMAKE_BINARY_DIR}/mavlink/${CONFIG_MAVLINK_DIALECT}
	SRCS
		SimulatorMavlink.cpp
		SimulatorMavlink.hpp
	DEPENDS
		mavlink_c_generate
		conversion
		geo
		drivers_accelerometer
		drivers_gyroscope
		drivers_magnetometer
	)

include(sitl_targets_flightgear.cmake)
include(sitl_targets_gazebo-classic.cmake)
include(sitl_targets_jmavsim.cmake)
include(sitl_targets_jsbsim.cmake)


# none_iris (legacy compatibility launch helper)
add_custom_target(none_iris
	COMMAND ${CMAKE_COMMAND} -E env PX4_SYS_AUTOSTART=10016 $<TARGET_FILE:px4>
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	DEPENDS
		px4
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris
	COMMENT "launching px4 none_iris (SYS_AUTOSTART=10016)"
)
