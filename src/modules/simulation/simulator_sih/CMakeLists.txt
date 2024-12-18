############################################################################
#
#   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
	MODULE modules__simulation__simulator_sih
	MAIN simulator_sih
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		aero.hpp
		sih.cpp
		sih.hpp
	DEPENDS
		mathlib
		drivers_accelerometer
		drivers_gyroscope
		geo
	)

if(PX4_PLATFORM MATCHES "posix")
	# create targets for sihsim
	set(models
		airplane
		quadx
		xvert
	)

	# find corresponding airframes
	file(GLOB sihsim_airframes
		RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_sihsim_*
	)

	# remove any .post files
	foreach(sihsim_airframe IN LISTS sihsim_airframes)
		if(sihsim_airframe MATCHES ".post")
			list(REMOVE_ITEM sihsim_airframes ${sihsim_airframe})
		endif()
	endforeach()
	list(REMOVE_DUPLICATES sihsim_airframes)

	foreach(model ${models})

		# match model to airframe
		set(airframe_model_only)
		set(airframe_sys_autostart)
		set(sihsim_airframe_found)
		foreach(sihsim_airframe IN LISTS sihsim_airframes)

			string(REGEX REPLACE ".*_sihsim_" "" airframe_model_only ${sihsim_airframe})
			string(REGEX REPLACE "_sihsim_.*" "" airframe_sys_autostart ${sihsim_airframe})

			if(model STREQUAL ${airframe_model_only})
				set(sihsim_airframe_found ${sihsim_airframe})
				break()
			endif()
		endforeach()

		if(sihsim_airframe_found)
			#message(STATUS "sihsim model: ${model} (${airframe_model_only}), airframe: ${sihsim_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		else()
			message(WARNING "sihsim missing model: ${model} (${airframe_model_only}), airframe: ${sihsim_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		endif()

		add_custom_target(sihsim_${model}
			COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=sihsim_${model} PX4_SIMULATOR=sihsim $<TARGET_FILE:px4>
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS px4
		)
	endforeach()

endif()
