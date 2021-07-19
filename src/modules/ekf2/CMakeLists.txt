############################################################################
#
#   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
#############################################################################

add_subdirectory(Utility)

px4_add_module(
	MODULE modules__ekf2
	MAIN ekf2
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
	INCLUDES
		EKF
	STACK_MAX
		3500
	SRCS
		EKF/airspeed_fusion.cpp
		EKF/baro_bias_estimator.cpp
		EKF/control.cpp
		EKF/covariance.cpp
		EKF/drag_fusion.cpp
		EKF/ekf.cpp
		EKF/ekf_helper.cpp
		EKF/EKFGSF_yaw.cpp
		EKF/estimator_interface.cpp
		EKF/gps_checks.cpp
		EKF/gps_yaw_fusion.cpp
		EKF/imu_down_sampler.cpp
		EKF/mag_control.cpp
		EKF/mag_fusion.cpp
		EKF/optflow_fusion.cpp
		EKF/sensor_range_finder.cpp
		EKF/sideslip_fusion.cpp
		EKF/terrain_estimator.cpp
		EKF/utils.cpp
		EKF/vel_pos_fusion.cpp

		EKF2.cpp
		EKF2.hpp
		EKF2Selector.cpp
		EKF2Selector.hpp

	DEPENDS
		geo
		perf
		EKF2Utility
		px4_work_queue
		world_magnetic_model
	UNITY_BUILD
	)

if(BUILD_TESTING)
	add_subdirectory(EKF)
	add_subdirectory(test)
endif()
