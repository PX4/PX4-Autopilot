############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

set(EKF_SRCS)
list(APPEND EKF_SRCS
	bias_estimator.cpp
	control.cpp
	covariance.cpp
	ekf.cpp
	ekf_helper.cpp
	estimator_interface.cpp
	fake_height_control.cpp
	fake_pos_control.cpp
	height_control.cpp
	imu_down_sampler.cpp
	output_predictor.cpp
	velocity_fusion.cpp
	position_fusion.cpp
	yaw_fusion.cpp
	zero_innovation_heading_update.cpp

	aid_sources/ZeroGyroUpdate.cpp
	aid_sources/ZeroVelocityUpdate.cpp
)

if(CONFIG_EKF2_AIRSPEED)
	list(APPEND EKF_SRCS airspeed_fusion.cpp)
endif()

if(CONFIG_EKF2_AUX_GLOBAL_POSITION)
	list(APPEND EKF_SRCS aux_global_position.cpp)
endif()

if(CONFIG_EKF2_AUXVEL)
	list(APPEND EKF_SRCS auxvel_fusion.cpp)
endif()

if(CONFIG_EKF2_BAROMETER)
	list(APPEND EKF_SRCS
		baro_height_control.cpp
	)
endif()

if(CONFIG_EKF2_DRAG_FUSION)
	list(APPEND EKF_SRCS drag_fusion.cpp)
endif()

if(CONFIG_EKF2_EXTERNAL_VISION)
	list(APPEND EKF_SRCS
		ev_control.cpp
		ev_height_control.cpp
		ev_pos_control.cpp
		ev_vel_control.cpp
		ev_yaw_control.cpp
	)
endif()

if(CONFIG_EKF2_GNSS)
	list(APPEND EKF_SRCS
		gnss_height_control.cpp
		gps_checks.cpp
		gps_control.cpp
		EKFGSF_yaw.cpp
	)
endif()

if(CONFIG_EKF2_GNSS_YAW)
	list(APPEND EKF_SRCS gps_yaw_fusion.cpp)
endif()

if(CONFIG_EKF2_GRAVITY_FUSION)
	list(APPEND EKF_SRCS gravity_fusion.cpp)
endif()

if(CONFIG_EKF2_MAGNETOMETER)
	list(APPEND EKF_SRCS
		mag_3d_control.cpp
		mag_control.cpp
		mag_fusion.cpp
		mag_heading_control.cpp
	)
endif()

if(CONFIG_EKF2_OPTICAL_FLOW)
	list(APPEND EKF_SRCS
		optical_flow_control.cpp
		optflow_fusion.cpp
	)
endif()

if(CONFIG_EKF2_RANGE_FINDER)
	list(APPEND EKF_SRCS
		range_finder_consistency_check.cpp
		range_height_control.cpp
		sensor_range_finder.cpp
	)
endif()

if(CONFIG_EKF2_SIDESLIP)
	list(APPEND EKF_SRCS sideslip_fusion.cpp)
endif()

if(CONFIG_EKF2_TERRAIN)
	list(APPEND EKF_SRCS terrain_estimator.cpp)
endif()

add_library(ecl_EKF
	${EKF_SRCS}
)

add_dependencies(ecl_EKF prebuild_targets)
target_include_directories(ecl_EKF PUBLIC ${EKF_GENERATED_DERIVATION_INCLUDE_PATH})
target_link_libraries(ecl_EKF PRIVATE geo world_magnetic_model)
target_compile_options(ecl_EKF PRIVATE -fno-associative-math)
