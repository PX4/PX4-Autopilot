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
		#-DDEBUG_BUILD
	INCLUDES
		EKF
	STACK_MAX
		3600
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
		EKF/gps_control.cpp
		EKF/gps_fusion.cpp
		EKF/gps_yaw_fusion.cpp
		EKF/height_fusion.cpp
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
		hysteresis
		perf
		EKF2Utility
		px4_work_queue
		world_magnetic_model
	UNITY_BUILD
	PUBLICATIONS
		"px4::msg::Ekf2Timestamps /ekf2_timestamps"
		"px4::msg::EkfGpsDrift /ekf_gps_drift"
		"px4::msg::EstimatorBaroBias /estimator_baro_bias"
		"px4::msg::EstimatorInnovations /estimator_innovations"
		"px4::msg::EstimatorInnovations /estimator_innovation_test_ratios"
		"px4::msg::EstimatorInnovations /estimator_innovation_variances"
		"px4::msg::EstimatorOpticalFlowVel /estimator_optical_flow_vel"
		"px4::msg::EstimatorSensorBias /estimator_sensor_bias"
		"px4::msg::EstimatorStates /estimator_states"
		"px4::msg::EstimatorStatus /estimator_status"
		"px4::msg::EstimatorStatusFlags /estimator_status_flags"
		"px4::msg::EstimatorEventFlags /estimator_event_flags"
		"px4::msg::VehicleOdometry /estimator_visual_odometry_aligned"
		"px4::msg::YawEstimatorStatus /yaw_estimator_status"

		"px4::msg::VehicleAttitude /vehicle_attitude"
		"px4::msg::VehicleLocalPosition /vehicle_local_position"
		"px4::msg::VehicleGlobalPosition /vehicle_global_position"
		"px4::msg::VehicleOdometry /vehicle_odometry"
		"px4::msg::Wind /wind"

		# multi-EKF
		"px4::msg::EstimatorSelectorStatus /estimator_selector_status"
		"px4::msg::VehicleAttitude /estimator_attitude"
		"px4::msg::VehicleLocalPosition /estimator_local_position"
		"px4::msg::VehicleGlobalPosition /estimator_global_position"
		"px4::msg::VehicleOdometry /estimator_odometry"
		"px4::msg::Wind /estimator_wind"
	SUBSCRIPTIONS
		"px4::msg::OpticalFlow /optical_flow"
	)

if(BUILD_TESTING)
	add_subdirectory(EKF)
	add_subdirectory(test)
endif()
