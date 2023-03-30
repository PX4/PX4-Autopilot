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
#############################################################################

add_subdirectory(Utility)

# Symforce code generation TODO:fixme
# execute_process(
#     COMMAND ${PYTHON_EXECUTABLE} -m symforce.symbolic
#     RESULT_VARIABLE PYTHON_SYMFORCE_EXIT_CODE
#     OUTPUT_QUIET
# )
# if(${PYTHON_SYMFORCE_EXIT_CODE} EQUAL 0)

# 	set(EKF_DERIVATION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EKF/python/ekf_derivation)

# 	set(EKF_GENERATED_SRC_FILES
# 		${EKF_DERIVATION_DIR}/generated/compute_airspeed_h_and_k.h
# 		${EKF_DERIVATION_DIR}/generated/compute_airspeed_innov_and_innov_var.h
# 		${EKF_DERIVATION_DIR}/generated/compute_sideslip_h_and_k.h
# 		${EKF_DERIVATION_DIR}/generated/compute_sideslip_innov_and_innov_var.h
# 		${EKF_DERIVATION_DIR}/generated/covariance_prediction.h
# 	)

# 	add_custom_command(
# 		OUTPUT ${EKF_GENERATED_SRC_FILES}
# 		COMMAND ${PYTHON_EXECUTABLE} derivation.py
# 		DEPENDS
# 			${EKF_DERIVATION_DIR}/derivation.py
# 			${EKF_DERIVATION_DIR}/derivation_utils.py

# 		WORKING_DIRECTORY ${EKF_DERIVATION_DIR}
# 		COMMENT "Symforce code generation"
# 		USES_TERMINAL
# 	)

# 	add_custom_target(ekf2_symforce_generate DEPENDS ${EKF_GENERATED_SRC_FILES})
# endif()

set(EKF_SRCS)
list(APPEND EKF_SRCS
	EKF/baro_height_control.cpp
	EKF/bias_estimator.cpp
	EKF/control.cpp
	EKF/covariance.cpp
	EKF/ekf.cpp
	EKF/ekf_helper.cpp
	EKF/EKFGSF_yaw.cpp
	EKF/estimator_interface.cpp
	EKF/fake_height_control.cpp
	EKF/fake_pos_control.cpp
	EKF/gnss_height_control.cpp
	EKF/gps_checks.cpp
	EKF/gps_control.cpp
	EKF/gravity_fusion.cpp
	EKF/height_control.cpp
	EKF/imu_down_sampler.cpp
	EKF/mag_control.cpp
	EKF/mag_fusion.cpp
	EKF/output_predictor.cpp
	EKF/vel_pos_fusion.cpp
	EKF/zero_innovation_heading_update.cpp
	EKF/zero_velocity_update.cpp
)

if(CONFIG_EKF2_AIRSPEED)
	list(APPEND EKF_SRCS EKF/airspeed_fusion.cpp)
endif()

if(CONFIG_EKF2_AUXVEL)
	list(APPEND EKF_SRCS EKF/auxvel_fusion.cpp)
endif()

if(CONFIG_EKF2_DRAG_FUSION)
	list(APPEND EKF_SRCS EKF/drag_fusion.cpp)
endif()

if(CONFIG_EKF2_EXTERNAL_VISION)
	list(APPEND EKF_SRCS
		EKF/ev_control.cpp
		EKF/ev_height_control.cpp
		EKF/ev_pos_control.cpp
		EKF/ev_vel_control.cpp
		EKF/ev_yaw_control.cpp
	)
endif()

if(CONFIG_EKF2_GNSS_YAW)
	list(APPEND EKF_SRCS EKF/gps_yaw_fusion.cpp)
endif()

if(CONFIG_EKF2_OPTICAL_FLOW)
	list(APPEND EKF_SRCS
		EKF/optical_flow_control.cpp
		EKF/optflow_fusion.cpp
	)
endif()

if(CONFIG_EKF2_RANGE_FINDER)
	list(APPEND EKF_SRCS
		EKF/range_finder_consistency_check.cpp
		EKF/range_height_control.cpp
		EKF/sensor_range_finder.cpp
		EKF/terrain_estimator.cpp
	)
endif()

if(CONFIG_EKF2_SIDESLIP)
	list(APPEND EKF_SRCS EKF/sideslip_fusion.cpp)
endif()

px4_add_module(
	MODULE modules__ekf2
	MAIN ekf2
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
		#-DDEBUG_BUILD
		#-O0
	INCLUDES
		EKF
	PRIORITY
		"SCHED_PRIORITY_MAX - 18" # max priority below high priority WQ threads
	STACK_MAX
		3600
	SRCS
		${EKF_SRCS}

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
	)

if(BUILD_TESTING)
	add_subdirectory(EKF)
	add_subdirectory(test)
endif()
