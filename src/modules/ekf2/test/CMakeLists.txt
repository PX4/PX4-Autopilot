############################################################################
#
#   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
add_subdirectory(sensor_simulator)
add_subdirectory(test_helper)

px4_add_unit_gtest(SRC test_EKF_accelerometer.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_airspeed_fusion_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_airspeed.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_basics.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_covariance_prediction_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_externalVision.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_flow.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_opt_flow_fusion_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_fusionLogic.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_gps.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_gps_yaw.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_gnss_yaw_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_height_fusion.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_imuSampling.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_initialization.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_mag.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_mag_3d_fusion_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_mag_declination_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_measurementSampling.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_ringbuffer.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_sideslip_fusion_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_terrain_estimator.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_utils.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_withReplayData.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_yaw_estimator.cpp LINKLIBS ecl_EKF ecl_sensor_sim ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_yaw_estimator_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_EKF_yaw_fusion_generated.cpp LINKLIBS ecl_EKF ecl_test_helper)
px4_add_unit_gtest(SRC test_SensorRangeFinder.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
px4_add_unit_gtest(SRC test_EKF_drag_fusion.cpp LINKLIBS ecl_EKF ecl_sensor_sim)
