############################################################################
#
#   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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

include(gtest.cmake)

add_subdirectory(sensor_simulator)
add_subdirectory(test_helper)

set(SRCS
	main.cpp
	test_EKF_basics.cpp
	test_EKF_ringbuffer.cpp
	test_EKF_measurementSampling.cpp
	test_EKF_imuSampling.cpp
	test_EKF_fusionLogic.cpp
	test_EKF_initialization.cpp
	test_EKF_gps_yaw.cpp
	test_EKF_gps.cpp
	test_EKF_externalVision.cpp
	test_EKF_airspeed.cpp
	test_EKF_withReplayData.cpp
	test_EKF_flow.cpp
	test_EKF_terrain_estimator.cpp
	test_SensorRangeFinder.cpp
	test_geo.cpp
	test_EKF_utils.cpp
   )
add_executable(ECL_GTESTS ${SRCS})

target_link_libraries(ECL_GTESTS gtest_main ecl_EKF ecl_sensor_sim ecl_test_helper)

add_test(NAME ECL_GTESTS COMMAND ECL_GTESTS)
