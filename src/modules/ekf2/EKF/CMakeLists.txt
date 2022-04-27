############################################################################
#
#   Copyright (c) 2015-2018 ECL Development Team. All rights reserved.
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

add_library(ecl_EKF
	airspeed_fusion.cpp
	baro_bias_estimator.cpp
	control.cpp
	covariance.cpp
	drag_fusion.cpp
	ekf.cpp
	ekf_helper.cpp
	EKFGSF_yaw.cpp
	estimator_interface.cpp
	fake_pos_control.cpp
	gps_checks.cpp
	gps_control.cpp
	gps_fusion.cpp
	gps_yaw_fusion.cpp
	height_fusion.cpp
	imu_down_sampler.cpp
	mag_control.cpp
	mag_fusion.cpp
	optflow_fusion.cpp
	range_finder_consistency_check.cpp
	sensor_range_finder.cpp
	sideslip_fusion.cpp
	terrain_estimator.cpp
	utils.cpp
	vel_pos_fusion.cpp
	zero_velocity_update.cpp
)

add_dependencies(ecl_EKF prebuild_targets)
target_link_libraries(ecl_EKF PRIVATE geo world_magnetic_model)
target_compile_options(ecl_EKF PRIVATE -fno-associative-math)
