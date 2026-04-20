############################################################################
#
#   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
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
	MODULE modules__temperature_compensation
	MAIN temperature_compensation
	SRCS
		TemperatureCompensationModule.cpp
		TemperatureCompensation.cpp
		temperature_calibration/accel.cpp
		temperature_calibration/baro.cpp
		temperature_calibration/gyro.cpp
		temperature_calibration/mag.cpp
		temperature_calibration/task.cpp
	MODULE_CONFIG
		temp_comp_params_accel.yaml
		temp_comp_params_accel_0.yaml
		temp_comp_params_accel_1.yaml
		temp_comp_params_accel_2.yaml
		temp_comp_params_accel_3.yaml
		temp_comp_params_baro.yaml
		temp_comp_params_baro_0.yaml
		temp_comp_params_baro_1.yaml
		temp_comp_params_baro_2.yaml
		temp_comp_params_baro_3.yaml
		temp_comp_params_gyro.yaml
		temp_comp_params_gyro_0.yaml
		temp_comp_params_gyro_1.yaml
		temp_comp_params_gyro_2.yaml
		temp_comp_params_gyro_3.yaml
		temp_comp_params_mag.yaml
		temp_comp_params_mag_0.yaml
		temp_comp_params_mag_1.yaml
		temp_comp_params_mag_2.yaml
		temp_comp_params_mag_3.yaml
	DEPENDS
		mathlib
	)
