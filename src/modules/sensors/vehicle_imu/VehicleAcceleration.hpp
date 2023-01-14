/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "IMU.hpp"

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;

namespace sensors
{

class VehicleAcceleration : public ModuleParams
{
public:
	VehicleAcceleration();
	~VehicleAcceleration() override = default;

	void ParametersUpdate();

	void PrintStatus();

	void updateAccel(IMU &imu, const matrix::Vector3f &accel_raw, const float sample_rate_hz);

private:
	uORB::Publication<vehicle_acceleration_s> _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};

	matrix::Vector3f _acceleration_prev{};

	uint32_t _selected_sensor_device_id{0};

	static constexpr float kDefaultAccelSampleFreqHz{800.f};
	static constexpr float kDefaultAccelCutoffFreqHz{30.f};
	math::LowPassFilter2p<matrix::Vector3f> _lp_filter{kDefaultAccelSampleFreqHz, kDefaultAccelCutoffFreqHz};
	float _filter_sample_rate{kDefaultAccelSampleFreqHz};

	bool _filter_config_check{true};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate
	)
};

} // namespace sensors
