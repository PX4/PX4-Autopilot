/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Integrator.hpp"

#include <sensor_corrections/SensorCorrections.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMU() = delete;
	VehicleIMU(uint8_t accel_index = 0, uint8_t gyro_index = 0);

	~VehicleIMU() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void ParametersUpdate(bool force = false);
	void Run() override;

	struct IntervalAverage {
		hrt_abstime timestamp_sample_last{0};
		float interval_sum{0.f};
		float interval_count{0.f};
		float update_interval{0.f};
	};

	bool UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample);
	void UpdateIntegratorConfiguration();
	void UpdateGyroVibrationMetrics(const matrix::Vector3f &delta_angle);
	void UpdateAccelVibrationMetrics(const matrix::Vector3f &delta_velocity);

	uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::PublicationMulti<vehicle_imu_status_s> _vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub;
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub;

	SensorCorrections _accel_corrections;
	SensorCorrections _gyro_corrections;

	Integrator _accel_integrator{}; // 200 Hz default
	Integrator _gyro_integrator{true};   // 200 Hz default, coning compensation enabled

	hrt_abstime _last_timestamp_sample_accel{0};
	hrt_abstime _last_timestamp_sample_gyro{0};

	IntervalAverage _accel_interval{};
	IntervalAverage _gyro_interval{};

	uint32_t _accel_error_count{0};
	uint32_t _gyro_error_count{0};

	matrix::Vector3f _delta_angle_prev{0.f, 0.f, 0.f};	// delta angle from the previous IMU measurement
	matrix::Vector3f _delta_velocity_prev{0.f, 0.f, 0.f};	// delta velocity from the previous IMU measurement
	float _accel_vibration_metric{0.f};	// high frequency vibration level in the IMU delta velocity data (m/s)
	float _gyro_vibration_metric{0.f};	// high frequency vibration level in the IMU delta angle data (rad)
	float _gyro_coning_vibration{0.f};	// Level of coning vibration in the IMU delta angles (rad^2)

	uint8_t _delta_velocity_clipping{0};
	uint32_t _delta_velocity_clipping_total[3] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate
	)
};

} // namespace sensors
