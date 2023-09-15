/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "data_validator/DataValidatorGroup.hpp"
#include "RingBuffer.hpp"

#include <Integrator.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/vehicle_optical_flow_vel.h>

namespace sensors
{

class VehicleOpticalFlow : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleOpticalFlow();
	~VehicleOpticalFlow() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void ClearAccumulatedData();
	void UpdateDistanceSensor();
	void UpdateSensorGyro();

	void Run() override;

	void ParametersUpdate();
	void SensorCorrectionsUpdate(bool force = false);

	static constexpr int MAX_SENSOR_COUNT = 3;

	uORB::Publication<vehicle_optical_flow_s> _vehicle_optical_flow_pub{ORB_ID(vehicle_optical_flow)};
	uORB::Publication<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_pub{ORB_ID(vehicle_optical_flow_vel)};

	uORB::Subscription _params_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	uORB::SubscriptionCallbackWorkItem _sensor_flow_sub{this, ORB_ID(sensor_optical_flow)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};

	sensors::IntegratorConing _gyro_integrator{};

	hrt_abstime _gyro_timestamp_sample_last{0};

	calibration::Gyroscope _gyro_calibration{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	matrix::Dcmf _flow_rotation{matrix::eye<float, 3>()};

	hrt_abstime _flow_timestamp_sample_last{0};
	matrix::Vector2f _flow_integral{};
	matrix::Vector3f _delta_angle{};
	uint32_t _integration_timespan_us{};
	float _distance_sum{NAN};
	uint8_t _distance_sum_count{0};
	uint16_t _quality_sum{0};
	uint8_t _accumulated_count{0};

	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
	hrt_abstime _last_range_sensor_update{0};

	bool _delta_angle_available{false};

	struct gyroSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		matrix::Vector3f data{};
		float dt{0.f};
	};

	struct rangeSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		float data{};
	};

	RingBuffer<gyroSample, 32> _gyro_buffer{};
	RingBuffer<rangeSample, 5> _range_buffer{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_FLOW_ROT>) _param_sens_flow_rot,
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXR>) _param_sens_flow_maxr,
		(ParamFloat<px4::params::SENS_FLOW_RATE>) _param_sens_flow_rate
	)
};
}; // namespace sensors
