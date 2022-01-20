/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_imu_fifo.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;

namespace sensors
{

class VehicleAcceleration : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleAcceleration();
	~VehicleAcceleration() override;

	void PrintStatus();
	bool Start();
	void Stop();

private:
	void Run() override;

	bool CalibrateAndPublish(const hrt_abstime &timestamp_sample, const matrix::Vector3f &acceleration_uncalibrated);

	inline float FilterAcceleration(int axis, float data[], int N = 1);

	void ParametersUpdate(bool force = false);

	void ResetFilters(const hrt_abstime &time_now_us);
	void SensorBiasUpdate(bool force = false);
	bool SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force = false);
	bool UpdateSampleRate();

	// scaled appropriately for current sensor
	matrix::Vector3f GetResetAcceleration() const;

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<vehicle_acceleration_s> _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::SubscriptionCallbackWorkItem _sensor_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_fifo_sub{this, ORB_ID(sensor_imu_fifo)};

	calibration::Accelerometer _calibration{};

	matrix::Vector3f _bias{};

	matrix::Vector3f _acceleration{};

	hrt_abstime _publish_interval_min_us{0};
	hrt_abstime _last_publish{0};

	float _filter_sample_rate_hz{NAN};

	math::LowPassFilter2p<float> _lp_filter[3] {};

	uint32_t _selected_sensor_device_id{0};

	bool _reset_filters{true};
	bool _fifo_available{false};
	bool _update_sample_rate{true};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": accel filter")};
	perf_counter_t _filter_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel filter reset")};
	perf_counter_t _selection_changed_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel selection changed")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate
	)
};

} // namespace sensors
