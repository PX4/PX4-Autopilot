/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <lib/conversion/rotation.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_acceleration.h>

class VehicleAcceleration : public ModuleParams, public px4::WorkItem
{
public:

	VehicleAcceleration();
	~VehicleAcceleration() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void CheckFilters();
	void ParametersUpdate(bool force = false);
	void SensorBiasUpdate(bool force = false);
	void SensorCorrectionsUpdate(bool force = false);
	bool SensorSelectionUpdate(bool force = false);

	static constexpr int MAX_SENSOR_COUNT = 3;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,

		(ParamInt<px4::params::SENS_BOARD_ROT>) _param_sens_board_rot,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _param_sens_board_x_off,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _param_sens_board_y_off,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _param_sens_board_z_off
	)

	uORB::Publication<vehicle_acceleration_s> _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};

	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};
	uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(sensor_accel), 0},
		{this, ORB_ID(sensor_accel), 1},
		{this, ORB_ID(sensor_accel), 2}
	};

	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	matrix::Dcmf _board_rotation;

	matrix::Vector3f _bias{0.f, 0.f, 0.f};
	matrix::Vector3f _offset{0.f, 0.f, 0.f};
	matrix::Vector3f _scale{1.f, 1.f, 1.f};

	matrix::Vector3f _acceleration_prev{0.f, 0.f, 0.f};

	hrt_abstime _last_publish{0};
	hrt_abstime _filter_check_last{0};
	static constexpr const float kInitialRateHz{1000.0f}; /**< sensor update rate used for initialization */
	float _update_rate_hz{kInitialRateHz}; /**< current rate-controller loop update rate in [Hz] */

	math::LowPassFilter2pVector3f _lp_filter{kInitialRateHz, 30.0f};

	float _filter_sample_rate{kInitialRateHz};
	int _sample_rate_incorrect_count{0};

	uint32_t _selected_sensor_device_id{0};
	uint8_t _selected_sensor_sub_index{0};
	int8_t _corrections_selected_instance{-1};
};
