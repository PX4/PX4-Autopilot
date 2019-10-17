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

#include "VehicleAngularAcceleration.hpp"

#include <px4_log.h>

using namespace matrix;
using namespace time_literals;

VehicleAngularAcceleration::VehicleAngularAcceleration() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_cycle_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_acceleration: cycle time")),
	_sensor_latency_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_acceleration: sensor latency"))
{
}

VehicleAngularAcceleration::~VehicleAngularAcceleration()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_sensor_latency_perf);
}

bool
VehicleAngularAcceleration::Start()
{
	_angular_velocity_prev.zero();

	// force initial updates
	ParametersUpdate(true);

	// Register callbacks
	_vehicle_angular_velocity_sub.registerCallback();

	return true;
}

void
VehicleAngularAcceleration::Stop()
{
	Deinit();

	// clear all registered callbacks
	_vehicle_angular_velocity_sub.unregisterCallback();
}

void
VehicleAngularAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// Low pass filter
		if (fabsf(_lp_filter.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
			_lp_filter.set_cutoff_frequency(_loop_update_rate_hz, _param_imu_dgyro_cutoff.get());
			_lp_filter.reset(_angular_velocity_prev);
		}
	}
}

void
VehicleAngularAcceleration::Run()
{
	perf_begin(_cycle_perf);

	vehicle_angular_velocity_s v_angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&v_angular_velocity)) {
		perf_set_elapsed(_sensor_latency_perf, hrt_elapsed_time(&v_angular_velocity.timestamp));

		ParametersUpdate();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((v_angular_velocity.timestamp_sample - _timestamp_sample_prev) / 1e6f), 0.0002f,
						 0.02f);
		_timestamp_sample_prev = v_angular_velocity.timestamp_sample;

		// Differentiate angular velocity
		Vector3f angular_velocity(v_angular_velocity.xyz);
		Vector3f angular_acceleration_raw = (angular_velocity - _angular_velocity_prev) / dt;
		_angular_velocity_prev = angular_velocity;

		// Apply low pass filter
		Vector3f angular_acceleration(_lp_filter.apply(angular_acceleration_raw));

		// Publish
		vehicle_angular_acceleration_s v_angular_acceleration;
		v_angular_acceleration.timestamp = hrt_absolute_time();
		v_angular_acceleration.timestamp_sample = v_angular_velocity.timestamp_sample;
		angular_acceleration.copyTo(v_angular_acceleration.xyz);

		_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

		// Calculate loop rate
		_dt_accumulator += dt;
		++_loop_counter;

		if (_dt_accumulator > 1.f) {
			const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
			_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
			_dt_accumulator = 0;
			_loop_counter = 0;

			if (fabsf(_lp_filter.get_sample_freq() - _loop_update_rate_hz) > 1.0f) {
				_lp_filter.set_cutoff_frequency(_loop_update_rate_hz, _param_imu_dgyro_cutoff.get());
			}
		}
	}

	perf_end(_cycle_perf);
}

void
VehicleAngularAcceleration::PrintStatus()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_sensor_latency_perf);
}
