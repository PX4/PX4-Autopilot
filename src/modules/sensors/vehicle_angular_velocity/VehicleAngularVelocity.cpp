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

#include "VehicleAngularVelocity.hpp"

#include <px4_log.h>

using namespace time_literals;
using namespace matrix;

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	WorkItem(px4::wq_configurations::rate_ctrl),
	_cycle_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: cycle time")),
	_interval_perf(perf_alloc(PC_INTERVAL, "vehicle_angular_velocity: interval")),
	_sensor_gyro_latency_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: sensor gyro latency"))
{
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_sensor_gyro_latency_perf);
}

bool
VehicleAngularVelocity::Start()
{
	// initialize thermal corrections as we might not immediately get a topic update (only non-zero values)
	_scale = Vector3f{1.0f, 1.0f, 1.0f};
	_offset.zero();
	_bias.zero();

	// force initial updates
	ParametersUpdate(true);
	SensorBiasUpdate(true);

	// needed to change the active sensor if the primary stops updating
	_sensor_selection_sub.register_callback();

	return SensorCorrectionsUpdate(true);
}

void
VehicleAngularVelocity::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto sub : _sensor_gyro_sub) {
		sub.unregister_callback();
	}

	_sensor_selection_sub.unregister_callback();
}

void
VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	if (_sensor_bias_sub.updated() || force) {
		sensor_bias_s bias;

		if (_sensor_bias_sub.copy(&bias)) {
			// TODO: should be checking device ID
			_bias(0) = bias.gyro_x_bias;
			_bias(1) = bias.gyro_y_bias;
			_bias(2) = bias.gyro_z_bias;
		}
	}
}

bool
VehicleAngularVelocity::SensorCorrectionsUpdate(bool force)
{
	// check if the selected gyro has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// TODO: should be checking device ID
		if (_selected_gyro == 0) {
			_offset = Vector3f{corrections.gyro_offset_0};
			_scale = Vector3f{corrections.gyro_scale_0};

		} else if (_selected_gyro == 1) {
			_offset = Vector3f{corrections.gyro_offset_1};
			_scale = Vector3f{corrections.gyro_scale_1};

		} else if (_selected_gyro == 2) {
			_offset = Vector3f{corrections.gyro_offset_2};
			_scale = Vector3f{corrections.gyro_scale_2};

		} else {
			_offset = Vector3f{0.0f, 0.0f, 0.0f};
			_scale = Vector3f{1.0f, 1.0f, 1.0f};
		}

		// update the latest gyro selection
		if (_selected_gyro != corrections.selected_gyro_instance) {
			if (corrections.selected_gyro_instance < MAX_GYRO_COUNT) {
				// clear all registered callbacks
				for (auto sub : _sensor_gyro_sub) {
					sub.unregister_callback();
				}

				const int gyro_new = corrections.selected_gyro_instance;

				if (_sensor_gyro_sub[gyro_new].register_callback()) {
					PX4_DEBUG("selected gyro changed %d -> %d", _selected_gyro, gyro_new);
					_selected_gyro = gyro_new;

					return true;
				}
			}
		}
	}

	return false;
}

void
VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// get transformation matrix from sensor/board to body frame
		const matrix::Dcmf board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		// fine tune the rotation
		const Dcmf board_rotation_offset(Eulerf(
				math::radians(_param_sens_board_x_off.get()),
				math::radians(_param_sens_board_y_off.get()),
				math::radians(_param_sens_board_z_off.get())));

		_board_rotation = board_rotation_offset * board_rotation;
	}
}

void
VehicleAngularVelocity::Run()
{
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// update corrections first to set _selected_gyro
	SensorCorrectionsUpdate();

	sensor_gyro_s sensor_gyro;

	if (_sensor_gyro_sub[_selected_gyro].update(&sensor_gyro)) {
		perf_set_elapsed(_sensor_gyro_latency_perf, hrt_elapsed_time(&sensor_gyro.timestamp));

		ParametersUpdate();
		SensorBiasUpdate();

		// get the raw gyro data and correct for thermal errors
		const Vector3f gyro{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z};

		// apply offsets and scale
		Vector3f rates{(gyro - _offset).emult(_scale)};

		// rotate corrected measurements from sensor to body frame
		rates = _board_rotation * rates;

		// correct for in-run bias errors
		rates -= _bias;

		vehicle_angular_velocity_s angular_velocity;
		angular_velocity.timestamp_sample = sensor_gyro.timestamp;
		rates.copyTo(angular_velocity.xyz);
		angular_velocity.timestamp = hrt_absolute_time();

		_vehicle_angular_velocity_pub.publish(angular_velocity);
	}

	perf_end(_cycle_perf);
}

void
VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected gyro: %d", _selected_gyro);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_sensor_gyro_latency_perf);
}
