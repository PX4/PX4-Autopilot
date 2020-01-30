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

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_lp_filter_velocity.set_cutoff_frequency(kInitialRateHz, _param_imu_gyro_cutoff.get());
	_notch_filter_velocity.setParameters(kInitialRateHz, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());

	_lp_filter_acceleration.set_cutoff_frequency(kInitialRateHz, _param_imu_dgyro_cutoff.get());
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();

	perf_free(_interval_perf);
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	ScheduleNow();
	return true;
}

void VehicleAngularVelocity::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}

	_sensor_selection_sub.unregisterCallback();
}

void VehicleAngularVelocity::CheckFilters()
{
	if ((hrt_elapsed_time(&_filter_check_last) > 100_ms)) {
		_filter_check_last = hrt_absolute_time();

		// calculate sensor update rate
		const float sample_interval_avg = perf_mean(_interval_perf);

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

			const float update_rate_hz = 1.0f / sample_interval_avg;

			if ((fabsf(update_rate_hz) > 0.0f) && PX4_ISFINITE(update_rate_hz)) {
				_update_rate_hz = update_rate_hz;

				// check if sample rate error is greater than 1%
				if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
					++_sample_rate_incorrect_count;
				}
			}
		}

		const bool sample_rate_updated = (_sample_rate_incorrect_count > 50);

		const bool lp_velocity_updated = (fabsf(_lp_filter_velocity.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f);
		const bool notch_updated = ((fabsf(_notch_filter_velocity.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
					    || (fabsf(_notch_filter_velocity.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f));

		const bool lp_acceleration_updated = (fabsf(_lp_filter_acceleration.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) >
						      0.01f);

		if (sample_rate_updated || lp_velocity_updated || notch_updated || lp_acceleration_updated) {
			PX4_INFO("updating filter, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
			_filter_sample_rate = _update_rate_hz;

			// update software low pass filters
			_lp_filter_velocity.set_cutoff_frequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
			_lp_filter_velocity.reset(_angular_velocity_prev);

			_notch_filter_velocity.setParameters(_filter_sample_rate, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
			_notch_filter_velocity.reset(_angular_velocity_prev);

			_lp_filter_acceleration.set_cutoff_frequency(_filter_sample_rate, _param_imu_dgyro_cutoff.get());
			_lp_filter_acceleration.reset(_angular_acceleration_prev);

			// reset state
			_sample_rate_incorrect_count = 0;
		}
	}
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.gyro_device_id == _selected_sensor_device_id) {
				_bias = Vector3f{bias.gyro_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

void VehicleAngularVelocity::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// selected sensor has changed, find updated index
		if ((_corrections_selected_instance < 0) || force) {
			_corrections_selected_instance = -1;

			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.gyro_device_ids[i] == _selected_sensor_device_id) {
					_corrections_selected_instance = i;
				}
			}
		}

		switch (_corrections_selected_instance) {
		case 0:
			_offset = Vector3f{corrections.gyro_offset_0};
			_scale = Vector3f{corrections.gyro_scale_0};
			break;
		case 1:
			_offset = Vector3f{corrections.gyro_offset_1};
			_scale = Vector3f{corrections.gyro_scale_1};
			break;
		case 2:
			_offset = Vector3f{corrections.gyro_offset_2};
			_scale = Vector3f{corrections.gyro_scale_2};
			break;
		default:
			_offset = Vector3f{0.f, 0.f, 0.f};
			_scale = Vector3f{1.f, 1.f, 1.f};
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				sensor_gyro_s report{};
				_sensor_sub[i].copy(&report);

				if ((report.device_id != 0) && (report.device_id == sensor_selection.gyro_device_id)) {
					if (_sensor_sub[i].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _selected_sensor_sub_index, i);

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();
						_offset = Vector3f{0.f, 0.f, 0.f};
						_scale = Vector3f{1.f, 1.f, 1.f};

						// force corrections reselection
						_corrections_selected_instance = -1;

						// reset sample rate monitor
						_sample_rate_incorrect_count = 0;

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
			_selected_sensor_device_id = 0;
			_selected_sensor_sub_index = 0;
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// get transformation matrix from sensor/board to body frame
		const Dcmf board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		// fine tune the rotation
		const Dcmf board_rotation_offset(Eulerf(
				radians(_param_sens_board_x_off.get()),
				radians(_param_sens_board_y_off.get()),
				radians(_param_sens_board_z_off.get())));

		_board_rotation = board_rotation_offset * board_rotation;
	}
}

void VehicleAngularVelocity::Run()
{
	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	bool sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();

	// process all outstanding messages
	while (sensor_updated || selection_updated) {
		selection_updated = false;

		sensor_gyro_s sensor_data;

		if (_sensor_sub[_selected_sensor_sub_index].copy(&sensor_data)) {

			if (sensor_updated) {
				perf_count_interval(_interval_perf, sensor_data.timestamp_sample);
			}

			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_prev) / 1e6f), 0.0002f, 0.02f);
			_timestamp_sample_prev = sensor_data.timestamp_sample;

			// get the sensor data and correct for thermal errors (apply offsets and scale)
			Vector3f angular_velocity_raw{(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z} - _offset).emult(_scale)};

			// rotate corrected measurements from sensor to body frame
			angular_velocity_raw = _board_rotation * angular_velocity_raw;

			// correct for in-run bias errors
			angular_velocity_raw -= _bias;

			// Differentiate angular velocity (after notch filter)
			const Vector3f angular_velocity_notched{_notch_filter_velocity.apply(angular_velocity_raw)};
			const Vector3f angular_acceleration_raw = (angular_velocity_notched - _angular_velocity_prev) / dt;

			_angular_velocity_prev = angular_velocity_notched;
			_angular_acceleration_prev = angular_acceleration_raw;

			CheckFilters();

			// Filter: apply low-pass
			const Vector3f angular_acceleration{_lp_filter_acceleration.apply(angular_acceleration_raw)};
			const Vector3f angular_velocity{_lp_filter_velocity.apply(angular_velocity_notched)};

			// publish once all new samples are processed
			sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();

			if (!sensor_updated) {
				bool publish = true;

				if (_param_imu_gyro_rate_max.get() > 0) {
					const uint64_t interval = 1e6f / _param_imu_gyro_rate_max.get();

					if (hrt_elapsed_time(&_last_publish) < interval) {
						publish = false;
					}
				}

				if (publish) {
					// Publish vehicle_angular_acceleration
					vehicle_angular_acceleration_s v_angular_acceleration;
					v_angular_acceleration.timestamp_sample = sensor_data.timestamp_sample;
					angular_acceleration.copyTo(v_angular_acceleration.xyz);
					v_angular_acceleration.timestamp = hrt_absolute_time();
					_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

					// Publish vehicle_angular_velocity
					vehicle_angular_velocity_s v_angular_velocity;
					v_angular_velocity.timestamp_sample = sensor_data.timestamp_sample;
					angular_velocity.copyTo(v_angular_velocity.xyz);
					v_angular_velocity.timestamp = hrt_absolute_time();
					_vehicle_angular_velocity_pub.publish(v_angular_velocity);

					_last_publish = v_angular_velocity.timestamp_sample;
					return;
				}
			}
		}
	}
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d)", _selected_sensor_device_id, _selected_sensor_sub_index);
	PX4_INFO("bias: [%.3f %.3f %.3f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));
	PX4_INFO("offset: [%.3f %.3f %.3f]", (double)_offset(0), (double)_offset(1), (double)_offset(2));
	PX4_INFO("scale: [%.3f %.3f %.3f]", (double)_scale(0), (double)_scale(1), (double)_scale(2));

	PX4_INFO("sample rate: %.3f Hz", (double)_update_rate_hz);
	PX4_INFO("low-pass filter cutoff: %.3f Hz", (double)_lp_filter_velocity.get_cutoff_freq());

	if (_notch_filter_velocity.getNotchFreq() > 0.0f) {
		PX4_INFO("notch filter freq: %.3f Hz\tbandwidth: %.3f Hz", (double)_notch_filter_velocity.getNotchFreq(),
			 (double)_notch_filter_velocity.getBandwidth());
	}
}
