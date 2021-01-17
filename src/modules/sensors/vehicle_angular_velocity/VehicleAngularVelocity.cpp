/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// gyro low pass
	for (auto &lp : _lp_filter_velocity) {
		lp.set_cutoff_frequency(kInitialRateHz, _param_imu_gyro_cutoff.get());
	}

	// notch filter
	for (auto &nf : _notch_filter_velocity) {
		nf.setParameters(kInitialRateHz, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
	}

	// angular acceleration low pass
	for (auto &lp : _lp_filter_acceleration) {
		lp.set_cutoff_frequency(kInitialRateHz, _param_imu_dgyro_cutoff.get());
	}
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();
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

	if (!SensorSelectionUpdate(true)) {
		_selected_sensor_sub_index = 0;
		_sensor_sub.registerCallback();
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAngularVelocity::CheckFilters()
{
	// calculate sensor update rate
	const float sample_interval_avg = _filter_sample_rate;

	if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

		_update_rate_hz = 1.e6f / sample_interval_avg;

		// check if sample rate error is greater than 1%
		if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
			PX4_DEBUG("resetting filters, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
			_reset_filters = true;
			_filter_sample_rate = _update_rate_hz;
		}

		if (_reset_filters || (_required_sample_updates == 0)) {
			if (_param_imu_gyro_rate_max.get() > 0) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_rate_max.get();
				const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
									(float)sensor_gyro_s::ORB_QUEUE_LENGTH);

				_sensor_sub.set_required_updates(samples);
				_required_sample_updates = samples;

			} else {
				_sensor_sub.set_required_updates(1);
				_required_sample_updates = 1;
			}
		}

		// publish interval
		if (_param_imu_gyro_rate_max.get() > 0) {
			const uint64_t imu_gyro_interval = 1e6f / _param_imu_gyro_rate_max.get();
			_publish_interval_min_us = imu_gyro_interval - (sample_interval_avg / 2);

		} else {
			_publish_interval_min_us = 0;
		}
	}

	// reset sample interval accumulator
	_timestamp_interval_last = 0;
	_sample_rate_determined = true;
}

void VehicleAngularVelocity::ResetFilters(const Vector3f &angular_velocity, const Vector3f &angular_acceleration)
{
	for (int axis = 0; axis < 3; axis++) {
		_lp_filter_velocity[axis].set_cutoff_frequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
		_lp_filter_velocity[axis].reset(angular_velocity(axis));
	}

	for (int axis = 0; axis < 3; axis++) {
		_notch_filter_velocity[axis].setParameters(_filter_sample_rate, _param_imu_gyro_nf_freq.get(),
				_param_imu_gyro_nf_bw.get());
		_notch_filter_velocity[axis].reset(angular_velocity(axis));
	}

	for (int axis = 0; axis < 3; axis++) {
		_lp_filter_acceleration[axis].set_cutoff_frequency(_filter_sample_rate, _param_imu_dgyro_cutoff.get());
		_lp_filter_acceleration[axis].reset(angular_acceleration(axis));
	}

	_reset_filters = false;
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

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

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {

			// see if the selected sensor publishes sensor_gyro_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {
						// unregister
						_sensor_sub.unregisterCallback();

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_fifo_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_sample_rate_determined = false;
						_required_sample_updates = 0;
						_reset_filters = true;
						_fifo_data_filtered_prev.zero();

						_fifo_available = true;

						return true;
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if ((sensor_gyro_sub.get().device_id != 0) && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						// unregister
						_sensor_fifo_sub.unregisterCallback();

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_sample_rate_determined = false;
						_required_sample_updates = 0;
						_reset_filters = true;

						_fifo_available = false;

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
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();

		// gyro low pass cutoff frequency changed
		for (auto &lp : _lp_filter_velocity) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

		// gyro notch filter frequency or bandwidth changed
		for (auto &nf : _notch_filter_velocity) {
			if ((fabsf(nf.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
			    || (fabsf(nf.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f)) {

				_reset_filters = true;
				break;
			}
		}

		// gyro derivative low pass cutoff changed
		for (auto &lp : _lp_filter_acceleration) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}
	}
}

float VehicleAngularVelocity::GetSampleRateForGyro(uint32_t device_id)
{
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().gyro_device_id == device_id) {
			if (imu_status.get().gyro_rate_hz > 0) {
				return imu_status.get().gyro_rate_hz;
			}
		}
	}

	return NAN;
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	if (_fifo_available) {

		// dynamic notch filter update
		if (_sample_rate_determined) {
			sensor_gyro_fft_s sensor_gyro_fft;

			if (_sensor_gyro_fft_sub.update(&sensor_gyro_fft) && (sensor_gyro_fft.device_id = _selected_sensor_device_id)) {
				for (int i = 0; i < MAX_NUM_FFT_PEAKS; i++) {
					for (int axis = 0; axis < 3; axis++) {

						float *peak_frequencies = nullptr;

						switch (axis) {
						case 0:
							peak_frequencies = sensor_gyro_fft.peak_frequencies_x;
							break;

						case 1:
							peak_frequencies = sensor_gyro_fft.peak_frequencies_y;
							break;

						case 2:
							peak_frequencies = sensor_gyro_fft.peak_frequencies_z;
							break;
						}

						if (PX4_ISFINITE(peak_frequencies[i]) && (peak_frequencies[i] > 10.f)
						    && fabsf(_dynamic_notch_filter[i][axis].getNotchFreq() - peak_frequencies[i]) > 0.1f) {

							_dynamic_notch_filter[i][axis].setParameters(_filter_sample_rate,
									peak_frequencies[i], sensor_gyro_fft.resolution_hz);

						} else {
							// disable
							_dynamic_notch_filter[i][axis].setParameters(_filter_sample_rate, 0, sensor_gyro_fft.resolution_hz);
						}
					}
				}
			}
		}

		// process all outstanding fifo messages
		sensor_gyro_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {

			if (sensor_fifo_data.samples > 0 && sensor_fifo_data.samples <= 32) {
				const int N = sensor_fifo_data.samples;
				const float dt_s = sensor_fifo_data.dt / 1e6f;

				if (_reset_filters) {
					if (!_sample_rate_determined) {
						// sample rate hasn't been calculated internally yet, try to get it from vehicle_imu_status
						const float gyro_rate_hz = GetSampleRateForGyro(sensor_fifo_data.device_id);

						if (PX4_ISFINITE(gyro_rate_hz)) {
							_filter_sample_rate = gyro_rate_hz * sensor_fifo_data.samples;
							PX4_DEBUG("using FIFO sample rate %.3f Hz", (double)_filter_sample_rate);
						}
					}

					Vector3f angular_velocity_reset{};
					angular_velocity_reset(0) = sensor_fifo_data.x[0];
					angular_velocity_reset(1) = sensor_fifo_data.y[0];
					angular_velocity_reset(2) = sensor_fifo_data.z[0];

					Vector3f angular_acceleration_reset{};

					if (sensor_fifo_data.samples >= 2) {
						angular_acceleration_reset(0) = (sensor_fifo_data.x[1] - sensor_fifo_data.x[0]) / dt_s;
						angular_acceleration_reset(1) = (sensor_fifo_data.y[1] - sensor_fifo_data.y[0]) / dt_s;
						angular_acceleration_reset(2) = (sensor_fifo_data.z[1] - sensor_fifo_data.z[0]) / dt_s;
					}

					ResetFilters(angular_velocity_reset, angular_acceleration_reset);
				}

				matrix::Vector3f angular_velocity_unscaled;
				matrix::Vector3f angular_acceleration_unscaled;

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering
					int16_t *raw_data = nullptr;

					switch (axis) {
					case 0:
						raw_data = sensor_fifo_data.x;
						break;

					case 1:
						raw_data = sensor_fifo_data.y;
						break;

					case 2:
						raw_data = sensor_fifo_data.z;
						break;
					}

					float data[N];

					for (int n = 0; n < N; n++) {
						data[n] = raw_data[n];
					}

					// Apply dynamic notch filter from FFT
					for (auto &dnf : _dynamic_notch_filter) {
						if (dnf[axis].getNotchFreq() > 10.f) {
							dnf[axis].applyDF1(data, N);
						}
					}

					// Apply general notch filter (IMU_GYRO_NF_FREQ)
					if (_notch_filter_velocity[axis].getNotchFreq() > 0.f) {
						_notch_filter_velocity[axis].apply(data, N);
					}

					// Apply general low-pass filter (IMU_GYRO_CUTOFF)
					_lp_filter_velocity[axis].apply(data, N);

					// save last filtered sample
					angular_velocity_unscaled(axis) = data[N - 1];


					// angular acceleration: Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
					for (int n = 0; n < N; n++) {
						float accel = (data[n] - _fifo_data_filtered_prev(axis)) / dt_s;
						angular_acceleration_unscaled(axis) = _lp_filter_acceleration[axis].apply(accel);
						_fifo_data_filtered_prev(axis) = data[n];
					}
				}

				// Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
				const Vector3f angular_velocity{_calibration.Correct(angular_velocity_unscaled * sensor_fifo_data.scale) - _bias};

				// Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
				const Vector3f angular_acceleration{_calibration.rotation() *angular_acceleration_unscaled * sensor_fifo_data.scale};

				// Publish
				if (!_sensor_fifo_sub.updated() && (sensor_fifo_data.timestamp_sample - _last_publish >= _publish_interval_min_us)) {
					Publish(sensor_fifo_data.timestamp_sample, angular_velocity, angular_acceleration);
					_last_publish = sensor_fifo_data.timestamp_sample;
				}
			}
		}

	} else {
		// process all outstanding messages
		sensor_gyro_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) / 1e6f), 0.0002f, 0.02f);
			_timestamp_sample_last = sensor_data.timestamp_sample;

			// Apply calibration, rotation, and correct for in-run bias errors
			Vector3f angular_velocity{_calibration.Correct(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z}) - _bias};
			Vector3f angular_acceleration{};

			if (_reset_filters) {
				if (!_sample_rate_determined) {
					// if sample rate was previously determined use it
					if (PX4_ISFINITE(_sensor_sample_rate[_selected_sensor_sub_index])) {
						_filter_sample_rate = _sensor_sample_rate[_selected_sensor_sub_index];

					} else {
						// sample rate hasn't been calculated internally yet, try to get it from vehicle_imu_status
						const float gyro_rate_hz = GetSampleRateForGyro(sensor_data.device_id);

						if (PX4_ISFINITE(gyro_rate_hz)) {
							_filter_sample_rate = gyro_rate_hz;
							PX4_DEBUG("using sample rate %.3f Hz", (double)_filter_sample_rate);
						}
					}
				}

				ResetFilters(angular_velocity, angular_acceleration);
			}

			for (int axis = 0; axis < 3; axis++) {
				// Apply general notch filter (IMU_GYRO_NF_FREQ)
				_notch_filter_velocity[axis].apply(&angular_velocity(axis), 1);

				// Apply general low-pass filter (IMU_GYRO_CUTOFF)
				_lp_filter_velocity[axis].apply(&angular_velocity(axis), 1);

				// Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
				float accel = (angular_velocity(axis) - _angular_velocity_last(axis)) / dt;
				angular_acceleration(axis) = _lp_filter_acceleration[axis].apply(accel);
				_angular_velocity_last(axis) = angular_velocity(axis);
			}

			// Publish
			if (!_sensor_sub.updated() && (sensor_data.timestamp_sample - _last_publish >= _publish_interval_min_us)) {
				Publish(sensor_data.timestamp_sample, angular_velocity, angular_acceleration);
				_last_publish = sensor_data.timestamp_sample;
			}
		}
	}
}

void VehicleAngularVelocity::Publish(const hrt_abstime &timestamp_sample, const Vector3f &angular_velocity,
				     const Vector3f &angular_acceleration)
{
	// Publish vehicle_angular_acceleration
	vehicle_angular_acceleration_s v_angular_acceleration;
	v_angular_acceleration.timestamp_sample = timestamp_sample;
	angular_acceleration.copyTo(v_angular_acceleration.xyz);
	v_angular_acceleration.timestamp = hrt_absolute_time();
	_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

	// Publish vehicle_angular_velocity
	vehicle_angular_velocity_s v_angular_velocity;
	v_angular_velocity.timestamp_sample = timestamp_sample;
	angular_velocity.copyTo(v_angular_velocity.xyz);
	v_angular_velocity.timestamp = hrt_absolute_time();
	_vehicle_angular_velocity_pub.publish(v_angular_velocity);

	_last_publish = timestamp_sample;
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d), rate: %.1f Hz %s",
		 _selected_sensor_device_id, _selected_sensor_sub_index, (double)_update_rate_hz, _fifo_available ? "FIFO" : "");
	PX4_INFO("estimated bias: [%.4f %.4f %.4f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
