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

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_acceleration_pub.advertise();
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		ScheduleNow();
	}

	return true;
}

void VehicleAcceleration::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_fifo_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

bool VehicleAcceleration::UpdateSampleRate()
{
	float sample_rate_hz = NAN;
	float publish_rate_hz = NAN;

	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().accel_device_id == _selected_sensor_device_id) {
			sample_rate_hz = imu_status.get().accel_raw_rate_hz;
			publish_rate_hz = imu_status.get().accel_rate_hz;
			break;
		}
	}

	// calculate sensor update rate
	if ((sample_rate_hz > 0) && PX4_ISFINITE(sample_rate_hz) && (publish_rate_hz > 0) && PX4_ISFINITE(publish_rate_hz)) {
		// check if sample rate error is greater than 1%
		const bool sample_rate_changed = (fabsf(sample_rate_hz - _filter_sample_rate_hz) / sample_rate_hz) > 0.01f;

		if (_update_sample_rate || sample_rate_changed
		    || (_filter_sample_rate_hz <= FLT_EPSILON) || !PX4_ISFINITE(_filter_sample_rate_hz)) {

			PX4_DEBUG("updating sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_hz, (double)sample_rate_hz);

			if (sample_rate_changed || !PX4_ISFINITE(_filter_sample_rate_hz)) {
				_reset_filters = true;
			}

			_filter_sample_rate_hz = sample_rate_hz;
			_update_sample_rate = false;

			if (_param_imu_integ_rate.get() > 0.f) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
				const float publish_interval_us = 1e6f / publish_rate_hz;

				const uint8_t samples = roundf(configured_interval_us / publish_interval_us);

				if (_fifo_available) {
					_sensor_fifo_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_imu_fifo_s::ORB_QUEUE_LENGTH));

				} else {
					_sensor_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_accel_s::ORB_QUEUE_LENGTH));
				}

				// publish interval (constrained 100 Hz - 8 kHz)
				_publish_interval_min_us = math::constrain((int)roundf(configured_interval_us - (publish_interval_us * 0.5f)), 125,
							   10000);

			} else {
				_sensor_sub.set_required_updates(1);
				_sensor_fifo_sub.set_required_updates(1);
				_publish_interval_min_us = 0;
			}
		}
	}

	return PX4_ISFINITE(_filter_sample_rate_hz) && (_filter_sample_rate_hz > 0);
}

void VehicleAcceleration::ResetFilters(const hrt_abstime &time_now_us)
{
	if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

		const Vector3f acceleration_uncalibrated{GetResetAcceleration()};

		PX4_INFO("reset acceleration [%.5f, %.5f, %.5f]", (double)acceleration_uncalibrated(0),
			 (double)acceleration_uncalibrated(1), (double)acceleration_uncalibrated(2));

		for (int axis = 0; axis < 3; axis++) {
			// acceleration low pass
			_lp_filter[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_accel_cutoff.get());
			_lp_filter[axis].reset(acceleration_uncalibrated(axis));
		}

		_reset_filters = false;
		perf_count(_filter_reset_perf);
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
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

		if (_estimator_sensor_bias_sub.copy(&bias)
		    && (bias.accel_device_id == _selected_sensor_device_id)
		    && bias.accel_bias_valid) {

			_bias = Vector3f{bias.accel_bias};

		} else {
			_bias.zero();
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		bool selected_device_id_valid = false;
		uint32_t device_id = sensor_selection.accel_device_id;
		uint32_t device_id_first_valid_imu = 0;

		// use vehicle_imu_status to do basic sensor selection validation
		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

			if (imu_status.advertised()
			    && (imu_status.get().timestamp != 0) && (time_now_us < imu_status.get().timestamp + 1_s)
			    && (imu_status.get().accel_device_id != 0)) {
				// vehicle_imu_status accel valid

				if ((device_id != 0) && (imu_status.get().accel_device_id == device_id)) {
					selected_device_id_valid = true;
				}

				// record first valid IMU as a backup option
				if (device_id_first_valid_imu == 0) {
					device_id_first_valid_imu = imu_status.get().accel_device_id;
				}
			}
		}

		// if no accel selected or healthy then use fallback
		if ((device_id == 0) || !selected_device_id_valid) {
			device_id = device_id_first_valid_imu;
		}

		if ((_selected_sensor_device_id != device_id) || force) {

			const bool device_id_valid = (device_id != 0);

			// see if the selected sensor publishes sensor_imu_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_imu_fifo_s> sensor_imu_fifo_sub{ORB_ID(sensor_imu_fifo), i};

				if (sensor_imu_fifo_sub.get().device_id != 0) {
					// if no accel was selected use the first valid sensor_imu_fifo
					if (!device_id_valid && (time_now_us < sensor_imu_fifo_sub.get().timestamp + 1_s)) {
						device_id = sensor_imu_fifo_sub.get().device_id;
					}

					if ((sensor_imu_fifo_sub.get().device_id == device_id)
					    && _sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {

						// make sure non-FIFO sub is unregistered
						_sensor_sub.unregisterCallback();

						_calibration.set_device_id(sensor_imu_fifo_sub.get().device_id);

						if (_calibration.enabled()) {
							_selected_sensor_device_id = sensor_imu_fifo_sub.get().device_id;

							_filter_sample_rate_hz = 1.f / (sensor_imu_fifo_sub.get().dt * 1e-6f);
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = true;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_imu_fifo:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							_selected_sensor_device_id = 0;
						}
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				if (sensor_accel_sub.advertised()
				    && (sensor_accel_sub.get().timestamp != 0)
				    && (sensor_accel_sub.get().device_id != 0)
				    && (time_now_us < sensor_accel_sub.get().timestamp + 1_s)) {

					// if no accel was selected use the first valid sensor_accel
					if (!device_id_valid) {
						device_id = sensor_accel_sub.get().device_id;
						PX4_WARN("no accel selected, using sensor_accel:%" PRIu8 " %" PRIu32, i, sensor_accel_sub.get().device_id);
					}

					if (sensor_accel_sub.get().device_id == device_id) {
						if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
							// make sure FIFO sub is unregistered
							_sensor_fifo_sub.unregisterCallback();

							_calibration.set_device_id(sensor_accel_sub.get().device_id);

							_selected_sensor_device_id = sensor_accel_sub.get().device_id;

							_filter_sample_rate_hz = NAN;
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = false;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_accel:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							PX4_ERR("unable to register callback for sensor_accel:%" PRIu8 " %" PRIu32,
								i, sensor_accel_sub.get().device_id);
						}
					}
				}
			}

			if (device_id != 0) {
				PX4_ERR("unable to find or subscribe to selected sensor (%" PRIu32 ")", device_id);
			}

			_selected_sensor_device_id = 0;
		}
	}

	return false;
}

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();

		// accel low pass cutoff frequency changed
		for (auto &lp : _lp_filter) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}
	}
}

Vector3f VehicleAcceleration::GetResetAcceleration() const
{
	if (_last_publish != 0) {
		// acceleration filtering is performed on raw unscaled data
		//  start with last valid vehicle body frame acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f acceleration_uncalibrated{_calibration.Uncorrect(_acceleration + _bias)};

		if (PX4_ISFINITE(acceleration_uncalibrated(0))
		    && PX4_ISFINITE(acceleration_uncalibrated(1))
		    && PX4_ISFINITE(acceleration_uncalibrated(2))) {

			return acceleration_uncalibrated;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

float VehicleAcceleration::FilterAcceleration(int axis, float data[], int N)
{
	// Apply general low-pass filter (IMU_ACCEL_CUTOFF)
	_lp_filter[axis].applyArray(data, N);

	// return last filtered sample
	return data[N - 1];
}

void VehicleAcceleration::Run()
{
	perf_begin(_cycle_perf);

	// backup schedule
	ScheduleDelayed(10_ms);

	const hrt_abstime time_now_us = hrt_absolute_time();

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate(time_now_us);

	if (selection_updated || _update_sample_rate) {
		if (!UpdateSampleRate()) {
			// sensor sample rate required to run
			perf_end(_cycle_perf);
			return;
		}
	}

	ParametersUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);

	if (_reset_filters) {
		ResetFilters(time_now_us);

		if (_reset_filters) {
			// not safe to run until filters configured
			perf_end(_cycle_perf);
			return;
		}
	}

	if (_fifo_available) {
		// process all outstanding fifo messages
		sensor_imu_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {
			const int N = sensor_fifo_data.samples;
			static constexpr int FIFO_SIZE_MAX = sizeof(sensor_fifo_data.accel_x) / sizeof(sensor_fifo_data.accel_x[0]);

			if ((sensor_fifo_data.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
				Vector3f acceleration_uncalibrated;

				int16_t *raw_data_array[] {sensor_fifo_data.accel_x, sensor_fifo_data.accel_y, sensor_fifo_data.accel_z};

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering
					float data[FIFO_SIZE_MAX];

					for (int n = 0; n < N; n++) {
						data[n] = sensor_fifo_data.accel_scale * raw_data_array[axis][n];
					}

					// save last filtered sample
					acceleration_uncalibrated(axis) = FilterAcceleration(axis, data, N);
				}

				// Publish
				if (!_sensor_fifo_sub.updated()) {
					if (CalibrateAndPublish(sensor_fifo_data.timestamp_sample, acceleration_uncalibrated)) {

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}

	} else {
		// process all outstanding messages
		sensor_accel_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			if (PX4_ISFINITE(sensor_data.x) && PX4_ISFINITE(sensor_data.y) && PX4_ISFINITE(sensor_data.z)) {

				Vector3f acceleration_uncalibrated;

				float raw_data_array[] {sensor_data.x, sensor_data.y, sensor_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy sensor sample to float array for filtering
					float data[1] {raw_data_array[axis]};

					// save last filtered sample
					acceleration_uncalibrated(axis) = FilterAcceleration(axis, data);
				}

				// Publish
				if (!_sensor_sub.updated()) {
					if (CalibrateAndPublish(sensor_data.timestamp_sample, acceleration_uncalibrated)) {

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}
	}

	// force reselection on timeout
	if (time_now_us > _last_publish + 500_ms) {
		SensorSelectionUpdate(true);
	}

	perf_end(_cycle_perf);
}

bool VehicleAcceleration::CalibrateAndPublish(const hrt_abstime &timestamp_sample,
		const Vector3f &acceleration_uncalibrated)
{
	if (timestamp_sample >= _last_publish + _publish_interval_min_us) {

		// Publish vehicle_acceleration
		vehicle_acceleration_s v_acceleration;
		v_acceleration.timestamp_sample = timestamp_sample;

		// Acceleration: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
		_acceleration = _calibration.Correct(acceleration_uncalibrated) - _bias;
		_acceleration.copyTo(v_acceleration.xyz);

		v_acceleration.timestamp = hrt_absolute_time();
		_vehicle_acceleration_pub.publish(v_acceleration);


		// shift last publish time forward, but don't let it get further behind than the interval
		_last_publish = math::constrain(_last_publish + _publish_interval_min_us,
						timestamp_sample - _publish_interval_min_us, timestamp_sample);

		return true;
	}

	return false;
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_acceleration] selected sensor: %" PRIu32
		     ", rate: %.1f Hz %s, estimated bias: [%.5f %.5f %.5f]\n",
		     _calibration.device_id(), (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
		     (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_filter_reset_perf);
	perf_print_counter(_selection_changed_perf);
}

} // namespace sensors
