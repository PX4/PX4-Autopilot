/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "VehicleImuFifo.hpp"

#include <px4_platform_common/log.h>

#include <float.h>

using namespace matrix;

using math::constrain;

namespace sensors
{

static constexpr uint8_t clipping(const int16_t samples[20], int16_t clip_limit, uint8_t len)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		if (abs(samples[n]) >= clip_limit) {
			clip_count++;
		}
	}

	return clip_count;
}

VehicleImuFifo::VehicleImuFifo(uint8_t accel_index, uint8_t gyro_index, const px4::wq_config_t &config) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_sensor_accel_sub(this, ORB_ID(sensor_accel_fifo), accel_index),
	_sensor_gyro_sub(this, ORB_ID(sensor_gyro_fifo), gyro_index)
{
	const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();

	_accel_integrator.set_reset_interval(configured_interval_us);
	_accel_integrator.set_reset_samples(sensor_accel_fifo_s::ORB_QUEUE_LENGTH);

	_gyro_integrator.set_reset_interval(configured_interval_us);
	_gyro_integrator.set_reset_samples(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// currently with lockstep every raw sample needs a corresponding vehicle_imu publication
	_sensor_accel_sub.set_required_updates(1);
	_sensor_gyro_sub.set_required_updates(1);
#else
	// schedule conservatively until the actual accel & gyro rates are known
	_sensor_accel_sub.set_required_updates(sensor_accel_fifo_s::ORB_QUEUE_LENGTH / 2);
	_sensor_gyro_sub.set_required_updates(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH / 2);
#endif

	// advertise immediately to ensure consistent ordering
	_vehicle_imu_pub.advertise();
	_vehicle_imu_status_pub.advertise();
}

VehicleImuFifo::~VehicleImuFifo()
{
	Stop();

	perf_free(_accel_generation_gap_perf);
	perf_free(_accel_update_perf);
	perf_free(_gyro_generation_gap_perf);
	perf_free(_gyro_update_perf);

	_vehicle_imu_pub.unadvertise();
	_vehicle_imu_status_pub.unadvertise();
}

bool VehicleImuFifo::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_gyro_sub.registerCallback();
	_sensor_accel_sub.registerCallback();
	ScheduleNow();
	return true;
}

void VehicleImuFifo::Stop()
{
	// clear all registered callbacks
	_sensor_accel_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

void VehicleImuFifo::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const auto imu_integ_rate_prev = _param_imu_integ_rate.get();

		updateParams();

		_accel_calibration.ParametersUpdate();
		_gyro_calibration.ParametersUpdate();

		// constrain IMU integration time 1-10 milliseconds (100-1000 Hz)
		int32_t imu_integration_rate_hz = constrain(_param_imu_integ_rate.get(),
						  100, math::max(_param_imu_gyro_ratemax.get(), 1000));

		if (imu_integration_rate_hz != _param_imu_integ_rate.get()) {
			PX4_WARN("IMU_INTEG_RATE updated %d -> %d", _param_imu_integ_rate.get(), imu_integration_rate_hz);
			_param_imu_integ_rate.set(imu_integration_rate_hz);
			_param_imu_integ_rate.commit_no_notification();
		}

		_imu_integration_interval_us = 1000000 / imu_integration_rate_hz;

		if (_param_imu_integ_rate.get() != imu_integ_rate_prev) {
			// force update
			UpdateIntegratorConfiguration();
		}
	}
}

bool VehicleImuFifo::UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample,
		uint8_t samples)
{
	bool updated = false;

	// conservative maximum time between samples to reject large gaps and reset averaging
	float max_interval_us = 10000; // 100 Hz
	float min_interval_us = 50;   // 20,000 Hz

	if (intavg.update_interval > 0) {
		// if available use previously calculated interval as bounds
		max_interval_us = 1.5f * intavg.update_interval;
		min_interval_us = 0.5f * intavg.update_interval;
	}

	const float interval_us = (timestamp_sample - intavg.timestamp_sample_last);

	if ((intavg.timestamp_sample_last > 0) && (interval_us < max_interval_us) && (interval_us > min_interval_us)) {

		intavg.interval_sum += interval_us;
		intavg.sample_count += samples;
		intavg.interval_count++;

		// periodically calculate sensor update rate
		if (intavg.interval_count > 10000 || ((intavg.update_interval <= FLT_EPSILON) && intavg.interval_count > 100)) {

			const float interval_avg = intavg.interval_sum / intavg.interval_count;
			const float sample_interval_avg = intavg.interval_sum / intavg.sample_count;

			if (PX4_ISFINITE(interval_avg) && (interval_avg > 0.f)) {
				// update if interval has changed by more than 0.5%
				if ((fabsf(intavg.update_interval - interval_avg) / intavg.update_interval) > 0.005f) {

					intavg.update_interval = interval_avg;
					intavg.sample_interval = sample_interval_avg;
					updated = true;
				}
			}

			// reset sample interval accumulator
			intavg.interval_sum = 0.f;
			intavg.sample_count = 0;
			intavg.interval_count = 0.f;
		}

	} else {
		// reset
		intavg.interval_sum = 0.f;
		intavg.sample_count = 0;
		intavg.interval_count = 0.f;
	}

	intavg.timestamp_sample_last = timestamp_sample;

	return updated;
}

void VehicleImuFifo::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	ParametersUpdate();

	if (!_accel_calibration.enabled() || !_gyro_calibration.enabled()) {
		return;
	}


	bool sensor_data_gap = false;
	bool update_integrator_config = false;

	// integrate queued gyro
	sensor_gyro_fifo_s sensor_gyro_fifo;

	while (_sensor_gyro_sub.update(&sensor_gyro_fifo)) {
		perf_count_interval(_gyro_update_perf, sensor_gyro_fifo.timestamp_sample);

		if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
			sensor_data_gap = true;
			perf_count(_gyro_generation_gap_perf);

			_gyro_interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured
			    && UpdateIntervalAverage(_gyro_interval, sensor_gyro_fifo.timestamp_sample, sensor_gyro_fifo.samples)) {
				update_integrator_config = true;
				_publish_status = true;
				_status.gyro_rate_hz = 1e6f / _gyro_interval.sample_interval;
			}
		}

		_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

		_gyro_calibration.set_device_id(sensor_gyro_fifo.device_id);

		if (sensor_gyro_fifo.error_count != _status.gyro_error_count) {
			_publish_status = true;
			_status.gyro_error_count = sensor_gyro_fifo.error_count;
		}

		_last_timestamp_sample_gyro = sensor_gyro_fifo.timestamp_sample;

		const float dt_s = sensor_gyro_fifo.dt * 1e-6f;
		const int N = sensor_gyro_fifo.samples;

		for (int n = 0; n < N; n++) {
			const Vector3f gyro_raw{
				sensor_gyro_fifo.scale *sensor_gyro_fifo.x[n],
				sensor_gyro_fifo.scale *sensor_gyro_fifo.y[n],
				sensor_gyro_fifo.scale *sensor_gyro_fifo.z[n]
			};

			_gyro_integrator.put(gyro_raw, dt_s);

			_gyro_sum += gyro_raw;
			_gyro_sum_count++;
		}

		_gyro_temperature += sensor_gyro_fifo.temperature;
		_gyro_temperature_count++;

		// break if interval is configured and we haven't fallen behind
		if (_intervals_configured && _gyro_integrator.integral_ready()
		    && (hrt_elapsed_time(&sensor_gyro_fifo.timestamp) < _imu_integration_interval_us) && !sensor_data_gap) {

			break;
		}
	}

	// update accel, stopping once caught up to the last gyro sample
	sensor_accel_fifo_s sensor_accel_fifo;

	while (_sensor_accel_sub.update(&sensor_accel_fifo)) {
		perf_count_interval(_accel_update_perf, sensor_accel_fifo.timestamp_sample);

		if (_sensor_accel_sub.get_last_generation() != _accel_last_generation + 1) {
			sensor_data_gap = true;
			perf_count(_accel_generation_gap_perf);

			_accel_interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured
			    && UpdateIntervalAverage(_accel_interval, sensor_accel_fifo.timestamp_sample, sensor_accel_fifo.samples)) {
				update_integrator_config = true;
				_publish_status = true;
				_status.accel_rate_hz = 1e6f / _accel_interval.sample_interval;
			}
		}

		_accel_last_generation = _sensor_accel_sub.get_last_generation();

		_accel_calibration.set_device_id(sensor_accel_fifo.device_id);

		if (sensor_accel_fifo.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = sensor_accel_fifo.error_count;
		}

		_last_timestamp_sample_accel = sensor_accel_fifo.timestamp_sample;

		const float dt_s = sensor_accel_fifo.dt * 1e-6f;
		const int N = sensor_accel_fifo.samples;

		// clipping
		Vector3f clip_count {
			(float)clipping(sensor_accel_fifo.x, UINT16_MAX - 1, N),
			(float)clipping(sensor_accel_fifo.y, UINT16_MAX - 1, N),
			(float)clipping(sensor_accel_fifo.z, UINT16_MAX - 1, N),
		};

		// rotate sensor clip counts into vehicle body frame
		const Vector3f clipping{_accel_calibration.rotation() *clip_count};

		// round to get reasonble clip counts per axis (after board rotation)
		if (fabsf(clipping(0)) > 0.f) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_X;
		}

		if (fabsf(clipping(1)) > 0.f) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Y;
		}

		if (fabsf(clipping(2)) > 0.f) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Z;
		}


		for (int n = 0; n < N; n++) {
			const Vector3f accel_raw{
				sensor_accel_fifo.scale *sensor_accel_fifo.x[n],
				sensor_accel_fifo.scale *sensor_accel_fifo.y[n],
				sensor_accel_fifo.scale *sensor_accel_fifo.z[n]
			};

			_accel_integrator.put(accel_raw, dt_s);

			_accel_sum += accel_raw;
			_accel_sum_count++;

			// break once caught up to gyro
			if (!sensor_data_gap && _intervals_configured) {
				// && (_last_timestamp_sample_accel >= (_last_timestamp_sample_gyro - 0.5f * _accel_interval.update_interval))) {

				if (_accel_integrator.integral_ready() && _gyro_integrator.integral_ready()) {
					Publish();
				}
			}
		}

		_accel_temperature += sensor_accel_fifo.temperature;
		_accel_temperature_count++;
	}

	if (sensor_data_gap) {
		_consecutive_data_gap++;

		// if there's consistently a gap in data start monitoring publication interval again
		if (_consecutive_data_gap > 10) {
			_intervals_configured = false;
		}

	} else {
		_consecutive_data_gap = 0;
	}

	// reconfigure integrators if calculated sensor intervals have changed
	if (update_integrator_config) {
		UpdateIntegratorConfiguration();
	}

	Publish();
}

void VehicleImuFifo::Publish()
{
	// publish if both accel & gyro integrators are ready
	if (_accel_integrator.integral_ready() && _gyro_integrator.integral_ready()) {

		uint32_t accel_integral_dt;
		uint32_t gyro_integral_dt;
		Vector3f delta_angle;
		Vector3f delta_velocity;

		if (_accel_integrator.reset(delta_velocity, accel_integral_dt)
		    && _gyro_integrator.reset(delta_angle, gyro_integral_dt)) {

			if (_accel_calibration.enabled() && _gyro_calibration.enabled()) {

				// delta angle: apply offsets, scale, and board rotation
				_gyro_calibration.SensorCorrectionsUpdate();
				const float gyro_dt_inv = 1.e6f / gyro_integral_dt;
				const Vector3f delta_angle_corrected{_gyro_calibration.Correct(delta_angle * gyro_dt_inv) / gyro_dt_inv};

				// delta velocity: apply offsets, scale, and board rotation
				_accel_calibration.SensorCorrectionsUpdate();
				const float accel_dt_inv = 1.e6f / accel_integral_dt;
				Vector3f delta_velocity_corrected{_accel_calibration.Correct(delta_velocity * accel_dt_inv) / accel_dt_inv};

				UpdateAccelVibrationMetrics(delta_velocity_corrected);
				UpdateGyroVibrationMetrics(delta_angle_corrected);

				// vehicle_imu_status
				//  publish before vehicle_imu so that error counts are available synchronously if needed
				if (_publish_status || (hrt_elapsed_time(&_status.timestamp) >= 100_ms)) {
					_status.accel_device_id = _accel_calibration.device_id();
					_status.gyro_device_id = _gyro_calibration.device_id();

					// mean accel
					const Vector3f accel_mean{_accel_calibration.Correct(_accel_sum / _accel_sum_count)};
					accel_mean.copyTo(_status.mean_accel);
					_status.temperature_accel = _accel_temperature / _accel_temperature_count;
					_accel_sum.zero();
					_accel_sum_count = 0;
					_accel_temperature = 0;
					_accel_temperature_count = 0;

					// mean gyro
					const Vector3f gyro_mean{_gyro_calibration.Correct(_gyro_sum / _gyro_sum_count)};
					gyro_mean.copyTo(_status.mean_gyro);
					_status.temperature_gyro = _gyro_temperature / _gyro_temperature_count;
					_gyro_sum.zero();
					_gyro_sum_count = 0;
					_gyro_temperature = 0;
					_gyro_temperature_count = 0;

					_status.timestamp = hrt_absolute_time();
					_vehicle_imu_status_pub.publish(_status);

					_publish_status = false;
				}


				// publish vehicle_imu
				vehicle_imu_s imu;
				imu.timestamp_sample = _last_timestamp_sample_gyro;
				imu.accel_device_id = _accel_calibration.device_id();
				imu.gyro_device_id = _gyro_calibration.device_id();
				delta_angle_corrected.copyTo(imu.delta_angle);
				delta_velocity_corrected.copyTo(imu.delta_velocity);
				imu.delta_angle_dt = gyro_integral_dt;
				imu.delta_velocity_dt = accel_integral_dt;
				imu.delta_velocity_clipping = _delta_velocity_clipping;
				imu.calibration_count = _accel_calibration.calibration_count() + _gyro_calibration.calibration_count();
				imu.timestamp = hrt_absolute_time();
				_vehicle_imu_pub.publish(imu);
			}

			// reset clip counts
			_delta_velocity_clipping = 0;

			return;
		}
	}
}

void VehicleImuFifo::UpdateIntegratorConfiguration()
{
	if ((_accel_interval.update_interval > 0) && (_gyro_interval.update_interval > 0)) {

		const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();

		// determine number of sensor samples that will get closest to the desired integration interval
		const uint8_t accel_pub_integral_samples = math::max(1.f,
				roundf(configured_interval_us / _accel_interval.update_interval));
		const uint8_t gyro_pub_integral_samples = math::max(1.f,
				roundf(configured_interval_us / _gyro_interval.update_interval));

		const uint8_t accel_sample_integral_samples = math::max(1.f,
				roundf(configured_interval_us / _accel_interval.sample_interval));
		const uint8_t gyro_sample_integral_samples = math::max(1.f,
				roundf(configured_interval_us / _gyro_interval.sample_interval));

		// let the gyro set the configuration and scheduling
		// accel integrator will be forced to reset when gyro integrator is ready
		_gyro_integrator.set_reset_samples(gyro_sample_integral_samples);
		_accel_integrator.set_reset_samples(accel_sample_integral_samples);

		// relaxed minimum integration time required
		_accel_integrator.set_reset_interval(roundf((accel_sample_integral_samples - 0.5f) * _accel_interval.sample_interval));
		_gyro_integrator.set_reset_interval(roundf((gyro_sample_integral_samples - 0.5f) * _gyro_interval.sample_interval));

		// gyro: find largest integer multiple of gyro_integral_samples
		for (int n = sensor_gyro_fifo_s::ORB_QUEUE_LENGTH; n > 0; n--) {
			if (gyro_pub_integral_samples % n == 0) {
				_sensor_gyro_sub.set_required_updates(n);

				// run when there are enough new gyro samples, unregister accel
				_sensor_accel_sub.unregisterCallback();

				_intervals_configured = true; // stop monitoring topic publication rates

				PX4_INFO("accel (%d), gyro (%d), accel samples: %d, gyro samples: %d, accel interval: %.1f (%.1f), gyro interval: %.1f (%.1f) sub samples: %d",
					 _accel_calibration.device_id(), _gyro_calibration.device_id(),
					 accel_pub_integral_samples, gyro_pub_integral_samples,
					 (double)_accel_interval.update_interval, (double)_accel_interval.sample_interval,
					 (double)_gyro_interval.update_interval, (double)_gyro_interval.sample_interval,
					 n);

				break;
			}
		}
	}
}

void VehicleImuFifo::UpdateAccelVibrationMetrics(const Vector3f &delta_velocity)
{
	// Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	const Vector3f delta_velocity_diff = delta_velocity - _delta_velocity_prev;
	_status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric + 0.01f * delta_velocity_diff.norm();

	_delta_velocity_prev = delta_velocity;
}

void VehicleImuFifo::UpdateGyroVibrationMetrics(const Vector3f &delta_angle)
{
	// Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	const Vector3f delta_angle_diff = delta_angle - _delta_angle_prev;
	_status.gyro_vibration_metric = 0.99f * _status.gyro_vibration_metric + 0.01f * delta_angle_diff.norm();

	// Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	const Vector3f coning_metric = delta_angle % _delta_angle_prev;
	_status.gyro_coning_vibration = 0.99f * _status.gyro_coning_vibration + 0.01f * coning_metric.norm();

	_delta_angle_prev = delta_angle;
}

void VehicleImuFifo::PrintStatus()
{
	if (_accel_calibration.device_id() == _gyro_calibration.device_id()) {
		PX4_INFO("%d - IMU ID: %d, accel interval: %.1f us, gyro interval: %.1f us", _vehicle_imu_pub.get_instance(),
			 _accel_calibration.device_id(),
			 (double)_accel_interval.update_interval, (double)_gyro_interval.update_interval);

	} else {
		PX4_INFO("%d - Accel ID: %d, interval: %.1f us, Gyro ID: %d, interval: %.1f us", _vehicle_imu_pub.get_instance(),
			 _accel_calibration.device_id(), (double)_accel_interval.update_interval, _gyro_calibration.device_id(),
			 (double)_gyro_interval.update_interval);
	}

	perf_print_counter(_accel_generation_gap_perf);
	perf_print_counter(_gyro_generation_gap_perf);
	perf_print_counter(_accel_update_perf);
	perf_print_counter(_gyro_update_perf);

	_accel_calibration.PrintStatus();
	_gyro_calibration.PrintStatus();
}

} // namespace sensors
