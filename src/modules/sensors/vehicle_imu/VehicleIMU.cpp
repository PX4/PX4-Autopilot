/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>

#include <float.h>

using namespace matrix;

using math::constrain;

namespace sensors
{

VehicleIMU::VehicleIMU(int instance, uint8_t accel_index, uint8_t gyro_index, const px4::wq_config_t &config) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_sensor_accel_sub(ORB_ID(sensor_accel), accel_index),
	_sensor_gyro_sub(this, ORB_ID(sensor_gyro), gyro_index),
	_instance(instance)
{
	_imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();

	_accel_integrator.set_reset_interval(_imu_integration_interval_us);
	_accel_integrator.set_reset_samples(sensor_accel_s::ORB_QUEUE_LENGTH);

	_gyro_integrator.set_reset_interval(_imu_integration_interval_us);
	_gyro_integrator.set_reset_samples(sensor_gyro_s::ORB_QUEUE_LENGTH);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// currently with lockstep every raw sample needs a corresponding vehicle_imu publication
	_sensor_gyro_sub.set_required_updates(1);
#else
	// schedule conservatively until the actual accel & gyro rates are known
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);
#endif

	// advertise immediately to ensure consistent ordering
	_vehicle_imu_pub.advertise();
	_vehicle_imu_status_pub.advertise();
}

VehicleIMU::~VehicleIMU()
{
	Stop();

	perf_free(_accel_generation_gap_perf);
	perf_free(_gyro_generation_gap_perf);

	_vehicle_imu_pub.unadvertise();
	_vehicle_imu_status_pub.unadvertise();
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_gyro_sub.registerCallback();
	ScheduleNow();
	return true;
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

bool VehicleIMU::ParametersUpdate(bool force)
{
	bool updated = false;

	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const auto imu_integ_rate_prev = _param_imu_integ_rate.get();

		updateParams();

		updated = true;

		const auto accel_calibration_count = _accel_calibration.calibration_count();
		const auto gyro_calibration_count = _gyro_calibration.calibration_count();
		_accel_calibration.ParametersUpdate();
		_gyro_calibration.ParametersUpdate();

		if (accel_calibration_count != _accel_calibration.calibration_count()) {
			// if calibration changed reset any existing learned calibration
			_accel_cal_available = false;
			_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;

			for (auto &learned_cal : _accel_learned_calibration) {
				learned_cal = {};
			}
		}

		if (gyro_calibration_count != _gyro_calibration.calibration_count()) {
			// if calibration changed reset any existing learned calibration
			_gyro_cal_available = false;
			_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;

			for (auto &learned_cal : _gyro_learned_calibration) {
				learned_cal = {};
			}
		}

		// constrain IMU integration time 1-20 milliseconds (50-1000 Hz)
		const int32_t imu_integration_rate_hz = constrain(_param_imu_integ_rate.get(), (int32_t)50, (int32_t)1000);

		if (imu_integration_rate_hz != _param_imu_integ_rate.get()) {
			PX4_WARN("IMU_INTEG_RATE updated %" PRId32 " -> %" PRIu32, _param_imu_integ_rate.get(), imu_integration_rate_hz);
			_param_imu_integ_rate.set(imu_integration_rate_hz);
			_param_imu_integ_rate.commit_no_notification();
		}

		_imu_integration_interval_us = 1e6f / imu_integration_rate_hz;

		if (_param_imu_integ_rate.get() != imu_integ_rate_prev) {
			// force update
			_update_integrator_config = true;
		}
	}

	return updated;
}

void VehicleIMU::Run()
{
	const hrt_abstime now_us = hrt_absolute_time();

	const bool parameters_updated = ParametersUpdate();

	if (!_accel_calibration.enabled() || !_gyro_calibration.enabled()) {
		_sensor_gyro_sub.unregisterCallback();
		ScheduleDelayed(1_s);
		return;
	}

	// backup schedule
	ScheduleDelayed(_backup_schedule_timeout_us);

	// check vehicle status for changes to armed state
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	// reset data gap monitor
	_data_gap = false;

	while (_sensor_gyro_sub.updated() || _sensor_accel_sub.updated()) {
		bool updated = false;

		bool consume_all_gyro = !_intervals_configured || _data_gap;

		// monitor scheduling latency and force catch up with latest gyro if falling behind
		if (_sensor_gyro_sub.updated() && (_gyro_update_latency_mean.count() > 100)
		    && (_gyro_update_latency_mean.mean()(1) > _gyro_interval_us * 1e-6f)) {

			PX4_DEBUG("gyro update mean sample latency: %.6f, publish latency %.6f",
				  (double)_gyro_update_latency_mean.mean()(0),
				  (double)_gyro_update_latency_mean.mean()(1));

			consume_all_gyro = true;
		}

		// update gyro until integrator ready and not falling behind
		if (!_gyro_integrator.integral_ready() || consume_all_gyro) {
			if (UpdateGyro()) {
				updated = true;
			}
		}


		// update accel until integrator ready and caught up to gyro
		while (_sensor_accel_sub.updated()
		       && (!_accel_integrator.integral_ready() || !_intervals_configured || _data_gap
			   || (_accel_timestamp_sample_last < (_gyro_timestamp_sample_last - 0.5f * _accel_interval_us)))) {

			if (UpdateAccel()) {
				updated = true;
			}
		}

		// reconfigure integrators if calculated sensor intervals have changed
		if (_update_integrator_config || !_intervals_configured) {
			UpdateIntegratorConfiguration();
		}

		// check for additional updates and that we're fully caught up before publishing
		if ((consume_all_gyro || _data_gap) && _sensor_gyro_sub.updated()) {
			continue;
		}

		// publish if both accel & gyro integrators are ready
		if (_intervals_configured && _accel_integrator.integral_ready() && _gyro_integrator.integral_ready()) {
			if (Publish()) {
				// record gyro publication latency and integrated samples
				if (_gyro_update_latency_mean.count() > 10000) {
					// reset periodically to avoid numerical issues
					_gyro_update_latency_mean.reset();
				}

				const float time_run_s = now_us * 1e-6f;

				_gyro_update_latency_mean.update(Vector2f{time_run_s - _gyro_timestamp_sample_last * 1e-6f, time_run_s - _gyro_timestamp_last * 1e-6f});

				break;
			}
		}

		// finish if there are no more updates, but didn't publish
		if (!updated) {
			break;
		}
	}

	if (_param_sens_imu_autocal.get() && !parameters_updated) {
		if ((_armed || !_accel_calibration.calibrated() || !_gyro_calibration.calibrated())
		    && (now_us > _in_flight_calibration_check_timestamp_last + 1_s)) {

			SensorCalibrationUpdate();
			_in_flight_calibration_check_timestamp_last = now_us;

		} else if (!_armed) {
			SensorCalibrationSaveAccel();
			SensorCalibrationSaveGyro();
		}
	}
}

bool VehicleIMU::UpdateAccel()
{
	bool updated = false;

	// integrate queued accel
	sensor_accel_s accel;

	if (_sensor_accel_sub.update(&accel)) {
		if (_sensor_accel_sub.get_last_generation() != _accel_last_generation + 1) {
			_data_gap = true;
			perf_count(_accel_generation_gap_perf);

			// reset average sample measurement
			_accel_interval_mean.reset();

		} else {
			// collect sample interval average for filters
			if (accel.timestamp_sample > _accel_timestamp_sample_last) {
				if ((_accel_timestamp_sample_last != 0) && (accel.samples > 0)) {
					float interval_us = accel.timestamp_sample - _accel_timestamp_sample_last;
					_accel_interval_mean.update(Vector2f{interval_us, interval_us / accel.samples});
				}

			} else {
				PX4_ERR("%d - accel %" PRIu32 " timestamp error timestamp_sample: %" PRIu64 ", previous timestamp_sample: %" PRIu64,
					_instance, accel.device_id, accel.timestamp_sample, _accel_timestamp_sample_last);
			}

			if (accel.timestamp < accel.timestamp_sample) {
				PX4_ERR("%d - accel %" PRIu32 " timestamp (%" PRIu64 ") < timestamp_sample (%" PRIu64 ")",
					_instance, accel.device_id, accel.timestamp, accel.timestamp_sample);
			}

			const int interval_count = _accel_interval_mean.count();
			const float interval_variance = _accel_interval_mean.variance()(0);

			// check measured interval periodically
			if ((_accel_interval_mean.valid() && (interval_count % 10 == 0))
			    && (!PX4_ISFINITE(_accel_interval_best_variance)
				|| (interval_variance < _accel_interval_best_variance)
				|| (interval_count > 1000))) {

				const float interval_mean = _accel_interval_mean.mean()(0);
				const float interval_mean_fifo = _accel_interval_mean.mean()(1);

				// update sample rate if previously invalid or changed
				const float interval_delta_us = fabsf(interval_mean - _accel_interval_us);
				const float percent_changed = interval_delta_us / _accel_interval_us;

				if (!PX4_ISFINITE(_accel_interval_us) || (percent_changed > 0.001f)) {
					if (PX4_ISFINITE(interval_mean) && PX4_ISFINITE(interval_mean_fifo) && PX4_ISFINITE(interval_variance)) {
						// update integrator configuration if interval has changed by more than 10%
						if (interval_delta_us > 0.1f * _accel_interval_us) {
							_update_integrator_config = true;
						}

						_accel_interval_us = interval_mean;
						_accel_interval_best_variance = interval_variance;

						_status.accel_rate_hz = 1e6f / interval_mean;
						_status.accel_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO
						_publish_status = true;

					} else {
						_accel_interval_mean.reset();
					}
				}
			}

			if (interval_count > 10000) {
				// reset periodically to prevent numerical issues
				_accel_interval_mean.reset();
			}
		}

		_accel_last_generation = _sensor_accel_sub.get_last_generation();

		_accel_calibration.set_device_id(accel.device_id);

		if (accel.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = accel.error_count;
		}


		// temperature average
		if (_accel_temperature_sum_count == 0) {
			_accel_temperature_sum = accel.temperature;

		} else {
			_accel_temperature_sum += accel.temperature;
		}

		_accel_temperature_sum_count++;


		const float dt = (accel.timestamp_sample - _accel_timestamp_sample_last) * 1e-6f;
		_accel_timestamp_sample_last = accel.timestamp_sample;

		const Vector3f accel_raw{accel.x, accel.y, accel.z};
		_raw_accel_mean.update(accel_raw);
		_accel_integrator.put(accel_raw, dt);

		updated = true;

		if (accel.clip_counter[0] > 0 || accel.clip_counter[1] > 0 || accel.clip_counter[2] > 0) {
			// rotate sensor clip counts into vehicle body frame
			const Vector3f clipping{_accel_calibration.rotation() *
						Vector3f{(float)accel.clip_counter[0], (float)accel.clip_counter[1], (float)accel.clip_counter[2]}};

			// round to get reasonble clip counts per axis (after board rotation)
			const uint8_t clip_x = roundf(fabsf(clipping(0)));
			const uint8_t clip_y = roundf(fabsf(clipping(1)));
			const uint8_t clip_z = roundf(fabsf(clipping(2)));

			_status.accel_clipping[0] += clip_x;
			_status.accel_clipping[1] += clip_y;
			_status.accel_clipping[2] += clip_z;

			if (clip_x > 0) {
				_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_X;
			}

			if (clip_y > 0) {
				_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Y;
			}

			if (clip_z > 0) {
				_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Z;
			}

			_publish_status = true;

			if (_accel_calibration.enabled() && (hrt_elapsed_time(&_last_accel_clipping_notify_time) > 3_s)) {
				// start notifying the user periodically if there's significant continuous clipping
				const uint64_t clipping_total = _status.accel_clipping[0] + _status.accel_clipping[1] + _status.accel_clipping[2];

				if (clipping_total > _last_accel_clipping_notify_total_count + 1000) {
					mavlink_log_critical(&_mavlink_log_pub, "Accel %" PRIu8 " clipping, not safe to fly!\t", _instance);
					/* EVENT
					 * @description Land now, and check the vehicle setup.
					 * Clipping can lead to fly-aways.
					 */
					events::send<uint8_t>(events::ID("vehicle_imu_accel_clipping"), events::Log::Critical,
							      "Accel {1} clipping, not safe to fly!", _instance);
					_last_accel_clipping_notify_time = accel.timestamp_sample;
					_last_accel_clipping_notify_total_count = clipping_total;
				}
			}
		}
	}

	return updated;
}

bool VehicleIMU::UpdateGyro()
{
	bool updated = false;

	// integrate queued gyro
	sensor_gyro_s gyro;

	if (_sensor_gyro_sub.update(&gyro)) {
		if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
			_data_gap = true;
			perf_count(_gyro_generation_gap_perf);

			// reset average sample measurement
			_gyro_interval_mean.reset();

		} else {
			// collect sample interval average for filters
			if (gyro.timestamp_sample > _gyro_timestamp_sample_last) {
				if ((_gyro_timestamp_sample_last != 0) && (gyro.samples > 0)) {
					float interval_us = gyro.timestamp_sample - _gyro_timestamp_sample_last;
					_gyro_interval_mean.update(Vector2f{interval_us, interval_us / gyro.samples});
				}

			} else {
				PX4_ERR("%d - gyro %" PRIu32 " timestamp error timestamp_sample: %" PRIu64 ", previous timestamp_sample: %" PRIu64,
					_instance, gyro.device_id, gyro.timestamp_sample, _gyro_timestamp_sample_last);
			}

			if (gyro.timestamp < gyro.timestamp_sample) {
				PX4_ERR("%d - gyro %" PRIu32 " timestamp (%" PRIu64 ") < timestamp_sample (%" PRIu64 ")",
					_instance, gyro.device_id, gyro.timestamp, gyro.timestamp_sample);
			}

			const int interval_count = _gyro_interval_mean.count();
			const float interval_variance = _gyro_interval_mean.variance()(0);

			// check measured interval periodically
			if ((_gyro_interval_mean.valid() && (interval_count % 10 == 0))
			    && (!PX4_ISFINITE(_gyro_interval_best_variance)
				|| (interval_variance < _gyro_interval_best_variance)
				|| (interval_count > 1000))) {

				const float interval_mean = _gyro_interval_mean.mean()(0);
				const float interval_mean_fifo = _gyro_interval_mean.mean()(1);

				// update sample rate if previously invalid or changed
				const float interval_delta_us = fabsf(interval_mean - _gyro_interval_us);
				const float percent_changed = interval_delta_us / _gyro_interval_us;

				if (!PX4_ISFINITE(_gyro_interval_us) || (percent_changed > 0.001f)) {
					if (PX4_ISFINITE(interval_mean) && PX4_ISFINITE(interval_mean_fifo) && PX4_ISFINITE(interval_variance)) {
						// update integrator configuration if interval has changed by more than 10%
						if (interval_delta_us > 0.1f * _gyro_interval_us) {
							_update_integrator_config = true;
						}

						_gyro_interval_us = interval_mean;
						_gyro_interval_best_variance = interval_variance;

						_status.gyro_rate_hz = 1e6f / interval_mean;
						_status.gyro_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO
						_publish_status = true;

					} else {
						_gyro_interval_mean.reset();
					}
				}
			}

			if (interval_count > 10000) {
				// reset periodically to prevent numerical issues
				_gyro_interval_mean.reset();
			}
		}

		_gyro_last_generation = _sensor_gyro_sub.get_last_generation();
		_gyro_timestamp_last = gyro.timestamp;

		_gyro_calibration.set_device_id(gyro.device_id);

		if (gyro.error_count != _status.gyro_error_count) {
			_publish_status = true;
			_status.gyro_error_count = gyro.error_count;
		}


		// temperature average
		if (_gyro_temperature_sum_count == 0) {
			_gyro_temperature_sum = gyro.temperature;

		} else {
			_gyro_temperature_sum += gyro.temperature;
		}

		_gyro_temperature_sum_count++;


		const float dt = (gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
		_gyro_timestamp_sample_last = gyro.timestamp_sample;

		const Vector3f gyro_raw{gyro.x, gyro.y, gyro.z};
		_raw_gyro_mean.update(gyro_raw);
		_gyro_integrator.put(gyro_raw, dt);

		updated = true;

		if (gyro.clip_counter[0] > 0 || gyro.clip_counter[1] > 0 || gyro.clip_counter[2] > 0) {
			// rotate sensor clip counts into vehicle body frame
			const Vector3f clipping{_gyro_calibration.rotation() *
						Vector3f{(float)gyro.clip_counter[0], (float)gyro.clip_counter[1], (float)gyro.clip_counter[2]}};

			// round to get reasonble clip counts per axis (after board rotation)
			const uint8_t clip_x = roundf(fabsf(clipping(0)));
			const uint8_t clip_y = roundf(fabsf(clipping(1)));
			const uint8_t clip_z = roundf(fabsf(clipping(2)));

			_status.gyro_clipping[0] += clip_x;
			_status.gyro_clipping[1] += clip_y;
			_status.gyro_clipping[2] += clip_z;

			if (clip_x > 0) {
				_delta_angle_clipping |= vehicle_imu_s::CLIPPING_X;
			}

			if (clip_y > 0) {
				_delta_angle_clipping |= vehicle_imu_s::CLIPPING_Y;
			}

			if (clip_z > 0) {
				_delta_angle_clipping |= vehicle_imu_s::CLIPPING_Z;
			}

			_publish_status = true;

			if (_gyro_calibration.enabled() && (hrt_elapsed_time(&_last_gyro_clipping_notify_time) > 3_s)) {
				// start notifying the user periodically if there's significant continuous clipping
				const uint64_t clipping_total = _status.gyro_clipping[0] + _status.gyro_clipping[1] + _status.gyro_clipping[2];

				if (clipping_total > _last_gyro_clipping_notify_total_count + 1000) {
					mavlink_log_critical(&_mavlink_log_pub, "Gyro %" PRIu8 " clipping, not safe to fly!\t", _instance);
					/* EVENT
					 * @description Land now, and check the vehicle setup.
					 * Clipping can lead to fly-aways.
					 */
					events::send<uint8_t>(events::ID("vehicle_imu_gyro_clipping"), events::Log::Critical,
							      "Gyro {1} clipping, not safe to fly!", _instance);
					_last_gyro_clipping_notify_time = gyro.timestamp_sample;
					_last_gyro_clipping_notify_total_count = clipping_total;
				}
			}
		}
	}

	return updated;
}

bool VehicleIMU::Publish()
{
	bool updated = false;

	vehicle_imu_s imu;
	Vector3f delta_angle;
	Vector3f delta_velocity;

	const Vector3f accumulated_coning_corrections = _gyro_integrator.accumulated_coning_corrections();

	if (_accel_integrator.reset(delta_velocity, imu.delta_velocity_dt)
	    && _gyro_integrator.reset(delta_angle, imu.delta_angle_dt)) {

		if (_accel_calibration.enabled() && _gyro_calibration.enabled()) {

			// delta angle: apply offsets, scale, and board rotation
			_gyro_calibration.SensorCorrectionsUpdate();
			const float gyro_dt_s = 1.e-6f * imu.delta_angle_dt;
			const Vector3f angular_velocity{_gyro_calibration.Correct(delta_angle / gyro_dt_s)};
			UpdateGyroVibrationMetrics(angular_velocity);
			const Vector3f delta_angle_corrected{angular_velocity * gyro_dt_s};

			// accumulate delta angle coning corrections
			_coning_norm_accum += accumulated_coning_corrections.norm() * gyro_dt_s;
			_coning_norm_accum_total_time_s += gyro_dt_s;


			// delta velocity: apply offsets, scale, and board rotation
			_accel_calibration.SensorCorrectionsUpdate();
			const float accel_dt_s = 1.e-6f * imu.delta_velocity_dt;
			const Vector3f acceleration{_accel_calibration.Correct(delta_velocity / accel_dt_s)};
			UpdateAccelVibrationMetrics(acceleration);
			const Vector3f delta_velocity_corrected{acceleration * accel_dt_s};

			// vehicle_imu_status
			//  publish before vehicle_imu so that error counts are available synchronously if needed
			if (_raw_accel_mean.valid() && _raw_gyro_mean.valid()
			    && (_publish_status || (hrt_elapsed_time(&_status.timestamp) >= 100_ms))) {

				_status.accel_device_id = _accel_calibration.device_id();

				// accel mean and variance
				Vector3f(_accel_calibration.rotation() * _raw_accel_mean.mean()).copyTo(_status.mean_accel);
				Vector3f(_accel_calibration.rotation() * _raw_accel_mean.variance()).copyTo(_status.var_accel);
				_raw_accel_mean.reset();

				// accel temperature
				_status.temperature_accel = _accel_temperature_sum / _accel_temperature_sum_count;
				_accel_temperature_sum = NAN;
				_accel_temperature_sum_count = 0;


				_status.gyro_device_id = _gyro_calibration.device_id();

				// gyro mean and variance
				Vector3f(_gyro_calibration.rotation() * _raw_gyro_mean.mean()).copyTo(_status.mean_gyro);
				Vector3f(_gyro_calibration.rotation() * _raw_gyro_mean.variance()).copyTo(_status.var_gyro);
				_raw_gyro_mean.reset();

				// Gyro delta angle coning metric = length of coning corrections averaged since last status publication
				_status.delta_angle_coning_metric = _coning_norm_accum / _coning_norm_accum_total_time_s;
				_coning_norm_accum = 0;
				_coning_norm_accum_total_time_s = 0;

				// gyro temperature
				_status.temperature_gyro = _gyro_temperature_sum / _gyro_temperature_sum_count;
				_gyro_temperature_sum = NAN;
				_gyro_temperature_sum_count = 0;


				_status.timestamp = hrt_absolute_time();
				_vehicle_imu_status_pub.publish(_status);

				_publish_status = false;
			}

			// publish vehicle_imu
			imu.timestamp_sample = _gyro_timestamp_sample_last;
			imu.accel_device_id = _accel_calibration.device_id();
			imu.gyro_device_id = _gyro_calibration.device_id();
			delta_angle_corrected.copyTo(imu.delta_angle);
			delta_velocity_corrected.copyTo(imu.delta_velocity);
			imu.delta_velocity_clipping = _delta_velocity_clipping;
			imu.accel_calibration_count = _accel_calibration.calibration_count();
			imu.gyro_calibration_count = _gyro_calibration.calibration_count();
			imu.timestamp = hrt_absolute_time();
			_vehicle_imu_pub.publish(imu);

			// reset clip counts
			_delta_velocity_clipping = 0;

			updated = true;
		}
	}

	return updated;
}

void VehicleIMU::UpdateIntegratorConfiguration()
{
	if (PX4_ISFINITE(_accel_interval_us) && PX4_ISFINITE(_gyro_interval_us)) {

		// determine number of sensor samples that will get closest to the desired integration interval
		uint8_t gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _gyro_interval_us));

		// if gyro samples exceeds queue depth, instead round to nearest even integer to improve scheduling options
		if (gyro_integral_samples > sensor_gyro_s::ORB_QUEUE_LENGTH) {
			gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _gyro_interval_us / 2) * 2);
		}

		uint32_t integration_interval_us = roundf(gyro_integral_samples * _gyro_interval_us);

		// accel follows gyro as closely as possible
		uint8_t accel_integral_samples = math::max(1, (int)roundf(integration_interval_us / _accel_interval_us));

		// let the gyro set the configuration and scheduling
		// relaxed minimum integration time required
		_accel_integrator.set_reset_interval(roundf((accel_integral_samples - 0.5f) * _accel_interval_us));
		_accel_integrator.set_reset_samples(accel_integral_samples);

		_gyro_integrator.set_reset_interval(roundf((gyro_integral_samples - 0.5f) * _gyro_interval_us));
		_gyro_integrator.set_reset_samples(gyro_integral_samples);

		_backup_schedule_timeout_us = math::constrain((int)math::min(sensor_accel_s::ORB_QUEUE_LENGTH * _accel_interval_us,
					      sensor_gyro_s::ORB_QUEUE_LENGTH * _gyro_interval_us) / 2, 1000, 20000);

		// gyro: find largest integer multiple of gyro_integral_samples
		for (int n = sensor_gyro_s::ORB_QUEUE_LENGTH; n > 0; n--) {
			if (gyro_integral_samples > sensor_gyro_s::ORB_QUEUE_LENGTH) {
				gyro_integral_samples /= 2;
			}

			if (gyro_integral_samples % n == 0) {
				_sensor_gyro_sub.set_required_updates(n);
				_sensor_gyro_sub.registerCallback();

				_intervals_configured = true;
				_update_integrator_config = false;

				PX4_DEBUG("accel (%" PRIu32 "), gyro (%" PRIu32 "), accel samples: %" PRIu8 ", gyro samples: %" PRIu8
					  ", accel interval: %.1f, gyro interval: %.1f sub samples: %d",
					  _accel_calibration.device_id(), _gyro_calibration.device_id(), accel_integral_samples, gyro_integral_samples,
					  (double)_accel_interval_us, (double)_gyro_interval_us, n);

				break;
			}
		}
	}
}

void VehicleIMU::UpdateAccelVibrationMetrics(const Vector3f &acceleration)
{
	// Accel high frequency vibe = filtered length of (acceleration - acceleration_prev)
	_status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric
					 + 0.01f * Vector3f(acceleration - _acceleration_prev).norm();

	_acceleration_prev = acceleration;
}

void VehicleIMU::UpdateGyroVibrationMetrics(const Vector3f &angular_velocity)
{
	// Gyro high frequency vibe = filtered length of (angular_velocity - angular_velocity_prev)
	_status.gyro_vibration_metric = 0.99f * _status.gyro_vibration_metric
					+ 0.01f * Vector3f(angular_velocity - _angular_velocity_prev).norm();

	_angular_velocity_prev = angular_velocity;
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_imu] %" PRIu8 " - Accel: %" PRIu32 ", interval: %.1f us (SD %.1f us), Gyro: %" PRIu32
		     ", interval: %.1f us (SD %.1f us)\n",
		     _instance,
		     _accel_calibration.device_id(), (double)_accel_interval_us, (double)sqrtf(_accel_interval_best_variance),
		     _gyro_calibration.device_id(), (double)_gyro_interval_us, (double)sqrtf(_gyro_interval_best_variance));

	PX4_DEBUG("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
		  (double)_gyro_update_latency_mean.mean()(0), (double)_gyro_update_latency_mean.mean()(1),
		  (double)(_gyro_interval_us * 1e-6f));

	perf_print_counter(_accel_generation_gap_perf);
	perf_print_counter(_gyro_generation_gap_perf);

	_accel_calibration.PrintStatus();
	_gyro_calibration.PrintStatus();
}

void VehicleIMU::SensorCalibrationUpdate()
{
	for (int i = 0; i < _estimator_sensor_bias_subs.size(); i++) {
		estimator_sensor_bias_s estimator_sensor_bias;

		if (_estimator_sensor_bias_subs[i].update(&estimator_sensor_bias)
		    && (hrt_elapsed_time(&estimator_sensor_bias.timestamp) < 1_s)) {

			// find corresponding accel bias
			if (estimator_sensor_bias.accel_bias_valid && estimator_sensor_bias.accel_bias_stable
			    && (estimator_sensor_bias.accel_device_id != 0)
			    && (estimator_sensor_bias.accel_device_id == _accel_calibration.device_id())) {

				const Vector3f bias{estimator_sensor_bias.accel_bias};

				_accel_learned_calibration[i].offset = _accel_calibration.BiasCorrectedSensorOffset(bias);
				_accel_learned_calibration[i].bias_variance = Vector3f{estimator_sensor_bias.accel_bias_variance};
				_accel_learned_calibration[i].valid = true;
				_accel_cal_available = true;
			}

			// find corresponding gyro calibration
			if (estimator_sensor_bias.gyro_bias_valid && estimator_sensor_bias.gyro_bias_stable
			    && (estimator_sensor_bias.gyro_device_id != 0)
			    && (estimator_sensor_bias.gyro_device_id == _gyro_calibration.device_id())) {

				const Vector3f bias{estimator_sensor_bias.gyro_bias};

				_gyro_learned_calibration[i].offset = _gyro_calibration.BiasCorrectedSensorOffset(bias);
				_gyro_learned_calibration[i].bias_variance = Vector3f{estimator_sensor_bias.gyro_bias_variance};
				_gyro_learned_calibration[i].valid = true;
				_gyro_cal_available = true;
			}
		}
	}
}

void VehicleIMU::SensorCalibrationSaveAccel()
{
	if (_accel_cal_available) {
		const Vector3f cal_orig{_accel_calibration.offset()};
		bool initialised = false;
		Vector3f offset_estimate{};
		Vector3f bias_variance{};

		// apply all valid saved offsets
		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_accel_learned_calibration[i].valid) {
				if (!initialised) {
					bias_variance = _accel_learned_calibration[i].bias_variance;
					offset_estimate = _accel_learned_calibration[i].offset;
					initialised = true;

				} else {
					for (int axis_index = 0; axis_index < 3; axis_index++) {
						const float sum_of_variances = _accel_learned_calibration[i].bias_variance(axis_index) + bias_variance(axis_index);
						const float k1 = bias_variance(axis_index) / sum_of_variances;
						const float k2 = _accel_learned_calibration[i].bias_variance(axis_index) / sum_of_variances;
						offset_estimate(axis_index) = k2 * offset_estimate(axis_index) + k1 * _accel_learned_calibration[i].offset(axis_index);
						bias_variance(axis_index) *= k2;
					}
				}

				// reset
				_accel_learned_calibration[i] = {};
			}
		}

		if (initialised && ((cal_orig - offset_estimate).longerThan(0.05f) || !_accel_calibration.calibrated())) {
			if (_accel_calibration.set_offset(offset_estimate)) {
				PX4_INFO("%s %d (%" PRIu32 ") offset committed: [%.3f %.3f %.3f]->[%.3f %.3f %.3f])",
					 _accel_calibration.SensorString(), _instance, _accel_calibration.device_id(),
					 (double)cal_orig(0), (double)cal_orig(1), (double)cal_orig(2),
					 (double)offset_estimate(0), (double)offset_estimate(1), (double)offset_estimate(2));

				// save parameters with preferred calibration slot to current sensor index
				if (_accel_calibration.ParametersSave(_sensor_accel_sub.get_instance())) {
					param_notify_changes();
				}

				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;
			}
		}

		// reset
		_accel_cal_available = false;
	}
}

void VehicleIMU::SensorCalibrationSaveGyro()
{
	if (_gyro_cal_available) {
		const Vector3f cal_orig{_gyro_calibration.offset()};
		bool initialised = false;
		Vector3f offset_estimate{};
		Vector3f bias_variance{};

		// apply all valid saved offsets
		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_gyro_learned_calibration[i].valid) {
				if (!initialised) {
					bias_variance = _gyro_learned_calibration[i].bias_variance;
					offset_estimate = _gyro_learned_calibration[i].offset;
					initialised = true;

				} else {
					for (int axis_index = 0; axis_index < 3; axis_index++) {
						const float sum_of_variances = _gyro_learned_calibration[i].bias_variance(axis_index) + bias_variance(axis_index);
						const float k1 = bias_variance(axis_index) / sum_of_variances;
						const float k2 = _gyro_learned_calibration[i].bias_variance(axis_index) / sum_of_variances;
						offset_estimate(axis_index) = k2 * offset_estimate(axis_index) + k1 * _gyro_learned_calibration[i].offset(axis_index);
						bias_variance(axis_index) *= k2;
					}
				}

				// reset
				_gyro_learned_calibration[i] = {};
			}
		}

		if (initialised && ((cal_orig - offset_estimate).longerThan(0.01f) || !_gyro_calibration.calibrated())) {
			if (_gyro_calibration.set_offset(offset_estimate)) {
				PX4_INFO("%s %d (%" PRIu32 ") offset committed: [%.3f %.3f %.3f]->[%.3f %.3f %.3f])",
					 _gyro_calibration.SensorString(), _instance, _gyro_calibration.device_id(),
					 (double)cal_orig(0), (double)cal_orig(1), (double)cal_orig(2),
					 (double)offset_estimate(0), (double)offset_estimate(1), (double)offset_estimate(2));

				// save parameters with preferred calibration slot to current sensor index
				if (_gyro_calibration.ParametersSave(_sensor_gyro_sub.get_instance())) {
					param_notify_changes();
				}

				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;
			}
		}

		// reset
		_gyro_cal_available = false;
	}
}

} // namespace sensors
