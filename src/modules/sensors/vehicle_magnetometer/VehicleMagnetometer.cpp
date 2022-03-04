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

#include "VehicleMagnetometer.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <lib/geo/geo.h>
#include <lib/sensor_calibration/Utilities.hpp>

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleMagnetometer::VehicleMagnetometer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	param_find("CAL_MAG_SIDES");
	param_find("CAL_MAG_ROT_AUTO");

	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(1000);

	ParametersUpdate(true);

	_vehicle_magnetometer_pub[0].advertise();
	_sensor_preflight_mag_pub.advertise();

	// if publishing multiple mags advertise instances immediately for existing calibrations
	if (!_param_sens_mag_mode.get()) {
		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			uint32_t device_id_mag = calibration::GetCalibrationParamInt32("MAG", "ID", i);

			if (device_id_mag != 0) {
				_vehicle_magnetometer_pub[i].advertise();
			}
		}
	}
}

VehicleMagnetometer::~VehicleMagnetometer()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleMagnetometer::Start()
{
	ScheduleNow();
	return true;
}

void VehicleMagnetometer::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

bool VehicleMagnetometer::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// Mag compensation type
		MagCompensationType mag_comp_typ = static_cast<MagCompensationType>(_param_mag_comp_typ.get());

		if (mag_comp_typ != _mag_comp_type) {
			// check mag power compensation type (change battery current subscription instance if necessary)
			switch (mag_comp_typ) {
			case MagCompensationType::Current_inst0:
				_battery_status_sub.ChangeInstance(0);
				break;

			case MagCompensationType::Current_inst1:
				_battery_status_sub.ChangeInstance(1);
				break;

			case MagCompensationType::Throttle:
				break;

			default:

				// ensure power compensation is disabled
				for (auto &cal : _calibration) {
					cal.UpdatePower(0.f);
				}

				break;
			}
		}

		_mag_comp_type = mag_comp_typ;


		if (!_armed) {
			bool calibration_updated = false;

			// update mag priority (CAL_MAGx_PRIO)
			for (int mag = 0; mag < MAX_SENSOR_COUNT; mag++) {
				const auto calibration_count = _calibration[mag].calibration_count();
				const int32_t priority_old = _calibration[mag].priority();
				_calibration[mag].ParametersUpdate();
				const int32_t priority_new = _calibration[mag].priority();

				if (priority_old != priority_new) {
					if (_priority[mag] == priority_old) {
						_priority[mag] = priority_new;

					} else {
						// change relative priority to incorporate any sensor faults
						int priority_change = priority_new - priority_old;
						_priority[mag] = math::constrain(_priority[mag] + priority_change, 1, 100);
					}
				}

				if (calibration_count != _calibration[mag].calibration_count()) {
					calibration_updated = true;
				}
			}

			if (calibration_updated) {
				// clear all
				for (auto &bias : _calibration_estimator_bias) {
					bias.zero();
				}

				for (auto &cal : _mag_cal) {
					cal = {};
				}

				_in_flight_mag_cal_available = false;
				_last_calibration_update = hrt_absolute_time();
			}

		}

		return true;
	}

	return false;
}

void VehicleMagnetometer::UpdateMagBiasEstimate()
{
	if (_magnetometer_bias_estimate_sub.updated()) {
		// Continuous mag calibration is running when not armed
		magnetometer_bias_estimate_s mag_bias_est;

		if (_magnetometer_bias_estimate_sub.copy(&mag_bias_est)) {
			bool parameters_notify = false;

			for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
				if (mag_bias_est.valid[mag_index] && (mag_bias_est.timestamp > _last_calibration_update)) {

					const Vector3f bias{mag_bias_est.bias_x[mag_index],
							    mag_bias_est.bias_y[mag_index],
							    mag_bias_est.bias_z[mag_index]};

					_calibration_estimator_bias[mag_index] = bias;

					// set initial mag calibration if disarmed, mag uncalibrated, and valid estimated bias available
					if (_param_sens_mag_autocal.get() && !_armed && mag_bias_est.stable[mag_index]
					    && (_calibration[mag_index].device_id() != 0) && !_calibration[mag_index].calibrated()) {

						// set initial mag calibration
						const Vector3f offset = _calibration[mag_index].BiasCorrectedSensorOffset(_calibration_estimator_bias[mag_index]);

						if (_calibration[mag_index].set_offset(offset)) {
							// save parameters with preferred calibration slot to current sensor index
							_calibration[mag_index].ParametersSave(mag_index);

							_calibration_estimator_bias[mag_index].zero();

							parameters_notify = true;
						}
					}
				}
			}

			if (parameters_notify) {
				param_notify_changes();
				_last_calibration_update = hrt_absolute_time();
			}
		}
	}
}

void VehicleMagnetometer::UpdateMagCalibration()
{
	// State variance assumed for magnetometer bias storage.
	// This is a reference variance used to calculate the fraction of learned magnetometer bias that will be used to update the stored value.
	// Larger values cause a larger fraction of the learned biases to be used.
	static constexpr float magb_vref = 2.5e-7f;
	static constexpr float min_var_allowed = magb_vref * 0.01f;
	static constexpr float max_var_allowed = magb_vref * 100.f;

	if (_armed) {
		static constexpr uint8_t mag_cal_size = sizeof(_mag_cal) / sizeof(_mag_cal[0]);

		for (int i = 0; i < math::min(_estimator_sensor_bias_subs.size(), mag_cal_size); i++) {
			estimator_sensor_bias_s estimator_sensor_bias;

			if (_estimator_sensor_bias_subs[i].update(&estimator_sensor_bias)) {

				const Vector3f bias{estimator_sensor_bias.mag_bias};
				const Vector3f bias_variance{estimator_sensor_bias.mag_bias_variance};

				const bool valid = (hrt_elapsed_time(&estimator_sensor_bias.timestamp) < 1_s)
						   && (estimator_sensor_bias.mag_device_id != 0) &&
						   estimator_sensor_bias.mag_bias_valid &&
						   estimator_sensor_bias.mag_bias_stable &&
						   (bias_variance.min() > min_var_allowed) && (bias_variance.max() < max_var_allowed);

				if (valid) {
					// find corresponding mag calibration
					for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
						if (_calibration[mag_index].device_id() == estimator_sensor_bias.mag_device_id) {

							_mag_cal[i].device_id = estimator_sensor_bias.mag_device_id;

							// readd estimated bias that was removed before publishing vehicle_magnetometer
							_mag_cal[i].offset = _calibration[mag_index].BiasCorrectedSensorOffset(bias) +
									     _calibration_estimator_bias[mag_index];

							_mag_cal[i].variance = bias_variance;

							_in_flight_mag_cal_available = true;
							break;
						}
					}
				}
			}
		}

	} else if (_in_flight_mag_cal_available) {
		// not armed and mag cal available
		bool calibration_param_save_needed = false;
		// iterate through available bias estimates and fuse them sequentially using a Kalman Filter scheme
		Vector3f state_variance{magb_vref, magb_vref, magb_vref};

		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			// apply all valid saved offsets
			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if ((_calibration[mag_index].device_id() != 0) && (_mag_cal[i].device_id == _calibration[mag_index].device_id())) {
					const Vector3f mag_cal_orig{_calibration[mag_index].offset()};

					// calculate weighting using ratio of variances and update stored bias values
					const Vector3f &observation{_mag_cal[i].offset};
					const Vector3f &obs_variance{_mag_cal[i].variance};

					const Vector3f innovation{mag_cal_orig - observation};
					const Vector3f innovation_variance{state_variance + obs_variance};
					const Vector3f kalman_gain{state_variance.edivide(innovation_variance)};

					// new offset
					const Vector3f mag_cal_offset{mag_cal_orig - innovation.emult(kalman_gain)};

					for (int axis_index = 0; axis_index < 3; axis_index++) {
						state_variance(axis_index) = fmaxf(state_variance(axis_index) * (1.f - kalman_gain(axis_index)), 0.f);
					}

					if (_calibration[mag_index].set_offset(mag_cal_offset)) {

						PX4_INFO("%d (%" PRIu32 ") EST:%d offset: [%.2f, %.2f, %.2f]->[%.2f, %.2f, %.2f] (full [%.3f, %.3f, %.3f])",
							 mag_index, _calibration[mag_index].device_id(), i,
							 (double)mag_cal_orig(0), (double)mag_cal_orig(1), (double)mag_cal_orig(2),
							 (double)mag_cal_offset(0), (double)mag_cal_offset(1), (double)mag_cal_offset(2),
							 (double)_mag_cal[i].offset(0), (double)_mag_cal[i].offset(1), (double)_mag_cal[i].offset(2));

						_calibration[mag_index].ParametersSave();

						calibration_param_save_needed = true;
					}

				}

				// clear
				_mag_cal[i] = {};
			}

			_calibration_estimator_bias[mag_index].zero();
		}

		if (calibration_param_save_needed) {
			param_notify_changes();
			_last_calibration_update = hrt_absolute_time();
		}

		_in_flight_mag_cal_available = false;
	}
}

void VehicleMagnetometer::UpdatePowerCompensation()
{
	if (_mag_comp_type != MagCompensationType::Disabled) {
		// update power signal for mag compensation
		if (_armed && (_mag_comp_type == MagCompensationType::Throttle)) {
			actuator_controls_s controls;

			if (_actuator_controls_0_sub.update(&controls)) {
				for (auto &cal : _calibration) {
					cal.UpdatePower(controls.control[actuator_controls_s::INDEX_THROTTLE]);
				}
			}

		} else if ((_mag_comp_type == MagCompensationType::Current_inst0)
			   || (_mag_comp_type == MagCompensationType::Current_inst1)) {

			battery_status_s bat_stat;

			if (_battery_status_sub.update(&bat_stat)) {
				float power = bat_stat.current_a * 0.001f; // current in [kA]

				for (auto &cal : _calibration) {
					cal.UpdatePower(power);
				}
			}

		} else {
			for (auto &cal : _calibration) {
				cal.UpdatePower(0.f);
			}
		}
	}
}

void VehicleMagnetometer::Run()
{
	perf_begin(_cycle_perf);

	const hrt_abstime time_now_us = hrt_absolute_time();

	const bool parameter_update = ParametersUpdate();

	// check vehicle status for changes to armed state
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	UpdatePowerCompensation();

	UpdateMagBiasEstimate();

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		const bool was_advertised = _advertised[uorb_index];

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_publication_timestamp[uorb_index] == 0)
			    || (time_now_us > _last_publication_timestamp[uorb_index] + 1_s)) {

				if (_sensor_sub[uorb_index].advertised()) {
					_advertised[uorb_index] = true;

				} else {
					_last_publication_timestamp[uorb_index] = time_now_us;
				}
			}
		}

		if (_advertised[uorb_index]) {
			sensor_mag_s report;

			while (_sensor_sub[uorb_index].update(&report)) {

				if (_calibration[uorb_index].device_id() != report.device_id) {
					_calibration[uorb_index].set_device_id(report.device_id);
					_priority[uorb_index] = _calibration[uorb_index].priority();
				}

				if (_calibration[uorb_index].enabled()) {

					if (!was_advertised) {
						if (uorb_index > 0) {
							/* the first always exists, but for each further sensor, add a new validator */
							if (!_voter.add_new_validator()) {
								PX4_ERR("failed to add validator for %s %i", _calibration[uorb_index].SensorString(), uorb_index);
							}
						}

						// advertise outputs in order if publishing all
						if (!_param_sens_mag_mode.get()) {
							for (int instance = 0; instance < uorb_index; instance++) {
								_vehicle_magnetometer_pub[instance].advertise();
							}
						}

						if (_selected_sensor_sub_index < 0) {
							_sensor_sub[uorb_index].registerCallback();
						}

						ParametersUpdate(true);
					}

					const Vector3f vect{_calibration[uorb_index].Correct(Vector3f{report.x, report.y, report.z}) - _calibration_estimator_bias[uorb_index]};

					float mag_array[3] {vect(0), vect(1), vect(2)};
					_voter.put(uorb_index, report.timestamp, mag_array, report.error_count, _priority[uorb_index]);

					_timestamp_sample_sum[uorb_index] += report.timestamp_sample;
					_data_sum[uorb_index] += vect;
					_data_sum_count[uorb_index]++;

					_last_data[uorb_index] = vect;

					updated[uorb_index] = true;
				}
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(time_now_us, &best_index);

	if (best_index >= 0) {
		// handle selection change (don't process on same iteration as parameter update)
		if ((_selected_sensor_sub_index != best_index) && !parameter_update) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_mag_mode.get()) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%" PRId8 " -> #%d", _calibration[_selected_sensor_sub_index].SensorString(),
						 _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	// Publish
	if (_param_sens_mag_rate.get() > 0) {
		int interval_us = 1e6f / _param_sens_mag_rate.get();
		const bool multi_mode = (_param_sens_mag_mode.get() == 1);

		for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
			if (updated[instance] && (_data_sum_count[instance] > 0)) {

				const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _data_sum_count[instance];

				if (timestamp_sample >= _last_publication_timestamp[instance] + interval_us) {

					bool publish = (time_now_us <= timestamp_sample + 1_s);

					if (!multi_mode && publish) {
						publish = (_selected_sensor_sub_index >= 0)
							  && (instance == _selected_sensor_sub_index)
							  && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR);
					}

					if (publish) {
						const Vector3f magnetometer_data = _data_sum[instance] / _data_sum_count[instance];

						// populate vehicle_magnetometer and publish
						vehicle_magnetometer_s out{};
						out.timestamp_sample = timestamp_sample;
						out.device_id = _calibration[instance].device_id();
						magnetometer_data.copyTo(out.magnetometer_ga);
						out.calibration_count = _calibration[instance].calibration_count();
						out.timestamp = hrt_absolute_time();

						if (multi_mode) {
							_vehicle_magnetometer_pub[instance].publish(out);

						} else {
							// otherwise only ever publish the first instance
							_vehicle_magnetometer_pub[0].publish(out);
						}
					}

					_last_publication_timestamp[instance] = timestamp_sample;

					// reset
					_timestamp_sample_sum[instance] = 0;
					_data_sum[instance].zero();
					_data_sum_count[instance] = 0;
				}
			}
		}
	}

	if (!parameter_update && _param_sens_mag_mode.get()) {
		CheckFailover(time_now_us);
	}

	if (!_armed) {
		calcMagInconsistency();
	}

	UpdateMagCalibration();

	UpdateStatus();

	// reschedule timeout
	ScheduleDelayed(50_ms);

	perf_end(_cycle_perf);
}

void VehicleMagnetometer::CheckFailover(const hrt_abstime &time_now_us)
{
	// check failover and report (save failover report for a cycle where parameters didn't update)
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index >= 0 && failover_index < MAX_SENSOR_COUNT) {

				if (time_now_us > _last_error_message + 3_s) {
					mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!\t",
							      _calibration[failover_index].SensorString(),
							      failover_index,
							      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
							      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
							      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));

					events::px4::enums::sensor_failover_reason_t failover_reason{};

					if (flags & DataValidator::ERROR_FLAG_NO_DATA) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::no_data; }

					if (flags & DataValidator::ERROR_FLAG_STALE_DATA) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::stale_data; }

					if (flags & DataValidator::ERROR_FLAG_TIMEOUT) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::timeout; }

					if (flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::high_error_count; }

					if (flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::high_error_density; }

					/* EVENT
					 * @description
					 * Land immediately and check the system.
					 */
					events::send<uint8_t, events::px4::enums::sensor_failover_reason_t>(
						events::ID("sensor_failover_mag"), events::Log::Emergency, "Magnetometer sensor #{1} failure: {2}", failover_index,
						failover_reason);

					_last_error_message = time_now_us;
				}

				// reduce priority of failed sensor to the minimum
				_priority[failover_index] = 1;
			}
		}

		_last_failover_count = _voter.failover_count();
	}
}

void VehicleMagnetometer::calcMagInconsistency()
{
	if (_selected_sensor_sub_index >= 0) {
		sensor_preflight_mag_s preflt{};

		const Vector3f primary_mag(_last_data[_selected_sensor_sub_index]); // primary mag field vector

		float mag_angle_diff_max = 0.0f; // the maximum angle difference
		unsigned check_index = 0; // the number of sensors the primary has been checked against

		// Check each sensor against the primary
		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			// check that the sensor we are checking against is not the same as the primary
			if (_advertised[i] && (_priority[i] > 0) && (i != _selected_sensor_sub_index)) {
				// calculate angle to 3D magnetic field vector of the primary sensor
				Vector3f current_mag{_last_data[i]};

				float angle_error = AxisAnglef(Quatf(current_mag, primary_mag)).angle();

				// complementary filter to not fail/pass on single outliers
				_mag_angle_diff[check_index] *= 0.95f;
				_mag_angle_diff[check_index] += 0.05f * angle_error;

				mag_angle_diff_max = math::max(mag_angle_diff_max, _mag_angle_diff[check_index]);

				// increment the check index
				check_index++;
			}

			// check to see if the maximum number of checks has been reached and break
			if (check_index >= 2) {
				break;
			}
		}

		// get the vector length of the largest difference and write to the combined sensor struct
		// will be zero if there is only one magnetometer and hence nothing to compare
		preflt.mag_inconsistency_angle = mag_angle_diff_max;

		preflt.timestamp = hrt_absolute_time();
		_sensor_preflight_mag_pub.publish(preflt);
	}
}

void VehicleMagnetometer::UpdateStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		sensors_status_s sensors_status{};
		sensors_status.device_id_primary = _calibration[_selected_sensor_sub_index].device_id();

		matrix::Vector3f mean{};
		int sensor_count = 0;

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if ((_calibration[sensor_index].device_id() != 0) && (_calibration[sensor_index].enabled())) {
				sensor_count++;
				mean += _last_data[sensor_index];
			}
		}

		if (sensor_count > 0) {
			mean /= sensor_count;
		}

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if (_calibration[sensor_index].device_id() != 0) {

				_sensor_diff[sensor_index] = 0.95f * _sensor_diff[sensor_index] + 0.05f * (_last_data[sensor_index] - mean);

				sensors_status.device_ids[sensor_index] = _calibration[sensor_index].device_id();
				sensors_status.inconsistency[sensor_index] = _sensor_diff[sensor_index].norm();
				sensors_status.healthy[sensor_index] = (_voter.get_sensor_state(sensor_index) == DataValidator::ERROR_FLAG_NO_ERROR);
				sensors_status.priority[sensor_index] = _voter.get_sensor_priority(sensor_index);
				sensors_status.enabled[sensor_index] = _calibration[sensor_index].enabled();
				sensors_status.external[sensor_index] = _calibration[sensor_index].external();

			} else {
				sensors_status.inconsistency[sensor_index] = NAN;
			}
		}

		sensors_status.timestamp = hrt_absolute_time();
		_sensors_status_mag_pub.publish(sensors_status);
	}
}

void VehicleMagnetometer::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO_RAW("[vehicle_magnetometer] selected %s: %" PRIu32 " (%" PRId8 ")\n",
			     _calibration[_selected_sensor_sub_index].SensorString(),
			     _calibration[_selected_sensor_sub_index].device_id(), _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_calibration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
