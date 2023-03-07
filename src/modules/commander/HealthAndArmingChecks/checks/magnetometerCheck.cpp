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

#include "magnetometerCheck.hpp"

#include <lib/sensor_calibration/Utilities.hpp>

using namespace time_literals;

void MagnetometerChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_sys_has_mag.get() == 0) {
		return;
	}

	bool had_failure = false;
	int num_enabled_and_valid_calibration = 0;

	for (int instance = 0; instance < _sensor_mag_sub.size(); instance++) {
		bool is_mag_fault = false;
		const bool is_required = instance == 0 || isMagRequired(instance, is_mag_fault);

		if (!is_required) {
			continue;
		}

		const bool exists = _sensor_mag_sub[instance].advertised();
		bool is_valid = false;
		bool is_calibration_valid = false;

		if (exists) {
			sensor_mag_s mag_data;
			is_valid = _sensor_mag_sub[instance].copy(&mag_data) && (mag_data.device_id != 0) && (mag_data.timestamp != 0)
				   && (hrt_elapsed_time(&mag_data.timestamp) < 1_s);

			if (context.status().hil_state == vehicle_status_s::HIL_STATE_ON) {
				is_calibration_valid = true;
				num_enabled_and_valid_calibration++;

			} else {
				int calibration_index = calibration::FindCurrentCalibrationIndex("MAG", mag_data.device_id);
				is_calibration_valid = (calibration_index >= 0);

				if (is_calibration_valid) {
					int priority = calibration::GetCalibrationParamInt32("MAG", "PRIO", calibration_index);

					if (priority > 0) {
						++num_enabled_and_valid_calibration;
					}
				}
			}

			reporter.setIsPresent(health_component_t::magnetometer);
		}

		const bool is_sensor_ok = is_valid && is_calibration_valid && !is_mag_fault;

		if (!is_sensor_ok) {
			if (!exists) {
				/* EVENT
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::magnetometer, events::ID("check_mag_missing"),
								events::Log::Error, "Compass sensor {1} missing", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Compass Sensor %u missing", instance);
				}

			} else if (!is_valid) {
				/* EVENT
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::magnetometer, events::ID("check_mag_no_data"),
								events::Log::Error, "No valid data from Compass {1}", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: No valid data from Compass %u", instance);
				}

			} else if (!is_calibration_valid) {
				/* EVENT
				 */
				reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::magnetometer,
								     events::ID("check_mag_not_calibrated"),
								     events::Log::Error, "Compass {1} uncalibrated", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Compass %u uncalibrated", instance);
				}

			} else if (is_mag_fault) {
				/* EVENT
				 * @description
				 * Recalibrate the compass and check the orientation.
				 */
				reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::magnetometer, events::ID("check_mag_fault"),
								     events::Log::Error, "Estimator fault due to Compass {1}", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Compass %u fault", instance);
				}
			}

			had_failure = true;
		}
	}

	if (!had_failure && !context.isArmed()) {
		consistencyCheck(context, reporter);

		if (num_enabled_and_valid_calibration < _param_sys_has_mag.get()) {
			/* EVENT
			 * @description
			 * Make sure all required sensors are working, enabled and calibrated.
			 *
			 * <profile name="dev">
			 * This check can be configured via <param>SYS_HAS_MAG</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::magnetometer,
					events::ID("check_mag_sys_has_mag_missing"),
					events::Log::Error, "Found {1} compass (required: {2})", num_enabled_and_valid_calibration, _param_sys_has_mag.get());

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Found %i compass (required: %" PRId32 ")",
						     num_enabled_and_valid_calibration, _param_sys_has_mag.get());
			}
		}
	}
}

bool MagnetometerChecks::isMagRequired(int instance, bool &mag_fault)
{
	sensor_mag_s sensor_mag;

	if (!_sensor_mag_sub[instance].copy(&sensor_mag)) {
		return false;
	}

	const uint32_t device_id = static_cast<uint32_t>(sensor_mag.device_id);

	if (device_id == 0) {
		return false;
	}

	bool is_used_by_nav = false;

	for (int i = 0; i < _estimator_status_sub.size(); i++) {
		estimator_status_s estimator_status;

		if (_estimator_status_sub[i].copy(&estimator_status) && estimator_status.mag_device_id == device_id) {
			mag_fault = estimator_status.control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT);
			is_used_by_nav = true;
			break;
		}
	}

	return is_used_by_nav;
}


void MagnetometerChecks::consistencyCheck(const Context &context, Report &reporter)
{
	if (_param_com_arm_mag_ang.get() < 0) { // Check disabled
		return;
	}

	sensor_preflight_mag_s sensors;

	if (!_sensor_preflight_mag_sub.copy(&sensors)) {
		// can happen if not advertised (yet)
		return;
	}

	// Use the difference between sensors to detect a bad calibration, orientation or magnetic interference.
	// If a single sensor is fitted, the value being checked will be zero so this check will always pass.
	if (sensors.mag_inconsistency_angle > math::radians<float>(_param_com_arm_mag_ang.get())) {
		int inconsistency_angle_deg = static_cast<int>(math::degrees<float>(sensors.mag_inconsistency_angle));
		/* EVENT
		 * @description
		 * Check the compass orientations and recalibrate.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_MAG_ANG</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<int16_t>(NavModes::All, health_component_t::magnetometer,
						     events::ID("check_mag_consistency"),
						     events::Log::Error, "Compass inconsistent by {1} degrees", inconsistency_angle_deg);

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Compasses %d° inconsistent",
					     inconsistency_angle_deg);
		}
	}
}
