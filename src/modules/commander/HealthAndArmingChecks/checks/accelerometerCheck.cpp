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

#include "accelerometerCheck.hpp"

#include <lib/sensor_calibration/Utilities.hpp>

using namespace time_literals;

void AccelerometerChecks::checkAndReport(const Context &context, Report &reporter)
{
	for (int instance = 0; instance < _sensor_accel_sub.size(); instance++) {
		const bool is_required = instance == 0 || isAccelRequired(instance);

		if (!is_required) {
			continue;
		}

		const bool exists = _sensor_accel_sub[instance].advertised();
		bool is_valid = false;
		bool is_calibration_valid = false;

		if (exists) {
			sensor_accel_s accel_data;
			is_valid = _sensor_accel_sub[instance].copy(&accel_data) && (accel_data.device_id != 0) && (accel_data.timestamp != 0)
				   && (hrt_elapsed_time(&accel_data.timestamp) < 1_s);

			if (context.status().hil_state == vehicle_status_s::HIL_STATE_ON) {
				is_calibration_valid = true;

			} else {
				is_calibration_valid = (calibration::FindCurrentCalibrationIndex("ACC", accel_data.device_id) >= 0);
			}

			reporter.setIsPresent(health_component_t::accel);
		}

		const bool is_sensor_ok = is_valid && is_calibration_valid;

		if (!is_sensor_ok) {
			if (!exists) {
				/* EVENT
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::accel, events::ID("check_accel_missing"),
								events::Log::Error, "Accelerometer sensor {1} missing", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Accel Sensor %u missing", instance);
				}

			} else if (!is_valid) {
				/* EVENT
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::accel, events::ID("check_accel_no_data"),
								events::Log::Error, "No valid data from Accelerometer {1}", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: No valid data from Accel %u", instance);
				}

			} else if (!is_calibration_valid) {
				/* EVENT
				 */
				reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::accel, events::ID("check_accel_not_calibrated"),
								     events::Log::Error, "Accelerometer {1} uncalibrated", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Accel %u uncalibrated", instance);
				}
			}
		}
	}
}

bool AccelerometerChecks::isAccelRequired(int instance)
{
	sensor_accel_s sensor_accel;

	if (!_sensor_accel_sub[instance].copy(&sensor_accel)) {
		return false;
	}

	const uint32_t device_id = static_cast<uint32_t>(sensor_accel.device_id);

	if (device_id == 0) {
		return false;
	}

	bool is_used_by_nav = false;

	for (int i = 0; i < _estimator_status_sub.size(); i++) {
		estimator_status_s estimator_status;

		if (_estimator_status_sub[i].copy(&estimator_status) && estimator_status.accel_device_id == device_id) {
			is_used_by_nav = true;
			break;
		}
	}

	return is_used_by_nav;
}
