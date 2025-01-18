/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "distanceSensorChecks.hpp"

using namespace time_literals;

void DistanceSensorChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_sys_has_num_dist.get() <= 0) {
		return;
	}

	for (int instance = 0; instance < _distance_sensor_sub.size(); instance++) {
		const bool exists = _distance_sensor_sub[instance].advertised();
		const bool is_mandatory = instance < _param_sys_has_num_dist.get();
		bool valid = false;

		if (exists) {
			distance_sensor_s dist_sens;
			valid = _distance_sensor_sub[instance].copy(&dist_sens) && ((hrt_elapsed_time(&dist_sens.timestamp) < 1_s)
					|| (dist_sens.mode == distance_sensor_s::MODE_DISABLED));
			reporter.setIsPresent(health_component_t::distance_sensor);
		}

		if (is_mandatory) {
			if (!exists) {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>SYS_HAS_NUM_DIST</param> parameter.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::distance_sensor,
								events::ID("check_distance_sensor_missing"),
								events::Log::Error, "Distance sensor {1} missing", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Distance Sensor %u missing", instance);
				}

			} else if (!valid) {
				/* EVENT
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::distance_sensor,
								events::ID("check_distance_sensor_invalid"),
								events::Log::Error, "No valid data from distance sensor {1}", instance);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: no valid data from distance sensor %u", instance);
				}
			}
		}
	}
}
