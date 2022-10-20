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

#include "geofenceCheck.hpp"

void GeofenceChecks::checkAndReport(const Context &context, Report &reporter)
{
	geofence_result_s geofence_result;

	if (!_geofence_result_sub.copy(&geofence_result)) {
		geofence_result = {};
	}

	reporter.failsafeFlags().primary_geofence_breached = geofence_result.primary_geofence_breached;

	if (geofence_result.primary_geofence_action != 0 && reporter.failsafeFlags().primary_geofence_breached) {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>GF_ACTION</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure<events::px4::enums::geofence_violation_reason_t>(NavModes::All, health_component_t::system,
				events::ID("check_gf_violation"),
				events::Log::Error, "Geofence violation: {1}",
				(events::px4::enums::geofence_violation_reason_t)geofence_result.geofence_violation_reason);

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Geofence violation");
		}
	}

	if (geofence_result.primary_geofence_action == geofence_result_s::GF_ACTION_RTL
	    && reporter.failsafeFlags().home_position_invalid) {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>GF_ACTION</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_gf_no_home"),
					    events::Log::Error, "Geofence RTL requires valid home");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Geofence RTL requires valid home");
		}
	}
}
