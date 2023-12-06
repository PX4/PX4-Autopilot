/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include "airspeedCheck.hpp"
#include <lib/circuit_breaker/circuit_breaker.h>

using namespace time_literals;

AirspeedChecks::AirspeedChecks()
	: _param_fw_airspd_max_handle(param_find("FW_AIRSPD_MAX"))
{
}

void AirspeedChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_sys_has_num_aspd.get() <= 0 ||
	    (context.status().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !context.status().is_vtol)) {
		return;
	}

	airspeed_validated_s airspeed_validated;

	if (_airspeed_validated_sub.copy(&airspeed_validated) && hrt_elapsed_time(&airspeed_validated.timestamp) < 2_s) {

		reporter.setIsPresent(health_component_t::differential_pressure);

		// Maximally allow the airspeed reading to be at FW_AIRSPD_MAX when arming. This is to catch very badly calibrated
		// airspeed sensors, but also high wind conditions that prevent a forward flight of the vehicle.
		float arming_max_airspeed_allowed = 20.f;
		param_get(_param_fw_airspd_max_handle, &arming_max_airspeed_allowed);

		/*
		 * Check if airspeed is declared valid or not by airspeed selector.
		 */
		if (!PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)) {

			/* EVENT
			 */
			reporter.healthFailure(NavModes::All, health_component_t::differential_pressure,
					       events::ID("check_airspeed_invalid"),
					       events::Log::Error, "Airspeed invalid");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Airspeed invalid");
			}
		}

		if (!context.isArmed() && fabsf(airspeed_validated.calibrated_airspeed_m_s) > arming_max_airspeed_allowed) {
			/* EVENT
			 * @description
			 * Current airspeed reading too high. Check if wind is below maximum airspeed and redo airspeed
			 * calibration if the measured airspeed does not correspond to wind conditions.
			 *
			 * <profile name="dev">
			 * Measured: {1:.1m/s}, limit: {2:.1m/s}.
			 *
			 * This check can be configured via <param>FW_AIRSPD_MAX</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure<float, float>(NavModes::None, health_component_t::differential_pressure,
					events::ID("check_airspeed_too_high"),
					events::Log::Error, "Airspeed too high", airspeed_validated.calibrated_airspeed_m_s, arming_max_airspeed_allowed);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Airspeed too high - check airspeed calibration");
			}
		}

	} else {

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Most likely the airspeed selector module is not running.
		 * This check can be configured via <param>SYS_HAS_NUM_ASPD</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::differential_pressure,
				       events::ID("check_airspeed_no_data"),
				       events::Log::Error, "No airspeed data");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Airspeed selector module down");
		}
	}

}
