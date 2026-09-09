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

#include "vtolCheck.hpp"

using namespace time_literals;

void VtolChecks::checkAndReport(const Context &context, Report &reporter)
{
#if !defined(CONFIG_MODULES_FW_MODE_MANAGER) || !defined(CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL)

	if (context.status().is_vtol_tailsitter) {
		/* EVENT
		 * @description
		 * The firmware does not include the fixed-wing position control modules required by tailsitter airframes.
		 * Enable CONFIG_MODULES_FW_MODE_MANAGER and CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL
		 * in the board configuration, then reinstall the firmware.
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_vtol_tailsitter_fw_pos_control_missing"),
					    events::Log::Error,
					    "Tailsitter requires fixed-wing position control");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(),
					     "Preflight Fail: Tailsitter requires fixed-wing position control");
		}
	}

#endif

	vtol_vehicle_status_s vtol_vehicle_status;

	if (_vtol_vehicle_status_sub.copy(&vtol_vehicle_status)) {
		reporter.failsafeFlags().vtol_fixed_wing_system_failure = vtol_vehicle_status.fixed_wing_system_failure;

		if (reporter.failsafeFlags().vtol_fixed_wing_system_failure) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_vtol_fixed_wing_system_failure"),
						    events::Log::Error,
						    "VTOL fixed-wing system failure detected. Verify reason for failure, and reboot the vehicle once confirmed safe");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_info(reporter.mavlink_log_pub(), "Preflight Fail: VTOL fixed-wing system failure detected\t");
			}
		}
	}
}
