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

#include "rcAndDataLinkCheck.hpp"

using namespace time_literals;

void RcAndDataLinkChecks::checkAndReport(const Context &context, Report &reporter)
{
	// RC
	manual_control_setpoint_s manual_control_setpoint;

	if (!_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
		manual_control_setpoint = {};
		reporter.failsafeFlags().manual_control_signal_lost = true;
	}

	// Check if RC is valid
	if (!manual_control_setpoint.valid
	    || hrt_elapsed_time(&manual_control_setpoint.timestamp) > _param_com_rc_loss_t.get() * 1_s) {

		if (!reporter.failsafeFlags().manual_control_signal_lost && _last_valid_manual_control_setpoint > 0) {

			events::send(events::ID("commander_rc_lost"), {events::Log::Info, events::LogInternal::Info},
				     "Manual control lost");
		}

		reporter.failsafeFlags().manual_control_signal_lost = true;

	} else {
		reporter.setIsPresent(health_component_t::remote_control);

		if (reporter.failsafeFlags().manual_control_signal_lost && _last_valid_manual_control_setpoint > 0) {
			float elapsed = hrt_elapsed_time(&_last_valid_manual_control_setpoint) * 1e-6f;
			events::send<float>(events::ID("commander_rc_regained"), events::Log::Info,
					    "Manual control regained after {1:.1} s", elapsed);
		}

		reporter.failsafeFlags().manual_control_signal_lost = false;
		_last_valid_manual_control_setpoint = manual_control_setpoint.timestamp;
	}

	// Manual control check is in modeCheck as mode requirement

	// GCS connection
	reporter.failsafeFlags().gcs_connection_lost = context.status().gcs_connection_lost;

	if (reporter.failsafeFlags().gcs_connection_lost) {

		// Prevent arming if we neither have RC nor a GCS connection. TODO: disabled for now due to MAVROS tests
		bool gcs_connection_required =  _param_nav_dll_act.get() > 0
						/*|| (rc_is_optional && reporter.failsafeFlags().manual_control_signal_lost) */;
		NavModes affected_modes = gcs_connection_required ? NavModes::All : NavModes::None;
		events::LogLevel log_level = gcs_connection_required ? events::Log::Error : events::Log::Info;
		/* EVENT
		 * @description
		 * To arm, at least a data link or manual control (RC) must be present.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>NAV_DLL_ACT</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::communication_links,
					    events::ID("check_rc_dl_no_dllink"),
					    log_level, "No connection to the ground control station");

		if (gcs_connection_required && reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "Preflight Fail: No connection to the ground control station\t");
		}

	} else {
		reporter.setIsPresent(health_component_t::communication_links);
	}
}
