/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "manualControlCheck.hpp"

using namespace time_literals;

void ManualControlChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (context.isArmed()) {
		return;
	}

	manual_control_switches_s manual_control_switches;

	if (_manual_control_switches_sub.copy(&manual_control_switches)) {

		// check action switches
		if (manual_control_switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::remote_control,
						    events::ID("check_man_control_rtl_engaged"),
						    events::Log::Error, "RTL switch engaged");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: RTL switch engaged");
			}
		}

		if (manual_control_switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::remote_control,
						    events::ID("check_man_control_kill_engaged"),
						    events::Log::Error, "Kill switch engaged");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Kill switch engaged");
			}
		}

		if (manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::remote_control,
						    events::ID("check_man_control_landing_gear_up"),
						    events::Log::Error, "Landing gear switch set in UP position");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Landing gear switch set in UP position");
			}
		}

	}
}
