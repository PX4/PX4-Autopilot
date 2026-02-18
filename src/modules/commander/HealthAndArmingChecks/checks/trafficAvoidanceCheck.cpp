/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "trafficAvoidanceCheck.hpp"


void TrafficAvoidanceChecks::checkAndReport(const Context &context, Report &reporter)
{
	NavModes affected_modes{NavModes::None}; // COM_ARM_TRAFF 1 - warning only, arming allowed, affected_modes stays None

	switch (_param_com_arm_traff.get()) {
	case 0:
		return; // Check disabled

	case 2:
		affected_modes = NavModes::All; // Disallow arming for all modes
		break;

	case 3:
		affected_modes = NavModes::Mission; // Disallow arming for mission
		break;
	}

	if (!context.status().traffic_avoidance_system_present) {
		/* EVENT
		 * @description
		 * Traffic avoidance system (ADSB/FLARM) failed to report. Make sure it is setup and connected properly.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_TRAFF</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::traffic_avoidance,
					    events::ID("check_traffic_avoidance_missing"),
					    events::Log::Error, "Traffic avoidance system missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Traffic avoidance system missing");
		}
	}
}
