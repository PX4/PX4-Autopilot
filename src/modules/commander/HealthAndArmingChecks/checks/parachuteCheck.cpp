/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "parachuteCheck.hpp"


using namespace time_literals;

void ParachuteChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (!_param_com_parachute.get()) {
		return;
	}

	if (!context.status().parachute_system_present) {
		/* EVENT
		 * @description
		 * Parachute system failed to report. Make sure it it setup and installed properly.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_PARACHUTE</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::parachute, events::ID("check_parachute_missing"),
				       events::Log::Error, "Parachute system missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Parachute system missing");
		}

	} else if (!context.status().parachute_system_healthy) {

		/* EVENT
		 * @description
		 * Parachute system reported being unhealth.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_PARACHUTE</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::parachute, events::ID("check_parachute_unhealthy"),
				       events::Log::Error, "Parachute system not ready");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Parachute system not ready");
		}
	}

	if (context.status().parachute_system_present) {
		reporter.setIsPresent(health_component_t::parachute);
	}
}
