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

void ParachuteChecks::checkAndReport(const Context &context, Report &reporter)
{
	int32_t fw_lnd_para_en = 0;

	if (_param_fw_lnd_para_en_handle != PARAM_INVALID) {
		param_get(_param_fw_lnd_para_en_handle, &fw_lnd_para_en);
	}

	if (fw_lnd_para_en == 1 && _param_com_parachute.get() < 1 && !_parachute_sub.advertised()) {
		// Parachute landing is enabled, but neither a monitored external parachute system is
		// configured nor is the parachute module present to release a local parachute output:
		// nothing can act on the release command.
		/* EVENT
		 * @description
		 * FW_LND_PARA_EN is set but no monitored parachute system is configured
		 * (COM_PARACHUTE) and the parachute module is not part of the firmware, so the
		 * parachute release on the landing approach would have no effect.
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::parachute,
					    events::ID("check_parachute_landing_no_consumer"),
					    events::Log::Warning, "Parachute landing enabled but no parachute system found");
	}

	if (_param_com_parachute.get() < 1) { // COM_PARACHUTE 0 disables the check
		return;
	}

	reporter.failsafeFlags().parachute_unhealthy = !context.status().parachute_system_present || !context.status().parachute_system_healthy;

	// COM_PARACHUTE 1 (Warning) only warns, higher values also prevent arming
	const bool is_error = _param_com_parachute.get() >= 2;
	const NavModes affected_modes = is_error ? NavModes::All : NavModes::None;
	const events::Log log_level = is_error ? events::Log::Error : events::Log::Warning;

	if (!context.status().parachute_system_present) {
		/* EVENT
		 * @description
		 * No MAVLink parachute heartbeat detected. Check connection, power, configuration.
		 *
		 * <profile name="dev">
		 * Configured by <param>COM_PARACHUTE</param>
		 * </profile>
		 */
		reporter.healthFailure(affected_modes, health_component_t::parachute, events::ID("check_parachute_missing"),
				       log_level, "Parachute system missing");

		if (reporter.mavlink_log_pub() && is_error) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Parachute system missing");
		}

	} else if (!context.status().parachute_system_healthy) {

		/* EVENT
		 * @description
		 * MAVLink parachute system reports unhealthy status.
		 *
		 * <profile name="dev">
		 * Configured by <param>COM_PARACHUTE</param>
		 * </profile>
		 */
		reporter.healthFailure(affected_modes, health_component_t::parachute, events::ID("check_parachute_unhealthy"),
				       log_level, "Parachute system not ready");

		if (reporter.mavlink_log_pub() && is_error) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Parachute system not ready");
		}
	}

	if (context.status().parachute_system_present) {
		reporter.setIsPresent(health_component_t::parachute);
	}
}
