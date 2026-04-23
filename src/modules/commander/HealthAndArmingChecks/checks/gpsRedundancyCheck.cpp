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

#include "gpsRedundancyCheck.hpp"

void GpsRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().gnss_lost = false;

	gnss_redundancy_status_s status;

	if (_gnss_redundancy_status_sub.copy(&status)) {
		const bool act_configured = (_param_com_gnss_loss_act.get() > 0);

		// Divergence triggers the failsafe only when the operator explicitly expects two
		// receivers (SYS_HAS_NUM_GNSS >= 2); otherwise it remains a warning.
		const bool divergence_triggers_failsafe = status.divergence_detected
				&& (status.num_receivers_required >= 2);

		reporter.failsafeFlags().gnss_lost = status.below_required || divergence_triggers_failsafe;

		if (status.below_required || status.dropped_below_peak) {
			const bool block_arming = status.below_required && act_configured;
			const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
			const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;
			const uint8_t expected = status.below_required ? status.num_receivers_required
						 : status.peak_receivers_fixed;

			if (status.num_receivers_online < status.num_receivers_fixed
			    || status.num_receivers_online < status.num_receivers_required) {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GNSS</param>.
				 * Configure the failsafe action with <param>COM_GNSS_LSS_ACT</param>.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t, uint8_t>(nav_modes, health_component_t::gps,
						events::ID("check_gps_redundancy_offline"),
						log_level,
						"GPS receiver offline: {1} of {2} online",
						status.num_receivers_online, expected);

			} else {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GNSS</param>.
				 * Configure the failsafe action with <param>COM_GNSS_LSS_ACT</param>.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t, uint8_t>(nav_modes, health_component_t::gps,
						events::ID("check_gps_redundancy_no_fix"),
						log_level,
						"GPS receiver lost 3D fix: {1} of {2} fixed",
						status.num_receivers_fixed, expected);
			}
		}

		if (status.divergence_detected) {
			const bool block_arming = divergence_triggers_failsafe && act_configured;
			const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
			const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;

			/* EVENT
			 * @description
			 * Two GNSS receivers report positions that are inconsistent with their reported accuracy.
			 *
			 * <profile name="dev">
			 * Configure the failsafe action with <param>COM_GNSS_LSS_ACT</param>.
			 * The failsafe action is only triggered when <param>SYS_HAS_NUM_GNSS</param> is set to 2.
			 * </profile>
			 */
			reporter.healthFailure<float, float>(nav_modes, health_component_t::gps,
							     events::ID("check_gps_position_divergence"),
							     log_level,
							     "GPS receivers disagree: {1:.1}m apart (gate {2:.1}m)",
							     (double)status.divergence_m, (double)status.divergence_gate_m);
		}
	}
}
