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

#include "gnssRedundancyCheck.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;
using namespace time_literals;

GnssRedundancyChecks::GnssRedundancyChecks()
{
	_divergence_hysteresis.set_hysteresis_time_from(false, 2_s);
	_divergence_hysteresis.set_hysteresis_time_from(true, 2_s);
}

void GnssRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	bool gps_online[GPS_MAX_INSTANCES] {};
	bool gnss_healthy[GPS_MAX_INSTANCES] {};
	float eph[GPS_MAX_INSTANCES] {};
	uint8_t healthy_count = 0;
	int selected = -1;

	sensors_status_gnss_s status{};
	const bool status_valid = _sensors_status_gnss_sub.copy(&status) && (hrt_elapsed_time(&status.timestamp) < 1_s);

	for (int i = 0; i < GPS_MAX_INSTANCES; i++) {
		sensor_gnss_s gnss{};

		if (_sensor_gnss_sub[i].copy(&gnss)
		    && (gnss.device_id != 0)
		    && (hrt_elapsed_time(&gnss.timestamp) < 1_s)) {
			gps_online[i] = true;

			// The sensors module indexes its status by the same sensor_gnss instance
			if (status_valid && (status.device_ids[i] == gnss.device_id) && status.healthy[i]) {
				gnss_healthy[i] = true;
				eph[i] = gnss.eph;
				healthy_count++;

				if (gnss.device_id == status.device_id_selected) {
					selected = i;
				}
			}
		}
	}

	// Track the highest healthy count seen to warn about GNSS loss regardless of SYS_HAS_NUM_GNSS
	if (healthy_count > _peak_healthy_count) {
		_peak_healthy_count = healthy_count;
	}

	// Position divergence check: flag if a healthy receiver disagrees with the selected one beyond their combined
	// uncertainty. Gate = 3 * RSS(eph), on the disagreement the sensors module finds after the lever arms.
	float divergence_m = 0.f;
	bool diverged = false;

	for (int i = 0; i < GPS_MAX_INSTANCES; i++) {
		if ((selected < 0) || (i == selected) || !gnss_healthy[i] || !PX4_ISFINITE(status.inconsistency[i])) {
			continue;
		}

		// Use quadrature sum for standard deviation of the difference taking the firmware dependent eph as standard
		// deviation and a heuristic factor of 3 because then it's unlikely just noise.
		const float divergence_gate_m = 3.f * Vector2f(eph[i], eph[selected]).length();
		divergence_m = math::max(divergence_m, status.inconsistency[i]);
		diverged |= status.inconsistency[i] > divergence_gate_m;
	}

	_divergence_hysteresis.set_state_and_update(diverged, hrt_absolute_time());

	const bool below_required = (_param_sys_has_num_gnss.get() > 0) && (healthy_count < _param_sys_has_num_gnss.get());
	const bool dropped_below_peak = (_peak_healthy_count > 1) && (healthy_count < _peak_healthy_count);
	const bool act_configured = (_param_com_gnssloss_act.get() > 0);

	// Divergence triggers the failsafe only when the operator explicitly expects two
	// receivers (SYS_HAS_NUM_GNSS >= 2); otherwise it remains a warning.
	const bool divergence_triggers_failsafe = _divergence_hysteresis.get_state() && (_param_sys_has_num_gnss.get() >= 2);

	reporter.failsafeFlags().gnss_lost = below_required || divergence_triggers_failsafe;

	if (below_required || dropped_below_peak) {
		const bool block_arming = below_required  && (act_configured || !context.isArmed());
		const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
		const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;
		const int expected = below_required ? _param_sys_has_num_gnss.get() : _peak_healthy_count;

		for (int i = 0; i < expected; i++) {
			if (!gps_online[i]) {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GNSS</param>.
				 * Configure the failsafe action with <param>COM_GNSSLOSS_ACT</param>.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t>(nav_modes, health_component_t::gps,
								events::ID("check_gnss_receiver_offline"),
								log_level, "GPS {1} offline", (uint8_t)i);

			} else if (!gnss_healthy[i]) {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GNSS</param>.
				 * Configure the failsafe action with <param>COM_GNSSLOSS_ACT</param>.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t>(nav_modes, health_component_t::gps,
								events::ID("check_gnss_receiver_unhealthy"),
								log_level, "GNSS {1} fails its checks", (uint8_t)i);
			}
		}
	}

	if (_divergence_hysteresis.get_state()) {
		const bool block_arming = divergence_triggers_failsafe && (act_configured || !context.isArmed());
		const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
		const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;

		/* EVENT
		 * @description
		 * Two GNSS receivers report positions that are inconsistent with their reported accuracy.
		 *
		 * <profile name="dev">
		 * Configure the failsafe action with <param>COM_GNSSLOSS_ACT</param>.
		 * The failsafe action is only triggered when <param>SYS_HAS_NUM_GNSS</param> is set to 2.
		 * </profile>
		 */
		reporter.healthFailure<float>(nav_modes, health_component_t::gps,
					      events::ID("check_gps_position_divergence"),
					      log_level,
					      "GPS receivers disagree by {1:.1}m",
					      (double)divergence_m);
	}
}
