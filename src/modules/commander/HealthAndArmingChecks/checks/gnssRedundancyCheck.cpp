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

using namespace matrix;
using namespace time_literals;

void GnssRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	bool gps_online[GPS_MAX_INSTANCES] {};
	bool gnss_healthy[GPS_MAX_INSTANCES] {};
	float eph[GPS_MAX_INSTANCES] {};
	float epv[GPS_MAX_INSTANCES] {};
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
				epv[i] = gnss.epv;
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

	// Position divergence check: flag a healthy receiver that disagrees with the selected one, after their antenna
	// positions, by more than the limits. Receivers with a poor reported accuracy disagree by more, so the gate widens to
	// 3 times their combined accuracy where that is larger: then the disagreement is unlikely to be noise.
	float divergence_h_m = 0.f;
	float divergence_v_m = 0.f;
	bool diverging = false;
	bool compared = false;

	for (int i = 0; i < GPS_MAX_INSTANCES; i++) {
		if ((selected < 0) || (i == selected) || !gnss_healthy[i]) {
			continue;
		}

		const float horizontal = status.inconsistency_horizontal[i];
		const float vertical = status.inconsistency_vertical[i];

		if (PX4_ISFINITE(horizontal)) {
			const float gate = math::max(_param_com_gnss_h_max.get(), 3.f * Vector2f(eph[i], eph[selected]).norm());
			diverging |= (horizontal > gate);
			divergence_h_m = math::max(divergence_h_m, horizontal);
			compared = true;
		}

		if (PX4_ISFINITE(vertical)) {
			const float gate = math::max(_param_com_gnss_v_max.get(), 3.f * Vector2f(epv[i], epv[selected]).norm());
			diverging |= (vertical > gate);
			divergence_v_m = math::max(divergence_v_m, vertical);
			compared = true;
		}
	}

	// Before arming, a disagreement is reported at once. In flight it has to persist for 2 s, so short glitches pass.
	_divergence_hysteresis.set_hysteresis_time_from(false, context.isArmed() ? 2_s : 0_s);
	// A disagreement clears after 2 s of continuous agreement, or at once when the receivers can't be compared
	_divergence_hysteresis.set_hysteresis_time_from(true, compared ? 2_s : 0_s);
	_divergence_hysteresis.set_state_and_update(diverging, hrt_absolute_time());

	const bool below_required = (_param_sys_has_num_gnss.get() > 0) && (healthy_count < _param_sys_has_num_gnss.get());
	const bool dropped_below_peak = (_peak_healthy_count > 1) && (healthy_count < _peak_healthy_count);

	// Divergence triggers the failsafe only when the operator explicitly expects two
	// receivers (SYS_HAS_NUM_GNSS >= 2); otherwise it remains a warning.
	const bool divergence_triggers_failsafe = _divergence_hysteresis.get_state() && (_param_sys_has_num_gnss.get() >= 2);

	reporter.failsafeFlags().gnss_lost = below_required || divergence_triggers_failsafe;
	const bool block_arming = reporter.failsafeFlags().gnss_lost && (_param_com_gnssloss_act.get() > 0);
	const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
	const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;

	if (below_required || dropped_below_peak) {
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
		/* EVENT
		 * @description
		 * Two GNSS receivers report positions that disagree by more than their antenna positions explain.
		 *
		 * <profile name="dev">
		 * Configure the limits with <param>COM_GNSS_H_MAX</param> and <param>COM_GNSS_V_MAX</param>.
		 * Configure the failsafe action with <param>COM_GNSSLOSS_ACT</param>.
		 * The failsafe action is only triggered when <param>SYS_HAS_NUM_GNSS</param> is set to 2.
		 * </profile>
		 */
		reporter.healthFailure<float, float>(nav_modes, health_component_t::gps,
						     events::ID("check_gps_position_divergence"),
						     log_level,
						     "GPS receivers disagree by {1:.1}m horizontally, {2:.1}m vertically",
						     divergence_h_m, divergence_v_m);
	}

	// The selection falls back to another receiver, so like any other receiver the primary blocks arming only when
	// SYS_HAS_NUM_GNSS counts it
	if (!context.isArmed() && status_valid && status.primary_offline) {
		/* EVENT
		 * @description
		 * Another receiver, if any, is used until the primary one publishes.
		 *
		 * <profile name="dev">
		 * The primary receiver is set with <param>SENS_GNSS_PRIME</param>, or is the moving base of a moving base pair.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::gps, events::ID("check_gnss_primary_offline"),
					    events::Log::Warning, "Primary GNSS receiver offline");
	}
}
