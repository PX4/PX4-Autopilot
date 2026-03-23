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

#include <lib/geo/geo.h>

using namespace time_literals;

// eph is firmware-dependent and not a rigorous 1-sigma, so 3 is a heuristic consistency gate
// rather than a precise statistical claim. It relaxes the gate automatically when receivers degrade.
static constexpr float GPS_DIVERGENCE_SIGMA = 3.0f;
static constexpr uint64_t GPS_DIVERGENCE_SUSTAIN = 2_s; // must be sustained before warning

void GpsRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	// Always reset — will be set below only when the condition is active
	reporter.failsafeFlags().gps_redundancy_lost = false;

	// Separate "online" (present + fresh data) from "fixed" (online + 3D fix).
	// online_count tracks receivers that are communicating; fixed_count tracks those
	// suitable for navigation. Keeping them separate lets us emit a more informative
	// warning: "receiver offline" vs "receiver lost fix".
	int online_count = 0;
	int fixed_count = 0;
	sensor_gps_s fixed_gps[GPS_MAX_INSTANCES] {};

	for (int i = 0; i < _sensor_gps_sub.size(); i++) {
		sensor_gps_s gps{};

		if (_sensor_gps_sub[i].copy(&gps)
		    && (gps.device_id != 0)
		    && (hrt_elapsed_time(&gps.timestamp) < 2_s)) {
			online_count++;

			if (gps.fix_type >= 3) {
				fixed_gps[fixed_count++] = gps;
			}
		}
	}

	// Position divergence check: warn if two fixed receivers disagree beyond their combined
	// uncertainty. The gate is: lever-arm separation + GPS_DIVERGENCE_SIGMA * RMS(eph0, eph1).
	// Using RMS(eph) means the gate tightens when both receivers are accurate and widens
	// automatically when one degrades, avoiding false alarms without a hard-coded threshold.
	// Pre-arm: warn immediately. In-flight: require GPS_DIVERGENCE_SUSTAIN to suppress
	// transient multipath spikes.
	bool divergence_active = false;

	if (fixed_count >= 2) {
		float north, east;
		get_vector_to_next_waypoint(fixed_gps[0].latitude_deg, fixed_gps[0].longitude_deg,
					    fixed_gps[1].latitude_deg, fixed_gps[1].longitude_deg,
					    &north, &east);

		const float divergence_m = sqrtf(north * north + east * east);
		const float rms_eph = sqrtf(fixed_gps[0].eph * fixed_gps[0].eph + fixed_gps[1].eph * fixed_gps[1].eph);
		const float dx = _param_gps0_offx.get() - _param_gps1_offx.get();
		const float dy = _param_gps0_offy.get() - _param_gps1_offy.get();
		const float expected_d = sqrtf(dx * dx + dy * dy);
		const float gate_m = expected_d + rms_eph * GPS_DIVERGENCE_SIGMA;

		// Pre-arm: trigger immediately so the operator can decide before takeoff.
		// In-flight: require sustained divergence to avoid false alarms from transient multipath.
		const uint64_t sustain = context.isArmed() ? GPS_DIVERGENCE_SUSTAIN : 0_s;

		if (divergence_m > gate_m) {
			if (_divergence_since == 0) {
				_divergence_since = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&_divergence_since) >= sustain) {
				divergence_active = true;
				const bool act = (_param_com_gps_loss_act.get() > 0);

				/* EVENT
				 * @description
				 * Two GPS receivers report positions that are inconsistent with their reported accuracy.
				 *
				 * <profile name="dev">
				 * Configure the failsafe action with <param>COM_GPS_LOSS_ACT</param>.
				 * </profile>
				 */
				reporter.healthFailure<float, float>(act ? NavModes::All : NavModes::None,
								     health_component_t::gps,
								     events::ID("check_gps_position_divergence"),
								     act ? events::Log::Error : events::Log::Warning,
								     "GPS receivers disagree: {1:.1}m apart (gate {2:.1}m)",
								     (double)divergence_m, (double)gate_m);
			}

		} else {
			_divergence_since = 0;
		}
	}

	// Track the highest fixed count seen — used to detect any GPS loss regardless of SYS_HAS_NUM_GPS
	if (fixed_count > _peak_fixed_count) {
		_peak_fixed_count = fixed_count;
	}

	const int required = _param_sys_has_num_gps.get();
	const bool below_required = (required > 0 && fixed_count < required);
	const bool dropped_below_peak = (_peak_fixed_count > 1 && fixed_count < _peak_fixed_count);

	if (!below_required && !dropped_below_peak && !divergence_active) {
		return;
	}

	reporter.failsafeFlags().gps_redundancy_lost = below_required || divergence_active;

	if (!below_required && !dropped_below_peak) {
		return;
	}

	// act==0: warn only, never blocks arming; act>0: blocks arming and shows red
	const bool block_arming = below_required && (_param_com_gps_loss_act.get() > 0);
	const NavModes nav_modes = block_arming ? NavModes::All : NavModes::None;
	const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;
	const uint8_t expected = below_required ? (uint8_t)required : (uint8_t)_peak_fixed_count;

	// Differentiate: if online_count is also low the receiver is offline; otherwise it lost fix.
	if (online_count < fixed_count || online_count < required) {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GPS</param>.
		 * Configure the failsafe action with <param>COM_GPS_LOSS_ACT</param>.
		 * </profile>
		 */
		reporter.healthFailure<uint8_t, uint8_t>(nav_modes, health_component_t::gps,
				events::ID("check_gps_redundancy_offline"),
				log_level,
				"GPS receiver offline: {1} of {2} online",
				(uint8_t)online_count, expected);

	} else {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GPS</param>.
		 * Configure the failsafe action with <param>COM_GPS_LOSS_ACT</param>.
		 * </profile>
		 */
		reporter.healthFailure<uint8_t, uint8_t>(nav_modes, health_component_t::gps,
				events::ID("check_gps_redundancy_no_fix"),
				log_level,
				"GPS receiver lost 3D fix: {1} of {2} fixed",
				(uint8_t)fixed_count, expected);
	}
}
