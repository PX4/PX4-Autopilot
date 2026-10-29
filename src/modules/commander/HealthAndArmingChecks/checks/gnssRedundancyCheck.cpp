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
#include <lib/geo/geo.h>

using namespace matrix;
using namespace time_literals;

GnssRedundancyChecks::GnssRedundancyChecks()
{
}

Vector3f GnssRedundancyChecks::antennaOffset(uint32_t device_id, int instance) const
{
	// Match by SENS_GNSSx_ID, or by sensor_gnss instance if no ID is configured (same as VehicleGPSPosition)
	const int32_t id0 = _param_sens_gnss0_id.get();
	const int32_t id1 = _param_sens_gnss1_id.get();
	const bool match_by_id = (id0 != 0) || (id1 != 0);

	if (match_by_id ? (static_cast<uint32_t>(id0) == device_id) : (instance == 0)) {
		return Vector3f(_param_sens_gnss0_offx.get(), _param_sens_gnss0_offy.get(), _param_sens_gnss0_offz.get());
	}

	if (match_by_id ? (static_cast<uint32_t>(id1) == device_id) : (instance == 1)) {
		return Vector3f(_param_sens_gnss1_offx.get(), _param_sens_gnss1_offy.get(), _param_sens_gnss1_offz.get());
	}

	return Vector3f{};
}

void GnssRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	bool gps_online[GPS_MAX_INSTANCES] {};
	bool gnss_healthy[GPS_MAX_INSTANCES] {};
	uint8_t healthy_count = 0;

	// Only the fields needed for the divergence check, to keep the full sensor_gnss_s copies off the stack
	struct HealthyPosition {
		double latitude;
		double longitude;
		double altitude_msl;
		Vector3f antenna_offset;
	} healthy_gnss[GPS_MAX_INSTANCES] {};

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
				healthy_gnss[healthy_count++] = {gnss.latitude, gnss.longitude, gnss.altitude_msl, antennaOffset(gnss.device_id, i)};
			}
		}
	}

	// Track the highest healthy count seen to warn about GNSS loss regardless of SYS_HAS_NUM_GNSS
	if (healthy_count > _peak_healthy_count) {
		_peak_healthy_count = healthy_count;
	}

	// The expected antenna baseline depends on heading and tilt, so the receivers are only compared with a valid attitude
	vehicle_attitude_s attitude{};
	const bool attitude_valid = _vehicle_attitude_sub.copy(&attitude) && (hrt_elapsed_time(&attitude.timestamp) < 1_s);
	const bool can_compare = (healthy_count >= 2) && attitude_valid;

	// Before arming, a disagreement blocks immediately. In flight, it has to persist for 2s to not react to short glitches.
	_divergence_hysteresis.set_hysteresis_time_from(false, context.isArmed() ? 2_s : 0_s);
	// A disagreement only clears after 2s of continuous agreement, or immediately if the receivers cannot be compared
	_divergence_hysteresis.set_hysteresis_time_from(true, can_compare ? 2_s : 0_s);

	// Position divergence check: flag if two healthy receivers disagree by more than COM_GNSS_H_MAX horizontally
	// or COM_GNSS_V_MAX vertically. The expected baseline between the antennas is rotated from body to NED with
	// the current attitude, so heading and tilt are accounted for and the full error vector is checked.
	// Until the heading is aligned, only the horizontal distances are compared, which does not depend on heading.
	float divergence_h_m = 0.f;
	float divergence_v_m = 0.f;

	if (can_compare) {
		// Measured baseline from receiver 0 to receiver 1 (NED)
		float north, east;
		get_vector_to_next_waypoint(healthy_gnss[0].latitude, healthy_gnss[0].longitude,
					    healthy_gnss[1].latitude, healthy_gnss[1].longitude,
					    &north, &east);
		const float down = -static_cast<float>(healthy_gnss[1].altitude_msl - healthy_gnss[0].altitude_msl);

		// Expected baseline from the antenna offsets
		const Vector3f expected = Quatf(attitude.q).rotateVector(healthy_gnss[1].antenna_offset - healthy_gnss[0].antenna_offset);

		// Yaw alignment of the primary EKF (the attitude source)
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_status_flags_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}

		estimator_status_flags_s estimator_status_flags{};
		// Published at ~1 Hz
		const bool heading_valid = _estimator_status_flags_sub.copy(&estimator_status_flags)
					   && (hrt_elapsed_time(&estimator_status_flags.timestamp) < 2_s)
					   && estimator_status_flags.cs_yaw_align;

		if (heading_valid) {
			divergence_h_m = Vector2f(north - expected(0), east - expected(1)).length();

		} else {
			divergence_h_m = fabsf(Vector2f(north, east).length() - Vector2f(expected(0), expected(1)).length());
		}

		// The vertical component does not depend on heading
		divergence_v_m = fabsf(down - expected(2));

		const bool diverging = (divergence_h_m > _param_com_gnss_h_max.get()) || (divergence_v_m > _param_com_gnss_v_max.get());
		_divergence_hysteresis.set_state_and_update(diverging, hrt_absolute_time());

	} else {
		_divergence_hysteresis.set_state_and_update(false, hrt_absolute_time());
	}

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
		 * Two GNSS receivers report positions that disagree by more than the configured limits.
		 *
		 * <profile name="dev">
		 * Configure the limits with <param>COM_GNSS_H_MAX</param> (horizontal) and <param>COM_GNSS_V_MAX</param> (vertical).
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
}
