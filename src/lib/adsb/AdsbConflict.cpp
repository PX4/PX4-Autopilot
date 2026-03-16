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

#include "AdsbConflict.h"
#include "geo/geo.h"

#include <uORB/topics/transponder_report.h>

#include <float.h>

matrix::Vector3f AdsbConflict::velocity_ned_from_transponder_report(const transponder_report_s &transponder_report)
{
	if (!PX4_ISFINITE(transponder_report.heading)) {
		return matrix::Vector3f(transponder_report.hor_velocity, 0.f, -transponder_report.ver_velocity);
	}

	return matrix::Vector3f(cosf(transponder_report.heading) * transponder_report.hor_velocity,
				sinf(transponder_report.heading) * transponder_report.hor_velocity,
				-transponder_report.ver_velocity);
}

void AdsbConflict::transponder_report_to_aircraft_state(const transponder_report_s &transponder_report,
		aircraft_state_s &traffic_state)
{
	traffic_state.lat_lon = matrix::Vector2d(transponder_report.lat, transponder_report.lon);
	traffic_state.altitude = transponder_report.altitude;
	traffic_state.heading = transponder_report.heading;
	traffic_state.velocity_ned = velocity_ned_from_transponder_report(transponder_report);
}

void AdsbConflict::uav_state_to_aircraft_state(const matrix::Vector2d &uav_lat_lon, const float uav_alt,
		const float uav_heading,
		const matrix::Vector3f &uav_vel_ned, aircraft_state_s &uav_state)
{
	uav_state.lat_lon = uav_lat_lon;
	uav_state.altitude = uav_alt;
	uav_state.heading = uav_heading;
	uav_state.velocity_ned = uav_vel_ned;
}

bool AdsbConflict::handle_traffic(const matrix::Vector2d &uav_lat_lon, const float uav_alt,
				  const float uav_heading,
				  const matrix::Vector3f &uav_vel_ned, const transponder_report_s &transponder_report, detect_and_avoid_s &daa_output)
{
	// Check input data not NAN
	if (!(PX4_ISFINITE(transponder_report.lat) && PX4_ISFINITE(transponder_report.lon)
	      && uav_lat_lon.isAllFinite())) {
		PX4_DEBUG("DAA lib: Invalid lat, lon, Early return");
		return false;
	}

	if (!(PX4_ISFINITE(transponder_report.hor_velocity) && PX4_ISFINITE(transponder_report.ver_velocity))) {
		PX4_DEBUG("DAA lib: Invalid traffic vel, Early return");
		return false;
	}

	if (!uav_vel_ned.isAllFinite()) {
		PX4_DEBUG("DAA lib: Invalid uav vel, Early return");
		return false;
	}

	if (_daa_standard == detect_and_avoid_s::DAA_STANDARD_CROSSTRACK
	    && (!PX4_ISFINITE(uav_heading) || !PX4_ISFINITE(transponder_report.heading))) {
		PX4_DEBUG("DAA lib: Invalid heading, Early return");
		return false;
	}

	// Process input data
	aircraft_state_s uav_state{};
	uav_state_to_aircraft_state(uav_lat_lon, uav_alt, uav_heading, uav_vel_ned, uav_state);

	aircraft_state_s traffic_state{};
	transponder_report_to_aircraft_state(transponder_report, traffic_state);

	// Use DAA standard to detect traffic
	daa_stats_s daa_stats{};
	daa_output.conflict_level = calculate_daa_stats(uav_state, traffic_state, daa_stats);
	daa_output.aircraft_dist_hor = daa_stats.aircraft_dist_hor;
	daa_output.aircraft_dist_vert = daa_stats.aircraft_dist_vert;
	daa_output.expected_min_dist_time = daa_stats.expected_min_dist_time_sec;

	return true;
}

uint8_t AdsbConflict::calculate_daa_stats(const aircraft_state_s &uav_state, const aircraft_state_s &traffic_state,
		daa_stats_s &daa_stats)
{
	switch (_daa_standard) {
	case detect_and_avoid_s::DAA_STANDARD_CROSSTRACK: {
			return _daaCrosstrack.calculate_daa_stats(uav_state, traffic_state, daa_stats);
		}

	case detect_and_avoid_s::DAA_STANDARD_F3442: {
			return _daaF3442.calculate_daa_stats(uav_state, traffic_state, daa_stats);
		}

	default: {
			break;
		}
	}

	return detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
}

bool AdsbConflict::try_setting_DAA_standard(const uint8_t standard)
{
	switch (standard) {
	case detect_and_avoid_s::DAA_STANDARD_F3442:
	case detect_and_avoid_s::DAA_STANDARD_CROSSTRACK:
		_daa_standard = standard;
		return true;

	default:
		PX4_DEBUG("DAA: Invalid Standard.");
		return false;
	}
}

bool AdsbConflict::try_updating_params()
{
	switch (_daa_standard) {
	case detect_and_avoid_s::DAA_STANDARD_CROSSTRACK:
		return _daaCrosstrack.try_setting_params();

	case detect_and_avoid_s::DAA_STANDARD_F3442:
	default:
		return _daaF3442.try_setting_params();
	}
}
