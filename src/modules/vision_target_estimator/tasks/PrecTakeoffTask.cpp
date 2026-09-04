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

/**
 * @file PrecTakeoffTask.cpp
 * @brief Implements the precision-takeoff VTE task.
 *
 * The task follows prec_takeoff_status published by Navigator. On start it seeds the position
 * estimator with home as an approximate pad position, so its GNSS bias from the true pad center
 * is estimated like the mission land point's.
 * Home fusion is gated by VTE_AID_MASK bit 5.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "PrecTakeoffTask.h"

#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include "../Position/VTEPosition.h"
#include "../common.h"

namespace vision_target_estimator
{

static constexpr hrt_abstime kGpsDataTimeoutUs = 1_s;
// vehicle_land_detected has a 1 Hz heartbeat, so allow one full period plus scheduling jitter.
static constexpr hrt_abstime kLandDetectedTimeoutUs = 2_s;

void PrecTakeoffTask::pollStatus()
{
	prec_takeoff_status_s status;

	if (_prec_takeoff_status_sub.update(&status)) {
		_is_taking_off = status.state == prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING;
	}
}

void PrecTakeoffTask::onActivate()
{
	_home_ref = {};
	_home_dist_warned = false;
	updateHomeReference(false);
}

void PrecTakeoffTask::onPosEstStart(VTEPosition &pos)
{
	// Replace any reference cached by the previous task, even while home aiding is disabled.
	const bool home_aid_enabled = pos.missionPosAidEnabled();

	if (_home_ref.valid || updateHomeReference(home_aid_enabled)) {
		pos.setMissionPosition(_home_ref.lat_deg, _home_ref.lon_deg, _home_ref.alt_m);

	} else {
		if (home_aid_enabled) {
			PX4_WARN("VTE for precision takeoff, home position cannot be used.");
		}

		pos.clearMissionPosition();
	}
}

bool PrecTakeoffTask::updateHomeReference(const bool report_distance_warning)
{
	home_position_s home;

	if (!_home_position_sub.copy(&home) || !home.valid_hpos || !home.valid_alt
	    || !PX4_ISFINITE(home.lat) || !PX4_ISFINITE(home.lon) || !PX4_ISFINITE(home.alt)) {
		return false;
	}

	// Only trust home while on the ground, where the vehicle sits on the pad.
	vehicle_land_detected_s land_detected;
	const hrt_abstime now = TimeSourceProvider::nowUs();

	if (!_vehicle_land_detected_sub.copy(&land_detected) || !land_detected.landed
	    || land_detected.timestamp == 0 || land_detected.timestamp > now
	    || now - land_detected.timestamp >= kLandDetectedTimeoutUs) {
		return false;
	}

	sensor_gps_s gps;

	if (!_vehicle_gps_position_sub.copy(&gps) || gps.fix_type < sensor_gps_s::FIX_TYPE_3D
	    || gps.timestamp_sample == 0 || gps.timestamp_sample > now
	    || now - gps.timestamp_sample >= kGpsDataTimeoutUs
	    || !PX4_ISFINITE(gps.latitude_deg) || !PX4_ISFINITE(gps.longitude_deg)) {
		return false;
	}

	const float dist_m = get_distance_to_next_waypoint(home.lat, home.lon, gps.latitude_deg, gps.longitude_deg);

	if (!PX4_ISFINITE(dist_m) || dist_m > kMaxHomeDistM) {
		if (report_distance_warning && !_home_dist_warned) {
			PX4_WARN("VTE for precision takeoff, home is %.1f m from the vehicle, not used.", (double)dist_m);
			_home_dist_warned = true;
		}

		return false;
	}

	_home_ref.valid = true;
	_home_ref.lat_deg = home.lat;
	_home_ref.lon_deg = home.lon;
	_home_ref.alt_m = home.alt;
	return true;
}

} // namespace vision_target_estimator
