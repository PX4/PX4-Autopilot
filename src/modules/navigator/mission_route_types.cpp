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
 * @file mission_route_types.cpp
 *
 * Implementations for the shared mission-route data types and helpers declared in
 * mission_route_types.h.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_types.h"

#include <px4_platform_common/log.h>

namespace mission_route
{

bool Position::valid() const
{
	return PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)
	       && !((fabs(lat) < kNullIslandThresholdDeg) && (fabs(lon) < kNullIslandThresholdDeg))
	       && (fabs(lat) <= 90.0) && (fabs(lon) <= 180.0);
}

bool isLandingCmd(uint16_t nav_cmd)
{
	return nav_cmd == NAV_CMD_LAND || nav_cmd == NAV_CMD_VTOL_LAND;
}

float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl)
{
	if (mission_item.altitude_is_relative) {
		return PX4_ISFINITE(home_altitude_amsl) ? mission_item.altitude + home_altitude_amsl : NAN;
	}

	return mission_item.altitude;
}

bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl, Position &position)
{
	if (safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT) {
		return false;
	}

	// Rally-point safe points are expected in global AMSL or global-relative-altitude frames.
	switch (safe_point_item.frame) {
	case NAV_FRAME_GLOBAL:
	case NAV_FRAME_GLOBAL_INT:
		position.alt = safe_point_item.altitude;
		break;

	case NAV_FRAME_GLOBAL_RELATIVE_ALT:
	case NAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
		if (!PX4_ISFINITE(home_altitude_amsl)) {
			return false;
		}

		position.alt = safe_point_item.altitude + home_altitude_amsl; // alt of safe point is rel to home
		break;

	default:
		PX4_WARN("RTL safe point frame %u unsupported", static_cast<unsigned>(safe_point_item.frame));
		return false;
	}

	position.lat = safe_point_item.lat;
	position.lon = safe_point_item.lon;
	return position.valid();
}

bool copyPositionToYawSetpoint(const Position &position, PositionYawSetpoint &setpoint)
{
	if (!position.valid()) {
		return false;
	}

	setpoint.lat = position.lat;
	setpoint.lon = position.lon;
	setpoint.alt = position.alt;
	setpoint.yaw = NAN;
	return true;
}

loiter_point_s makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl)
{
	loiter_point_s approach{};
	approach.lat = mission_item.lat;
	approach.lon = mission_item.lon;
	approach.height_m = getAbsoluteAltitudeForMissionItem(mission_item, home_altitude_amsl);
	approach.loiter_radius_m = mission_item.loiter_radius;
	return approach;
}

} // namespace mission_route
