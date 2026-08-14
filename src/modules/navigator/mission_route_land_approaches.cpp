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
 * @file mission_route_land_approaches.cpp
 *
 * VTOL landing-approach queries over mission-route safe-point records.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_land_approaches.h"

#include "mission_route_types.h"

#include <lib/geo/geo.h>

namespace mission_route
{

namespace
{

bool scanVtolLandApproachBlock(const Provider &provider, int safe_point_index, float home_altitude_amsl,
			       land_approaches_s *result)
{
	uint8_t valid_approach_counter = 0;
	const int safe_point_count = provider.safePointCount();

	for (int current_seq = safe_point_index + 1; current_seq < safe_point_count; ++current_seq) {
		mission_item_s mission_item{};

		if (!provider.loadSafePointItem(current_seq, mission_item)) {
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_RALLY_POINT) {
			// A rally point starts the next block, break
			break;
		}

		if (mission_item.nav_cmd != NAV_CMD_LOITER_TO_ALT
		    || valid_approach_counter >= land_approaches_s::num_approaches_max) {
			continue;
		}

		const loiter_point_s approach = makeVtolLandApproachPoint(mission_item, home_altitude_amsl);

		if (approach.isValid()) {
			if (result) {
				result->approaches[valid_approach_counter] = approach;
				valid_approach_counter++;

			} else {
				return true; // Early exit for the check-only path
			}
		}
	}

	return result ? (valid_approach_counter > 0) : false;
}

bool findAssociatedSafePointIndex(const Provider &provider, const PositionYawSetpoint &rtl_position,
				  float home_altitude_amsl, int &safe_point_index, mission_item_s &safe_point_item)
{
	for (int current_seq = 0; current_seq < provider.safePointCount(); ++current_seq) {
		mission_item_s mission_item{};

		if (!provider.loadSafePointItem(current_seq, mission_item)) {
			break;
		}

		if (mission_item.nav_cmd != NAV_CMD_RALLY_POINT) {
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(mission_item, home_altitude_amsl, safe_point_position)) {
			continue;
		}

		const float dist_to_safepoint = get_distance_to_next_waypoint(safe_point_position.lat, safe_point_position.lon,
						rtl_position.lat, rtl_position.lon);

		if (dist_to_safepoint < kLandApproachAssociationDistanceM) {
			safe_point_index = current_seq;
			safe_point_item = mission_item;
			return true;
		}
	}

	return false;
}

} // namespace

land_approaches_s getVtolLandApproachesNearLocation(const Provider &provider,
		const PositionYawSetpoint &rtl_position, float home_altitude_amsl)
{
	int safe_point_index = -1;
	mission_item_s safe_point_item{};

	if (!findAssociatedSafePointIndex(provider, rtl_position, home_altitude_amsl, safe_point_index, safe_point_item)) {
		return {};
	}

	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.land_location_lat_lon = matrix::Vector2d(safe_point_item.lat, safe_point_item.lon);
	scanVtolLandApproachBlock(provider, safe_point_index, home_altitude_amsl, &vtol_land_approaches);
	return vtol_land_approaches;
}

land_approaches_s getVtolLandApproachesAtSafePointIndex(const Provider &provider, int safe_point_index,
		float home_altitude_amsl)
{
	mission_item_s safe_point_item{};
	Position safe_point_position{};

	if (!provider.loadSafePointItem(safe_point_index, safe_point_item)
	    || safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT
	    || !extractSafePointPosition(safe_point_item, home_altitude_amsl, safe_point_position)) {
		return {};
	}

	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.land_location_lat_lon = matrix::Vector2d(safe_point_position.lat, safe_point_position.lon);
	scanVtolLandApproachBlock(provider, safe_point_index, home_altitude_amsl, &vtol_land_approaches);
	return vtol_land_approaches;
}

bool hasVtolLandApproachesNearLocation(const Provider &provider, const PositionYawSetpoint &rtl_position,
				       float home_altitude_amsl)
{
	int safe_point_index = -1;
	mission_item_s safe_point_item{};

	return findAssociatedSafePointIndex(provider, rtl_position, home_altitude_amsl, safe_point_index, safe_point_item)
	       && hasVtolLandApproachesAtSafePointIndex(provider, safe_point_index, home_altitude_amsl);
}

bool hasVtolLandApproachesAtSafePointIndex(const Provider &provider, int safe_point_index,
		float home_altitude_amsl)
{
	return scanVtolLandApproachBlock(provider, safe_point_index, home_altitude_amsl, nullptr);
}

bool anySafePointHasVtolLandApproach(const Provider &provider, float home_altitude_amsl)
{
	const int safe_point_count = provider.safePointCount();

	for (int safe_point_index = 0; safe_point_index < safe_point_count; ++safe_point_index) {
		mission_item_s safe_point_item{};

		// Only rally points with a valid position can anchor a landing-approach block.
		if (!provider.loadSafePointItem(safe_point_index, safe_point_item)
		    || safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT) {
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(safe_point_item, home_altitude_amsl, safe_point_position)) {
			continue;
		}

		if (hasVtolLandApproachesAtSafePointIndex(provider, safe_point_index, home_altitude_amsl)) {
			return true;
		}
	}

	return false;
}

} // namespace mission_route
