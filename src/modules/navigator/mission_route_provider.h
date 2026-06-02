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
 * @file mission_route_provider.h
 *
 * Mission-route planner data-source interface.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"
#include "safe_point_land.hpp"

#include <stdint.h>

namespace mission_route
{

/**
 * @brief Data source used by the planner.
 *
 * Navigator passes MissionRouteCache here. Tests can pass an in-memory provider,
 * which keeps the route geometry code independent from dataman and uORB.
 */
class Provider
{
public:
	virtual ~Provider() = default;

	virtual int missionCount() const = 0;
	virtual bool loadMissionItem(int index, mission_item_s &mission_item) const = 0;
	virtual int safePointCount() const = 0;
	virtual bool loadSafePointItem(int index, mission_item_s &safe_point_item) const = 0;

	// Default scans backward for NAV_CMD_LAND / NAV_CMD_VTOL_LAND.
	virtual bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const;
	/**
	 * @brief Find the mission takeoff item.
	 *
	 * The default implementation skips leading non-position setup commands but stops as soon as the
	 * mission reaches its first other position-bearing item.
	 */
	virtual bool getMissionTakeoffItem(int32_t &index, mission_item_s &takeoff_item) const;

	/**
	 * @brief Read the landing-approach block associated with the first valid rally point near rtl_position.
	 *
	 * A block starts at the associated rally point and contains the consecutive NAV_CMD_LOITER_TO_ALT
	 * items that follow it. The next rally point starts a new block.
	 * Invalid rally points are skipped so a later nearby valid rally point can still be considered.
	 */
	virtual land_approaches_s getVtolLandApproachesNearLocation(const PositionYawSetpoint &rtl_position,
			float home_altitude_amsl) const;
	virtual bool hasVtolLandApproachesNearLocation(const PositionYawSetpoint &rtl_position,
			float home_altitude_amsl) const;
	virtual bool hasVtolLandApproachesAtSafePointIndex(int safe_point_index, float home_altitude_amsl) const;
	virtual bool anySafePointHasVtolLandApproach(float home_altitude_amsl) const;

protected:
	/**
	 * @brief Scan the rally block following one safe point for valid landing approaches.
	 *
	 * A block is the consecutive NAV_CMD_LOITER_TO_ALT items after safe_point_index; the next
	 * rally point starts a different block and stops the scan.
	 *
	 * If result is non-null, all valid approaches are collected into it.
	 * If result is null, returns true on the first valid approach (early exit).
	 *
	 * @return true if at least one valid approach was found.
	 */
	bool scanVtolLandApproachBlock(int safe_point_index, float home_altitude_amsl, land_approaches_s *result) const;

private:
	/**
	 * @brief Find the first rally point whose block should be associated with rtl_position.
	 *
	 * Invalid rally points are skipped so nearby valid fallbacks can still be associated.
	 * On success, safe_point_index and safe_point_item are populated with the rally point that starts the block.
	 */
	bool findAssociatedSafePointIndex(const PositionYawSetpoint &rtl_position, float home_altitude_amsl,
					  int &safe_point_index, mission_item_s &safe_point_item) const;
};

} // namespace mission_route
