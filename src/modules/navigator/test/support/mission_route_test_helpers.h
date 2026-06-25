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
 * @file mission_route_test_helpers.h
 *
 * Shared mission-route test helpers.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <gtest/gtest.h>

#include "mission_route_provider.h"
#include "mission_route_types.h"
#include "vector_mission_item_store.h"

#include <lib/geo/geo.h>

#include <vector>

namespace navigator_test
{

class VectorMissionRouteProvider : public mission_route::Provider
{
public:
	VectorMissionRouteProvider(const std::vector<mission_item_s> &mission_items,
				   const std::vector<mission_item_s> &safe_point_items,
				   const std::vector<int32_t> &faulty_mission_indices = {},
				   const std::vector<int32_t> &faulty_safe_point_indices = {},
				   int32_t land_index = -1)
	{
		_mission_items.setItems(mission_items);
		_safe_point_items.setItems(safe_point_items);
		_mission_items.setLoadFailureIndices(faulty_mission_indices);
		_safe_point_items.setLoadFailureIndices(faulty_safe_point_indices);
		_land_index = land_index;
	}

	int missionCount() const override { return static_cast<int>(_mission_items.itemCount()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		++_mission_load_count;
		return _mission_items.loadItem(index, mission_item);
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.itemCount()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		++_safe_point_load_count;
		return _safe_point_items.loadItem(index, safe_point_item);
	}

	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const override
	{
		if (_land_index < 0 || _land_index >= missionCount()) {
			return false;
		}

		mission_item_s item{};

		if (!loadMissionItem(_land_index, item)) {
			return false;
		}

		if (item.nav_cmd != NAV_CMD_LAND && item.nav_cmd != NAV_CMD_VTOL_LAND) {
			return false;
		}

		index = _land_index;
		land_item = item;
		return true;
	}

	bool scanVtolLandApproachBlockForTest(int safe_point_index, float home_altitude_amsl,
					      land_approaches_s *result) const
	{
		return scanVtolLandApproachBlock(safe_point_index, home_altitude_amsl, result);
	}

	void resetCounters() const
	{
		_mission_load_count = 0;
		_safe_point_load_count = 0;
	}

	int missionLoadCount() const { return _mission_load_count; }
	int safePointLoadCount() const { return _safe_point_load_count; }

private:
	VectorMissionItemStore _mission_items{};
	VectorMissionItemStore _safe_point_items{};
	int32_t _land_index{-1};
	mutable int _mission_load_count{0};
	mutable int _safe_point_load_count{0};
};

static inline mission_item_s makePositionItem(double lat, double lon, float alt,
		uint16_t nav_cmd = NAV_CMD_WAYPOINT)
{
	mission_item_s item{};
	item.lat = lat;
	item.lon = lon;
	item.altitude = alt;
	item.nav_cmd = nav_cmd;
	item.frame = NAV_FRAME_GLOBAL;
	item.altitude_is_relative = false;
	item.autocontinue = true;
	return item;
}

static inline mission_item_s makePositionItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint16_t nav_cmd = NAV_CMD_WAYPOINT, bool autocontinue = true)
{
	double lat = base_lat;
	double lon = base_lon;
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &lat, &lon);

	mission_item_s item = makePositionItem(lat, lon, alt, nav_cmd);
	item.autocontinue = autocontinue;
	return item;
}

static inline mission_item_s makeTakeoffItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_TAKEOFF, false);
}

static inline mission_item_s makeLandItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_LAND, false);
}

static inline mission_item_s makeTakeoffItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_TAKEOFF);
	item.autocontinue = false;
	return item;
}

static inline mission_item_s makeLandItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_LAND);
	item.autocontinue = false;
	return item;
}

static inline mission_item_s makeDoJump(int16_t jump_target_index, uint16_t repeat_count,
					uint16_t current_count = 0)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_JUMP;
	item.do_jump_mission_index = jump_target_index;
	item.do_jump_repeat_count = repeat_count;
	item.do_jump_current_count = current_count;
	return item;
}

static inline mission_item_s makeVtolTransitionItem(uint8_t target_state)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(target_state);
	return item;
}

static inline mission_item_s makeSafePointFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

static inline mission_item_s makeSafePointAbsolute(double lat, double lon, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

static inline mission_route::Position makePositionFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	mission_route::Position position{};
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &position.lat, &position.lon);
	position.alt = alt;
	return position;
}

static inline mission_route::Position makePositionAbsolute(double lat, double lon, float alt)
{
	return mission_route::Position{lat, lon, alt};
}

namespace route_test_reference
{
static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;
}

static constexpr double kLatLonToleranceDeg = 1e-7;
static constexpr float kAltitudeTolerance = 2.0f;
static constexpr float kDistanceTolerance = 5.0f;

} // namespace navigator_test

using navigator_test::VectorMissionRouteProvider;
using VectorProvider = navigator_test::VectorMissionRouteProvider;
using navigator_test::kAltitudeTolerance;
using navigator_test::kDistanceTolerance;
using navigator_test::kLatLonToleranceDeg;
using navigator_test::makeDoJump;
using navigator_test::makeLandItem;
using navigator_test::makeLandItemFromOffset;
using navigator_test::makePositionAbsolute;
using navigator_test::makePositionFromOffset;
using navigator_test::makePositionItem;
using navigator_test::makePositionItemFromOffset;
using navigator_test::makeSafePointAbsolute;
using navigator_test::makeSafePointFromOffset;
using navigator_test::makeTakeoffItem;
using navigator_test::makeTakeoffItemFromOffset;
using navigator_test::makeVtolTransitionItem;
