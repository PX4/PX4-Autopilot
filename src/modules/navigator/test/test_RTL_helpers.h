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
 * @file test_RTL_helpers.h
 *
 * Shared helpers for MissionRoutePlanner unit tests.
 * Provides in-memory Provider implementations, mission item factory
 * functions, default configs, tolerance constants, and verification
 * helpers.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <gtest/gtest.h>

#include "mission_route_planner.h"

#include <lib/geo/geo.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include <cmath>
#include <vector>

// ============================================================================
// Provider implementations for testing
// ============================================================================

/**
 * @brief In-memory planner provider for all MissionRoutePlanner tests.
 *
 * Supports two optional capabilities on top of basic vector-backed access:
 * - Fault injection: indices listed in faulty_mission_indices / faulty_safe_point_indices
 *   will fail to load, simulating SD card read errors.
 * - Load counting: missionLoadCount() / safePointLoadCount() track how many reads
 *   the planner performed, useful for verifying batching behavior.
 *
 * When constructed with just mission + safe-point vectors (the common case),
 * fault lists are empty and counters can be ignored.
 */
class VectorProvider : public MissionRoutePlanner::Provider
{
public:
	VectorProvider(std::vector<mission_item_s> mission_items,
		       std::vector<mission_item_s> safe_point_items,
		       std::vector<int> faulty_mission_indices = {},
		       std::vector<int> faulty_safe_point_indices = {}) :
		_mission_items(std::move(mission_items)),
		_safe_point_items(std::move(safe_point_items)),
		_faulty_mission_indices(std::move(faulty_mission_indices)),
		_faulty_safe_point_indices(std::move(faulty_safe_point_indices))
	{
	}

	int missionCount() const override { return static_cast<int>(_mission_items.size()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		++_mission_load_count;

		for (int fi : _faulty_mission_indices) {
			if (fi == index) {
				return false;
			}
		}

		if (index < 0 || index >= missionCount()) {
			return false;
		}

		mission_item = _mission_items[index];
		return true;
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.size()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		++_safe_point_load_count;

		for (int fi : _faulty_safe_point_indices) {
			if (fi == index) {
				return false;
			}
		}

		if (index < 0 || index >= safePointCount()) {
			return false;
		}

		safe_point_item = _safe_point_items[index];
		return true;
	}

	void resetCounters() const
	{
		_mission_load_count = 0;
		_safe_point_load_count = 0;
	}

	int missionLoadCount() const { return _mission_load_count; }
	int safePointLoadCount() const { return _safe_point_load_count; }

private:
	std::vector<mission_item_s> _mission_items;
	std::vector<mission_item_s> _safe_point_items;
	std::vector<int> _faulty_mission_indices;
	std::vector<int> _faulty_safe_point_indices;
	mutable int _mission_load_count{0};
	mutable int _safe_point_load_count{0};
};

// ============================================================================
// Mission item factory helpers
// ============================================================================

// These helpers stay header-only because only a small set of navigator test TUs include
// them today. If reuse widens or the bodies become materially larger, move them into a
// shared test-support .cpp to avoid duplicated code generation.

/** @brief Build an absolute mission item at the given coordinates. */
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

/** @brief Build a mission item from a local offset relative to the reference point. */
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

/** @brief Build a takeoff mission item with a fixed landing-style altitude. */
static inline mission_item_s makeTakeoffItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_TAKEOFF, false);
}

/** @brief Build a land mission item with an absolute altitude. */
static inline mission_item_s makeLandItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_LAND, false);
}

/** @brief Build a takeoff mission item at absolute GPS coordinates. */
static inline mission_item_s makeTakeoffItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_TAKEOFF);
	item.autocontinue = false;
	return item;
}

/** @brief Build a land mission item at absolute GPS coordinates. */
static inline mission_item_s makeLandItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_LAND);
	item.autocontinue = false;
	return item;
}

/** @brief Build a DO_JUMP item for loop handling tests. */
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

/** @brief Build a VTOL transition command for route-state tests. */
static inline mission_item_s makeVtolTransitionItem(uint8_t target_state)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(target_state);
	return item;
}

/** @brief Build a rally point from a local offset relative to the reference point. */
static inline mission_item_s makeSafePointFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

/** @brief Build a rally point at absolute GPS coordinates. */
static inline mission_item_s makeSafePointAbsolute(double lat, double lon, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

/** @brief Build a route-planner position from a local offset relative to the reference point. */
static inline MissionRoutePlanner::Position makePositionFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	MissionRoutePlanner::Position position{};
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &position.lat, &position.lon);
	position.alt = alt;
	return position;
}

/** @brief Build a route-planner position at absolute GPS coordinates. */
static inline MissionRoutePlanner::Position makePositionAbsolute(double lat, double lon, float alt)
{
	return MissionRoutePlanner::Position{lat, lon, alt};
}

// ============================================================================
// Default config helpers
// ============================================================================

/** @brief Default route-planner config for route-following tests (multicopter). */
static inline MissionRoutePlanner::Config defaultConfig()
{
	MissionRoutePlanner::Config config{};
	config.parameters.vehicle_projection_search_dist = 60.f;
	config.parameters.safe_point_projection_search_dist = 60.f;
	config.parameters.acceptance_radius = 10.f;
	config.parameters.direct_acceptance_radius = 10.f;
	config.parameters.home_altitude_amsl = 500.f;
	config.state.is_multicopter = true;
	return config;
}

/** @brief Config for fixed-wing vehicle tests. */
static inline MissionRoutePlanner::Config fwConfig()
{
	MissionRoutePlanner::Config config = defaultConfig();
	config.state.is_multicopter = false;
	config.state.is_fixed_wing = true;
	config.parameters.u_turn_penalty_m = 4000.f;
	return config;
}

// ============================================================================
// Shared reference geometry and tolerance constants
// ============================================================================

namespace rtl_test_reference
{
static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;
}

static constexpr double kLatLonToleranceDeg = 1e-7;     // ~1 cm at equator
static constexpr float kAltitudeTolerance = 2.0f;       // meters
static constexpr float kDistanceTolerance = 5.0f;       // meters

// ============================================================================
// Test fixture base class
// ============================================================================

/**
 * @brief Base fixture for MissionRoutePlanner tests.
 *
 * Provides default config, projection context, and failure reason as
 * class members so individual tests do not need to repeat the same
 * 3-line declaration block.  Tests that need a different config
 * (e.g. fwConfig()) simply reassign the member.
 */
class MissionRoutePlannerTestBase : public ::testing::Test
{
protected:
	MissionRoutePlanner::Config config = defaultConfig();
	MissionRoutePlanner::ProjectionContext ctx{};
	MissionRoutePlanner::FailureReason reason{};
};
