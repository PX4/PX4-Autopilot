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
 * @file test_mission_block_terrain.cpp
 *
 * Mission items with a terrain-relative altitude (NAV_FRAME_GLOBAL_TERRAIN_ALT): their position setpoint carries
 * agl_alt, they are reached by the distance to the ground, and items built from their setpoint do not take over the
 * stale AMSL estimate of the setpoint.
 */

#include <gtest/gtest.h>

#include "mission_block.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>

namespace
{

constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kHomeAlt = 400.f;
constexpr float kTerrainAlt = 450.f; // terrain below the vehicle
constexpr float kVehicleAlt = 480.f;
constexpr float kItemAgl = 30.f;

} // namespace

class MissionBlockTerrainTestPeer : public MissionBlock
{
public:
	explicit MissionBlockTerrainTestPeer(Navigator *navigator)
		: MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {}

	mission_item_s &item() { return _mission_item; }
	bool reached() { return is_mission_item_reached_or_completed(); }
	void reset() { reset_mission_item_reached(); }
	using MissionBlock::mission_item_to_position_setpoint;
	using MissionBlock::setLoiterItemFromCurrentPositionSetpoint;
};

class MissionBlockTerrainTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};
	MissionBlockTerrainTestPeer _block{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		vehicle_status_s *status = _navigator.get_vstatus();
		status->timestamp = hrt_absolute_time();
		status->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status->is_vtol = false;

		_navigator.get_land_detected()->landed = false;
		_navigator.get_home_position()->alt = kHomeAlt;

		vehicle_global_position_s *global = _navigator.get_global_position();
		global->timestamp = hrt_absolute_time();
		global->lat = kLat;
		global->lon = kLon;
		global->alt = kVehicleAlt;
		setTerrain(true, kVehicleAlt - kTerrainAlt);

		mission_item_s &item = _block.item();
		item = {};
		item.nav_cmd = NAV_CMD_WAYPOINT;
		item.lat = kLat;
		item.lon = kLon;
		item.altitude = kItemAgl;
		item.frame = NAV_FRAME_GLOBAL_TERRAIN_ALT;
		item.yaw = NAN;
		item.acceptance_radius = NAN;
		item.time_inside = 0.f;
		_block.reset();
	}

	void setTerrain(bool valid, float dist_bottom)
	{
		vehicle_global_position_s *global = _navigator.get_global_position();
		global->terrain_alt = global->alt - dist_bottom;
		global->terrain_alt_valid = valid;

		vehicle_local_position_s *local = _navigator.get_local_position();
		local->timestamp = hrt_absolute_time();
		local->dist_bottom = dist_bottom;
		local->dist_bottom_valid = valid;
	}
};

TEST_F(MissionBlockTerrainTest, SetpointCarriesAglAltitude)
{
	position_setpoint_s sp{};
	ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));

	EXPECT_FLOAT_EQ(sp.agl_alt, kItemAgl);
	// alt is the estimate with the terrain below the vehicle
	EXPECT_FLOAT_EQ(sp.alt, kTerrainAlt + kItemAgl);

	// without a terrain estimate, the estimate assumes the terrain at the home altitude
	setTerrain(false, NAN);
	ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));
	EXPECT_FLOAT_EQ(sp.agl_alt, kItemAgl);
	EXPECT_FLOAT_EQ(sp.alt, kHomeAlt + kItemAgl);
}

TEST_F(MissionBlockTerrainTest, TerrainIntFrameCarriesAglAltitude)
{
	// the deprecated _INT synonym is stored as uploaded and handled the same
	_block.item().frame = NAV_FRAME_GLOBAL_TERRAIN_ALT_INT;

	position_setpoint_s sp{};
	ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));
	EXPECT_FLOAT_EQ(sp.agl_alt, kItemAgl);
}

TEST_F(MissionBlockTerrainTest, OtherFramesHaveNoAglAltitude)
{
	position_setpoint_s sp{};

	for (const uint8_t frame : {NAV_FRAME_GLOBAL, NAV_FRAME_GLOBAL_RELATIVE_ALT, NAV_FRAME_GLOBAL_INT, NAV_FRAME_GLOBAL_RELATIVE_ALT_INT}) {
		_block.item().frame = frame;
		sp.agl_alt = 0.f;
		ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));
		EXPECT_FALSE(PX4_ISFINITE(sp.agl_alt));
	}
}

TEST_F(MissionBlockTerrainTest, ReachedByDistanceToGround)
{
	// the AMSL altitude of the vehicle is far from the item's AGL value, the distance to the ground decides
	setTerrain(true, kItemAgl + 0.1f);
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockTerrainTest, NotReachedAtWrongHeightAboveGround)
{
	setTerrain(true, kItemAgl + 20.f);
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockTerrainTest, ReachedHorizontallyWithoutDistanceToGround)
{
	setTerrain(false, NAN);
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockTerrainTest, LoiterFromSetpointKeepsTerrainFrame)
{
	position_setpoint_s sp{};
	ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));

	mission_item_s loiter{};
	_block.setLoiterItemFromCurrentPositionSetpoint(loiter, sp);
	EXPECT_EQ(loiter.frame, NAV_FRAME_GLOBAL_TERRAIN_ALT);
	EXPECT_FLOAT_EQ(loiter.altitude, kItemAgl);
}

TEST_F(MissionBlockTerrainTest, AmslItemFromSetpointUsesCurrentTerrain)
{
	position_setpoint_s sp{};
	ASSERT_TRUE(_block.mission_item_to_position_setpoint(_block.item(), &sp));

	// the vehicle flew on over higher terrain: the setpoint estimate is stale
	setTerrain(true, 10.f); // terrain now at kVehicleAlt - 10

	mission_item_s loiter{};
	_block.setLoiterItemFromCurrentPositionSetpoint(loiter, sp, false);
	EXPECT_EQ(loiter.frame, NAV_FRAME_GLOBAL);
	EXPECT_FLOAT_EQ(loiter.altitude, kVehicleAlt - 10.f + kItemAgl);

	// without a terrain estimate, keep the vehicle altitude
	setTerrain(false, NAN);
	_block.setLoiterItemFromCurrentPositionSetpoint(loiter, sp, false);
	EXPECT_FLOAT_EQ(loiter.altitude, kVehicleAlt);
}
