/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <navigator/terrain_follower_wrapper.hpp>

using namespace matrix;
using Vector2d = matrix::Vector2<double>;

class FakeTerrainProvider : public terrain::TerrainProvider
{
public:
	FakeTerrainProvider() :
		TerrainProvider(200)
	{

	}

	virtual ~FakeTerrainProvider() {};

	bool lookup(double lat, double lon, float &alt_msl_m) override
	{
		alt_msl_m = _terrain_value_to_return;
		return true;
	}

	void setTerrainValueToReturn(float val) { _terrain_value_to_return = val; }

private:
	float _terrain_value_to_return{0.0f};

};

class TerrainFollowerStateSetter
{
public:
	TerrainFollowerStateSetter(TerrainFollowerWrapper &terrain_follower):
		_terrain_follower(terrain_follower)
	{

		_terrain_follower.setHomeAltitude(_home_alt_amsl_m);
		_ref.initReference(_home_global(0), _home_global(1));
	}

	~TerrainFollowerStateSetter() {};

	matrix::Vector3<double> setCurrentPosNEU(const Vector3f &pos_NEU)
	{
		double lat, lon;
		_ref.reproject(pos_NEU(0), pos_NEU(1), lat, lon);
		_terrain_follower.setCurrentPosition(Vector2d(lat, lon), pos_NEU(2) + _home_alt_amsl_m);
		return matrix::Vector3<double> (lat, lon, pos_NEU(2) + _home_alt_amsl_m);
	}

	mission_item_s setTargetMissionItem(Vector3f pos_NEU)
	{
		double lat, lon;
		_ref.reproject(pos_NEU(0), pos_NEU(1), lat, lon);
		Vector2d target_pos_lat_lon(lat, lon);

		mission_item_s target_mission_item = {};
		target_mission_item.lat = target_pos_lat_lon(0);
		target_mission_item.lon = target_pos_lat_lon(1);

		target_mission_item.altitude = pos_NEU(2); // relative to home
		target_mission_item.altitude_is_relative = true;

		_terrain_follower.setCurrentMissionItem(target_mission_item);

		return target_mission_item;
	}

	float getHomeAlt() { return _home_alt_amsl_m; }

	const Vector2d &getHomePosition() { return _home_global; }

private:

	TerrainFollowerWrapper &_terrain_follower;

	Vector2d _home_global = Vector2d(42.1, 8.2);
	float _home_alt_amsl_m = 200.0;

	MapProjection _ref;

};

TEST(TerrainFollowerTest, FixedWing)
{
	FakeTerrainProvider _fake_terrain;
	TerrainFollowerWrapper terrain_follower;
	terrain_follower.enable();
	terrain_follower.setTerrainProvider(&_fake_terrain);
	TerrainFollowerStateSetter state_setter(terrain_follower);

	const float mission_alt_rel_m = 20.0f;
	const float ground_speed = 20.0f;
	bool is_hovercraft = false;
	const float loiter_radius = 60.0f;

	terrain_follower.setGroundSpeed(ground_speed);

	terrain_follower.setIsHoverCraft(is_hovercraft);

	terrain_follower.setLoiterRadius(loiter_radius);

	Vector3f current_pos_local_NEU(0, 0, mission_alt_rel_m);
	state_setter.setCurrentPosNEU(current_pos_local_NEU);

	// set target position 2km ahead to the north, at 20m above home
	Vector3f target_pos_local_NEU(2000, 0, mission_alt_rel_m);
	state_setter.setTargetMissionItem(target_pos_local_NEU);

	_fake_terrain.setTerrainValueToReturn(state_setter.getHomeAlt() + 100.0f); // this violates terrain

	// there is terrain violation, so should return true
	bool ret = terrain_follower.updateIntermediateMissionItem(false);
	ASSERT_TRUE(ret == true);

	mission_item_s intermediate_mission_item = terrain_follower.getIntermediateMissionItem();

	// expect altitude to be 200m above home (100m clearance + 100m terrain + 50m to be within band between min dist and max dist)
	ASSERT_EQ(intermediate_mission_item.altitude, state_setter.getHomeAlt() + 100.0f + 100.0f + 50.0f);

	// position should be equal to our current position which is the home position
	ASSERT_EQ(intermediate_mission_item.lat, state_setter.getHomePosition()(0));
	ASSERT_EQ(intermediate_mission_item.lon, state_setter.getHomePosition()(1));
}

TEST(TerrainFollowerTest, test)
{
	FakeTerrainProvider _fake_terrain;
	TerrainFollowerWrapper terrain_follower;
	terrain_follower.enable();
	terrain_follower.setTerrainProvider(&_fake_terrain);
	TerrainFollowerStateSetter state_setter(terrain_follower);


	Vector3f current_pos_local_NEU(0, 0, 20.0f);
	state_setter.setCurrentPosNEU(current_pos_local_NEU);

	// not target item set, should return false
	bool ret = terrain_follower.updateIntermediateMissionItem(false);
	ASSERT_TRUE(ret == false);


	// set target position 2km ahead to the north, at 20m above home
	Vector3f target_pos_local_NEU(2000, 0, 20.0f);
	state_setter.setTargetMissionItem(target_pos_local_NEU);

	_fake_terrain.setTerrainValueToReturn(state_setter.getHomeAlt() +
					      10.0f); // this only gives a 10m alt clearance, min is 100m

	// expect to return true, as we are violating terrain
	ret = terrain_follower.updateIntermediateMissionItem(false);

	ASSERT_TRUE(ret = true);

	mission_item_s intermediate_mission_item = terrain_follower.getIntermediateMissionItem();

	// expect intermediate mission item with altitude that does not violate (100m min dist, 10m terrain, 50m to be within band between min and max dist)
	ASSERT_EQ(intermediate_mission_item.altitude, state_setter.getHomeAlt() + 100.0f + 10.0f + 50.0f);

	// expect the terrain follower to tell use that we have an intermediate item
	ASSERT_TRUE(terrain_follower.hasIntermediateMissionItem());

	// we are now reaching the intermediate waypoint
	Vector3f new_pos = current_pos_local_NEU + Vector3f(100.0f, 0, 0);
	state_setter.setCurrentPosNEU(new_pos);

	// we expect this to return true as we need to insert more intermediate waypoint in order not to violate terrain
	ASSERT_TRUE(terrain_follower.updateIntermediateMissionItem(true));

	intermediate_mission_item = terrain_follower.getIntermediateMissionItem();

	// expect intermediate mission item with altitude that does not violate (100m min dist, 10m terrain, 50m to be within band between min and max dist)
	ASSERT_EQ(intermediate_mission_item.altitude, state_setter.getHomeAlt() + 100.0f + 10.0f + 50.0f);

	// change the terrain so that no more intermediate waypoint needs to be inserted
	_fake_terrain.setTerrainValueToReturn(state_setter.getHomeAlt() - 100.0f); // this give plenty of clearance

	// intermediate mission item has been reached and terrain does not violate. this still returns true since we need to reload the original mission item
	ASSERT_TRUE(terrain_follower.updateIntermediateMissionItem(true));

	// this indicates that there are no more intermediate mission items and thus the mission items from the original mission should be reloaded
	ASSERT_FALSE(terrain_follower.hasIntermediateMissionItem());

	// we are withing check_distance of the original target mission item and are currently not tracking any intermediate items
	new_pos(0) = 1950.0f;
	state_setter.setCurrentPosNEU(new_pos);

	// we are currently not tracking an intermediate waypoint and have come close to the target mission item, don't need to update
	ASSERT_FALSE(terrain_follower.updateIntermediateMissionItem(false));

	// original mission item still valid
	ASSERT_TRUE(terrain_follower.isCurrentMissionItemValid());

	// we have reached the target item, now the mission item is invalidated
	ASSERT_FALSE(terrain_follower.updateIntermediateMissionItem(true));
	ASSERT_FALSE(terrain_follower.isCurrentMissionItemValid());
}
