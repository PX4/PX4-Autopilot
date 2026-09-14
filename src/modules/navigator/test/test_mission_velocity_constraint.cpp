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
 * @file test_mission_velocity_constraint.cpp
 *
 * Tests for the velocity constraint the navigator puts on the next setpoint: the gate deciding whether the
 * vehicle really flies through a waypoint instead of stopping at it, and the walk along the mission that
 * derives how fast the vehicle may leave the next waypoint.
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <mathlib/math/TrajMath.hpp>
#include <px4_platform_common/posix.h>
#include <uORB/topics/vehicle_status.h>

#include <vector>

using matrix::Vector3f;

class MissionVelocityConstraintTestPeer : public Mission
{
public:
	explicit MissionVelocityConstraintTestPeer(Navigator *navigator) : Mission(navigator) {}

	void useMissionDataman(dm_item_t dataman_id, int32_t count)
	{
		_mission.mission_dataman_id = static_cast<uint8_t>(dataman_id);
		_mission.count = count;
	}

	/* The lookahead only reads items from the cache, so prime it the way updateDatamanCache() does. */
	void cacheItems(dm_item_t dataman_id, int32_t count)
	{
		_dataman_cache.invalidate();

		for (int32_t index = 0; index < count; index++) {
			_dataman_cache.load(dataman_id, static_cast<uint32_t>(index));
		}

		const hrt_abstime start = hrt_absolute_time();

		while (_dataman_cache.isLoading() && (hrt_elapsed_time(&start) < 1_s)) {
			_dataman_cache.update();
			px4_usleep(1000);
		}
	}

	void dropCachedItems()
	{
		_dataman_cache.invalidate();
	}

	using Mission::isFlownThroughWithoutStopping;
	using Mission::setNextVelocityConstraint;
	using MissionBlock::mission_item_to_position_setpoint;
};

static constexpr double kOriginLat{47.0};
static constexpr double kOriginLon{8.0};

static mission_item_s makeWaypoint(double lat = kOriginLat, double lon = kOriginLon)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_WAYPOINT;
	item.autocontinue = true;
	item.lat = lat;
	item.lon = lon;
	item.altitude = 100.f;
	return item;
}

/* Plain waypoint at the given distance and bearing from the origin */
static mission_item_s makeWaypointAt(float bearing_rad, float distance_m)
{
	double lat{0.0};
	double lon{0.0};
	waypoint_from_heading_and_distance(kOriginLat, kOriginLon, bearing_rad, distance_m, &lat, &lon);
	return makeWaypoint(lat, lon);
}

static mission_item_s makeCommand(uint16_t nav_cmd)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.autocontinue = true;
	return item;
}

class MissionVelocityConstraintTest : public NavigatorDatamanTestBase
{
protected:
	static constexpr dm_item_t kMissionDataman{DM_KEY_WAYPOINTS_OFFBOARD_0};
	static constexpr float kNorth{0.f};
	static constexpr float kEast{M_PI_F / 2.f};

	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(kMissionDataman));
		/* get_time_inside() only holds a plain waypoint for a rotary wing, same as brake_for_hold, and the
		 * constraint is only produced for a rotary wing. */
		_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	void writeMission(const std::vector<mission_item_s> &items)
	{
		for (size_t index = 0; index < items.size(); index++) {
			mission_item_s item = items[index];
			ASSERT_TRUE(_dataman_client.writeSync(kMissionDataman, static_cast<uint32_t>(index),
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		_mission.useMissionDataman(kMissionDataman, static_cast<int32_t>(items.size()));
		_mission.cacheItems(kMissionDataman, static_cast<int32_t>(items.size()));
	}

	/* Run the navigator side for a triplet made of items[current_index] and items[next_index] */
	Vector3f constraintFor(const std::vector<mission_item_s> &items, int32_t current_index, int32_t next_index,
			       bool direction_backward = false)
	{
		position_setpoint_s current{};
		position_setpoint_s next{};
		_mission.mission_item_to_position_setpoint(items[current_index], &current);
		_mission.mission_item_to_position_setpoint(items[next_index], &next);
		EXPECT_FALSE(PX4_ISFINITE(next.velocity_constraint[0])) << "constraint must start out unknown";

		_mission.setNextVelocityConstraint(current, items[next_index], next_index, next, direction_backward);

		return Vector3f(next.velocity_constraint);
	}

	float cruiseSpeed() const { return _navigator.get_multicopter_trajectory_limits().max_speed_xy; }

	DatamanClient _dataman_client{};
	Navigator _navigator{};
	MissionVelocityConstraintTestPeer _mission{&_navigator};
};

/* ---------------------------------------------------------------------------------------------------------
 * Gate: does the vehicle fly through the waypoint
 * -------------------------------------------------------------------------------------------------------*/

TEST_F(MissionVelocityConstraintTest, PlainWaypointIsFlownThrough)
{
	// GIVEN: two consecutive plain waypoints
	const std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	writeMission(items);

	// THEN: the first one is passed at speed
	EXPECT_TRUE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionVelocityConstraintTest, WaypointWithTimeInsideStopsTheVehicle)
{
	// GIVEN: a waypoint the vehicle has to hold at
	std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	items[0].time_inside = 5.f;
	writeMission(items);

	// THEN: the vehicle must be able to stop there
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionVelocityConstraintTest, WaypointWithoutAutocontinueStopsTheVehicle)
{
	// GIVEN: a waypoint the mission does not continue past by itself
	std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	items[0].autocontinue = false;
	writeMission(items);

	// THEN: stop
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionVelocityConstraintTest, LoiterStopsTheVehicle)
{
	// GIVEN: a loiter item instead of a plain waypoint
	std::vector<mission_item_s> items{makeCommand(NAV_CMD_LOITER_TIME_LIMIT), makeWaypoint()};
	items[0].time_inside = 10.f;
	writeMission(items);

	// THEN: stop
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionVelocityConstraintTest, LandingStopsTheVehicle)
{
	// GIVEN: a landing instead of a plain waypoint
	const std::vector<mission_item_s> items{makeCommand(NAV_CMD_LAND), makeWaypoint()};
	writeMission(items);

	// THEN: stop
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionVelocityConstraintTest, DelayBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a delay between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DELAY), makeWaypoint()};
	writeMission(items);

	// THEN: stop, the vehicle waits at the waypoint
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionVelocityConstraintTest, VtolTransitionBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a transition between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_VTOL_TRANSITION), makeWaypoint()};
	writeMission(items);

	// THEN: stop
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionVelocityConstraintTest, DoJumpBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a jump between the waypoint and the following position item, which may redirect the mission
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_JUMP), makeWaypoint()};
	writeMission(items);

	// THEN: stop, the following item is not necessarily the one the jump leads to
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionVelocityConstraintTest, PayloadCommandWithTimeoutBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a payload command the vehicle waits for between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_GRIPPER), makeWaypoint()};
	writeMission(items);

	// THEN: stop
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionVelocityConstraintTest, HarmlessCommandBetweenPositionItemsIsFlownThrough)
{
	// GIVEN: a command that does not hold the vehicle between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_CHANGE_SPEED), makeWaypoint()};
	writeMission(items);

	// THEN: the waypoint is still flown through
	EXPECT_TRUE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionVelocityConstraintTest, ItemsInBetweenAreCheckedWhenFlyingBackwards)
{
	// GIVEN: a mission flown backwards (reverse RTL), with a delay between the two position items
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DELAY), makeWaypoint()};
	writeMission(items);

	// THEN: the delay is found in between and stops the vehicle, a plain waypoint after it is flown through
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[2], 2, 0));
	EXPECT_TRUE(_mission.isFlownThroughWithoutStopping(items[2], 2, 1));
}

TEST_F(MissionVelocityConstraintTest, UncachedItemInBetweenStopsTheVehicle)
{
	// GIVEN: an item in between that is not in the dataman cache
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_CHANGE_SPEED), makeWaypoint()};
	writeMission(items);
	_mission.dropCachedItems();

	// THEN: the cache miss is treated like an item that stops the vehicle
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

/* ---------------------------------------------------------------------------------------------------------
 * Constraint: how fast may the vehicle leave the next waypoint
 * -------------------------------------------------------------------------------------------------------*/

TEST_F(MissionVelocityConstraintTest, StraightMissionAllowsCruiseSpeedThroughNext)
{
	// GIVEN: a straight line of waypoints heading north, spaced further apart than the braking distance
	const float spacing = 2.f * _navigator.get_multicopter_braking_distance(cruiseSpeed());
	const std::vector<mission_item_s> items{
		makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, spacing), makeWaypointAt(kNorth, 2.f * spacing),
		makeWaypointAt(kNorth, 3.f * spacing)};
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: it may leave the next waypoint at cruise speed, heading north
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_NEAR(constraint.norm(), cruiseSpeed(), 1e-3f);
	EXPECT_NEAR(constraint(0), cruiseSpeed(), 1e-3f);
	EXPECT_NEAR(constraint(1), 0.f, 1e-3f);
	EXPECT_FLOAT_EQ(constraint(2), 0.f);
}

TEST_F(MissionVelocityConstraintTest, CloseWaypointsAreNoStopOnAStraightPath)
{
	// GIVEN: a survey-like entry: the next waypoint only a few metres past the current one on a straight line
	// that continues far beyond, all of them within the acceptance radius of each other
	const float spacing = 2.f;
	const float horizon = _navigator.get_multicopter_braking_distance(cruiseSpeed());
	const std::vector<mission_item_s> items{
		makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, spacing), makeWaypointAt(kNorth, 2.f * spacing),
		makeWaypointAt(kNorth, 2.f * spacing + 2.f * horizon)};
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: the walk goes past the close waypoints and lets the vehicle carry cruise speed through next
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_NEAR(constraint.norm(), cruiseSpeed(), 1e-3f);
}

TEST_F(MissionVelocityConstraintTest, ShortLastSegmentLimitsTheSpeedLeavingNext)
{
	// GIVEN: after the next waypoint the mission ends a few metres further
	const float last_segment = 2.f;
	const std::vector<mission_item_s> items{
		makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, 30.f), makeWaypointAt(kNorth, 30.f + last_segment)};
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: it may only leave next as fast as a stop at the end of the short segment allows
	const math::trajectory::VehicleDynamicLimits limits = _navigator.get_multicopter_trajectory_limits();
	const float expected = math::trajectory::computeMaxSpeedFromDistance(limits.max_jerk, limits.max_acc_xy,
			       last_segment, 0.f);
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_GT(constraint.norm(), 0.f);
	EXPECT_LT(constraint.norm(), cruiseSpeed());
	EXPECT_NEAR(constraint.norm(), expected, 1e-2f);
}

TEST_F(MissionVelocityConstraintTest, CornerAfterNextLimitsTheSpeedAndSetsTheDirection)
{
	// GIVEN: the path turns by 90 degrees at the next waypoint, then continues east for a long way
	const float spacing = 30.f;
	std::vector<mission_item_s> items{makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, spacing)};
	double lat{0.0};
	double lon{0.0};
	waypoint_from_heading_and_distance(items[1].lat, items[1].lon, kEast, 200.f, &lat, &lon);
	items.push_back(makeWaypoint(lat, lon));
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: the turn at next limits the speed below cruise, and the constraint points east, along the segment
	// after next
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_GT(constraint.norm(), 0.f);
	EXPECT_LT(constraint.norm(), cruiseSpeed());
	EXPECT_NEAR(constraint(0), 0.f, 1e-2f);
	EXPECT_GT(constraint(1), 0.f);
}

TEST_F(MissionVelocityConstraintTest, StopAtNextGivesZeroConstraint)
{
	// GIVEN: the next waypoint has a hold time
	std::vector<mission_item_s> items{makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, 30.f), makeWaypointAt(kNorth, 60.f)};
	items[1].time_inside = 5.f;
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: the vehicle stops at next, which is a known zero, not an unknown
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_FLOAT_EQ(constraint.norm(), 0.f);
}

TEST_F(MissionVelocityConstraintTest, EndOfMissionAfterNextGivesZeroConstraint)
{
	// GIVEN: the next waypoint is the last one
	const std::vector<mission_item_s> items{makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, 30.f)};
	writeMission(items);

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: stop
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_FLOAT_EQ(constraint.norm(), 0.f);
}

TEST_F(MissionVelocityConstraintTest, CacheMissAfterNextGivesZeroConstraint)
{
	// GIVEN: a straight mission whose items are not in the dataman cache
	const std::vector<mission_item_s> items{makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, 30.f), makeWaypointAt(kNorth, 60.f)};
	writeMission(items);
	_mission.dropCachedItems();

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: the walk ends at next and treats it as a stop, the safe assumption
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_FLOAT_EQ(constraint.norm(), 0.f);
}

TEST_F(MissionVelocityConstraintTest, FixedWingLeavesTheConstraintUnknown)
{
	// GIVEN: a straight mission flown by a fixed wing
	const std::vector<mission_item_s> items{makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, 30.f), makeWaypointAt(kNorth, 60.f)};
	writeMission(items);
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	// WHEN: the vehicle flies from the first to the second waypoint
	const Vector3f constraint = constraintFor(items, 0, 1);

	// THEN: only the multicopter trajectory planner consumes it, nothing is produced
	EXPECT_FALSE(constraint.isAllFinite());
}

TEST_F(MissionVelocityConstraintTest, WalkFollowsTheMissionBackwards)
{
	// GIVEN: a straight mission flown backwards, from the last waypoint towards the first
	const float spacing = 30.f;
	const std::vector<mission_item_s> items{
		makeWaypointAt(kNorth, 0.f), makeWaypointAt(kNorth, spacing), makeWaypointAt(kNorth, 2.f * spacing),
		makeWaypointAt(kNorth, 3.f * spacing)};
	writeMission(items);

	// WHEN: the vehicle flies from the last to the third waypoint
	const Vector3f constraint = constraintFor(items, 3, 2, true);

	// THEN: it may leave the third waypoint at cruise speed, heading south
	ASSERT_TRUE(constraint.isAllFinite());
	EXPECT_NEAR(constraint.norm(), cruiseSpeed(), 1e-3f);
	EXPECT_LT(constraint(0), 0.f);
}
