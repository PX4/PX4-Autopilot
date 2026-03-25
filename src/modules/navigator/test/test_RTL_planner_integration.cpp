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
 * @file test_RTL_planner_integration.cpp
 *
 * End-to-end tests for MissionRoutePlanner::planRouteToGoal.
 * Covers fallback to mission endpoints, join altitude handling,
 * branch-off caching, safe-point selection, error handling,
 * faulty provider degradation, VTOL mission planning, and
 * stage-machine contract verification.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

#include <cmath>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

// ============================================================================
// Test fixture
// ============================================================================

class RtlPlannerIntegrationTest : public MissionRoutePlannerTestBase
{
protected:
	MissionRoutePlanner::Plan plan{};
};

// =============================================================================
// GROUP 1: Fallback to mission endpoints when no safe points exist
// =============================================================================

// WHY: When no safe points are configured, the planner must fall back to mission endpoints.
// WHAT: A vehicle close to the landing waypoint should select MissionLand as goal.
TEST_F(RtlPlannerIntegrationTest, FallsBackToMissionLandWhenNoSafePoints)
{
	// GIVEN: A 4-waypoint mission (takeoff -> wp -> wp -> land) with no safe points.
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle is at N+260, close to the landing waypoint.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 260.f, 0.f, kAlt + 25.f);
	config = fwConfig();

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 2, config, plan, &reason);

	// THEN: Planning succeeds and selects MissionLand.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_NEAR(plan.selection.goal_position.lat, items[3].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, items[3].lon, kLatLonToleranceDeg);
	EXPECT_FALSE(plan.selection.path.direction_reversed);
}

// WHY: When the vehicle is near takeoff and the path back is shorter, the planner should prefer MissionTakeoff.
// WHAT: Vehicle at N+40 with a long mission extending to N+5200 should select takeoff with a reversed path.
TEST_F(RtlPlannerIntegrationTest, FallsBackToMissionTakeoffWhenPathIsShorter)
{
	// GIVEN: A 4-waypoint mission where land is far away (N+5200).
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 5100.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5200.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle is at N+40, close to takeoff.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt + 10.f);
	config = fwConfig();
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds and selects MissionTakeoff with reversed direction.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_NEAR(plan.selection.goal_position.lat, items[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, items[0].lon, kLatLonToleranceDeg);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_TRUE(plan.selection.path.u_turn_required);
	EXPECT_NEAR(plan.selection.path.dist, 4040.f, kDistanceTolerance);
}

// WHY: Reverse fallback to the takeoff endpoint must land at the configured home AMSL reference,
//      not at the takeoff climb altitude stored in the mission item.
// WHAT: A MissionTakeoff fallback keeps the takeoff XY position but rewrites the goal altitude to home AMSL.
TEST_F(RtlPlannerIntegrationTest, MissionTakeoffFallbackUsesHomeAltitudeReference)
{
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, 650.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, 670.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 5100.f, 0.f, 680.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5200.f, 0.f, 640.f),
	};
	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, 655.f);
	config = fwConfig();
	config.home_altitude_amsl = 600.f;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	ASSERT_TRUE(planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason));
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_NEAR(plan.selection.goal_position.lat, items[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, items[0].lon, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.alt, 600.f, kAltitudeTolerance);
}

// WHY: Mission smart rejoin and normal-mode planner users must be able to rewind an exhausted DO_JUMP loop
//      when that path is shorter than continuing to the other loop exit.
// WHAT: On loop segment [2->0] near idx 2 with no repeats left -> first target becomes idx 2.
TEST_F(RtlPlannerIntegrationTest, FindNominalPathToGoalChoosesShortestExhaustedLoopExit)
{
	auto items = std::vector<mission_item_s> {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 2),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
	};
	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);
	config = defaultConfig();
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 0;

	MissionRoutePlanner::ProjectionContext projection_context{};
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt);
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_pos, 0, config, projection_context, &reason));
	ASSERT_TRUE(projection_context.loop_ctx.valid());
	EXPECT_EQ(projection_context.mission_loops_remaining, 0);

	const MissionRoutePlanner::Path path = planner.findNominalPathToGoal(4,
					       projection_context.dist_along_to_route_end,
					       projection_context, config);
	ASSERT_TRUE(path.valid());
	EXPECT_EQ(path.first_item_index, projection_context.loop_ctx.segment.start.idx);
	EXPECT_FALSE(path.direction_reversed);
}

// WHY: When a DO_JUMP still has repeats remaining, normal mission rejoin must continue the active loop
//      instead of rewinding it, even if the opposite exit would be shorter.
// WHAT: On loop segment [2->0] with one repeat left -> first target stays at the loop end.
TEST_F(RtlPlannerIntegrationTest, FindNominalPathToGoalKeepsLoopEndWhileRepeatsRemain)
{
	auto items = std::vector<mission_item_s> {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 1),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
	};
	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);
	config = defaultConfig();
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 1;

	MissionRoutePlanner::ProjectionContext projection_context{};
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt);
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_pos, 0, config, projection_context, &reason));
	ASSERT_TRUE(projection_context.loop_ctx.valid());
	EXPECT_GT(projection_context.mission_loops_remaining, 0);

	const MissionRoutePlanner::Path path = planner.findNominalPathToGoal(4,
					       projection_context.dist_along_to_route_end,
					       projection_context, config);
	ASSERT_TRUE(path.valid());
	EXPECT_EQ(path.first_item_index, projection_context.loop_ctx.segment.end.idx);
	EXPECT_FALSE(path.direction_reversed);
}

// WHY: RTL type 6 treats DO_JUMP items as geometry only and must ignore remaining loop counts
//      when choosing the shortest return path.
// WHAT: planRouteToGoal clears the effective loop counter and rewinds the loop when that exit is shorter.
TEST_F(RtlPlannerIntegrationTest, PlanRouteToGoalIgnoresPendingLoopsForRtl)
{
	auto items = std::vector<mission_item_s> {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 1),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
	};
	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);
	config = defaultConfig();
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 1;

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt);
	ASSERT_TRUE(planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason));
	ASSERT_TRUE(plan.projection_context.loop_ctx.valid());
	EXPECT_GT(plan.projection_context.loop_ctx.segment.loops_remaining, 0);
	EXPECT_EQ(plan.projection_context.mission_loops_remaining, 0);
	EXPECT_EQ(plan.selection.path.first_item_index, plan.projection_context.loop_ctx.segment.start.idx);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
}

// WHY: A safe point with an invalid frame should be rejected, causing fallback to a mission endpoint.
// WHAT: One safe point with frame=15 is treated as invalid; planner falls back to an endpoint.
TEST_F(RtlPlannerIntegrationTest, FallsBackWhenAllSafePointsInvalid)
{
	// GIVEN: The default dataset mission with one safe point that has an invalid frame (15).
	auto items = default_dataset::mission();
	std::vector<mission_item_s> bad_safe_points;
	{
		mission_item_s sp{};
		sp.lat = 46.105;
		sp.lon = 2.300;
		sp.altitude = 500.f;
		sp.nav_cmd = NAV_CMD_RALLY_POINT;
		sp.frame = 15;   // invalid frame
		sp.altitude_is_relative = false;
		bad_safe_points.push_back(sp);
	}
	VectorProvider provider(items, bad_safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle at N+50 from first waypoint.
	auto vehicle_pos = makePositionFromOffset(items[0].lat, items[0].lon, 50.f, 0.f, 560.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds but no safe point was found; goal is a mission endpoint.
	ASSERT_TRUE(ok);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionLand ||
		    plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionTakeoff);
}

// =============================================================================
// GROUP 2: Altitude handling near landing
// =============================================================================

// WHY: When a fixed-wing vehicle is close to the land waypoint, it should skip the altitude requirement.
// WHAT: Vehicle near land at alt=523 gets skip_altitude_requirement=true and join alt=vehicle alt.
TEST_F(RtlPlannerIntegrationTest, SkipsAltitudeRequirementNearLand)
{
	// GIVEN: A 3-waypoint mission (takeoff -> wp -> land).
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle at (N+200, E+0, alt=523), near landing.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, 523.f);
	config = fwConfig();
	config.acceptance_radius = 20.f;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);

	// THEN: MissionLand is selected, skip_altitude_requirement is true, join alt matches vehicle alt.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_TRUE(plan.join_context.skip_altitude_requirement);
	EXPECT_NEAR(plan.join_context.projection.alt, 523.f, kAltitudeTolerance);
}

// WHY: Corner dataset missions with the vehicle near land should also skip altitude requirements.
// WHAT: Vehicle near the corner dataset land waypoint gets skip_altitude_requirement=true.
TEST_F(RtlPlannerIntegrationTest, CornerMission_SkipAltitudeNearLand)
{
	// GIVEN: Corner dataset mission with no safe points.
	auto items = corner_dataset::mission();
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle very near the land waypoint.
	auto vehicle_pos = makePositionAbsolute(46.10451291425605, 2.3176006267546034, 462.2f);
	config = fwConfig();
	config.acceptance_radius = 100.f;
	config.vehicle_projection_search_dist = 10.f;
	config.safe_point_projection_search_dist = 10.f;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 12, config, plan, &reason);

	// THEN: MissionLand is selected and skip_altitude_requirement keeps the current altitude.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_TRUE(plan.join_context.skip_altitude_requirement);
	EXPECT_NEAR(plan.join_context.projection.alt, 462.2f, kAltitudeTolerance);
}

// WHY: Relative-altitude mission items must be converted with home_altitude_amsl before endpoint fallback.
// WHAT: A relative-altitude landing item produces an absolute MissionLand goal altitude.
TEST_F(RtlPlannerIntegrationTest, RelativeAltitudeMissionLandUsesHomeAltitude)
{
	// GIVEN: A simple mission stored with relative altitudes and no safe points.
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, 60.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, 20.f),
	};

	for (mission_item_s &item : items) {
		item.altitude_is_relative = true;
		item.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;
	}

	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 320.f, 0.f, 645.f);
	config = defaultConfig();
	config.home_altitude_amsl = 600.f;
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal falls back to the mission landing endpoint.
	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);

	// THEN: The goal altitude is converted from relative altitude to AMSL.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_NEAR(plan.selection.goal_position.alt, 620.f, kAltitudeTolerance);
	EXPECT_NEAR(plan.selection.goal_position.lat, items.back().lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, items.back().lon, kLatLonToleranceDeg);
}

// =============================================================================
// GROUP 3: Full plan with safe points (no fallback)
// =============================================================================

// WHY: With no rally points, the planner should fall back to MissionLand when the vehicle is past the midpoint.
// WHAT: Vehicle in the latter part of the corner mission selects MissionLand with forward direction.
TEST_F(RtlPlannerIntegrationTest, SrpFallbackToLandWhenNoRallyPoints)
{
	// GIVEN: Corner dataset mission with empty safe points.
	auto items = corner_dataset::mission();
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10451291425605, 2.3176006267546034, 560.f);
	config = fwConfig();
	config.vehicle_projection_search_dist = 10.f;
	config.safe_point_projection_search_dist = 10.f;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called with mission_index=4.
	bool ok = planner.planRouteToGoal(vehicle_pos, 4, config, plan, &reason);

	// THEN: Planning succeeds, selects MissionLand, direction_reversed=false.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_FALSE(plan.selection.path.direction_reversed);
}

// WHY: With no rally points and vehicle near takeoff, the planner should fall back to MissionTakeoff.
// WHAT: Vehicle near the start of the corner mission selects MissionTakeoff with reversed direction.
TEST_F(RtlPlannerIntegrationTest, SrpFallbackToTakeoffWhenNoRallyPoints)
{
	// GIVEN: Corner dataset mission with empty safe points.
	auto items = corner_dataset::mission();
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.101868043118436, 2.3261360396086284, 540.f);
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called with mission_index=0.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds, selects MissionTakeoff with reversed direction.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
}

// WHY: When a valid rally point exists, the planner must select it instead of falling back to an endpoint.
// WHAT: Corner dataset with safe points and vehicle 5m along takeoff->wp1 bearing selects SafePoint.
TEST_F(RtlPlannerIntegrationTest, SafePointFoundDoesNotUseFallback)
{
	// GIVEN: Corner dataset mission with safe points.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	// Place vehicle 5m along the bearing from takeoff to wp1.
	float bearing = get_bearing_to_next_waypoint(items[0].lat, items[0].lon, items[1].lat, items[1].lon);
	double vehicle_lat, vehicle_lon;
	waypoint_from_heading_and_distance(items[0].lat, items[0].lon, bearing, 5.f, &vehicle_lat, &vehicle_lon);
	auto vehicle_pos = makePositionAbsolute(vehicle_lat, vehicle_lon, items[0].altitude + 10.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds, a safe point is found, and goal type is SafePoint.
	ASSERT_TRUE(ok);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
}

// =============================================================================
// GROUP 5: Error handling
// =============================================================================

// WHY: An empty mission has no waypoints to follow; planning must fail cleanly.
// WHAT: planRouteToGoal returns false with reason NoValidWaypoints.
TEST_F(RtlPlannerIntegrationTest, FailsWithEmptyMission)
{
	// GIVEN: An empty mission.
	std::vector<mission_item_s> empty_mission{};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(empty_mission, no_safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails with NoValidWaypoints.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidWaypoints);
}

// WHY: A single waypoint cannot form a segment; planning must fail.
// WHAT: planRouteToGoal returns false with reason NoValidWaypoints.
TEST_F(RtlPlannerIntegrationTest, FailsWithSingleWaypoint)
{
	// GIVEN: A mission with only one waypoint.
	auto items = std::vector<mission_item_s> {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails with NoValidWaypoints.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidWaypoints);
}

class RtlPlannerInvalidVehiclePositionTest : public MissionRoutePlannerTestBase,
	public ::testing::WithParamInterface<std::pair<double, double>>
{
protected:
	MissionRoutePlanner::Plan plan{};
};

// WHY: Full route planning should reject invalid vehicle positions before any mission search starts.
// WHAT: planRouteToGoal returns false with FailureReason::NoValidGlobalPos for each invalid position.
TEST_P(RtlPlannerInvalidVehiclePositionTest, RejectsInvalidVehiclePosition)
{
	// GIVEN: The default dataset mission and an invalid vehicle position.
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);
	const auto [lat, lon] = GetParam();

	MissionRoutePlanner::Position vehicle_pos{};
	vehicle_pos.lat = lat;
	vehicle_pos.lon = lon;
	vehicle_pos.alt = 550.f;

	// WHEN: planRouteToGoal is called with the invalid position.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails with the explicit invalid-global-position reason.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidGlobalPos);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidVehiclePositions,
	RtlPlannerInvalidVehiclePositionTest,
	::testing::Values(
		std::make_pair(NAN, kBaseLon),
		std::make_pair(kBaseLat, NAN),
		std::make_pair(INFINITY, kBaseLon),
		std::make_pair(kBaseLat, INFINITY),
		std::make_pair(0.0, 0.0),
		std::make_pair(90.0, kBaseLon),
		std::make_pair(kBaseLat, 180.0),
		std::make_pair(91.0, kBaseLon),
		std::make_pair(kBaseLat, 181.0)
	)
);

// =============================================================================
// NEW TESTS: Safe-point priority and near-takeoff guarding
// =============================================================================

// WHY: When a safe point is available near the route midpoint, it should be preferred over mission endpoints.
// WHAT: A 4-waypoint straight mission with a safe point near the midpoint selects SafePoint.
TEST_F(RtlPlannerIntegrationTest, PlanRouteSelectsSafePointOverEndpoint)
{
	// GIVEN: A 4-waypoint straight-line mission with a safe point near the midpoint.
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt + 10.f),
	};
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	// Vehicle near the start of the mission.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds and selects the safe point, not a mission endpoint.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_NE(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionLand);
	EXPECT_NE(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
}

// WHY: When the vehicle is near takeoff but a valid safe point exists, the planner must NOT
//      fall back to MissionTakeoff. The safe point should take priority.
// WHAT: Vehicle within 5m of takeoff in the corner dataset with safe points selects SafePoint.
TEST_F(RtlPlannerIntegrationTest, PlanRouteNearTakeoffWithSafePointDoesNotFallbackToTakeoff)
{
	// GIVEN: Corner dataset mission with safe points. Vehicle within 5m of takeoff.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	// Place vehicle 5m from takeoff along takeoff->wp1 bearing.
	float bearing = get_bearing_to_next_waypoint(items[0].lat, items[0].lon, items[1].lat, items[1].lon);
	double vehicle_lat, vehicle_lon;
	waypoint_from_heading_and_distance(items[0].lat, items[0].lon, bearing, 5.f, &vehicle_lat, &vehicle_lon);
	auto vehicle_pos = makePositionAbsolute(vehicle_lat, vehicle_lon, items[0].altitude + 5.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds, a safe point is found, and goal type is SafePoint (NOT MissionTakeoff).
	ASSERT_TRUE(ok);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_NE(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
}

// =============================================================================
// GROUP 6: Graceful degradation on load failures (using VectorProvider fault injection)
// =============================================================================

// WHY: If the SD card fails to read a mission item mid-scan, the planner must not crash
//      or produce an invalid plan. It should either fail gracefully with a clear reason
//      or succeed with the items it could read.
// WHAT: A faulty provider that fails on a mid-mission index causes the planner to fail
//       with an appropriate failure reason.
TEST_F(RtlPlannerIntegrationTest, FaultyMissionItemMidScanCausesGracefulFailure)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt + 60.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {}, {2}, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	MissionRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	EXPECT_FALSE(ok);
	EXPECT_EQ(fail_reason, MissionRoutePlanner::FailureReason::InternalError);
}

// WHY: If the first AND second mission items fail to load, the planner cannot build any
//      segments. This must fail cleanly without crashing.
// WHAT: All initial position items failing causes planning failure.
TEST_F(RtlPlannerIntegrationTest, AllInitialPositionItemsFaultyFailsGracefully)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {}, {0, 1}, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);
	config = defaultConfig();

	MissionRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	EXPECT_FALSE(ok);
	EXPECT_EQ(fail_reason, MissionRoutePlanner::FailureReason::NoSegmentsFound);
}

// WHY: If all safe points fail to load but the mission is intact, the planner should
//      still succeed by falling back to a mission endpoint (MissionLand/MissionTakeoff).
// WHAT: Faulty safe points do not prevent planning; the planner falls back to an endpoint.
TEST_F(RtlPlannerIntegrationTest, AllFaultySafePointsFallBackToEndpoint)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 50.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 750.f, -50.f, kAlt + 20.f),
	};

	VectorProvider provider(mission, safe_points, {}, {0, 1});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 25.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionLand
		    || plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionTakeoff);
}

// WHY: If only one safe point out of several fails to load, the planner should still
//      evaluate the remaining safe points and select one.
// WHAT: One faulty safe point does not prevent other safe points from being selected.
TEST_F(RtlPlannerIntegrationTest, OneFaultySafePointDoesNotBlockOthers)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 30.f, kAlt + 10.f),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 750.f, -30.f, kAlt + 40.f),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1250.f, 20.f, kAlt + 60.f),
	};

	VectorProvider provider(mission, safe_points, {}, {1});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 25.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_NE(plan.selection.safe_point_index, 1);
}

// WHY: A failed land-item read must not crash the planner or produce a half-valid fallback plan.
// WHAT: A faulty land item fails cleanly with NoValidWaypoints.
TEST_F(RtlPlannerIntegrationTest, FaultyLandItemFailsCleanly)
{
	// GIVEN: A mission whose land item cannot be loaded.
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};
	VectorProvider provider(mission, {}, {3}, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 20.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails explicitly instead of returning a partially-populated fallback.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidWaypoints);
}

// =============================================================================
// GROUP 7: VTOL mission planning
// =============================================================================

// WHY: When a VTOL in FW mode triggers RTL near the start of the mission, the planner should
//      select MissionTakeoff with direction_reversed=true. The executor must then fly the route
//      in reverse and eventually land at ground level.
// WHAT: Vehicle at N+100 on a long VTOL mission gets a reversed path to takeoff.
TEST_F(RtlPlannerIntegrationTest, FwVehicleReversesToTakeoffWhenNearStart)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 4000.f, 0.f, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 5000.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = fwConfig();
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_NEAR(plan.selection.goal_position.lat, mission[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, mission[0].lon, kLatLonToleranceDeg);
}

// WHY: When using the default_dataset, a VTOL vehicle near the start in FW mode should
//      get a plan that can be executed through the full stage machine including transitions.
// WHAT: Plan builds successfully with safe-point selection and the path includes segments
//       that cross VTOL transition boundaries.
TEST_F(RtlPlannerIntegrationTest, DefaultDatasetVtolPlanBuildsSucessfully)
{
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10830, 2.2995, 575.f);
	config = fwConfig();
	config.vehicle_velocity_north = default_dataset::kVel;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 4, config, plan, &reason);

	ASSERT_TRUE(ok);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_GT(plan.selection.path.dist, 0.f);
	EXPECT_TRUE(plan.projection_context.valid());
}

// =============================================================================
// GROUP 8: Stage-machine contract verification
// =============================================================================
//
// These planner-level contract tests stay focused on the Plan fields consumed by
// the executor. Lightweight executor stage tests now live in test_mission_base_vtol.cpp,
// where they can exercise RtlMissionSafePointFollow::setNextMissionItem() without
// constructing a full Navigator stack.

// WHY: The executor transitions FollowRoute → BranchOff when current_seq == branch_off_index.
//      This requires the plan to produce a valid branch_off_segment whose end index (nominal)
//      or start index (reversed) serves as the trigger.
// WHAT: When a safe point is selected, the plan's branchOffIndex() returns a valid mission index.
TEST_F(RtlPlannerIntegrationTest, PlanProvidesValidBranchOffIndexForSafePoint)
{
	// GIVEN: A simple mission whose safe point must be reached by branching off the route.
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt + 10.f),
	};
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: A valid safe-point branch-off index is produced.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	ASSERT_TRUE(plan.selection.safe_point_found);
	const int32_t branch_idx = plan.selection.branchOffIndex();
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_GE(branch_idx, 0);
	EXPECT_LT(branch_idx, static_cast<int32_t>(items.size()));
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
	EXPECT_TRUE(plan.selection.branch_off_projection.valid());
	EXPECT_GE(plan.selection.path.first_item_index, 0);
	EXPECT_LT(plan.selection.path.first_item_index, static_cast<int32_t>(items.size()));
	EXPECT_LE(plan.selection.path.first_item_index, branch_idx);
}

// WHY: The executor goes straight to LandAtGoal when direct_to_safe_point is set,
//      skipping JoinRoute and FollowRoute entirely. The plan must provide a valid
//      goal_position for the landing item.
// WHAT: When direct_to_safe_point is true, all landing fields are populated.
TEST_F(RtlPlannerIntegrationTest, DirectToSafePointPlanHasCompleteLandingFields)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 255.f, 0.f, kAlt + 50.f),
	};

	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = defaultConfig();
	config.is_multicopter = true;
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 5.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	ASSERT_TRUE(plan.selection.direct_to_safe_point);
	EXPECT_TRUE(plan.selection.goal_position.valid());
	EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.lat));
	EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.lon));
	EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.alt));
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
}

// WHY: When the executor reaches the mission endpoint in a non-safe-point plan,
//      it needs the goal_position to build the landing item. The plan must populate
//      this correctly for both MissionLand and MissionTakeoff goals.
// WHAT: Endpoint fallback plan has a valid goal_position matching the endpoint.
TEST_F(RtlPlannerIntegrationTest, EndpointFallbackPlanHasValidGoalPosition)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 750.f, 0.f, kAlt + 60.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);

	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_position.valid());

	if (plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionLand) {
		EXPECT_NEAR(plan.selection.goal_position.lat, mission.back().lat, kLatLonToleranceDeg);
		EXPECT_NEAR(plan.selection.goal_position.lon, mission.back().lon, kLatLonToleranceDeg);

	} else if (plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionTakeoff) {
		EXPECT_NEAR(plan.selection.goal_position.lat, mission.front().lat, kLatLonToleranceDeg);
		EXPECT_NEAR(plan.selection.goal_position.lon, mission.front().lon, kLatLonToleranceDeg);
	}

	EXPECT_GE(plan.selection.path.first_item_index, 0);
	EXPECT_LT(plan.selection.path.first_item_index, static_cast<int32_t>(mission.size()));
}

// WHY: If the mission only contains NAV_CMD_WAYPOINT items (no actual takeoff/land commands),
//      the endpoint fallback must NOT label a random waypoint as MissionTakeoff or MissionLand.
//      Doing so would cause the executor to land at the wrong altitude (CFIT risk).
// WHAT: planRouteToGoal fails when the mission has no takeoff or land items and no safe points.
TEST_F(RtlPlannerIntegrationTest, WaypointOnlyMissionRejectsEndpointFallback)
{
	// GIVEN: A mission with only NAV_CMD_WAYPOINT items (no takeoff/land).
	auto items = std::vector<mission_item_s> {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 100.f),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 25.f);
	config = defaultConfig();

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);

	// THEN: Planning fails because no safe point or valid endpoint candidate exists.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidCandidateFound);
}
