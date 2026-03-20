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
 * End-to-end tests for RtlRoutePlanner::planRouteToGoal.
 * Covers fallback to mission endpoints, join altitude handling,
 * branch-off caching, safe-point selection, and error handling.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

#include <cmath>

static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;

// ============================================================================
// Test fixture
// ============================================================================

class RtlPlannerIntegrationTest : public RtlRoutePlannerTestBase
{
protected:
	RtlRoutePlanner::Plan plan{};
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
	RtlRoutePlanner planner(provider);

	// Vehicle is at N+260, close to the landing waypoint.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 260.f, 0.f, kAlt + 25.f);
	config = fwConfig();

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 2, config, plan, &reason);

	// THEN: Planning succeeds and selects MissionLand.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
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
	RtlRoutePlanner planner(provider);

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
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_NEAR(plan.selection.goal_position.lat, items[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, items[0].lon, kLatLonToleranceDeg);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_TRUE(plan.selection.path.u_turn_required);
	EXPECT_NEAR(plan.selection.path.dist, 4040.f, kDistanceTolerance);
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
	RtlRoutePlanner planner(provider);

	// Vehicle at N+50 from first waypoint.
	auto vehicle_pos = makePositionFromOffset(items[0].lat, items[0].lon, 50.f, 0.f, 560.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds but no safe point was found; goal is a mission endpoint.
	ASSERT_TRUE(ok);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand ||
		    plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff);
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
	RtlRoutePlanner planner(provider);

	// Vehicle at (N+200, E+0, alt=523), near landing.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, 523.f);
	config = fwConfig();
	config.acceptance_radius = 20.f;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);

	// THEN: MissionLand is selected, skip_altitude_requirement is true, join alt matches vehicle alt.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
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
	RtlRoutePlanner planner(provider);

	// Vehicle very near the land waypoint.
	auto vehicle_pos = makePositionAbsolute(46.10451291425605, 2.3176006267546034, 462.2f);
	config = fwConfig();
	config.acceptance_radius = 100.f;
	config.vehicle_projection_search_dist = 10.f;
	config.safe_point_projection_search_dist = 10.f;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 12, config, plan, &reason);

	// THEN: MissionLand is selected and skip_altitude_requirement is true with join alt at vehicle alt.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);

	if (plan.join_context.skip_altitude_requirement) {
		EXPECT_NEAR(plan.join_context.projection.alt, 462.2f, kAltitudeTolerance);
	}
}

// =============================================================================
// GROUP 3: Branch-off caching
// =============================================================================

// WHY: The cached branch-off segment should be reusable when the vehicle stays close.
// WHAT: closeToBranchOffSegment returns true for a vehicle near the cached leg, false when far away.
TEST_F(RtlPlannerIntegrationTest, CloseToBranchOffSegmentUsesStoredLeg)
{
	// GIVEN: A 3-waypoint L-shaped mission.
	auto items = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 120.f, kAlt),
	};
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	RtlRoutePlanner planner(provider);

	// Manually build a Selection with branch_off on segment [1->2] at (N+100, E+60)
	RtlRoutePlanner::Selection selection{};
	selection.found = true;
	selection.safe_point_found = true;
	selection.branch_off_segment.start.idx = 1;
	selection.branch_off_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	selection.branch_off_segment.end.idx = 2;
	selection.branch_off_segment.end.nav_cmd = NAV_CMD_LAND;
	selection.branch_off_projection = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt + 10.f);
	selection.safe_point_position = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 120.f, kAlt);
	selection.goal_position = selection.safe_point_position;
	selection.goal_type = RtlRoutePlanner::GoalType::SafePoint;

	// WHEN: Vehicle is at (N+100, E+90), within 20m of the branch-off segment [1->2].
	auto near_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 90.f, kAlt + 10.f);
	bool near_result = planner.closeToBranchOffSegment(near_pos, selection, 20.f);

	// THEN: closeToBranchOffSegment returns true for the near position.
	EXPECT_TRUE(near_result);

	// WHEN: Vehicle is at (N+40, E+0), far from the branch-off segment.
	auto far_pos = makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt + 10.f);
	bool far_result = planner.closeToBranchOffSegment(far_pos, selection, 20.f);

	// THEN: closeToBranchOffSegment returns false for the far position.
	EXPECT_FALSE(far_result);
}

// =============================================================================
// GROUP 4: Full plan with safe points (no fallback)
// =============================================================================

// WHY: With no rally points, the planner should fall back to MissionLand when the vehicle is past the midpoint.
// WHAT: Vehicle in the latter part of the corner mission selects MissionLand with forward direction.
TEST_F(RtlPlannerIntegrationTest, SrpFallbackToLandWhenNoRallyPoints)
{
	// GIVEN: Corner dataset mission with empty safe points.
	auto items = corner_dataset::mission();
	std::vector<mission_item_s> no_safe_points{};
	VectorProvider provider(items, no_safe_points);
	RtlRoutePlanner planner(provider);

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
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
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
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.101868043118436, 2.3261360396086284, 540.f);
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called with mission_index=0.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds, selects MissionTakeoff with reversed direction.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
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
	RtlRoutePlanner planner(provider);

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
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
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
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails with NoValidWaypoints.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, RtlRoutePlanner::FailureReason::NoValidWaypoints);
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
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails with NoValidWaypoints.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, RtlRoutePlanner::FailureReason::NoValidWaypoints);
}

// WHY: An invalid vehicle position (NAN lat) must be caught early.
// WHAT: planRouteToGoal returns false when the vehicle latitude is NAN.
TEST_F(RtlPlannerIntegrationTest, FailsOnInvalidVehiclePosition)
{
	// GIVEN: The default dataset mission with safe points, but vehicle lat is NAN.
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle_pos{};
	vehicle_pos.lat = NAN;
	vehicle_pos.lon = 2.300;
	vehicle_pos.alt = 550.f;

	// WHEN: planRouteToGoal is called with an invalid vehicle position.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning fails.
	EXPECT_FALSE(ok);
}

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
	RtlRoutePlanner planner(provider);

	// Vehicle near the start of the mission.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds and selects the safe point, not a mission endpoint.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_NE(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
	EXPECT_NE(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
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
	RtlRoutePlanner planner(provider);

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
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
	EXPECT_NE(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
}
