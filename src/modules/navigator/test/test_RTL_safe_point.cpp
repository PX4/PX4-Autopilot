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
 * @file test_RTL_safe_point.cpp
 *
 * Unit tests for RtlRoutePlanner safe-point (rally point) evaluation and
 * selection during route-following RTL. Covers:
 *
 * - Basic safe-point selection: shortest along-route path, direct-to, invalid points
 * - Default dataset MC/FW selection: forward/reverse direction, acceptance radius,
 *   invalid rally skipping, all-behind fallback
 * - Fixed-wing u-turn penalty: verifies that the 4 km penalty shifts FW
 *   selection toward forward rally points where geometry warrants it
 * - Corner dataset selection: sharp corners, loop segments, small segments,
 *   stacked landing waypoints
 * - Batched scanning efficiency: verifies single mission scan for multiple safe points
 * - Loop handling: DO_JUMP projection, reverse jump choice, remaining iterations
 * - Velocity edge cases: zero velocity and orthogonal velocity for FW u-turn logic
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

// ============================================================================
// Common constants for offset-based tests
// ============================================================================

static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;

// ============================================================================
// Test fixture
// ============================================================================

class RtlSafePointTest : public RtlRoutePlannerTestBase {};

// ============================================================================
// GROUP 1: Basic safe point selection
// ============================================================================

// WHY: The planner must prefer the safe point reachable via the shortest along-route distance.
// WHAT: selectSafePoint returns the closer rally point (index 1) on segment [0-1].
TEST_F(RtlSafePointTest, PrefersShortestAlongRoutePath)
{
	// GIVEN: 4-wp square mission with two safe points at different along-route distances.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 95.f, 80.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 20.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 5.f, kAlt);

	// WHEN: Vehicle projects onto segment [0-1] near the start.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The closer safe point (index 1) is selected, branching off segment [0-1].
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
}

// WHY: When a safe point is within the direct acceptance radius, the planner should skip route following.
// WHAT: selectSafePoint returns direct_to_safe_point=true for a nearby rally point.
TEST_F(RtlSafePointTest, SupportsDirectToSafePoint)
{
	// GIVEN: 3-wp straight mission with one safe point very close to the vehicle.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 14.f, 3.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: Vehicle is near the safe point within direct acceptance radius.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Direct-to flag is set, safe point index is 0.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.direct_to_safe_point);
	EXPECT_EQ(selection.safe_point_index, 0);
}

// WHY: When all safe points have an invalid frame, none should be selected.
// WHAT: selectSafePoint returns found=false when every rally point is invalid.
TEST_F(RtlSafePointTest, ReturnsEmptyWhenAllSafePointsInvalid)
{
	// GIVEN: 3-wp mission with 3 safe points using unsupported frame=15.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> invalid_safe_points;

	for (int i = 0; i < 3; ++i) {
		mission_item_s item{};
		item.nav_cmd = NAV_CMD_RALLY_POINT;
		item.lat = kBaseLat;
		item.lon = kBaseLon;
		item.altitude = kAlt;
		item.frame = 15; // Invalid frame
		invalid_safe_points.push_back(item);
	}

	VectorProvider provider{mission, invalid_safe_points};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: Vehicle projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: No safe point is found.
	EXPECT_FALSE(selection.found);
}

// ============================================================================
// GROUP 2: Default dataset - MC selection
// ============================================================================

// WHY: MC should pick the closest rally point even when it is behind the vehicle (reverse direction).
// WHAT: Rally 1 (on seg 0-1) is selected with direction_reversed=true.
TEST_F(RtlSafePointTest, DefaultMission_ClosestBehindReverse_MC)
{
	// GIVEN: Default 16-item mission with 7 rally points, MC config.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// WHEN: Vehicle at mission_index=2 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 1 is selected in reverse direction.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: MC should pick a forward rally point when it is the closest along-route option.
// WHAT: Rally 0 (on seg 5-7, ahead) is selected for the vehicle at mission_index=5.
TEST_F(RtlSafePointTest, DefaultMission_ClosestForwardAhead_MC)
{
	// GIVEN: Default mission, MC config, vehicle flying with velocity (15,-15).
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = -15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);

	// WHEN: Vehicle at mission_index=5 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 5, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 0 is selected (forward, on segment 5-7 ahead of vehicle).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
}

// WHY: When the vehicle is within the acceptance radius of a rally point, direct-to should be selected.
// WHAT: Rally 4 near takeoff is selected with direct_to_safe_point=true.
TEST_F(RtlSafePointTest, DefaultMission_WithinAcceptanceRadius)
{
	// GIVEN: Default mission, MC config. Vehicle near rally 4.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.09681253236241, 2.2993209050608376, 838.48f);

	// WHEN: Vehicle at mission_index=1 is within acceptance radius of rally 4.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 4 is selected as direct-to safe point.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 4);
	EXPECT_TRUE(selection.direct_to_safe_point);
}

// WHY: When all rally points are behind the vehicle, MC should still pick the closest reverse one.
// WHAT: Rally 0 is selected with direction_reversed=true when vehicle is near the end of the route.
TEST_F(RtlSafePointTest, DefaultMission_AllBehind_MC)
{
	// GIVEN: Default mission, MC config. Vehicle near mission_index=15 (end of route).
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);

	// WHEN: Vehicle at mission_index=15 with all rally points behind.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 15, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Closest reverse rally is selected (rally 0), direction reversed.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: Corrupted rally point data (NAN lat) must be gracefully skipped.
// WHAT: Selection still succeeds and branch_off_projection has finite coordinates.
TEST_F(RtlSafePointTest, DefaultMission_InvalidRallyPointSkipped)
{
	// GIVEN: Default mission with rally[0].lat set to NAN (corrupted).
	auto safe_points = default_dataset::safePoints();
	safe_points[0].lat = NAN;

	VectorProvider provider{default_dataset::mission(), safe_points};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -15.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);

	// WHEN: Vehicle at mission_index=13 with corrupted rally 0.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: A valid safe point is still found, and branch_off projection coordinates are finite.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(std::isfinite(selection.branch_off_projection.lat));
	EXPECT_TRUE(std::isfinite(selection.branch_off_projection.lon));
}

// ============================================================================
// GROUP 3: Default dataset - FW selection (u-turn penalty)
// ============================================================================

// WHY: For this geometry the reverse path is still shorter even with the FW u-turn penalty.
// WHAT: FW picks rally 1 in reverse, same as MC.
TEST_F(RtlSafePointTest, DefaultMission_ClosestBehindReverse_FW)
{
	// GIVEN: Default mission, FW config. Same position as DefaultMission_ClosestBehindReverse_MC.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// WHEN: Vehicle at mission_index=2 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: FW still picks rally 1 reverse (u-turn penalty doesn't make forward cheaper).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: The 4km u-turn penalty should make FW prefer a more distant forward rally over a closer reverse one.
// WHAT: FW picks rally B (index 1, forward) instead of closer rally A (index 0, reverse).
TEST_F(RtlSafePointTest, FWUturnPenaltySelectsForwardOverCloserReverse)
{
	// GIVEN: 5-wp straight mission (takeoff to land, 500m spacing). Rally A 200m behind, Rally B 600m ahead.
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),   // A: ~200m behind vehicle
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),  // B: ~600m ahead of vehicle
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);

	// WHEN: FW vehicle at N+500, flying north along the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: FW picks rally B (forward, index 1) due to 4km u-turn penalty making reverse more expensive.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_FALSE(selection.path.direction_reversed);
}

// WHY: MC has no u-turn penalty, so it should always pick the closest rally regardless of direction.
// WHAT: MC picks rally A (index 0, reverse, closer) for the same geometry as FWUturnPenalty test.
TEST_F(RtlSafePointTest, MCNoUturnPenaltySelectsClosestReverse)
{
	// GIVEN: Same 5-wp straight mission and rally points as FWUturnPenalty test, but MC config.
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),   // A: ~200m behind
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),  // B: ~600m ahead
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);

	// WHEN: MC vehicle at N+500, flying north along the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: MC picks rally A (reverse, index 0, closer without u-turn penalty).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// ============================================================================
// GROUP 4: Corner dataset selection
// ============================================================================

// WHY: Corner missions with sharp turns must still correctly identify the closest rally in reverse.
// WHAT: MC picks rally 1 in reverse direction for the corner mission.
TEST_F(RtlSafePointTest, CornerMission_RallyOnCorner_MC)
{
	// GIVEN: Corner 16-item mission with 8 rally points, MC config.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// WHEN: Vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 1 is selected in reverse.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: FW must handle corner projections properly and find a valid safe point.
// WHAT: FW finds a safe point; if rally 0 is selected, branch_off is on segment [4-5].
TEST_F(RtlSafePointTest, CornerMission_CornerProjectionHandled_FW)
{
	// GIVEN: Corner mission, FW config. Same vehicle position as CornerMission_RallyOnCorner_MC.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// WHEN: FW vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: A safe point is found. If rally 0, branch_off is on [4-5]; direction is forward.
	ASSERT_TRUE(selection.found);

	if (selection.safe_point_index == 0) {
		EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
		EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
		EXPECT_FALSE(selection.path.direction_reversed);
	}
}

// WHY: Rally points whose loop-segment candidate would create an invalid path must be excluded.
// WHAT: MC picks rally 4 instead of rally 3 whose loop-segment candidate is excluded.
TEST_F(RtlSafePointTest, CornerMission_BackNoTransition_MC)
{
	// GIVEN: Corner mission, MC config. Vehicle at index 7 near a transition boundary.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);

	// WHEN: Vehicle at mission_index=7.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 7, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 4 is selected (rally 3's loop-segment candidate is excluded).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 4);
}

// WHY: Small segments near the end of the mission must be handled without skipping valid rally points.
// WHAT: MC picks rally 5 for a vehicle near the small segments at the end of the corner mission.
TEST_F(RtlSafePointTest, CornerMission_SmallSegmentFront_MC)
{
	// GIVEN: Corner mission, MC config. Vehicle near small segments at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);

	// WHEN: Vehicle at mission_index=13.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 5 is selected.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 5);
}

// WHY: Rally 0 should project onto segment [4-5] and not onto a corner artifact.
// WHAT: If rally 0 is selected, branch_off is on segment [4-5].
TEST_F(RtlSafePointTest, CornerMission_Rally0ProjectedOntoCornerWp4)
{
	// GIVEN: Corner mission, MC config. Vehicle at mission_index=5 flying reverse.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);

	// WHEN: Vehicle at mission_index=5.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 5, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: If rally 0 is selected, it should branch off on segment [4-5].
	ASSERT_TRUE(selection.found);

	if (selection.safe_point_index == 0) {
		EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
		EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
	}
}

// WHY: Rally 6 near the stacked landing waypoint must project onto the correct segment.
// WHAT: If rally 6 is selected, branch_off is on segment [14-15].
TEST_F(RtlSafePointTest, CornerMission_Rally6ProjectedOnLandCorner)
{
	// GIVEN: Corner mission, FW config. Vehicle near stacked landing at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10368934085859, 2.3183612137416754, 510.f);

	// WHEN: Vehicle at mission_index=13 with FW config.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: If rally 6 is selected, branch_off is on segment [14-15].
	ASSERT_TRUE(selection.found);

	if (selection.safe_point_index == 6) {
		EXPECT_EQ(selection.branch_off_segment.start.idx, 14);
		EXPECT_EQ(selection.branch_off_segment.end.idx, 15);
	}
}

// WHY: The DO_JUMP loop segment must be traversed correctly during safe-point selection.
// WHAT: Selection succeeds and produces a valid branch_off when the vehicle is on a loop segment.
TEST_F(RtlSafePointTest, CornerMission_LoopSegmentIsHandled)
{
	// GIVEN: Corner mission with DO_JUMP at index 8, MC config. Vehicle on loop area.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelFast;
	config.vehicle_velocity_east = 0.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10264815827885, 2.321939748532329, 600.f);

	// WHEN: Vehicle at mission_index=3.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 3, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Loop segment is handled; selection has a valid branch_off segment.
	ASSERT_TRUE(selection.found);
	EXPECT_GE(selection.branch_off_segment.start.idx, 0);
	EXPECT_GE(selection.branch_off_segment.end.idx, 0);
}

// ============================================================================
// GROUP 5: Batched scanning efficiency
// ============================================================================

// WHY: Safe-point projection should scan the mission once for all safe points, not once per safe point.
// WHAT: 4-wp mission with 6 safe points: missionLoadCount bounded by 2*M (not M*S), safePointLoadCount == 6.
TEST_F(RtlSafePointTest, ScansMissionOnceForBatch_Simple)
{
	// GIVEN: 4-wp square mission with 6 safe points, using CountingProvider.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, -20.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 40.f, 30.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 90.f, 50.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 110.f, 10.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 70.f, 120.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 110.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	// WHEN: Safe point selection is run, counting provider accesses.
	provider.resetCounters();
	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Mission is scanned efficiently — load count is O(M) not O(M*S).
	//       For M=4, S=6: O(M*S) would be 24 which must NOT pass.
	//       Bound of 2*M = 8 catches an unbatched regression while allowing reasonable overhead.
	const int mission_count = static_cast<int>(mission.size());
	ASSERT_TRUE(selection.found);
	EXPECT_LE(provider.missionLoadCount(), 2 * mission_count)
			<< "Mission should be scanned in one batch pass, not once per safe point";
	EXPECT_EQ(provider.safePointLoadCount(), 6);
}

// WHY: Batched scanning must also work for larger real-world-like missions.
// WHAT: Default dataset mission with 7 safe points: missionLoadCount bounded by 2*M, safePointLoadCount == 7.
TEST_F(RtlSafePointTest, ScansMissionOnceForBatch_DefaultDataset)
{
	// GIVEN: Default 16-item mission with 7 rally points, using CountingProvider.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	// WHEN: Safe point selection is run on the default dataset.
	provider.resetCounters();
	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Mission loads are bounded by O(M) not O(M*S).
	//       For M=16, S=7: O(M*S) would be 112 which must NOT pass.
	//       Bound of 2*M = 32 catches an unbatched regression.
	ASSERT_TRUE(selection.found);
	EXPECT_LE(provider.missionLoadCount(), 2 * provider.missionCount())
			<< "Mission should be scanned in one batch pass, not once per safe point";
	EXPECT_EQ(provider.safePointLoadCount(), 7);
}

// ============================================================================
// GROUP 6: Loop handling in safe point selection
// ============================================================================

// WHY: When the vehicle is on a loop segment, the planner must correctly evaluate reverse-jump paths.
// WHAT: Safe point on a loop segment triggers direction_reversed=true, first_item_index=2, branch_off on [1-2].
TEST_F(RtlSafePointTest, HandlesLoopProjectionAndReverseJumpChoice)
{
	// GIVEN: 4-item mission with DO_JUMP(0, repeat 1, current 1). Vehicle on loop segment midpoint.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 1, 1),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 95.f, 50.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};

	// GIVEN: Manually built ctx with loop segment [2->0, is_loop=true].
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, kAlt);
	const RtlRoutePlanner::Position loop_start =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	const RtlRoutePlanner::Position loop_end =
		makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const float loop_segment_length = get_distance_to_next_waypoint(loop_start.lat, loop_start.lon,
					  loop_end.lat, loop_end.lon);

	ctx.vehicle_pos = vehicle_position;
	ctx.mission_index = 2;
	ctx.seg_candidate.segment.start.idx = 2;
	ctx.seg_candidate.segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	ctx.seg_candidate.segment.end.idx = 0;
	ctx.seg_candidate.segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	ctx.seg_candidate.segment.is_loop = true;
	ctx.seg_candidate.segment_positions.start = loop_start;
	ctx.seg_candidate.segment_positions.end = loop_end;
	ctx.seg_candidate.projection = vehicle_position;
	ctx.seg_candidate.dist.xtrack = 0.f;
	ctx.seg_candidate.dist.along = 200.f + loop_segment_length / 2.f;
	ctx.seg_candidate.dist.segment_length = loop_segment_length;
	ctx.seg_candidate.dist.on_segment = loop_segment_length / 2.f;
	ctx.loop_ctx.segment = ctx.seg_candidate.segment;
	ctx.loop_ctx.segment_positions = ctx.seg_candidate.segment_positions;
	ctx.loop_ctx.along.start = 200.f;
	ctx.loop_ctx.along.end = 0.f;

	ASSERT_TRUE(ctx.valid());
	ASSERT_TRUE(ctx.loop_ctx.valid());

	// WHEN: selectSafePoint evaluates the loop context.
	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Reverse path is selected with first_item_index=2, branch_off on [1-2].
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.path.first_item_index, 2);
	EXPECT_TRUE(selection.path.direction_reversed);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 1);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 2);
}

// WHY: The planner must succeed even when DO_JUMP has remaining iterations.
// WHAT: planRouteToGoal succeeds with a valid plan despite pending loop iterations.
TEST_F(RtlSafePointTest, HandlesLoopWithRemainingIterations)
{
	// GIVEN: 5-item mission with DO_JUMP(0, repeat 3, current 1). Safe point nearby.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 3, 1),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);

	// WHEN: planRouteToGoal is called with pending loop iterations.
	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 1, config, plan, &failure_reason));

	// THEN: Plan is valid and a goal is found.
	ASSERT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
}

// ============================================================================
// NEW TESTS: Velocity edge cases for FW u-turn logic
// ============================================================================

// WHY: Without velocity info, FW cannot determine heading and should not apply a u-turn penalty.
// WHAT: FW with vehicle_velocity_valid=false picks the closest safe point regardless of direction.
TEST_F(RtlSafePointTest, FWWithZeroVelocityPicksShortestPath)
{
	// GIVEN: Same straight mission as FWUturnPenalty test, but vehicle_velocity_valid=false.
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),   // A: ~200m behind
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),  // B: ~600m ahead
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = false;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);

	// WHEN: FW vehicle without valid velocity data.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Without velocity info, FW picks the closest safe point (rally A, index 0).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
}

// WHY: Orthogonal velocity (perpendicular to route) should not trigger a u-turn penalty.
// WHAT: FW with velocity=(0,15) (east, perpendicular to north-south route) should not require u-turn.
TEST_F(RtlSafePointTest, FWWithOrthogonalVelocityNoUturn)
{
	// GIVEN: Same straight mission. FW config, velocity=(0,15) perpendicular to route.
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),   // A: ~200m behind
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),  // B: ~600m ahead
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 0.f;
	config.vehicle_velocity_east = 15.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);

	// WHEN: FW vehicle flying east (orthogonal to north-south route).
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Orthogonal velocity should not trigger u-turn; no u-turn required on selected path.
	ASSERT_TRUE(selection.found);
	EXPECT_FALSE(selection.path.u_turn_required);
}

// =============================================================================
// GROUP 7: DO_JUMP loop planning via planRouteToGoal
// =============================================================================

// WHY: When the vehicle is inside a DO_JUMP loop, the planner must correctly model
//      the loop edges and select a goal reachable via the loop geometry. The executor
//      relies on this to advance through the mission without following DO_JUMP control flow.
// WHAT: Vehicle on the corner_dataset loop area gets a valid plan with a safe point.
TEST_F(RtlSafePointTest, VehicleInsideDoJumpLoopGetsValidPlan)
{
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10214, 2.31760, kAlt + 150.f);
	config = defaultConfig();
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 5;

	RtlRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
}

// WHY: The planner must handle the DO_JUMP loop edge correctly when computing along-route
//      distances. A safe point near the loop must be reachable via the loop geometry.
// WHAT: Safe point on jump segment 7→2 is selected when the vehicle is in the loop.
TEST_F(RtlSafePointTest, SafePointOnDoJumpLoopSegmentIsReachable)
{
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	config = defaultConfig();
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 3;

	RtlRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);

	if (plan.selection.safe_point_found) {
		EXPECT_GE(plan.selection.safe_point_index, 0);
		EXPECT_TRUE(plan.selection.safe_point_position.valid());
		EXPECT_TRUE(plan.selection.branch_off_projection.valid());
	}
}

// WHY: A mission with an exhausted DO_JUMP (current_count == repeat_count) should be
//      treated as a straight-through mission with no loop edges.
// WHAT: Planning succeeds and does not create loop context when DO_JUMP is exhausted.
TEST_F(RtlSafePointTest, ExhaustedDoJumpTreatedAsStraightThrough)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 30.f),
		makeDoJump(1, 3, 3),  // exhausted: current == repeat
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt + 40.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt),
	};

	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	RtlRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_FALSE(plan.projection_context.loop_ctx.valid());
}

// =============================================================================
// GROUP 8: Direct-to-safe-point shortcut (MC vs FW)
// =============================================================================

// WHY: When a multicopter is very close to a safe point, the planner should select
//      direct_to_safe_point=true so the executor flies straight there without following the route.
// WHAT: MC vehicle within direct_acceptance_radius of a rally point gets direct-to-safe-point.
TEST_F(RtlSafePointTest, McDirectToNearbySafePoint)
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
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = defaultConfig();
	config.is_multicopter = true;
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 5.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	RtlRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.direct_to_safe_point);
}

// WHY: Fixed-wing vehicles cannot hover, so the planner should NOT select direct-to-safe-point
//      even when the safe point is very close. The vehicle must follow the route to the branch-off.
// WHAT: FW vehicle near a safe point does NOT get direct_to_safe_point=true.
TEST_F(RtlSafePointTest, FwDoesNotGetDirectToSafePoint)
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
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = fwConfig();
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	RtlRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.direct_to_safe_point);
}
