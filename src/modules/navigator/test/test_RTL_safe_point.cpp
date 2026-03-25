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
 * Unit tests for MissionRoutePlanner safe-point (rally point) evaluation and
 * selection during route-following RTL. Covers:
 *
 * - Basic safe-point selection: lowest overall path cost, direct-to, invalid points
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

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

// ============================================================================
// Test fixture
// ============================================================================

class RtlSafePointTest : public MissionRoutePlannerTestBase {};

// ============================================================================
// GROUP 1: Basic safe point selection
// ============================================================================

// WHY: The planner must prefer the safe point reachable via the lowest overall path cost.
// WHAT: selectSafePoint returns the closer rally point (index 1) on segment [0-1].
TEST_F(RtlSafePointTest, PrefersLowestOverallPathCost)
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
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 5.f, kAlt);

	// WHEN: Vehicle projects onto segment [0-1] near the start.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The closer safe point (index 1) is selected, branching off segment [0-1].
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
}

// WHY: The scorer must add the final branch-off leg to avoid preferring a short along-route detour
//      that still produces a longer total return path.
// WHAT: The farther-along but near-route safe point beats the near-along but far-off-route safe point.
TEST_F(RtlSafePointTest, IncludesBranchOffLegInSafePointRanking)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 15.f, 80.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 5.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
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
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: Vehicle is near the safe point within direct acceptance radius.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: Vehicle projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: No safe point is found.
	EXPECT_FALSE(selection.found);
}

// WHY: Relative-altitude rally points must be converted to AMSL using home altitude before planning.
// WHAT: A GLOBAL_RELATIVE_ALT safe point uses home_altitude_amsl for the selected goal altitude.
TEST_F(RtlSafePointTest, RelativeAltitudeSafePointUsesHomeAltitude)
{
	// GIVEN: A straight mission and one rally point stored in relative-altitude frame.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 90.f, 15.f, 40.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	config.home_altitude_amsl = 620.f;
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: The vehicle projects onto the route and selects the rally point.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The selected goal altitude is converted from relative altitude to AMSL.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_NEAR(selection.goal_position.alt, 660.f, kAltitudeTolerance);
	EXPECT_NEAR(selection.safe_point_position.alt, 660.f, kAltitudeTolerance);
}

// WHY: Relative-altitude rally points are invalid without a finite home altitude reference.
// WHAT: A GLOBAL_RELATIVE_ALT safe point is rejected when home_altitude_amsl is not finite.
TEST_F(RtlSafePointTest, RelativeAltitudeSafePointRequiresFiniteHomeAltitude)
{
	// GIVEN: The same mission geometry but without a valid home altitude reference.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 90.f, 15.f, 40.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	config.home_altitude_amsl = NAN;
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// WHEN: The planner evaluates the safe point without a valid AMSL reference.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The relative-altitude safe point is skipped instead of producing a bogus altitude.
	EXPECT_FALSE(selection.found);
	EXPECT_FALSE(selection.safe_point_found);
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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// WHEN: Vehicle at mission_index=2 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = -15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);

	// WHEN: Vehicle at mission_index=5 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 5, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.09681253236241, 2.2993209050608376, 838.48f);

	// WHEN: Vehicle at mission_index=1 is within acceptance radius of rally 4.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);

	// WHEN: Vehicle at mission_index=15 with all rally points behind.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 15, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -15.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);

	// WHEN: Vehicle at mission_index=13 with corrupted rally 0.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// WHEN: Vehicle at mission_index=2 projects onto the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: FW still picks rally 1 reverse (u-turn penalty doesn't make forward cheaper).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: The 4km u-turn penalty should make FW prefer a more distant forward rally over a closer reverse one.
// WHAT: FW picks rally B (index 1, forward) instead of closer rally A (index 0, reverse).
TEST_F(RtlSafePointTest, FWUturnPenaltySelectsForwardOverCloserReverse)
{
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const MissionRoutePlanner::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	// WHEN: FW vehicle at N+500, flying north along the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: FW picks rally B (forward, index 1) due to 4km u-turn penalty making reverse more expensive.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_FALSE(selection.path.direction_reversed);
}

// WHY: A VTOL already transitioning to fixed-wing must use the same u-turn penalty logic as FW.
// WHAT: vehicle_in_transition_to_fw selects the forward rally just like a fixed-wing vehicle.
TEST_F(RtlSafePointTest, TransitionToFwUsesFixedWingUturnPenalty)
{
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = defaultConfig();
	config.is_multicopter = false;
	config.vehicle_in_transition_to_fw = true;
	config.u_turn_penalty_m = 4000.f;
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const MissionRoutePlanner::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	// WHEN: The planner evaluates a vehicle that is already committed to a front transition.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The planner applies FW-style u-turn avoidance and keeps the forward rally.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_FALSE(selection.path.direction_reversed);
	EXPECT_FALSE(selection.path.u_turn_required);
}

// WHY: MC has no u-turn penalty, so it should always pick the closest rally regardless of direction.
// WHAT: MC picks rally A (index 0, reverse, closer) for the same geometry as FWUturnPenalty test.
TEST_F(RtlSafePointTest, MCNoUturnPenaltySelectsClosestReverse)
{
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const MissionRoutePlanner::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	// WHEN: MC vehicle at N+500, flying north along the route.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// WHEN: Vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 1 is selected in reverse.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// WHY: FW corner handling must stay deterministic because the branch-off segment feeds the executor.
// WHAT: With total-cost scoring, FW selects the route-nearer rally 0 and branches off on segment [4-5].
TEST_F(RtlSafePointTest, CornerMission_CornerProjectionHandled_FW)
{
	// GIVEN: Corner mission, FW config. Same vehicle position as CornerMission_RallyOnCorner_MC.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// WHEN: FW vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 0 is selected with the expected forward branch-off geometry.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
	EXPECT_TRUE(selection.branch_off_projection.valid());
	EXPECT_FALSE(selection.path.direction_reversed);
}

// WHY: Rally points whose loop-segment candidate would create an invalid path must still be allowed
//      to win through a valid nominal-segment projection when that yields the lowest total return cost.
// WHAT: MC picks rally 3 once the final branch-off leg is included in the scorer.
TEST_F(RtlSafePointTest, CornerMission_BackNoTransition_MC)
{
	// GIVEN: Corner mission, MC config. Vehicle at index 7 near a transition boundary.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);

	// WHEN: Vehicle at mission_index=7.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 7, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 3 is selected through its valid non-loop projection.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 3);
}

// WHY: Small segments near the end of the mission must be handled without skipping valid rally points.
// WHAT: MC picks rally 5 for a vehicle near the small segments at the end of the corner mission.
TEST_F(RtlSafePointTest, CornerMission_SmallSegmentFront_MC)
{
	// GIVEN: Corner mission, MC config. Vehicle near small segments at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);

	// WHEN: Vehicle at mission_index=13.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 5 is selected.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 5);
}

// WHY: Reverse-flight corner selection must stay deterministic across the 5->7 leg near the MC transition.
// WHAT: The reverse corner scenario selects rally 2 and branches off on segment [5-7].
TEST_F(RtlSafePointTest, CornerMission_ReverseCornerScenarioSelectsRally2OnSegment5To7)
{
	// GIVEN: Corner mission, MC config. Vehicle at mission_index=5 flying reverse.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = -corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);

	// WHEN: Vehicle at mission_index=5.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 5, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 2 is selected and its branch-off stays anchored on segment [5-7].
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 2);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 5);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 7);
	EXPECT_TRUE(selection.branch_off_projection.valid());
}

// WHY: The stacked landing corner is easy to regress because segment [14-15] has zero XY length.
// WHAT: The land-corner scenario selects the land-corner rally 6 and still branches off on segment [14-15].
TEST_F(RtlSafePointTest, CornerMission_LandCornerScenarioSelectsRally6OnSegment14To15)
{
	// GIVEN: Corner mission, FW config. Vehicle near stacked landing at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10368934085859, 2.3183612137416754, 510.f);

	// WHEN: Vehicle at mission_index=13 with FW config.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 13, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Rally 6 is selected on the stacked landing segment rather than a neighboring corner.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 6);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 14);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 15);
	EXPECT_TRUE(selection.branch_off_projection.valid());
}

// WHY: The DO_JUMP loop segment must be traversed correctly during safe-point selection.
// WHAT: Selection succeeds and produces a valid branch_off when the vehicle is on a loop segment.
TEST_F(RtlSafePointTest, CornerMission_LoopSegmentIsHandled)
{
	// GIVEN: Corner mission with DO_JUMP at index 8, MC config. Vehicle on loop area.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = corner_dataset::kVelFast;
	config.vehicle_velocity_east = 0.f;

	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10264815827885, 2.321939748532329, 600.f);

	// WHEN: Vehicle at mission_index=3.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 3, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Loop segment is handled; selection resolves to a safe point with a valid branch-off.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_TRUE(selection.branch_off_segment.valid());
	EXPECT_TRUE(selection.branch_off_projection.valid());
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
	// GIVEN: 4-wp square mission with 6 safe points, using VectorProvider.
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
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	// WHEN: Safe point selection is run, counting provider accesses.
	provider.resetCounters();
	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	// GIVEN: Default 16-item mission with 7 rally points, using VectorProvider.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 2, config, ctx, nullptr));

	// WHEN: Safe point selection is run on the default dataset.
	provider.resetCounters();
	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Mission loads are bounded by O(M) not O(M*S).
	//       For M=16, S=7: O(M*S) would be 112 which must NOT pass.
	//       Bound of 2*M = 32 catches an unbatched regression.
	ASSERT_TRUE(selection.found);
	EXPECT_LE(provider.missionLoadCount(), 2 * provider.missionCount())
			<< "Mission should be scanned in one batch pass, not once per safe point";
	EXPECT_EQ(provider.safePointLoadCount(), 7);
}

// WHY: The planner now uses a simple usability bitmask, so a masked-out safe point must be
//      skipped without any callback indirection or planner-side policy knowledge.
// WHAT: The closest safe point is masked out and the next allowed one is selected.
TEST_F(RtlSafePointTest, UsableSafePointBitmaskSkipsRejectedCandidate)
{
	// GIVEN: Two valid rally points where the closer one is masked out by the caller.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 15.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 80.f, 20.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	config.usable_safe_point_bitmask = 1ULL << 1;

	// WHEN: Safe-point selection runs with only safe point 1 marked as usable.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The masked-out closer candidate is skipped and safe point 1 is returned.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 1);
}

// WHY: When the usability bitmask rejects every safe point, selectSafePoint must report that
//      no safe-point destination is available so the caller can fall back appropriately.
// WHAT: Valid safe points with a zero mask return found=false.
TEST_F(RtlSafePointTest, UsableSafePointBitmaskCanRejectAllSafePoints)
{
	// GIVEN: Two valid rally points with a usability bitmask that rejects every one of them.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 15.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 80.f, 20.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	config.usable_safe_point_bitmask = 0;

	// WHEN: Safe-point selection runs with no usable safe-point bits enabled.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: The planner reports that no safe-point destination is available.
	EXPECT_FALSE(selection.found);
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
	MissionRoutePlanner planner{provider};

	// GIVEN: Manually built ctx with loop segment [2->0, is_loop=true].
	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, kAlt);
	const MissionRoutePlanner::Position loop_start =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	const MissionRoutePlanner::Position loop_end =
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
	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	MissionRoutePlanner planner{provider};
	MissionRoutePlanner::Plan plan{};
	MissionRoutePlanner::FailureReason failure_reason{MissionRoutePlanner::FailureReason::Unknown};

	const MissionRoutePlanner::Position vehicle_position =
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
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = false;

	const MissionRoutePlanner::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	// WHEN: FW vehicle without valid velocity data.
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

	// THEN: Without velocity info, FW picks the closest safe point (rally A, index 0).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
}

// WHY: Orthogonal velocity (perpendicular to route) should not trigger a u-turn penalty.
// WHAT: FW with velocity=(0,15) (east, perpendicular to north-south route) should not require u-turn.
TEST_F(RtlSafePointTest, FWWithOrthogonalVelocityNoUturn)
{
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 0.f;
	config.vehicle_velocity_east = 15.f;

	const MissionRoutePlanner::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	// WHEN: FW vehicle flying east (orthogonal to north-south route).
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, nullptr));

	const MissionRoutePlanner::Selection selection = planner.selectSafePoint(ctx, config);

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
	// GIVEN: The vehicle is inside the active DO_JUMP loop and the planner knows the last flown loop edge.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

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

	MissionRoutePlanner::Plan plan{};

	// WHEN: The planner builds a safe-point return from inside the loop.
	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	// THEN: The plan remains loop-aware and produces a complete safe-point branch-off.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.projection_context.loop_ctx.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
	EXPECT_TRUE(plan.selection.branch_off_projection.valid());
}

// WHY: Loop planning must remain deterministic even when the cheapest safe point is reached via a nominal segment.
// WHAT: The loop scenario selects rally 3 and branches off on nominal segment [7-9].
TEST_F(RtlSafePointTest, LoopScenarioSelectsRally3OnSegment7To9)
{
	// GIVEN: A safe point lies on the active jump segment 7->2 while the vehicle is inside that loop.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

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

	MissionRoutePlanner::Plan plan{};

	// WHEN: The planner evaluates a safe point that projects onto the loop edge itself.
	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	// THEN: The planner chooses the cheapest reachable rally and keeps a valid nominal branch-off segment.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.projection_context.loop_ctx.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 3);
	EXPECT_TRUE(plan.selection.safe_point_position.valid());
	EXPECT_TRUE(plan.selection.branch_off_projection.valid());
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
	EXPECT_FALSE(plan.selection.branch_off_segment.is_loop);
	EXPECT_EQ(plan.selection.branch_off_segment.start.idx, 7);
	EXPECT_EQ(plan.selection.branch_off_segment.end.idx, 9);
}

// WHY: A mission with an exhausted DO_JUMP (current_count == repeat_count) should be
//      treated as a straight-through mission with no loop edges.
// WHAT: Planning succeeds and does not create loop context when DO_JUMP is exhausted.
TEST_F(RtlSafePointTest, ExhaustedDoJumpTreatedAsStraightThrough)
{
	// GIVEN: A mission with a DO_JUMP whose current count already exhausted its repeats.
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
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	MissionRoutePlanner::Plan plan{};

	// WHEN: The planner scans the route after the DO_JUMP has been exhausted.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning stays on the nominal route and still returns a complete safe-point plan.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_FALSE(plan.projection_context.loop_ctx.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 0);
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
}

// =============================================================================
// GROUP 8: Branch-Off Geometry
// =============================================================================

// WHY: closeToBranchOffSegment is pure branch geometry logic and should stay covered in a unit-style test.
// WHAT: A point exactly on the returned branch-off leg is considered close to that leg.
TEST_F(RtlSafePointTest, CloseToBranchOffSegmentUsesBranchGeometry)
{
	// GIVEN: A deterministic safe-point plan with a valid branch-off segment.
	auto mission = std::vector<mission_item_s> {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt + 10.f),
	};
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	MissionRoutePlanner::Plan plan{};

	// WHEN: A safe-point plan is built and the cached branch-off leg is checked.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: The branch-off projection lies on the branch leg used by the executor.
	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	ASSERT_TRUE(plan.selection.safe_point_found);
	ASSERT_TRUE(plan.selection.branch_off_projection.valid());
	EXPECT_TRUE(planner.closeToBranchOffSegment(plan.selection.branch_off_projection,
			plan.selection, config.acceptance_radius));
}

// =============================================================================
// GROUP 9: Direct-to-safe-point shortcut (MC vs FW)
// =============================================================================

struct DirectToSafePointCase {
	const char *name;
	bool is_multicopter;
	float vehicle_velocity_north;
	bool expect_direct;
};

class RtlSafePointDirectShortcutTest : public RtlSafePointTest,
	public ::testing::WithParamInterface<DirectToSafePointCase> {};

TEST_P(RtlSafePointDirectShortcutTest, PlansDirectShortcutOnlyForMulticopter)
{
	const DirectToSafePointCase &scenario = GetParam();
	VectorProvider provider(direct_to_safe_point_dataset::mission(), direct_to_safe_point_dataset::safePoints());
	MissionRoutePlanner planner(provider);

	auto vehicle_pos = direct_to_safe_point_dataset::vehiclePosition();
	config = scenario.is_multicopter ? defaultConfig() : fwConfig();
	config.is_multicopter = scenario.is_multicopter;
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = scenario.vehicle_velocity_north;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	MissionRoutePlanner::Plan plan{};
	bool ok = planner.planRouteToGoal(vehicle_pos, direct_to_safe_point_dataset::kMissionIndex, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << MissionRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 0);
	EXPECT_EQ(plan.selection.direct_to_safe_point, scenario.expect_direct);
}

INSTANTIATE_TEST_SUITE_P(
	VehicleType,
	RtlSafePointDirectShortcutTest,
	::testing::Values(
		DirectToSafePointCase{"Multicopter", true, 5.f, true},
		DirectToSafePointCase{"FixedWing", false, 15.f, false}
	),
	[](const ::testing::TestParamInfo<DirectToSafePointCase> &param_info)
{
	return param_info.param.name;
}
);
