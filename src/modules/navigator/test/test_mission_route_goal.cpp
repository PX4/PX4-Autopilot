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
 * @file test_mission_route_goal.cpp
 *
 * Mission-route goal-selection tests.
 *
 * To visualize data, use Tools/navigator_mission_planner_visualizer/mission_planner_tools.py
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "support/mission_route_test_helpers.h"
#include "test_mission_route_data.h"

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

class MissionRouteGoalTest : public MissionRouteTestBase {};

// Cost includes the branch-off leg, so the near-route safe point beats the near-along one.
TEST_F(MissionRouteGoalTest, SelectsSafePointWithLowestTotalCostIncludingBranchOffLeg)
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
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 1, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
}

// With several direct shortcuts available, the lowest-cost safe point still wins.
TEST_F(MissionRouteGoalTest, DirectShortcutUsesSelectedSafePointNotUploadOrder)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 19.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 12.f, 1.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	mission_route::RoutePlan plan{};
	ASSERT_TRUE(planRouteToGoal(planner, vehicle_position, 1, config, plan, reason))
			<< mission_route::failureReasonString(reason);

	ASSERT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.skip_route_to_safe_point);
	EXPECT_EQ(plan.selection.safe_point_index, 1);
}

// All safe points have an invalid frame, so none is selected.
TEST_F(MissionRouteGoalTest, ReturnsEmptyWhenAllSafePointsInvalid)
{
	// 3-wp mission with 3 safe points using unsupported frame=15.
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
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// Vehicle projects onto the route.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 1, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// No safe point is found.
	EXPECT_FALSE(selection.found);
}

// No safe points are available, so selectBestGoal falls back to the configured mission land item.
TEST_F(MissionRouteGoalTest, MissionEndpointFallbackUsesConfiguredLandIndex)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};
	const int32_t land_index = 2;

	VectorProvider provider{mission, {}, {}, {}, land_index};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);

	mission_route::RoutePlan plan{};
	ASSERT_TRUE(planRouteToGoal(planner, vehicle_position, 1, config, plan, reason))
			<< mission_route::failureReasonString(reason);

	ASSERT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_NEAR(plan.selection.goal_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.alt, mission[land_index].altitude, kAltitudeTolerance);
}

// A relative-altitude safe point is converted to AMSL using home altitude.
TEST_F(MissionRouteGoalTest, RelativeAltitudeSafePointUsesHomeAltitude)
{
	// A straight mission and one rally point stored in relative-altitude frame.
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
	config.parameters.home_altitude_amsl = 620.f;
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	// The vehicle projects onto the route and selects the rally point.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 1, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// The selected goal altitude is converted from relative altitude to AMSL.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_NEAR(selection.goal_position.alt, 660.f, kAltitudeTolerance);
	EXPECT_NEAR(selection.safe_point_position.alt, 660.f, kAltitudeTolerance);
}

// MC picks the closest rally even when it is behind the vehicle (rally 1, reversed).
TEST_F(MissionRouteGoalTest, DefaultMissionClosestBehindReverseMC)
{
	// Default 16-item mission with 7 rally points, MC config.
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = 15.f;
	config.state.velocity_ne(1) = 15.f;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// Vehicle at mission_index=2 projects onto the route.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 2, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 1 is selected in reverse direction.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// MC picks the closest rally when it is ahead (rally 0).
TEST_F(MissionRouteGoalTest, DefaultMissionClosestForwardAheadMC)
{
	// Default mission, MC config, vehicle flying with velocity (15,-15).
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = 15.f;
	config.state.velocity_ne(1) = -15.f;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);

	// Vehicle at mission_index=5 projects onto the route.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 5, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 0 is selected (forward, on segment 5-7 ahead of vehicle).
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
}

// All rallies are behind, so MC picks the closest one in reverse (rally 0).
TEST_F(MissionRouteGoalTest, DefaultMissionAllBehindMC)
{
	// Default mission, MC config. Vehicle near mission_index=15 (end of route).
	VectorProvider provider{default_dataset::mission(), default_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = 15.f;
	config.state.velocity_ne(1) = 15.f;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);

	// Vehicle at mission_index=15 with all rally points behind.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 15, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Closest reverse rally is selected (rally 0), direction reversed.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// A corrupted rally (NaN lat) is skipped; selection still succeeds with finite coordinates.
TEST_F(MissionRouteGoalTest, DefaultMissionInvalidRallyPointSkipped)
{
	// Default mission with rally[0].lat set to NAN (corrupted).
	auto safe_points = default_dataset::safePoints();
	safe_points[0].lat = NAN;

	VectorProvider provider{default_dataset::mission(), safe_points};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = -15.f;
	config.state.velocity_ne(1) = 15.f;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);

	// Vehicle at mission_index=13 with corrupted rally 0.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 13, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// A valid safe point is still found, and branch_off projection coordinates are finite.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(std::isfinite(selection.branch_off_projection.lat));
	EXPECT_TRUE(std::isfinite(selection.branch_off_projection.lon));
}

enum class UturnVehicle { Multicopter, FixedWing, TransitionToFw };

struct UturnPenaltyCase {
	const char *name;
	UturnVehicle vehicle;
	bool velocity_valid;
	float velocity_north;
	float velocity_east;
	int expected_safe_point_index; // -1 to skip
	int expected_reversed;         // -1 to skip, else 0/1
	int expected_uturn_required;   // -1 to skip, else 0/1
};

class MissionRouteGoalUturnPenaltyTest : public MissionRouteTestBase,
	public ::testing::WithParamInterface<UturnPenaltyCase> {};

TEST_P(MissionRouteGoalUturnPenaltyTest, SelectsRallyAccordingToUturnPenalty)
{
	const UturnPenaltyCase &scenario = GetParam();
	VectorProvider provider{uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints()};
	MissionRoutePlanner planner{provider};

	config = scenario.vehicle == UturnVehicle::FixedWing ? fwConfig() : defaultConfig();

	if (scenario.vehicle == UturnVehicle::TransitionToFw) {
		config.state.in_transition_to_fw = true;
		config.parameters.u_turn_penalty_m = 4000.f;
	}

	config.state.velocity_valid = scenario.velocity_valid;
	config.state.velocity_ne(0) = scenario.velocity_north;
	config.state.velocity_ne(1) = scenario.velocity_east;

	const mission_route::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();

	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, uturn_penalty_dataset::kMissionIndex, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);

	if (scenario.expected_safe_point_index >= 0) {
		EXPECT_EQ(selection.safe_point_index, scenario.expected_safe_point_index);
	}

	if (scenario.expected_reversed >= 0) {
		EXPECT_EQ(selection.path.direction_reversed, scenario.expected_reversed != 0);
	}

	if (scenario.expected_uturn_required >= 0) {
		EXPECT_EQ(selection.path.u_turn_required, scenario.expected_uturn_required != 0);
	}
}

INSTANTIATE_TEST_SUITE_P(
	UturnPenalty,
	MissionRouteGoalUturnPenaltyTest,
	::testing::Values(
		// FW penalty: closer reverse rally A is dropped for the farther forward rally B.
		UturnPenaltyCase{"FixedWingPenaltySelectsForwardOverCloserReverse", UturnVehicle::FixedWing, true, 15.f, 0.f, 1, 0, -1},
		// A VTOL already transitioning to FW applies the same penalty as a fixed-wing.
		UturnPenaltyCase{"TransitionToFwUsesFixedWingPenalty", UturnVehicle::TransitionToFw, true, 15.f, 0.f, 1, 0, 0},
		// MC has no penalty, so it keeps the closest (reverse) rally A.
		UturnPenaltyCase{"MulticopterNoPenaltySelectsClosestReverse", UturnVehicle::Multicopter, true, 15.f, 0.f, 0, 1, -1},
		// Without a usable velocity, FW cannot detect a u-turn and falls back to the closest rally A.
		UturnPenaltyCase{"FixedWingInvalidVelocitySelectsClosestReverse", UturnVehicle::FixedWing, false, 0.f, 0.f, 0, -1, -1},
		// Velocity orthogonal to the route is not a u-turn, so no penalty is applied.
		UturnPenaltyCase{"FixedWingOrthogonalVelocityHasNoUturn", UturnVehicle::FixedWing, true, 0.f, 15.f, -1, -1, 0}
	),
	[](const ::testing::TestParamInfo<UturnPenaltyCase> &param_info)
{
	return param_info.param.name;
}
);

// On the corner mission, MC picks the closest rally in reverse (rally 1).
TEST_F(MissionRouteGoalTest, CornerMissionRallyOnCornerMC)
{
	// Corner 16-item mission with 8 rally points, MC config.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = -corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = -corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// Vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 2, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 1 is selected in reverse.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_TRUE(selection.path.direction_reversed);
}

// FW picks the route-nearer rally 0 and branches off on [4-5].
TEST_F(MissionRouteGoalTest, CornerMissionCornerProjectionHandledFW)
{
	// Corner mission, FW config. Same vehicle position as CornerMissionRallyOnCornerMC.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = -corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = -corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);

	// FW vehicle at mission_index=2 on the corner mission.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 2, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 0 is selected with the expected forward branch-off geometry.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
	EXPECT_TRUE(selection.branch_off_projection.valid());
	EXPECT_FALSE(selection.path.direction_reversed);
}

// A rally with an invalid loop candidate still wins via its nominal projection (rally 3).
TEST_F(MissionRouteGoalTest, CornerMissionBackNoTransitionMC)
{
	// Corner mission, MC config. Vehicle at index 7 near a transition boundary.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = -corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);

	// Vehicle at mission_index=7.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 7, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 3 is selected through its valid non-loop projection.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 3);
}

// Small end-of-mission segments must not hide valid rallies (rally 5).
TEST_F(MissionRouteGoalTest, CornerMissionSmallSegmentFrontMC)
{
	// Corner mission, MC config. Vehicle near small segments at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);

	// Vehicle at mission_index=13.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 13, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 5 is selected.
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.safe_point_index, 5);
}

// Reverse-flight corner case: rally 2, branch-off on [5-7].
TEST_F(MissionRouteGoalTest, CornerMissionReverseCornerScenarioSelectsRally2OnSegment5To7)
{
	// Corner mission, MC config. Vehicle at mission_index=5 flying reverse.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = -corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = -corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);

	// Vehicle at mission_index=5.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 5, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 2 is selected and its branch-off stays anchored on segment [5-7].
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, 2);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 5);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 7);
	EXPECT_TRUE(selection.branch_off_projection.valid());
}

// Zero-length land segment [14-15]: rally 6 still branches off there.
TEST_F(MissionRouteGoalTest, CornerMissionLandCornerScenarioSelectsRally6OnSegment14To15)
{
	// Corner mission, FW config. Vehicle near stacked landing at mission_index=13.
	VectorProvider provider{corner_dataset::mission(), corner_dataset::safePoints()};
	MissionRoutePlanner planner{provider};
	config = fwConfig();
	config.state.velocity_valid = true;
	config.state.velocity_ne(0) = corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = corner_dataset::kVelDiag;

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10368934085859, 2.3183612137416754, 510.f);

	// Vehicle at mission_index=13 with FW config.
	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 13, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// Rally 6 is selected on the stacked landing segment rather than a neighboring corner.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, 6);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 14);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 15);
	EXPECT_TRUE(selection.branch_off_projection.valid());
}

// One mission scan covers all safe points: loads stay near 2*M, not M*S.
TEST_F(MissionRouteGoalTest, ScansMissionOnceForBatchSimple)
{
	// 4-wp square mission with 6 safe points, using VectorProvider.
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
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt);

	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 1, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	// Safe point selection is run, counting provider accesses.
	provider.resetCounters();
	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// 2*M catches unbatched O(M*S) regressions while allowing modest overhead.
	const int mission_count = static_cast<int>(mission.size());
	ASSERT_TRUE(selection.found);
	EXPECT_LE(provider.missionLoadCount(), 2 * mission_count)
			<< "mission scan was repeated";
	EXPECT_EQ(provider.safePointLoadCount(), 6);
}

// Lowest cost safe point must be picked even if after the first batch
TEST_F(MissionRouteGoalTest, SelectsBestSafePointBeyondFirstBatch)
{
	// Straight north mission (single segment).
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};

	// The single best (near-route) rally point: ~5 m off-route next to the vehicle projection.
	const mission_item_s best_safe_point = makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 5.f, kAlt);

	// Fill the entire first batch with far rally points,
	// then place the best rally point at the start of the second batch.
	std::vector<mission_item_s> safe_points;
	safe_points.reserve(static_cast<size_t>(mission_route::kMaxSafePointBatch) + 1);

	for (uint16_t i = 0; i < mission_route::kMaxSafePointBatch; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f + 5.f * i, 500.f, kAlt));
	}

	const int32_t best_index = static_cast<int32_t>(mission_route::kMaxSafePointBatch);
	safe_points.push_back(best_safe_point);

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);

	ASSERT_TRUE(collectVehicleProjection(planner, vehicle_position, 1, config, ctx, reason))
			<< mission_route::failureReasonString(reason);

	provider.resetCounters();
	const mission_route::GoalSelection selection = planner.selectSafePoint(ctx, config);

	// The near-route rally point in the second batch wins despite a full first batch of valid candidates.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, best_index);

	// Every configured rally point is read exactly once.
	EXPECT_EQ(provider.safePointLoadCount(), static_cast<int>(safe_points.size()));

	// The mission is scanned once per batch (two here), not once per safe point.
	const int num_batches = (static_cast<int>(safe_points.size()) + mission_route::kMaxSafePointBatch - 1)
				/ mission_route::kMaxSafePointBatch;
	EXPECT_LE(provider.missionLoadCount(), 2 * static_cast<int>(mission.size()) * num_batches)
			<< "mission scan should run per batch, not per safe point";
}


// When the vehicle is on a loop segment, the planner must correctly evaluate reverse-jump paths.
TEST_F(MissionRouteGoalTest, HandlesLoopProjectionAndReverseJumpChoice)
{
	// The DO_JUMP(target 0) at index 3 forms the loop edge [2->0]; index 4 lies past the jump so the
	// scanner actually walks the loop edge (a jump as the last item is never reached by the scan).
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),     // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),   // 1
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt), // 2
		makeDoJump(0, 3, 0),                                               // 3: active loop, jumps to 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt), // 4: continues past the jump
	};

	// Safe point projects onto nominal segment [1-2], just short of the loop start along the route.
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 205.f, 100.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};

	// Anchor the projection to the active loop edge [2->0] and put the vehicle on its midpoint.
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 3;

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);

	mission_route::RoutePlan plan{};
	ASSERT_TRUE(planRouteToGoal(planner, vehicle_position, 0, config, plan, reason))
			<< mission_route::failureReasonString(reason);

	EXPECT_TRUE(plan.projection_context.loop_context.valid());
	EXPECT_TRUE(plan.projection_context.route_projection.segment.is_loop);

	// Reverse jump is selected with first_item_index=2, branching off on [1-2].
	ASSERT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.path.first_item_index, 2);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_EQ(plan.selection.branch_off_segment.start.idx, 1);
	EXPECT_EQ(plan.selection.branch_off_segment.end.idx, 2);
}

// Planning succeeds even when the DO_JUMP still has iterations left.
TEST_F(MissionRouteGoalTest, HandlesLoopWithRemainingIterations)
{
	// 5-item mission with DO_JUMP(0, repeat 3, current 1). Safe point nearby.
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
	mission_route::RoutePlan plan{};
	mission_route::FailureReason failure_reason{mission_route::FailureReason::kUnknown};

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);

	// planRouteToGoal is called with pending loop iterations.
	ASSERT_TRUE(planRouteToGoal(planner, vehicle_position, 1, config, plan, failure_reason))
			<< mission_route::failureReasonString(failure_reason);

	// Plan is valid and a goal is found.
	ASSERT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
}

// Loop geometry must produce a reachable safe-point plan without executing DO_JUMP.
TEST_F(MissionRouteGoalTest, VehicleInsideDoJumpLoopGetsValidPlan)
{
	// The vehicle is inside the active DO_JUMP loop and the planner knows the last flown loop edge.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_position = makePositionAbsolute(46.10214, 2.31760, kAlt + 150.f);
	config = defaultConfig();
	config.state.velocity_ne(0) = corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = -corner_dataset::kVelDiag;
	config.state.velocity_valid = true;

	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 5;

	mission_route::RoutePlan plan{};

	// The planner builds a safe-point return from inside the loop.
	bool ok = planRouteToGoal(planner, vehicle_position, 7, config, plan, reason);

	// The plan remains loop-aware and produces a complete safe-point branch-off.
	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.projection_context.loop_context.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
	EXPECT_TRUE(plan.selection.branch_off_projection.valid());
}

// Loop case where the cheapest path is nominal: rally 3, branch-off on [7-9].
TEST_F(MissionRouteGoalTest, LoopScenarioSelectsRally3OnSegment7To9)
{
	// A safe point lies on the active jump segment 7->2 while the vehicle is inside that loop.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	MissionRoutePlanner planner(provider);

	auto vehicle_position = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	config = defaultConfig();
	config.state.velocity_ne(0) = corner_dataset::kVelDiag;
	config.state.velocity_ne(1) = corner_dataset::kVelDiag;
	config.state.velocity_valid = true;

	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 3;

	mission_route::RoutePlan plan{};

	// The planner evaluates a safe point that projects onto the loop edge itself.
	bool ok = planRouteToGoal(planner, vehicle_position, 7, config, plan, reason);

	// The planner chooses the cheapest reachable rally and keeps a valid nominal branch-off segment.
	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	EXPECT_TRUE(plan.projection_context.loop_context.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 3);
	EXPECT_TRUE(plan.selection.safe_point_position.valid());
	EXPECT_TRUE(plan.selection.branch_off_projection.valid());
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
	EXPECT_FALSE(plan.selection.branch_off_segment.is_loop);
	EXPECT_EQ(plan.selection.branch_off_segment.start.idx, 7);
	EXPECT_EQ(plan.selection.branch_off_segment.end.idx, 9);
}

// Exhausted DO_JUMP items behave like straight-through mission items.
TEST_F(MissionRouteGoalTest, ExhaustedDoJumpTreatedAsStraightThrough)
{
	// A mission with a DO_JUMP whose current count already exhausted its repeats.
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

	auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.state.velocity_ne(0) = 10.f;
	config.state.velocity_ne(1) = 0.f;
	config.state.velocity_valid = true;

	mission_route::RoutePlan plan{};

	// The planner scans the route after the DO_JUMP has been exhausted.
	bool ok = planRouteToGoal(planner, vehicle_position, 0, config, plan, reason);

	// Planning stays on the nominal route and still returns a complete safe-point plan.
	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_FALSE(plan.projection_context.loop_context.valid());
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 0);
	EXPECT_TRUE(plan.selection.branch_off_segment.valid());
}

// closeToBranchOffSegment checks both crosstrack and altitude.
TEST_F(MissionRouteGoalTest, CloseToBranchOffSegmentChecksCrosstrackAndAltitude)
{
	// A deterministic safe-point plan with a valid branch-off segment.
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

	auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);
	config = defaultConfig();
	config.state.velocity_ne(0) = 10.f;
	config.state.velocity_ne(1) = 0.f;
	config.state.velocity_valid = true;

	mission_route::RoutePlan plan{};

	// A safe-point plan is built and the cached branch-off leg is checked.
	bool ok = planRouteToGoal(planner, vehicle_position, 0, config, plan, reason);

	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	ASSERT_TRUE(plan.selection.safe_point_found);
	ASSERT_TRUE(plan.selection.branch_off_projection.valid());

	// A query point exactly on the branch-off leg, at its interpolated altitude, is close.
	EXPECT_TRUE(planner.closeToBranchOffSegment(plan.selection.branch_off_projection, plan.selection,
			config.parameters.acceptance_radius, config.parameters.altitude_acceptance_radius));

	// Reuse the horizontal point with a bad altitude.
	auto high_position = plan.selection.branch_off_projection;
	high_position.alt += 5.f * config.parameters.altitude_acceptance_radius;
	EXPECT_FALSE(planner.closeToBranchOffSegment(high_position, plan.selection,
			config.parameters.acceptance_radius, config.parameters.altitude_acceptance_radius));
}

// Being near the selected branch-off leg skips route following.
TEST_F(MissionRouteGoalTest, SelectedBranchLegShortcutSkipsRouteForSelectedGoal)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};

	auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 10.f, kAlt);
	config = defaultConfig();
	config.parameters.direct_acceptance_radius = 20.f;
	config.parameters.acceptance_radius = 20.f;
	config.state.velocity_ne(0) = 5.f;
	config.state.velocity_ne(1) = 0.f;
	config.state.velocity_valid = true;

	mission_route::RoutePlan plan{};
	const bool ok = planRouteToGoal(planner, vehicle_position, 0, config, plan, reason);

	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	ASSERT_TRUE(plan.selection.safe_point_found);
	EXPECT_FALSE(get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
			plan.selection.goal_position.lat, plan.selection.goal_position.lon)
		     < config.parameters.direct_acceptance_radius);
	EXPECT_TRUE(plan.selection.skip_route_to_safe_point);
}
struct DirectToSafePointCase {
	const char *name;
	bool use_fw_config;
	float velocity_north;
	bool expect_direct;
};

class MissionRouteGoalDirectShortcutTest : public MissionRouteTestBase,
	public ::testing::WithParamInterface<DirectToSafePointCase> {};

TEST_P(MissionRouteGoalDirectShortcutTest, PlansDirectShortcutForSelectedSafePoint)
{
	const DirectToSafePointCase &scenario = GetParam();
	VectorProvider provider(direct_to_safe_point_dataset::mission(), direct_to_safe_point_dataset::safePoints());
	MissionRoutePlanner planner(provider);

	auto vehicle_position = direct_to_safe_point_dataset::vehiclePosition();
	config = scenario.use_fw_config ? fwConfig() : defaultConfig();
	config.parameters.direct_acceptance_radius = 20.f;
	config.state.velocity_ne(0) = scenario.velocity_north;
	config.state.velocity_ne(1) = 0.f;
	config.state.velocity_valid = true;

	mission_route::RoutePlan plan{};
	bool ok = planRouteToGoal(planner, vehicle_position, direct_to_safe_point_dataset::kMissionIndex, config, plan, reason);

	ASSERT_TRUE(ok) << "Failure reason: " << mission_route::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.selection.safe_point_index, 0);
	EXPECT_EQ(plan.selection.skip_route_to_safe_point, scenario.expect_direct);
}

INSTANTIATE_TEST_SUITE_P(
	VehicleType,
	MissionRouteGoalDirectShortcutTest,
	::testing::Values(
		DirectToSafePointCase{"Multicopter", false, 5.f, true},
		DirectToSafePointCase{"FixedWing", true, 15.f, true}
	),
	[](const ::testing::TestParamInfo<DirectToSafePointCase> &param_info)
{
	return param_info.param.name;
}
);
