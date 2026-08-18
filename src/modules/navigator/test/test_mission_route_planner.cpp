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
 * @file test_mission_route_planner.cpp
 *
 * Mission-route public planner-operation tests.
 *
 * To visualize data, use Tools/navigator_mission_planner_visualizer/mission_planner_tools.py
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_planner.h"
#include "support/mission_route_test_helpers.h"
#include "test_mission_route_data.h"

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

static constexpr uint16_t kSafePointBatchSize{CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE};

class MissionRoutePlannerTest : public ::testing::Test {};

static mission_route::MissionResumeRequest makeMissionResumeRequest(const mission_route::Position &vehicle_position,
		int32_t mission_index)
{
	mission_route::MissionResumeRequest request{};
	request.vehicle_position = vehicle_position;
	request.mission_index = mission_index;
	request.home_altitude_amsl = 500.f;
	request.projection_search_distance_m = 60.f;
	request.acceptance_radius_m = 10.f;
	return request;
}

static mission_route::RouteToGoalRequest makeRouteToGoalRequest(const mission_route::Position &vehicle_position,
		int32_t mission_index)
{
	mission_route::RouteToGoalRequest request{};
	request.vehicle_position = vehicle_position;
	request.mission_index = mission_index;
	request.home_altitude_amsl = 500.f;
	request.projection_search_distance_m = 60.f;
	request.safe_point_projection_search_distance_m = 60.f;
	request.acceptance_radius_m = 10.f;
	request.direct_goal_acceptance_radius_m = 10.f;
	request.altitude_acceptance_radius_m = 10.f;
	return request;
}

/**
 * Provider that injects one mission-item read failure after safe-point selection starts.
 *
 * The vehicle projection therefore succeeds, the first safe-point route scan observes the
 * failure, and all later route scans and endpoint reads can proceed normally.
 */
class OneShotSafePointScanFailureProvider : public mission_route::Provider
{
public:
	OneShotSafePointScanFailureProvider(const std::vector<mission_item_s> &mission_items,
					    const std::vector<mission_item_s> &safe_point_items,
					    int32_t failed_mission_index) :
		_failed_mission_index(failed_mission_index)
	{
		_delegate.setMissionItems(mission_items);
		_delegate.setSafePointItems(safe_point_items);
	}

	int missionCount() const override { return _delegate.missionCount(); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		if (_failure_armed && !_failure_injected && index == _failed_mission_index) {
			_failure_injected = true;
			return false;
		}

		return _delegate.loadMissionItem(index, mission_item);
	}

	int safePointCount() const override { return _delegate.safePointCount(); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		const bool success = _delegate.loadSafePointItem(index, safe_point_item);
		_failure_armed = true;
		return success;
	}

	bool failureInjected() const { return _failure_injected; }

private:
	VectorProvider _delegate{};
	int32_t _failed_mission_index{-1};
	mutable bool _failure_armed{false};
	mutable bool _failure_injected{false};
};

static mission_item_s makeVtolApproachFromOffset(float north_m, float east_m, float altitude)
{
	mission_item_s item = makePositionItemFromOffset(kBaseLat, kBaseLon, north_m, east_m, altitude,
			      NAV_CMD_LOITER_TO_ALT);
	item.loiter_radius = 30.f;
	return item;
}

// Cost includes the branch-off leg, so the near-route safe point beats the near-along one.
TEST_F(MissionRoutePlannerTest, SelectsSafePointWithLowestTotalCostIncludingBranchOffLeg)
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

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
}

// With several direct shortcuts available, the lowest-cost safe point still wins.
TEST_F(MissionRoutePlannerTest, DirectShortcutUsesSelectedSafePointNotUploadOrder)
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

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_TRUE(plan.fly_direct_to_goal);
	EXPECT_EQ(plan.safe_point_index, 1);
}

// All safe points have an invalid frame and no mission endpoint is available as a fallback.
TEST_F(MissionRoutePlannerTest, FailsWhenAllSafePointsAreInvalidAndNoEndpointExists)
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

	VectorProvider provider = makeRouteProvider(mission, invalid_safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

// A closer rally cannot borrow the landing approach belonging to the next rally block.
TEST_F(MissionRoutePlannerTest, RequiredVtolApproachSelectsFartherEligibleSafePoint)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 15.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 80.f, 20.f, kAlt),
		makeVtolApproachFromOffset(95.f, 20.f, kAlt + 20.f),
	};

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 1);
}

// If no rally has the required approach, normal endpoint fallback remains available.
TEST_F(MissionRoutePlannerTest, RequiredVtolApproachFallsBackToMissionEndpoint)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 15.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 320.f, 20.f, kAlt),
	};
	const int32_t land_index = 2;

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.mission_land_index = land_index;
	request.require_vtol_approach = true;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_NEAR(plan.goal_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.goal_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
}

// Invalid approach records do not end a rally block when a valid approach follows.
TEST_F(MissionRoutePlannerTest, RequiredVtolApproachScansTheWholeRallyBlock)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	mission_item_s invalid_relative_approach = makeVtolApproachFromOffset(45.f, 15.f, 20.f);
	invalid_relative_approach.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;
	invalid_relative_approach.altitude_is_relative = true;
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 15.f, kAlt),
		invalid_relative_approach,
		makeVtolApproachFromOffset(55.f, 20.f, kAlt + 20.f),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 120.f, 10.f, kAlt),
	};

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	request.home_altitude_amsl = NAN;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
}

// The published index points to a waypoint even though a later LAND exists. The planner must
// reject that exact item and use takeoff fallback instead of rescanning the mission for LAND.
TEST_F(MissionRoutePlannerTest, MissionEndpointFallbackDoesNotRescanNonLandConfiguredIndex)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};
	const int32_t non_land_index = 2;

	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.mission_land_index = non_land_index;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);
	EXPECT_NEAR(plan.goal_position.lat, mission[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.goal_position.lon, mission[0].lon, kLatLonToleranceDeg);
}

// The published land item can precede other position items. Score it at its own route location,
// not at the end of the complete route.
TEST_F(MissionRoutePlannerTest, MissionEndpointFallbackUsesLandRouteDistance)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};
	const int32_t land_index = 3;

	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.mission_land_index = land_index;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_NEAR(plan.goal_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.goal_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
}

// A relative-altitude safe point is converted to AMSL using home altitude.
TEST_F(MissionRoutePlannerTest, RelativeAltitudeSafePointUsesHomeAltitude)
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

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.home_altitude_amsl = 620.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// The selected goal altitude is converted from relative altitude to AMSL.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_NEAR(plan.goal_position.alt, 660.f, kAltitudeTolerance);
}

// MC picks the closest rally even when it is behind the vehicle (rally 1, reversed).
TEST_F(MissionRoutePlannerTest, DefaultMissionClosestBehindReverseMC)
{
	// Default 16-item mission with 7 rally points, MC config.
	VectorProvider provider = makeRouteProvider(default_dataset::mission(), default_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 1 is selected in reverse direction.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_TRUE(plan.direction_reversed);
}

// MC picks the closest rally when it is ahead (rally 0).
TEST_F(MissionRoutePlannerTest, DefaultMissionClosestForwardAheadMC)
{
	// Default mission, MC config, vehicle flying with velocity (15,-15).
	VectorProvider provider = makeRouteProvider(default_dataset::mission(), default_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 5);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = -15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 0 is selected (forward, on segment 5-7 ahead of vehicle).
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_FALSE(plan.direction_reversed);
}

// All rallies are behind, so MC picks the closest one in reverse (rally 0).
TEST_F(MissionRoutePlannerTest, DefaultMissionAllBehindMC)
{
	// Default mission, MC config. Vehicle near mission_index=15 (end of route).
	VectorProvider provider = makeRouteProvider(default_dataset::mission(), default_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 15);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Closest reverse rally is selected (rally 0), direction reversed.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_TRUE(plan.direction_reversed);
}

// A corrupted rally (NaN lat) is skipped; selection still succeeds with finite coordinates.
TEST_F(MissionRoutePlannerTest, DefaultMissionInvalidRallyPointSkipped)
{
	// Default mission with rally[0].lat set to NAN (corrupted).
	auto safe_points = default_dataset::safePoints();
	safe_points[0].lat = static_cast<double>(NAN);

	VectorProvider provider = makeRouteProvider(default_dataset::mission(), safe_points);
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 13);
	request.velocity_north_m_s = -15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// A valid safe point is still found, and branch-off coordinates are finite.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(std::isfinite(plan.branch_off_position.lat));
	EXPECT_TRUE(std::isfinite(plan.branch_off_position.lon));
}

enum class UturnVehicle { Multicopter, FixedWing, TransitionToFw };

struct UturnPenaltyCase {
	const char *name;
	UturnVehicle vehicle;
	float velocity_north;
	float velocity_east;
	int expected_safe_point_index;
	bool expected_reversed;
};

class MissionRoutePlannerUturnPenaltyTest : public ::testing::Test,
	public ::testing::WithParamInterface<UturnPenaltyCase> {};

TEST_P(MissionRoutePlannerUturnPenaltyTest, SelectsRallyAccordingToUturnPenalty)
{
	const UturnPenaltyCase &scenario = GetParam();
	VectorProvider provider = makeRouteProvider(uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position = uturn_penalty_dataset::vehiclePosition();
	mission_route::RouteToGoalRequest request =
		makeRouteToGoalRequest(vehicle_position, uturn_penalty_dataset::kMissionIndex);
	request.is_fixed_wing = scenario.vehicle == UturnVehicle::FixedWing;
	request.in_transition_to_fw = scenario.vehicle == UturnVehicle::TransitionToFw;
	request.velocity_north_m_s = scenario.velocity_north;
	request.velocity_east_m_s = scenario.velocity_east;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, scenario.expected_safe_point_index);
	EXPECT_EQ(plan.direction_reversed, scenario.expected_reversed);
}

INSTANTIATE_TEST_SUITE_P(
	UturnPenalty,
	MissionRoutePlannerUturnPenaltyTest,
	::testing::Values(
		// FW penalty: closer reverse rally A is dropped for the farther forward rally B.
UturnPenaltyCase{
	"FixedWingPenaltySelectsForwardOverCloserReverse",
	UturnVehicle::FixedWing, 15.f, 0.f, 1, false
},
// A VTOL already transitioning to FW applies the same penalty as a fixed-wing.
UturnPenaltyCase{
	"TransitionToFwUsesFixedWingPenalty",
	UturnVehicle::TransitionToFw, 15.f, 0.f, 1, false
},
// MC has no penalty, so it keeps the closest (reverse) rally A.
UturnPenaltyCase{
	"MulticopterNoPenaltySelectsClosestReverse",
	UturnVehicle::Multicopter, 15.f, 0.f, 0, true
},
// Without a usable velocity, FW cannot detect a u-turn and falls back to the closest rally A.
UturnPenaltyCase{
	"FixedWingInvalidVelocitySelectsClosestReverse",
	UturnVehicle::FixedWing, NAN, NAN, 0, true
},
// Velocity orthogonal to the route is not a u-turn, so no penalty is applied.
UturnPenaltyCase{
	"FixedWingOrthogonalVelocityHasNoUturn",
	UturnVehicle::FixedWing, 0.f, 15.f, 0, true
}
	),
[](const ::testing::TestParamInfo<UturnPenaltyCase> &param_info)
{
	return param_info.param.name;
}
);

// On the corner mission, MC picks the closest rally in reverse (rally 1).
TEST_F(MissionRoutePlannerTest, CornerMissionRallyOnCornerMC)
{
	// Corner 16-item mission with 8 rally points, MC config.
	VectorProvider provider = makeRouteProvider(corner_dataset::mission(), corner_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 1 is selected in reverse.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_TRUE(plan.direction_reversed);
}

// FW picks the route-nearer rally 0 and branches off on [4-5].
TEST_F(MissionRoutePlannerTest, CornerMissionCornerProjectionHandledFW)
{
	// Corner mission, FW config. Same vehicle position as CornerMissionRallyOnCornerMC.
	VectorProvider provider = makeRouteProvider(corner_dataset::mission(), corner_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.is_fixed_wing = true;
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 0 is selected with the expected forward branch-off geometry.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, 5);
	EXPECT_TRUE(plan.branch_off_position.valid());
	EXPECT_FALSE(plan.direction_reversed);
}

// A rally with an invalid loop candidate still wins via its nominal projection (rally 3).
TEST_F(MissionRoutePlannerTest, CornerMissionBackNoTransitionMC)
{
	// Corner mission, MC config. Vehicle at index 7 near a transition boundary.
	VectorProvider provider = makeRouteProvider(corner_dataset::mission(), corner_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 3 is selected through its valid non-loop projection.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 3);
}

// Small end-of-mission segments must not hide valid rallies (rally 5).
TEST_F(MissionRoutePlannerTest, CornerMissionSmallSegmentFrontMC)
{
	// Corner mission, MC config. Vehicle near small segments at mission_index=13.
	VectorProvider provider = makeRouteProvider(corner_dataset::mission(), corner_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 13);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 5 is selected.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 5);
}

// Reverse-flight corner case: rally 2, branch-off on [5-7].
TEST_F(MissionRoutePlannerTest, CornerMissionReverseCornerScenarioSelectsRally2OnSegment5To7)
{
	// Corner mission, MC config. Vehicle at mission_index=5 flying reverse.
	VectorProvider provider = makeRouteProvider(corner_dataset::mission(), corner_dataset::safePoints());
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 5);
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// Rally 2 is selected and its branch-off leaves the route at index 7.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 2);
	EXPECT_EQ(plan.branch_off_mission_item_index, 7);
	EXPECT_TRUE(plan.branch_off_position.valid());
}

// A lower-cost safe point in a later scan batch must beat valid candidates from the first batch.
TEST_F(MissionRoutePlannerTest, LowerCostSafePointInLaterBatchWins)
{
	// Straight north mission (single segment).
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};

	// The single best (near-route) rally point: ~5 m off-route next to the vehicle projection.
	const mission_item_s best_safe_point = makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 5.f, kAlt);

	// Fill the first scan batch, then add the winner as the only item in the second batch.
	std::vector<mission_item_s> safe_points;
	safe_points.reserve(static_cast<size_t>(kSafePointBatchSize) + 1);

	for (uint16_t i = 0; i < kSafePointBatchSize; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f + 5.f * i, 500.f, kAlt));
	}

	const int32_t best_index = static_cast<int32_t>(kSafePointBatchSize);
	safe_points.push_back(best_safe_point);

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	// The second-batch safe point beats every valid candidate from the first batch.
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, best_index);
}

// A failed safe-point route scan does not suppress a valid mission endpoint fallback.
TEST_F(MissionRoutePlannerTest, SafePointScanFailureFallsBackToMissionEndpoint)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 10.f, kAlt),
	};
	const int32_t land_index = 2;
	OneShotSafePointScanFailureProvider provider{mission, safe_points, 1};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.mission_land_index = land_index;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_TRUE(provider.failureInjected());
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_NEAR(plan.goal_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.goal_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
}

// A later safe-point scan can still provide the winner after an earlier scan failed.
TEST_F(MissionRoutePlannerTest, LaterSafePointWinnerSurvivesEarlierScanFailure)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points;
	safe_points.reserve(static_cast<size_t>(kSafePointBatchSize) + 1);

	for (uint16_t i = 0; i < kSafePointBatchSize; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f + i, 500.f, kAlt));
	}

	const int32_t winner_index = static_cast<int32_t>(safe_points.size());
	safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 5.f, kAlt));
	OneShotSafePointScanFailureProvider provider{mission, safe_points, 1};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_TRUE(provider.failureInjected());
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, winner_index);
}

// Mission resume follows the route in nominal direction toward the mission endpoint.
TEST_F(MissionRoutePlannerTest, PlansNominalMissionResumeJoinToNextItem)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt);
	const mission_route::MissionResumeRequest request = makeMissionResumeRequest(vehicle_position, 1);
	mission_route::MissionResumePlan plan{};

	const mission_route::FailureReason status = planner.planMissionResumeJoin(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.join_position.valid());
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_EQ(plan.first_mission_item_index, 1);
	EXPECT_FALSE(plan.use_current_altitude);
}

// When the projected join directly targets LAND, execution keeps the vehicle's live altitude.
TEST_F(MissionRoutePlannerTest, MissionResumeNearLandUsesCurrentAltitudeForJoin)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 95.f, 0.f, kAlt + 75.f);
	const mission_route::MissionResumeRequest request = makeMissionResumeRequest(vehicle_position, 1);
	mission_route::MissionResumePlan plan{};

	const mission_route::FailureReason status = planner.planMissionResumeJoin(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.first_mission_item_index, 1);
	EXPECT_TRUE(plan.use_current_altitude);
	EXPECT_FLOAT_EQ(plan.join_position.alt, vehicle_position.alt);
}

// Active jump identity must either be absent or name two distinct in-range mission positions.
TEST_F(MissionRoutePlannerTest, RejectsInvalidActiveJumpAnchorsAndClearsPlans)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);
	const mission_route::ActiveJumpAnchor invalid_anchors[] {{0, -1}, {1, 1}, {0, 3}};

	for (const mission_route::ActiveJumpAnchor &anchor : invalid_anchors) {
		mission_route::MissionResumeRequest mission_request = makeMissionResumeRequest(vehicle_position, 1);
		mission_request.active_jump_anchor = anchor;
		mission_route::MissionResumePlan mission_plan{};
		mission_plan.join_position = vehicle_position;
		mission_plan.first_mission_item_index = 1;
		ASSERT_TRUE(mission_plan.valid());

		const mission_route::FailureReason mission_status =
			planner.planMissionResumeJoin(mission_request, mission_plan);

		EXPECT_EQ(mission_status, mission_route::FailureReason::kInvalidRequest);
		EXPECT_FALSE(mission_plan.valid());
		EXPECT_EQ(mission_plan.first_mission_item_index, -1);

		mission_route::RouteToGoalRequest route_request = makeRouteToGoalRequest(vehicle_position, 1);
		route_request.active_jump_anchor = anchor;
		mission_route::RouteToGoalPlan route_plan{};
		route_plan.join_position = vehicle_position;
		route_plan.first_mission_item_index = 1;
		route_plan.goal_type = mission_route::GoalType::kMissionTakeoff;
		route_plan.goal_position = vehicle_position;
		ASSERT_TRUE(route_plan.valid());

		const mission_route::FailureReason route_status = planner.planRouteToGoal(route_request, route_plan);

		EXPECT_EQ(route_status, mission_route::FailureReason::kInvalidRequest);
		EXPECT_FALSE(route_plan.valid());
		EXPECT_EQ(route_plan.goal_type, mission_route::GoalType::kNone);
	}
}

// Mission resumes finish the active loop iteration while RTL deliberately ignores pending repeats.
TEST_F(MissionRoutePlannerTest, MissionPreservesAndRtlClearsActiveLoopState)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),     // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),   // 1
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt), // 2
		makeDoJump(0, 3, 0),                                               // 3
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt), // 4
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 205.f, 100.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	mission_route::MissionResumeRequest mission_request = makeMissionResumeRequest(vehicle_position, 0);
	mission_request.active_jump_anchor = {2, 0};
	mission_route::RouteToGoalRequest rtl_request = makeRouteToGoalRequest(vehicle_position, 0);
	rtl_request.active_jump_anchor = {2, 0};
	mission_route::MissionResumePlan mission_plan{};
	mission_route::RouteToGoalPlan rtl_plan{};

	const mission_route::FailureReason mission_status = planner.planMissionResumeJoin(mission_request, mission_plan);
	const mission_route::FailureReason rtl_status = planner.planRouteToGoal(rtl_request, rtl_plan);

	ASSERT_EQ(mission_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(mission_status);
	ASSERT_EQ(rtl_status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(rtl_status);
	ASSERT_TRUE(mission_plan.active_jump_anchor.valid());
	EXPECT_EQ(mission_plan.active_jump_anchor.start_index, 2);
	EXPECT_EQ(mission_plan.active_jump_anchor.target_index, 0);
	ASSERT_TRUE(rtl_plan.active_jump_anchor.valid());
	EXPECT_EQ(rtl_plan.active_jump_anchor.start_index, 2);
	EXPECT_EQ(rtl_plan.active_jump_anchor.target_index, 0);

	// Mission completes the active [2->0] jump before continuing toward the route end.
	EXPECT_EQ(mission_plan.first_mission_item_index, 0);
	EXPECT_FALSE(mission_plan.direction_reversed);

	// RTL ignores pending repeats and takes the cheaper reverse path to the safe point on [1->2].
	EXPECT_EQ(rtl_plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(rtl_plan.first_mission_item_index, 2);
	EXPECT_TRUE(rtl_plan.direction_reversed);
	EXPECT_EQ(rtl_plan.branch_off_mission_item_index, 1);
}

// Loop case where the cheapest path is nominal: rally 3, branch-off on [7-9].
TEST_F(MissionRoutePlannerTest, LoopScenarioSelectsRally3OnSegment7To9)
{
	// A safe point lies on the active jump segment 7->2 while the vehicle is inside that loop.
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider = makeRouteProvider(items, safe_points);
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	request.active_jump_anchor = {7, 2};
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.active_jump_anchor.valid());
	EXPECT_EQ(plan.active_jump_anchor.start_index, 7);
	EXPECT_EQ(plan.active_jump_anchor.target_index, 2);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 3);
	EXPECT_TRUE(plan.goal_position.valid());
	EXPECT_TRUE(plan.branch_off_position.valid());
	EXPECT_EQ(plan.branch_off_mission_item_index, 9);
}

// Route following is skipped only when the vehicle is close to the selected branch leg
// in both cross-track and altitude.
TEST_F(MissionRoutePlannerTest, SelectedBranchLegShortcutRequiresCrosstrackAndAltitudeProximity)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt),
	};

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 10.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 0);
	request.direct_goal_acceptance_radius_m = 20.f;
	request.acceptance_radius_m = 20.f;
	request.velocity_north_m_s = 5.f;
	request.velocity_east_m_s = 0.f;
	mission_route::RouteToGoalPlan close_plan{};

	const mission_route::FailureReason close_status = planner.planRouteToGoal(request, close_plan);
	ASSERT_EQ(close_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(close_status);
	ASSERT_EQ(close_plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
			close_plan.goal_position.lat, close_plan.goal_position.lon)
		     < request.direct_goal_acceptance_radius_m);
	EXPECT_TRUE(close_plan.fly_direct_to_goal);

	// At the correct altitude but outside the cross-track tolerance, no skip.
	const mission_route::Position off_leg_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 10.f, kAlt);
	request.vehicle_position = off_leg_position;
	mission_route::RouteToGoalPlan off_leg_plan{};
	const mission_route::FailureReason off_leg_status = planner.planRouteToGoal(request, off_leg_plan);
	ASSERT_EQ(off_leg_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(off_leg_status);
	ASSERT_EQ(off_leg_plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(get_distance_to_next_waypoint(off_leg_position.lat, off_leg_position.lon,
			off_leg_plan.goal_position.lat, off_leg_plan.goal_position.lon)
		     < request.direct_goal_acceptance_radius_m);
	EXPECT_FALSE(off_leg_plan.fly_direct_to_goal);

	// On the branch leg but outside the altitude tolerance, no skip.
	mission_route::Position high_position = vehicle_position;
	high_position.alt += 5.f * request.altitude_acceptance_radius_m;
	request.vehicle_position = high_position;
	mission_route::RouteToGoalPlan high_plan{};
	const mission_route::FailureReason high_status = planner.planRouteToGoal(request, high_plan);
	ASSERT_EQ(high_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(high_status);
	ASSERT_EQ(high_plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(high_plan.fly_direct_to_goal);
}
