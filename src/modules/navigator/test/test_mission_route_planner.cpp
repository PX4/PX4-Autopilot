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

class MissionLoadCountingProvider : public VectorProvider
{
public:
	MissionLoadCountingProvider(const std::vector<mission_item_s> &mission_items,
				    const std::vector<mission_item_s> &safe_point_items)
	{
		setMissionItems(mission_items);
		setSafePointItems(safe_point_items);
	}

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		_mission_load_count++;
		return VectorProvider::loadMissionItem(index, mission_item);
	}

	int missionLoadCount() const { return _mission_load_count; }

private:
	mutable int _mission_load_count{0};
};

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

// Injects one mission read failure after a chosen safe-point record is loaded.
class OneShotSafePointScanFailureProvider : public mission_route::Provider
{
public:
	OneShotSafePointScanFailureProvider(const std::vector<mission_item_s> &mission_items,
					    const std::vector<mission_item_s> &safe_point_items,
					    int32_t failed_mission_index,
					    int32_t arm_at_safe_point_index = 0) :
		_failed_mission_index(failed_mission_index),
		_arm_at_safe_point_index(arm_at_safe_point_index)
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

		if (success && index >= _arm_at_safe_point_index) {
			_failure_armed = true;
		}

		return success;
	}

	bool failureInjected() const { return _failure_injected; }

private:
	VectorProvider _delegate{};
	int32_t _failed_mission_index{-1};
	int32_t _arm_at_safe_point_index{0};
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

// Vehicle on the first route segment, 10 m past the start (these missions run north at east 0).
static mission_route::Position vehicleOnFirstSegment()
{
	return makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
}

// Owns a vector-backed provider and a planner bound to it: the common one-line test setup.
class TestPlanner
{
public:
	explicit TestPlanner(const std::vector<mission_item_s> &mission_items,
			     const std::vector<mission_item_s> &safe_point_items = {}) :
		_provider(makeRouteProvider(mission_items, safe_point_items)),
		_planner(_provider)
	{}

	mission_route::FailureReason planRouteToGoal(const mission_route::RouteToGoalRequest &request,
			mission_route::RouteToGoalPlan &plan)
	{
		return _planner.planRouteToGoal(request, plan);
	}

	mission_route::FailureReason planMissionResumeJoin(const mission_route::MissionResumeRequest &request,
			mission_route::MissionResumePlan &plan)
	{
		return _planner.planMissionResumeJoin(request, plan);
	}

	VectorProvider &provider() { return _provider; }

private:
	VectorProvider _provider;
	MissionRoutePlanner _planner;
};

// ---- Mission resume join ----

// Mission resume follows the route in nominal direction toward the mission endpoint.
TEST_F(MissionRoutePlannerTest, PlansNominalMissionResumeJoinToNextItem)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};
	TestPlanner planner(mission);
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
	TestPlanner planner(mission);
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

// An exhausted selected jump keeps its identity without forcing another repeat.
TEST_F(MissionRoutePlannerTest, ExhaustedJumpKeepsIdentityWithoutForcingRepeat)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt),
		makeDoJump(0, 3, 3),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt),
	};

	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	mission_route::MissionResumeRequest request = makeMissionResumeRequest(vehicle_position, 0);
	request.active_jump_anchor = {3};
	mission_route::MissionResumePlan plan{};

	const mission_route::FailureReason status = planner.planMissionResumeJoin(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.active_jump_anchor.jump_item_index, 3);
	EXPECT_EQ(plan.first_mission_item_index, 2);
	EXPECT_FALSE(plan.direction_reversed);
}

// ---- Safe-point cost selection ----

// Cost includes the branch-off leg, so the near-route safe point beats the near-along one.
TEST_F(MissionRoutePlannerTest, SelectsSafePointWithLowestTotalCostIncludingBranchOffLeg)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 15.f, 80.f, kAlt), // near along-route distance, 80 m branch-off leg
		makeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 5.f, kAlt),  // near-route: 5 m branch-off leg, lowest total cost
	};

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
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
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 19.f, kAlt), // first in upload order, 19 m from the vehicle
		makeSafePointFromOffset(kBaseLat, kBaseLon, 12.f, 1.f, kAlt),  // lowest cost, ~2 m away: selected and flown direct
	};

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_TRUE(plan.fly_direct_to_goal);
	EXPECT_EQ(plan.safe_point_index, 1);
}

// A lower-cost safe point in a later scan batch must beat valid candidates from the first batch.
TEST_F(MissionRoutePlannerTest, LowerCostSafePointInLaterBatchWins)
{
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

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, best_index);
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

	TestPlanner planner(mission, safe_points);

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

// ---- VTOL approach requirements ----

// An approach block belongs to the rally right before it in the file. Rally 0 is closer to the
// vehicle but has no approach of its own, so with require_vtol_approach rally 1 must win.
TEST_F(MissionRoutePlannerTest, RequiredVtolApproachSelectsFartherEligibleSafePoint)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 15.f, kAlt), // closer rally, no approach
		makeSafePointFromOffset(kBaseLat, kBaseLon, 80.f, 20.f, kAlt), // farther rally, owns the approach below
		makeVtolApproachFromOffset(95.f, 20.f, kAlt + 20.f),
	};

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
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

	TestPlanner planner(mission, safe_points);
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

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	request.home_altitude_amsl = NAN;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
}

// Approach records do not consume rally-point slots in a projection batch.
TEST_F(MissionRoutePlannerTest, ApproachRecordsDoNotConsumeSafePointBatchCapacity)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points;
	safe_points.reserve(2 * static_cast<size_t>(kSafePointBatchSize));

	for (uint16_t i = 0; i < kSafePointBatchSize; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 20.f + i, 20.f, kAlt));
		safe_points.push_back(makeVtolApproachFromOffset(20.f + i, 30.f, kAlt + 20.f));
	}

	MissionLoadCountingProvider provider{mission, safe_points};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_LE(provider.missionLoadCount(), 4 * static_cast<int>(mission.size()));
}

// ---- LAND and mission endpoint fallback ----

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

	TestPlanner planner(mission);
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

// The route ends at the first LAND item, not at the last uploaded item.
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

	MissionLoadCountingProvider provider{mission, {}};
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
	EXPECT_EQ(provider.missionLoadCount(), 6); // the waypoint after LAND is never loaded
}

// LAND as the first position leaves no route segment to plan.
TEST_F(MissionRoutePlannerTest, MissionLandAsFirstPositionHasNoRouteSegment)
{
	std::vector<mission_item_s> mission{
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
	};
	const int32_t land_index = 1;

	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, land_index);
	request.mission_land_index = land_index;
	mission_route::RouteToGoalPlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	EXPECT_EQ(status, mission_route::FailureReason::kNoSegmentsFound);
	EXPECT_FALSE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

// Items after LAND cannot move a safe-point branch beyond the LAND endpoint.
TEST_F(MissionRoutePlannerTest, SafePointAfterMissionLandBranchesFromLandEndpoint)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 900.f, 10.f, kAlt),
	};
	const int32_t land_index = 2;

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.mission_land_index = land_index;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, land_index);
	EXPECT_NEAR(plan.branch_off_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.branch_off_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
}

// All safe points have an invalid frame and no mission endpoint is available as a fallback.
TEST_F(MissionRoutePlannerTest, FailsWhenAllSafePointsAreInvalidAndNoEndpointExists)
{
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

	TestPlanner planner(mission, invalid_safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

// ---- Robustness to bad data and read failures ----

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

// A valid first-batch winner survives a later batch scan failure.
TEST_F(MissionRoutePlannerTest, EarlierSafePointWinnerSurvivesLaterScanFailure)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points;
	safe_points.reserve(static_cast<size_t>(kSafePointBatchSize) + 1);
	safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 5.f, kAlt));

	for (uint16_t i = 1; i <= kSafePointBatchSize; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f + i, 500.f, kAlt));
	}

	OneShotSafePointScanFailureProvider provider{mission, safe_points, 1, kSafePointBatchSize};
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_TRUE(provider.failureInjected());
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
}

// A corrupted rally (NaN lat) is skipped; selection falls back to another valid rally.
TEST_F(MissionRoutePlannerTest, DefaultMissionInvalidRallyPointSkipped)
{
	auto safe_points = default_dataset::safePoints();
	TestPlanner planner(default_dataset::mission(), safe_points);

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 13);
	request.velocity_north_m_s = -15.f;
	request.velocity_east_m_s = 15.f;

	// Baseline: with valid data rally 2 is the winner.
	mission_route::RouteToGoalPlan baseline_plan{};
	const mission_route::FailureReason baseline_status = planner.planRouteToGoal(request, baseline_plan);
	ASSERT_EQ(baseline_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(baseline_status);
	ASSERT_EQ(baseline_plan.safe_point_index, 2);

	// Corrupt the winner: it must be skipped and the plan must stay finite.
	safe_points[2].lat = static_cast<double>(NAN);
	planner.provider().setSafePointItems(safe_points);
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_NE(plan.safe_point_index, 2);
	EXPECT_TRUE(std::isfinite(plan.branch_off_position.lat));
	EXPECT_TRUE(std::isfinite(plan.branch_off_position.lon));
}

// A relative-altitude safe point is converted to AMSL using home altitude.
TEST_F(MissionRoutePlannerTest, RelativeAltitudeSafePointUsesHomeAltitude)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 90.f, 15.f, 40.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	};

	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 1);
	request.home_altitude_amsl = 620.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_NEAR(plan.goal_position.alt, 660.f, kAltitudeTolerance); // 620 m home + 40 m relative
}

// ---- DO_JUMP handling ----

// Active jump identity must be empty or a valid mission item index.
TEST_F(MissionRoutePlannerTest, RejectsInvalidActiveJumpAnchorsAndClearsPlans)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);
	const mission_route::ActiveJumpAnchor invalid_anchors[] {{-2}, {3}};

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
		EXPECT_TRUE(mission_plan.active_jump_anchor.empty());

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
		EXPECT_EQ(route_plan.first_mission_item_index, -1);
		EXPECT_TRUE(route_plan.active_jump_anchor.empty());
		EXPECT_EQ(route_plan.goal_type, mission_route::GoalType::kNone);
	}
}

// A stored jump index is rejected if that mission command has changed.
TEST_F(MissionRoutePlannerTest, RejectsStaleJumpAnchorAfterCommandChanges)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt),
		makeDoJump(0, 3, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt),
	};
	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	mission_route::MissionResumeRequest mission_request = makeMissionResumeRequest(vehicle_position, 0);
	mission_request.active_jump_anchor = {3};
	mission_route::RouteToGoalRequest route_request = makeRouteToGoalRequest(vehicle_position, 0);
	route_request.active_jump_anchor = {3};

	mission[3].nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	planner.provider().setMissionItems(mission);

	mission_route::MissionResumePlan mission_plan{};
	mission_plan.join_position = vehicle_position;
	mission_plan.first_mission_item_index = 1;
	ASSERT_TRUE(mission_plan.valid());
	mission_route::RouteToGoalPlan route_plan{};
	route_plan.join_position = vehicle_position;
	route_plan.first_mission_item_index = 1;
	route_plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	route_plan.goal_position = vehicle_position;
	ASSERT_TRUE(route_plan.valid());

	const mission_route::FailureReason mission_status =
		planner.planMissionResumeJoin(mission_request, mission_plan);
	const mission_route::FailureReason route_status = planner.planRouteToGoal(route_request, route_plan);

	EXPECT_EQ(mission_status, mission_route::FailureReason::kInvalidRequest);
	EXPECT_FALSE(mission_plan.valid());
	EXPECT_EQ(mission_plan.first_mission_item_index, -1);
	EXPECT_TRUE(mission_plan.active_jump_anchor.empty());
	EXPECT_EQ(route_status, mission_route::FailureReason::kInvalidRequest);
	EXPECT_FALSE(route_plan.valid());
	EXPECT_EQ(route_plan.first_mission_item_index, -1);
	EXPECT_TRUE(route_plan.active_jump_anchor.empty());
	EXPECT_EQ(route_plan.goal_type, mission_route::GoalType::kNone);
}

// Mission honors active repeats while RTL ignores their path.
TEST_F(MissionRoutePlannerTest, MissionHonorsAndRtlIgnoresActiveJumpRepeats)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),     // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),   // 1
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt), // 2
		makeDoJump(0, 3, 0),                                                // 3
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt), // 4
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 205.f, 100.f, kAlt),
	};
	TestPlanner planner(mission, safe_points);

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	mission_route::MissionResumeRequest mission_request = makeMissionResumeRequest(vehicle_position, 0);
	mission_request.active_jump_anchor = {3};
	mission_route::RouteToGoalRequest rtl_request = makeRouteToGoalRequest(vehicle_position, 0);
	rtl_request.active_jump_anchor = {3};
	mission_route::MissionResumePlan mission_plan{};
	mission_route::RouteToGoalPlan rtl_plan{};

	const mission_route::FailureReason mission_status = planner.planMissionResumeJoin(mission_request, mission_plan);
	const mission_route::FailureReason rtl_status = planner.planRouteToGoal(rtl_request, rtl_plan);

	ASSERT_EQ(mission_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(mission_status);
	ASSERT_EQ(rtl_status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(rtl_status);
	ASSERT_TRUE(mission_plan.active_jump_anchor.valid());
	EXPECT_EQ(mission_plan.active_jump_anchor.jump_item_index, 3);
	ASSERT_TRUE(rtl_plan.active_jump_anchor.valid());
	EXPECT_EQ(rtl_plan.active_jump_anchor.jump_item_index, 3);

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
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	request.active_jump_anchor = {8};
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.active_jump_anchor.valid());
	EXPECT_EQ(plan.active_jump_anchor.jump_item_index, 8);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 3);
	EXPECT_TRUE(plan.goal_position.valid());
	EXPECT_TRUE(plan.branch_off_position.valid());
	EXPECT_EQ(plan.branch_off_mission_item_index, 9);
}

// ---- Realistic dataset scenarios ----

// MC picks the closest rally even when it is behind the vehicle (rally 1, reversed).
TEST_F(MissionRoutePlannerTest, DefaultMissionClosestBehindReverseMC)
{
	TestPlanner planner(default_dataset::mission(), default_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_TRUE(plan.direction_reversed);
}

// MC picks the closest rally when it is ahead (rally 0).
TEST_F(MissionRoutePlannerTest, DefaultMissionClosestForwardAheadMC)
{
	TestPlanner planner(default_dataset::mission(), default_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 5);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = -15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_FALSE(plan.direction_reversed);
}

// All rallies are behind, so MC picks the closest one in reverse (rally 0).
TEST_F(MissionRoutePlannerTest, DefaultMissionAllBehindMC)
{
	TestPlanner planner(default_dataset::mission(), default_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 15);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_TRUE(plan.direction_reversed);
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
	TestPlanner planner(uturn_penalty_dataset::mission(), uturn_penalty_dataset::safePoints());

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

INSTANTIATE_TEST_SUITE_P(UturnPenalty, MissionRoutePlannerUturnPenaltyTest, ::testing::Values(
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
}), [](const ::testing::TestParamInfo<UturnPenaltyCase> &param_info)
{
	return param_info.param.name;
});

// On the corner mission, MC picks the closest rally in reverse (rally 1).
TEST_F(MissionRoutePlannerTest, CornerMissionRallyOnCornerMC)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_TRUE(plan.direction_reversed);
}

// FW picks the route-nearer rally 0 and branches off on [4-5].
TEST_F(MissionRoutePlannerTest, CornerMissionCornerProjectionHandledFW)
{
	// Same vehicle position as CornerMissionRallyOnCornerMC, FW instead of MC.
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 2);
	request.is_fixed_wing = true;
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

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
	// Vehicle near a transition boundary.
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 3);
}

// Small end-of-mission segments must not hide valid rally points (rally 5).
TEST_F(MissionRoutePlannerTest, CornerMissionSmallSegmentFrontMC)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 13);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 5);
}

// Reverse-flight corner case: rally 2, branch-off on [5-7].
TEST_F(MissionRoutePlannerTest, CornerMissionReverseCornerScenarioSelectsRally2OnSegment5To7)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 5);
	request.current_route_direction_reversed = true;
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 2);
	EXPECT_EQ(plan.branch_off_mission_item_index, 7);
	EXPECT_TRUE(plan.branch_off_position.valid());
}

// A rally can branch from a stacked zero-length LAND segment.
TEST_F(MissionRoutePlannerTest, CornerMissionLandCornerScenarioSelectsRally6OnSegment14To15)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());
	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10368934085859, 2.3183612137416754, 510.f);
	mission_route::RouteToGoalRequest request = makeRouteToGoalRequest(vehicle_position, 13);
	request.is_fixed_wing = true;
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	mission_route::RouteToGoalPlan plan{};

	const mission_route::FailureReason status = planner.planRouteToGoal(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 6);
	EXPECT_EQ(plan.branch_off_mission_item_index, 15);
	EXPECT_TRUE(plan.branch_off_position.valid());
}
