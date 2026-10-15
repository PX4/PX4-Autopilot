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
#include "mission_route_projection.h"
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

static mission_route::RtlRouteRequest makeRtlRouteRequest(const mission_route::Position &vehicle_position,
		int32_t mission_index)
{
	mission_route::RtlRouteRequest request{};
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

	mission_route::FailureReason planRtlRoute(const mission_route::RtlRouteRequest &request,
			mission_route::RtlRoutePlan &plan)
	{
		return _planner.planRtlRoute(request, plan);
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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 1);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 0);
	request.direct_goal_acceptance_radius_m = 20.f;
	request.acceptance_radius_m = 20.f;
	request.velocity_north_m_s = 5.f;
	request.velocity_east_m_s = 0.f;
	mission_route::RtlRoutePlan close_plan{};

	const mission_route::FailureReason close_status = planner.planRtlRoute(request, close_plan);
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
	mission_route::RtlRoutePlan off_leg_plan{};
	const mission_route::FailureReason off_leg_status = planner.planRtlRoute(request, off_leg_plan);
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
	mission_route::RtlRoutePlan high_plan{};
	const mission_route::FailureReason high_status = planner.planRtlRoute(request, high_plan);
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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 1);
}

// ---- LAND and mission endpoint fallback ----

// Returning to TAKEOFF uses home altitude, never the takeoff item's climb altitude.
TEST_F(MissionRoutePlannerTest, MissionTakeoffFallbackRequiresHomeAltitude)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	TestPlanner planner(mission);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	request.home_altitude_amsl = kAlt - 100.f;
	mission_route::RtlRoutePlan plan{};

	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);
	EXPECT_FLOAT_EQ(plan.goal_position.alt, request.home_altitude_amsl);

	request.home_altitude_amsl = NAN;
	EXPECT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNoValidSafePoints);
	EXPECT_FALSE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

// RTL during the descent onto a stacked LAND with the mission landing as goal: a small drift toward
// the approach leg must not turn the join into a climb. LAND stays the target at the current altitude.
TEST_F(MissionRoutePlannerTest, RtlDuringStackedLandingKeepsCurrentAltitude)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 98.f, 0.5f, kAlt + 10.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 3);
	request.mission_land_index = 3;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_EQ(plan.first_mission_item_index, 3);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_TRUE(plan.use_current_altitude);
	EXPECT_TRUE(plan.fly_direct_to_goal);
	EXPECT_FLOAT_EQ(plan.join_position.alt, vehicle_position.alt);
}

// RTL during the climb above TAKEOFF with the takeoff location as goal: the vehicle returns at its
// current altitude instead of first climbing to the top of the stack.
TEST_F(MissionRoutePlannerTest, RtlDuringStackedTakeoffClimbKeepsCurrentAltitude)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
	};
	TestPlanner planner(mission);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 2.f, 0.5f, kAlt + 20.f);
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);
	EXPECT_EQ(plan.first_mission_item_index, 0);
	EXPECT_TRUE(plan.direction_reversed);
	EXPECT_TRUE(plan.use_current_altitude);
	EXPECT_TRUE(plan.fly_direct_to_goal);
	EXPECT_FLOAT_EQ(plan.join_position.alt, vehicle_position.alt);
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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

// ---- Robustness to bad data ----

// A corrupted rally (NaN lat) is skipped; selection falls back to another valid rally.
TEST_F(MissionRoutePlannerTest, DefaultMissionInvalidRallyPointSkipped)
{
	auto safe_points = default_dataset::safePoints();
	TestPlanner planner(default_dataset::mission(), safe_points);

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.11057010025454, 2.2972410253925846, 461.4f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 13);
	request.velocity_north_m_s = -15.f;
	request.velocity_east_m_s = 15.f;

	// Baseline: with valid data rally 2 is the winner.
	mission_route::RtlRoutePlan baseline_plan{};
	const mission_route::FailureReason baseline_status = planner.planRtlRoute(request, baseline_plan);
	ASSERT_EQ(baseline_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(baseline_status);
	ASSERT_EQ(baseline_plan.safe_point_index, 2);

	// Corrupt the winner: it must be skipped and the plan must stay finite.
	safe_points[2].lat = static_cast<double>(NAN);
	planner.provider().setSafePointItems(safe_points);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.home_altitude_amsl = 620.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_NEAR(plan.goal_position.alt, 660.f, kAltitudeTolerance); // 620 m home + 40 m relative
}

// ---- DO_JUMP handling ----

// Mission completes the active repeat; RTL chooses the shorter exit from the same jump.
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
	mission_route::RtlRouteRequest rtl_request = makeRtlRouteRequest(vehicle_position, 0);
	rtl_request.active_jump_anchor = {3};
	mission_route::MissionResumePlan mission_plan{};
	mission_route::RtlRoutePlan rtl_plan{};

	const mission_route::FailureReason mission_status = planner.planMissionResumeJoin(mission_request, mission_plan);
	const mission_route::FailureReason rtl_status = planner.planRtlRoute(rtl_request, rtl_plan);

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

// The selected jump remains a usable branch-off when both vehicle and safe point project onto it.
TEST_F(MissionRoutePlannerTest, ActiveJumpRemainsAvailableForSafePointProjection)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 1000.f, kAlt),
		makeDoJump(0, 3, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 1000.f, kAlt),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 300.f, kAlt),
	};
	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 0);
	request.active_jump_anchor = {3};
	request.safe_point_projection_search_distance_m = 0.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.active_jump_anchor.jump_item_index, 3);
	EXPECT_TRUE(plan.direction_reversed);
	EXPECT_EQ(plan.first_mission_item_index, 2);
	EXPECT_EQ(plan.branch_off_mission_item_index, 2);
}

// ---- U-turn penalty ----

enum class UturnVehicle { Multicopter, FixedWing };

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
	mission_route::RtlRouteRequest request =
		makeRtlRouteRequest(vehicle_position, uturn_penalty_dataset::kMissionIndex);
	request.is_fixed_wing = scenario.vehicle == UturnVehicle::FixedWing;
	request.velocity_north_m_s = scenario.velocity_north;
	request.velocity_east_m_s = scenario.velocity_east;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
// MC has no penalty, so it keeps the closest (reverse) rally A.
UturnPenaltyCase{
	"MulticopterNoPenaltySelectsClosestReverse",
	UturnVehicle::Multicopter, 15.f, 0.f, 0, true
},
// Velocity orthogonal to the route is not a u-turn, so no penalty is applied.
UturnPenaltyCase{
	"FixedWingOrthogonalVelocityHasNoUturn",
	UturnVehicle::FixedWing, 0.f, 15.f, 0, true
}), [](const ::testing::TestParamInfo<UturnPenaltyCase> &param_info)
{
	return param_info.param.name;
});

// ---- VTOL segment state and transition actions ----

static constexpr uint8_t kVtolUndefined = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;
static constexpr uint8_t kVtolMc = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
static constexpr uint8_t kVtolFw = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

static mission_route::Segment makeRouteSegment(int32_t start_idx, int32_t end_idx)
{
	mission_route::Segment segment{};
	segment.start.idx = start_idx;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = end_idx;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	return segment;
}

// Mission with a front transition before wp 3 and a back transition before wp 5.
static std::vector<mission_item_s> missionWithTransitions()
{
	return {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(kVtolFw),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeVtolTransitionItem(kVtolMc),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
}

// The segment state comes from the last DO_VTOL_TRANSITION before it, else the upload state.
TEST_F(MissionRoutePlannerTest, VtolStateForSegmentUsesLastTransitionBeforeSegment)
{
	VectorProvider provider = makeRouteProvider(missionWithTransitions(), {});

	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(0, 1), kVtolMc), kVtolMc);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(0, 1), kVtolUndefined), kVtolUndefined);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(1, 3), kVtolMc), kVtolFw);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(3, 5), kVtolFw), kVtolMc);
}

// U-turn penalty mission (straight north, rallies behind and ahead), with an optional
// DO_VTOL_TRANSITION inserted at index 1 so every segment past the takeoff shares its state.
static std::vector<mission_item_s> uturnMissionWithTransition(uint8_t transition_target)
{
	std::vector<mission_item_s> mission = uturn_penalty_dataset::mission();

	if (transition_target != kVtolUndefined) {
		mission.insert(mission.begin() + 1, makeVtolTransitionItem(transition_target));
	}

	return mission;
}

// MC segments and MC vehicles never take the u-turn penalty: the vehicle is, or will be, in MC
// for the turn. A back transition reports as MC (Commander sets the rotary-wing type), so the MC
// rows cover it. FW keeps the penalty only toward FW or unknown segments. Plans also expose the
// transition missed at branch-in.
TEST_F(MissionRoutePlannerTest, VtolUturnPenaltyAcrossTransitionStates)
{
	struct VtolUturnCase {
		const char *name;
		bool is_vtol;
		bool is_fixed_wing;
		bool in_transition_to_fw;
		uint8_t transition_target; // kVtolUndefined = no transition item in the mission
		uint8_t upload_state;
		int expected_safe_point_index;
		bool expected_reversed;
		mission_route::VtolTransitionAction expected_action;
	};

	using Action = mission_route::VtolTransitionAction;

	const VtolUturnCase cases[] = {
		// FW on an MC segment: no penalty (a u-turn would BT + FT, then re-BT at the segment).
		{"FwOnMcSegmentSkipsPenalty", true, true, false, kVtolMc, kVtolUndefined, 0, true, Action::kBackTransition},
		// MC toward a FW segment: still no penalty, the front transition comes at branch-in.
		{"McOnFwSegmentRequestsFrontTransition", true, false, false, kVtolFw, kVtolUndefined, 0, true, Action::kFrontTransition},
		// FW on a FW segment: penalty applies, nothing to transition.
		{"FwOnFwSegmentKeepsPenalty", true, true, false, kVtolFw, kVtolUndefined, 1, false, Action::kNone},
		// Unknown segment state never suppresses the penalty or requests a transition.
		{"FwWithUnknownSegmentStateKeepsPenalty", true, true, false, kVtolUndefined, kVtolUndefined, 1, false, Action::kNone},
	};

	for (const VtolUturnCase &test_case : cases) {
		SCOPED_TRACE(test_case.name);
		TestPlanner planner(uturnMissionWithTransition(test_case.transition_target),
				    uturn_penalty_dataset::safePoints());

		// The inserted transition item shifts the targeted waypoint by one.
		const int32_t mission_index = (test_case.transition_target == kVtolUndefined) ? 1 : 2;
		mission_route::RtlRouteRequest request =
			makeRtlRouteRequest(uturn_penalty_dataset::vehiclePosition(), mission_index);
		request.is_vtol = test_case.is_vtol;
		request.is_fixed_wing = test_case.is_fixed_wing;
		request.in_transition_to_fw = test_case.in_transition_to_fw;
		request.vtol_state_on_mission_upload = test_case.upload_state;
		request.velocity_north_m_s = 15.f;
		request.velocity_east_m_s = 0.f;
		mission_route::RtlRoutePlan plan{};

		const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

		ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
		EXPECT_EQ(plan.safe_point_index, test_case.expected_safe_point_index);
		EXPECT_EQ(plan.direction_reversed, test_case.expected_reversed);
		EXPECT_EQ(plan.vtol_transition_action, test_case.expected_action);
	}
}

// ---- Realistic dataset scenarios ----

// FW picks the route-nearer rally 0 and branches off on [4-5].
TEST_F(MissionRoutePlannerTest, CornerMissionCornerProjectionHandledFW)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);
	request.is_fixed_wing = true;
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, 5);
	EXPECT_TRUE(plan.branch_off_position.valid());
	EXPECT_FALSE(plan.direction_reversed);
}

// A rally can branch from a stacked zero-length LAND segment.
TEST_F(MissionRoutePlannerTest, CornerMissionLandCornerScenarioSelectsRally6OnSegment14To15)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());
	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10368934085859, 2.3183612137416754, 510.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 13);
	request.is_fixed_wing = true;
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 6);
	EXPECT_EQ(plan.branch_off_mission_item_index, 15);
	EXPECT_TRUE(plan.branch_off_position.valid());
}
