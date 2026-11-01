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
#include "mission_route_path_checker.h"
#include "mission_route_projection.h"
#include "support/mission_route_test_helpers.h"
#include "test_mission_route_data.h"

#include <cinttypes>
#include <cstdio>
#include <functional>

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

static constexpr uint16_t kSafePointBatchSize{CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE};

class MissionRoutePlannerTest : public ::testing::Test {};

class RecordingPathChecker : public mission_route::PathChecker
{
public:
	std::function<bool(const mission_route::ProjectionReference &, const mission_route::RouteProjectionCandidate &)>
	candidate_clear = [](const auto &, const auto &) { return true; };
	std::function<bool(const mission_route::ProjectionReferenceBatch &)>
	batch_available = [](const auto &) { return true; };
	bool path_available{true};
	bool path_clear{true};
	mutable std::vector<unsigned> batch_sizes;
	mutable unsigned path_checks{0};
	mutable mission_route::Position checked_start{};
	mutable mission_route::Position checked_end{};

	bool pruneCandidates(mission_route::ProjectionReferenceBatch &batch) const override
	{
		batch_sizes.push_back(batch.count);

		if (!batch_available(batch)) {
			return false;
		}

		for (uint8_t i = 0; i < batch.count; ++i) {
			auto &reference = batch.items[i];
			auto &buffer = reference.candidate_buffer;
			uint8_t retained = 0;

			for (uint8_t j = 0; j < buffer.count; ++j) {
				if (candidate_clear(reference, buffer.candidates[j])) {
					buffer.candidates[retained++] = buffer.candidates[j];
				}
			}

			buffer.count = retained;
		}

		return true;
	}

	bool checkPath(const mission_route::Position &start, const mission_route::Position &end, bool &clear) const override
	{
		++path_checks;
		checked_start = start;
		checked_end = end;
		clear = path_available && path_clear;
		return path_available;
	}
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

TEST_F(MissionRoutePlannerTest, GeofencePrunesVehicleCandidatesBeforeContinuitySelection)
{
	// Two nearby parallel legs: each remains usable when the preferred leg is blocked.
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 1000.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 1000.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission, {});
	const auto vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 500.f, kAlt);

	for (int32_t blocked_end : {1, 3}) {
		SCOPED_TRACE(blocked_end);
		auto request = makeMissionResumeRequest(vehicle, blocked_end);
		request.projection_search_distance_m = 150.f;
		mission_route::MissionResumePlan baseline{};
		ASSERT_EQ(MissionRoutePlanner{provider}.planMissionResumeJoin(request, baseline), mission_route::FailureReason::kNone);
		ASSERT_EQ(baseline.first_mission_item_index, blocked_end);

		RecordingPathChecker checker;
		checker.candidate_clear = [blocked_end](const auto &, const auto & candidate) {
			return candidate.segment.end.idx != blocked_end;
		};
		MissionRoutePlanner planner{provider, &checker};
		mission_route::MissionResumePlan plan{};
		ASSERT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
		ASSERT_TRUE(plan.valid());
		EXPECT_EQ(plan.first_mission_item_index, blocked_end == 1 ? 3 : 1);
		EXPECT_EQ(checker.batch_sizes, std::vector<unsigned>({1}));
	}
}

// The prune removes every vehicle candidate: no route, and no second walk of the route.
TEST_F(MissionRoutePlannerTest, GeofenceRejectsAllVehicleCandidates)
{
	const std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};
	VectorProvider baseline_provider = makeRouteProvider(mission);
	const auto request = makeMissionResumeRequest(vehicleOnFirstSegment(), 1);
	mission_route::MissionResumePlan plan{};
	ASSERT_EQ(MissionRoutePlanner{baseline_provider}.planMissionResumeJoin(request, plan),
		  mission_route::FailureReason::kNone);

	RecordingPathChecker checker;
	checker.candidate_clear = [](const auto &, const auto &) { return false; };
	VectorProvider provider = makeRouteProvider(mission);
	MissionRoutePlanner planner{provider, &checker};
	EXPECT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_FALSE(plan.valid());
	EXPECT_LE(provider.missionLoadCount(), baseline_provider.missionLoadCount());

	auto rtl_request = makeRtlRouteRequest(request.vehicle_position, 1);
	mission_route::RtlRoutePlan rtl_plan{};
	EXPECT_EQ(planner.planRtlRoute(rtl_request, rtl_plan), mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_FALSE(rtl_plan.valid());
}

TEST_F(MissionRoutePlannerTest, UnavailableGeofenceRejectsVehicleJoinForMissionAndRtl)
{
	const std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission, {});
	RecordingPathChecker checker;
	checker.batch_available = [](const auto &) { return false; };
	MissionRoutePlanner planner{provider, &checker};
	mission_route::MissionResumePlan mission_plan{};
	EXPECT_EQ(planner.planMissionResumeJoin(makeMissionResumeRequest(vehicleOnFirstSegment(), 1), mission_plan),
		  mission_route::FailureReason::kGeofenceUnavailable);
	EXPECT_FALSE(mission_plan.valid());

	auto request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	mission_route::RtlRoutePlan rtl_plan{};
	EXPECT_EQ(planner.planRtlRoute(request, rtl_plan), mission_route::FailureReason::kGeofenceUnavailable);
	EXPECT_FALSE(rtl_plan.valid());
}

// Blocked safe points in the first batch lose to a clear one in the next batch, with one walk per batch.
TEST_F(MissionRoutePlannerTest, GeofencePrunesCheaperSafePointsAcrossBatches)
{
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points;

	for (uint16_t i = 0; i < kSafePointBatchSize; ++i) {
		safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 110.f, 5.f, kAlt));
	}

	safe_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt));
	const auto vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const auto request = makeRtlRouteRequest(vehicle, 1);
	VectorProvider baseline_provider = makeRouteProvider(mission, safe_points);
	mission_route::RtlRoutePlan baseline{};
	ASSERT_EQ(MissionRoutePlanner{baseline_provider}.planRtlRoute(request, baseline), mission_route::FailureReason::kNone);
	ASSERT_LT(baseline.safe_point_index, kSafePointBatchSize);

	RecordingPathChecker checker;
	checker.candidate_clear = [](const auto & reference, const auto &) {
		return reference.source_index < 0 || reference.source_index >= kSafePointBatchSize;
	};
	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, kSafePointBatchSize);
	EXPECT_EQ(checker.batch_sizes, std::vector<unsigned>({1, kSafePointBatchSize, 1}));
	EXPECT_LE(provider.missionLoadCount(), baseline_provider.missionLoadCount());
}

TEST_F(MissionRoutePlannerTest, GeofenceBlockedSafePointsFallBackToMissionLand)
{
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission,
	{makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt)});
	RecordingPathChecker checker;
	checker.candidate_clear = [](const auto & reference, const auto &) { return reference.source_index < 0; };
	MissionRoutePlanner planner{provider, &checker};
	auto request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_EQ(plan.safe_point_index, -1);
}

TEST_F(MissionRoutePlannerTest, UnavailableLaterGeofenceBatchDiscardsEarlierWinnerAndEndpointFallback)
{
	const std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};
	const std::vector<mission_item_s> safe_points(static_cast<size_t>(kSafePointBatchSize) + 1,
			makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt));
	VectorProvider provider = makeRouteProvider(mission, safe_points);
	RecordingPathChecker checker;
	checker.batch_available = [](const auto & batch) {
		return batch.items[0].source_index < kSafePointBatchSize;
	};
	MissionRoutePlanner planner{provider, &checker};
	auto request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(MissionRoutePlanner{provider}.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kGeofenceUnavailable);
	EXPECT_FALSE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
	EXPECT_EQ(checker.batch_sizes, std::vector<unsigned>({1, kSafePointBatchSize, 1}));
}

TEST_F(MissionRoutePlannerTest, GeofenceChecksDirectShortcutFromVehicleToSelectedGoal)
{
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission,
	{makeSafePointFromOffset(kBaseLat, kBaseLon, 12.f, 1.f, kAlt)});
	RecordingPathChecker checker;
	MissionRoutePlanner planner{provider, &checker};
	const auto request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.fly_direct_to_goal);
	ASSERT_EQ(checker.path_checks, 1u);
	EXPECT_DOUBLE_EQ(checker.checked_start.lat, request.vehicle_position.lat);
	EXPECT_DOUBLE_EQ(checker.checked_start.lon, request.vehicle_position.lon);
	EXPECT_FLOAT_EQ(checker.checked_start.alt, request.vehicle_position.alt);
	EXPECT_DOUBLE_EQ(checker.checked_end.lat, plan.goal_position.lat);
	EXPECT_DOUBLE_EQ(checker.checked_end.lon, plan.goal_position.lon);
	EXPECT_FLOAT_EQ(checker.checked_end.alt, plan.goal_position.alt);

	checker.path_clear = false;
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	EXPECT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.fly_direct_to_goal);

	checker.path_available = false;
	EXPECT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kGeofenceUnavailable);
	EXPECT_FALSE(plan.valid());
}

// Near LAND the join keeps the live altitude and flies to the landing item. That segment gets its own fence check,
// ending above the item at the landing leg's altitude.
TEST_F(MissionRoutePlannerTest, GeofenceChecksMissionLandingShortcutAtCurrentAltitude)
{
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	VectorProvider provider = makeRouteProvider(mission, {});
	const auto vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 95.f, 0.f, kAlt + 75.f);
	const auto request = makeMissionResumeRequest(vehicle, 1);
	mission_route::MissionResumePlan plan{};

	// Without a fence checker the join is the same, with nothing to check.
	ASSERT_EQ(MissionRoutePlanner{provider}.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
	EXPECT_TRUE(plan.use_current_altitude);

	RecordingPathChecker checker;
	MissionRoutePlanner planner{provider, &checker};
	ASSERT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.use_current_altitude);
	EXPECT_EQ(plan.first_mission_item_index, 1);
	EXPECT_FLOAT_EQ(plan.join_position.alt, vehicle.alt);
	ASSERT_EQ(checker.path_checks, 1u);
	EXPECT_FLOAT_EQ(checker.checked_start.alt, vehicle.alt);
	EXPECT_FLOAT_EQ(checker.checked_end.alt, kAlt); // the landing leg's altitude
	EXPECT_DOUBLE_EQ(checker.checked_end.lat, mission[1].lat);
	EXPECT_DOUBLE_EQ(checker.checked_end.lon, mission[1].lon);

	checker.path_clear = false;
	EXPECT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNoValidPath);
	EXPECT_FALSE(plan.valid());

	checker.path_available = false;
	EXPECT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kGeofenceUnavailable);
	EXPECT_FALSE(plan.valid());
}

TEST_F(MissionRoutePlannerTest, BlockedDirectLandingCannotFallBackToAnAlreadyReachedRouteJoin)
{
	const std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	VectorProvider provider = makeRouteProvider(mission, {});
	const auto vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 95.f, 0.f, kAlt + 75.f);
	auto request = makeRtlRouteRequest(vehicle, 1);
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(MissionRoutePlanner{provider}.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.fly_direct_to_goal);
	ASSERT_TRUE(plan.use_current_altitude);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);

	// Model an altitude restriction: joining horizontally at current altitude is clear,
	// but descending to the selected endpoint is blocked. The nearby join cannot detour it.
	class NoDescentPathChecker : public RecordingPathChecker
	{
	public:
		bool checkPath(const mission_route::Position &start, const mission_route::Position &end, bool &clear) const override
		{
			const bool available = RecordingPathChecker::checkPath(start, end, clear);
			clear = clear && end.alt >= start.alt;
			return available;
		}
	} checker;
	MissionRoutePlanner planner{provider, &checker};
	EXPECT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNoValidPath);
	EXPECT_FALSE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kNone);
}

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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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

// The follower flies the branch-off leg level and descends at the rally point, so the shortcut
// expects the branch-off altitude anywhere on the leg.
TEST_F(MissionRoutePlannerTest, BranchLegShortcutUsesBranchOffAltitude)
{
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt - 50.f),
	};

	TestPlanner planner(mission, safe_points);

	// Halfway along the leg at the branch-off altitude.
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(
				makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 25.f, kAlt), 0);
	request.direct_goal_acceptance_radius_m = 20.f;
	request.acceptance_radius_m = 20.f;
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_TRUE(plan.fly_direct_to_goal);

	// Halfway down a straight line to the rally point is not on the flown leg.
	request.vehicle_position.alt = kAlt - 25.f;
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.fly_direct_to_goal);
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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	request.home_altitude_amsl = NAN;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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

	VectorProvider provider = makeRouteProvider(mission, safe_points);
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position = vehicleOnFirstSegment();
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.require_vtol_approach = true;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_LE(provider.missionLoadCount(), 4 * static_cast<int>(mission.size()));
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

// Without home altitude, an absolute-altitude LAND still works even when TAKEOFF is closer.
TEST_F(MissionRoutePlannerTest, MissingHomeAltitudeKeepsAbsoluteMissionLandFallback)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 100.f),
	};
	TestPlanner planner(mission);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	request.home_altitude_amsl = NAN;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_FLOAT_EQ(plan.goal_position.alt, mission[2].altitude);
}

// Missing home altitude does not reject a usable rally point with an absolute altitude.
TEST_F(MissionRoutePlannerTest, MissingHomeAltitudeKeepsAbsoluteSafePointGoal)
{
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 50.f, kAlt - 100.f),
	};
	TestPlanner planner(mission, safe_points);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicleOnFirstSegment(), 1);
	request.home_altitude_amsl = NAN;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_FLOAT_EQ(plan.goal_position.alt, safe_points[0].altitude);
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

	VectorProvider provider = makeRouteProvider(mission);
	provider.setMissionLoadFailures({4}); // reading the waypoint after LAND would fail the plan
	MissionRoutePlanner planner{provider};
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_NEAR(plan.goal_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.goal_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
	// One route scan, the branch-in segment's two items and the two endpoint lookups.
	EXPECT_LE(provider.missionLoadCount(), 8);
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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, land_index);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidWaypoints);
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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, land_index);
	EXPECT_NEAR(plan.branch_off_position.lat, mission[land_index].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.branch_off_position.lon, mission[land_index].lon, kLatLonToleranceDeg);
}

// Waypoints above LAND share its latitude/longitude. They must not create an extra branch-off
// at LAND when the rally's closest point on the incoming leg is inside that leg.
TEST_F(MissionRoutePlannerTest, StackedLandKeepsInteriorSafePointBranchOff)
{
	// Top view, north increases to the right (metres). B, B' and LAND differ only in altitude.
	// A(0) -------- Q(50) -------- B/B'/LAND(100) -- V(105)
	//               |
	//               R (10 m east)
	// Expected return: V -> landing location -> Q -> R.
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt),
	};
	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 105.f, 0.f, kAlt + 75.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);
	request.current_route_direction_reversed = true;
	request.safe_point_projection_search_distance_m = 50.f; // Would admit the ~51 m endpoint candidate.
	request.fw_u_turn_penalty_m = 0.f;
	// Disable route-skip shortcuts to test branch-off selection itself.
	request.acceptance_radius_m = 0.f;
	request.direct_goal_acceptance_radius_m = 0.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_FALSE(plan.fly_direct_to_goal);
	EXPECT_TRUE(plan.direction_reversed);
	// In reverse, Q replaces the target A (index 0) on the A -> B segment.
	EXPECT_EQ(plan.branch_off_mission_item_index, 0);
	EXPECT_NEAR(plan.branch_off_position.lat, safe_points[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.branch_off_position.lon, mission[0].lon, kLatLonToleranceDeg);
}

// Takeoff and its next two waypoints share latitude/longitude. The rally must project onto
// the outgoing horizontal leg, without an extra branch-off at the takeoff location.
TEST_F(MissionRoutePlannerTest, StackedTakeoffKeepsInteriorSafePointBranchOff)
{
	// Top view, north increases to the right (metres). T/T1/T2 are the vertical climb.
	// V(-5) -- T/T1/T2(0) -------- Q(50) -------- C(100)
	//                              |
	//                              R (10 m east)
	// Expected return: V -> takeoff location at T2's altitude -> Q -> R.
	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt + 75.f),
	};
	TestPlanner planner(mission, safe_points);
	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, -5.f, 0.f, kAlt + 75.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.safe_point_projection_search_distance_m = 50.f; // Would admit the ~51 m takeoff candidate.
	request.fw_u_turn_penalty_m = 0.f;
	// Disable route-skip shortcuts to test branch-off selection itself.
	request.acceptance_radius_m = 0.f;
	request.direct_goal_acceptance_radius_m = 0.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_FALSE(plan.fly_direct_to_goal);
	EXPECT_FALSE(plan.direction_reversed);
	// In nominal direction, Q replaces the target C (index 3) on the T2 -> C segment.
	EXPECT_EQ(plan.branch_off_mission_item_index, 3);
	EXPECT_NEAR(plan.branch_off_position.lat, safe_points[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.branch_off_position.lon, mission[0].lon, kLatLonToleranceDeg);
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

// While TAKEOFF is the current item no segment contains its index, since nothing precedes it.
// Inward drift must still return to TAKEOFF at the current altitude instead of climbing first.
TEST_F(MissionRoutePlannerTest, RtlDuringInitialStackedTakeoffKeepsCurrentAltitude)
{
	for (const bool preceding_command : {false, true}) {
		std::vector<mission_item_s> mission{
			makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 25.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		};

		if (preceding_command) {
			mission_item_s speed_change{};
			speed_change.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
			mission.insert(mission.begin(), speed_change);
		}

		const int32_t takeoff_index = preceding_command ? 1 : 0;
		TestPlanner planner(mission);

		// Cover drift directly along the route as well as slightly off to one side.
		for (const float east_m : {0.f, 0.5f}) {
			SCOPED_TRACE(::testing::Message() << "TAKEOFF index=" << takeoff_index << ", east=" << east_m);
			const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 2.f, east_m, kAlt + 10.f);
			const auto request = makeRtlRouteRequest(vehicle_position, takeoff_index);
			mission_route::RtlRoutePlan plan{};
			ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
			ASSERT_TRUE(plan.valid());
			EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);
			EXPECT_EQ(plan.first_mission_item_index, takeoff_index);
			EXPECT_TRUE(plan.direction_reversed);
			EXPECT_TRUE(plan.use_current_altitude);
			EXPECT_TRUE(plan.fly_direct_to_goal);
			EXPECT_FLOAT_EQ(plan.join_position.alt, vehicle_position.alt);
		}
	}
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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	const mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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

		mission_route::RtlRouteRequest route_request = makeRtlRouteRequest(vehicle_position, 1);
		route_request.active_jump_anchor = anchor;
		mission_route::RtlRoutePlan route_plan{};
		route_plan.join_position = vehicle_position;
		route_plan.first_mission_item_index = 1;
		route_plan.goal_type = mission_route::GoalType::kMissionTakeoff;
		route_plan.goal_position = vehicle_position;
		ASSERT_TRUE(route_plan.valid());

		const mission_route::FailureReason route_status = planner.planRtlRoute(route_request, route_plan);

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
	mission_route::RtlRouteRequest route_request = makeRtlRouteRequest(vehicle_position, 0);
	route_request.active_jump_anchor = {3};

	mission[3].nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	planner.provider().setMissionItems(mission);

	mission_route::MissionResumePlan mission_plan{};
	mission_plan.join_position = vehicle_position;
	mission_plan.first_mission_item_index = 1;
	ASSERT_TRUE(mission_plan.valid());
	mission_route::RtlRoutePlan route_plan{};
	route_plan.join_position = vehicle_position;
	route_plan.first_mission_item_index = 1;
	route_plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	route_plan.goal_position = vehicle_position;
	ASSERT_TRUE(route_plan.valid());

	const mission_route::FailureReason mission_status =
		planner.planMissionResumeJoin(mission_request, mission_plan);
	const mission_route::FailureReason route_status = planner.planRtlRoute(route_request, route_plan);

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

// Even an unusable jump defines the geometric search window, matching pre-flight projections.
TEST_F(MissionRoutePlannerTest, InactiveJumpStillDefinesSafePointProjectionWindow)
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
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 1);
	request.safe_point_projection_search_distance_m = 0.f;
	mission_route::RtlRoutePlan plan{};

	// With zero margin only the jump remains. It cannot be used from the nominal vehicle segment.
	EXPECT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNoValidCandidateFound);
	EXPECT_FALSE(plan.valid());

	// A wider geometric window also admits the nominal leg, which has a usable return path.
	request.safe_point_projection_search_distance_m = 400.f;
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
	EXPECT_TRUE(plan.active_jump_anchor.empty());
}

// Square A-B-C-D closed by a DO_JUMP from D back to A, then on to E.
static std::vector<mission_item_s> squareLoopMission()
{
	return {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),     // 0 A
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kAlt),   // 1 B
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt), // 2 C
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),   // 3 D
		makeDoJump(0, 3, 0),                                                // 4 jump D -> A
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),   // 5 E
	};
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

// The closing edge bounds the window as before the flight, and on the vehicle's own loop it is also a branch-off.
// From just past D, corner D (102 m off) costs less than the edge (20 m off), but only a wide margin admits it.
TEST_F(MissionRoutePlannerTest, ActiveJumpStillNarrowsSafePointProjectionWindow)
{
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, -20.f, kAlt),
	};
	TestPlanner planner(squareLoopMission(), safe_points);
	mission_route::RtlRouteRequest request =
		makeRtlRouteRequest(makePositionFromOffset(kBaseLat, kBaseLon, 199.f, 0.f, kAlt), 0);
	request.active_jump_anchor = {4};

	struct MarginCase {
		float margin_m;
		int32_t first_mission_item_index;
		int32_t branch_off_mission_item_index;
	};

	// 30 m: along the jump edge toward A. 200 m: back to D, then branch off at corner D on [D-E].
	for (const MarginCase &margin_case : {MarginCase{30.f, 0, 0}, MarginCase{200.f, 3, 5}}) {
		SCOPED_TRACE(::testing::Message() << "margin " << margin_case.margin_m);
		request.safe_point_projection_search_distance_m = margin_case.margin_m;
		mission_route::RtlRoutePlan plan{};
		const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

		ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
		ASSERT_TRUE(plan.valid());
		EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
		EXPECT_EQ(plan.safe_point_index, 0);
		EXPECT_EQ(plan.active_jump_anchor.jump_item_index, 4);
		EXPECT_FALSE(plan.direction_reversed);
		EXPECT_EQ(plan.first_mission_item_index, margin_case.first_mission_item_index);
		EXPECT_EQ(plan.branch_off_mission_item_index, margin_case.branch_off_mission_item_index);
	}
}

// A DO_JUMP over commands only: its edge [1->4] and the nominal leg [1->4] are twins. On either, the safe
// point branches off ahead on [1->4], not after flying to item 4 and back.
TEST_F(MissionRoutePlannerTest, ActiveJumpEdgeTwinBranchesOffAhead)
{
	mission_item_s speed_change{};
	speed_change.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),   // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt), // 1
		speed_change,                                                     // 2 jump target
		makeDoJump(2, 2, 2),                                              // 3 exhausted, edge [1->4]
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt), // 4
	};
	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 175.f, 40.f, kAlt),
	};
	TestPlanner planner(mission, safe_points);
	mission_route::RtlRouteRequest request =
		makeRtlRouteRequest(makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 5.f, kAlt), 4);

	for (const int32_t anchor : {3, -1}) {
		SCOPED_TRACE(::testing::Message() << "anchor " << anchor);
		request.active_jump_anchor = {anchor};
		mission_route::RtlRoutePlan plan{};
		const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

		ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
		ASSERT_TRUE(plan.valid());
		EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
		EXPECT_EQ(plan.active_jump_anchor.jump_item_index, anchor);
		EXPECT_FALSE(plan.direction_reversed);
		EXPECT_EQ(plan.first_mission_item_index, 4);
		EXPECT_EQ(plan.branch_off_mission_item_index, 4);
	}
}

// Loop case where the cheapest path is nominal: rally 3, branch-off on [7-9].
TEST_F(MissionRoutePlannerTest, LoopScenarioSelectsRally3OnSegment7To9)
{
	// A safe point lies on the active jump segment 7->2 while the vehicle is inside that loop.
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	request.active_jump_anchor = {8};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 5);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = -15.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 15);
	request.velocity_north_m_s = 15.f;
	request.velocity_east_m_s = 15.f;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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
	mission_route::RtlRouteRequest request =
		makeRtlRouteRequest(vehicle_position, uturn_penalty_dataset::kMissionIndex);
	request.is_fixed_wing = scenario.vehicle == UturnVehicle::FixedWing;
	request.in_transition_to_fw = scenario.vehicle == UturnVehicle::TransitionToFw;
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

// ---- VTOL segment state and transition actions ----

static constexpr uint8_t kVtolUndefined = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;
static constexpr uint8_t kVtolMc = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
static constexpr uint8_t kVtolFw = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

static mission_route::Segment makeRouteSegment(int32_t start_idx, int32_t end_idx, int32_t jump_item_index = -1)
{
	mission_route::Segment segment{};
	segment.start.idx = start_idx;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = end_idx;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	segment.jump_item_index = jump_item_index;
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

// A preceding transition defines the mode; an undefined prefix uses the next transition.
TEST_F(MissionRoutePlannerTest, VtolStateForSegmentUsesLastTransitionBeforeSegment)
{
	VectorProvider provider = makeRouteProvider(missionWithTransitions(), {});

	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(0, 1)), kVtolMc);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(1, 3)), kVtolFw);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(3, 5)), kVtolMc);
}

// Invalid or out-of-range segments have no defined mode.
TEST_F(MissionRoutePlannerTest, VtolStateForSegmentIsUndefinedForInvalidSegments)
{
	VectorProvider provider = makeRouteProvider(missionWithTransitions(), {});

	EXPECT_EQ(mission_route::vtolStateForSegment(provider, mission_route::Segment{}), kVtolUndefined);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(3, 99)), kVtolUndefined);
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(3, 1, 99)), kVtolUndefined);
}

// An unreadable item is skipped and the scan continues toward the route start.
TEST_F(MissionRoutePlannerTest, VtolStateForSegmentSkipsUnreadableItems)
{
	VectorProvider provider = makeRouteProvider(missionWithTransitions(), {});
	provider.setMissionLoadFailures({4});

	// The back transition at 4 is unreadable, so the front transition at 2 wins.
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, makeRouteSegment(3, 5)), kVtolFw);
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
		int expected_safe_point_index;
		bool expected_reversed;
		mission_route::VtolTransitionAction expected_action;
	};

	using Action = mission_route::VtolTransitionAction;

	const VtolUturnCase cases[] = {
		// FW on an MC segment: no penalty (a u-turn would BT + FT, then re-BT at the segment).
		{"FwOnMcSegmentSkipsPenalty", true, true, false, kVtolMc, 0, true, Action::kBackTransition},
		// Transitioning to FW toward an MC segment: same skip as FW.
		{"TransitionToFwOnMcSegmentSkipsPenalty", true, false, true, kVtolMc, 0, true, Action::kBackTransition},
		// MC never u-turns and needs no transition for an MC segment.
		{"McOnMcSegmentNoPenaltyNoAction", true, false, false, kVtolMc, 0, true, Action::kNone},
		// MC toward a FW segment: still no penalty, the front transition comes at branch-in.
		{"McOnFwSegmentRequestsFrontTransition", true, false, false, kVtolFw, 0, true, Action::kFrontTransition},
		// Without transition items, the vehicle stays in MC and keeps the closest route.
		{"McWithUndefinedSegmentStateNoAction", true, false, false, kVtolUndefined, 0, true, Action::kNone},
		// FW on a FW segment: penalty applies, nothing to transition.
		{"FwOnFwSegmentKeepsPenalty", true, true, false, kVtolFw, 1, false, Action::kNone},
		// Unknown segment state never suppresses the penalty or requests a transition.
		{"FwWithUnknownSegmentStateKeepsPenalty", true, true, false, kVtolUndefined, 1, false, Action::kNone},
		// Non-VTOL: the segment state stays unresolved, so transition items change nothing.
		{"NonVtolKeepsPenaltyOnMcSegment", false, true, false, kVtolMc, 1, false, Action::kNone},
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

// A MC VTOL resuming onto a FW segment gets the missing front transition in the resume plan.
TEST_F(MissionRoutePlannerTest, MissionResumePlanRequestsFrontTransitionOnFwSegment)
{
	TestPlanner planner(uturnMissionWithTransition(kVtolFw));

	mission_route::MissionResumeRequest request =
		makeMissionResumeRequest(uturn_penalty_dataset::vehiclePosition(), 2);
	request.is_vtol = true;
	mission_route::MissionResumePlan plan{};

	ASSERT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.vtol_transition_action, mission_route::VtolTransitionAction::kFrontTransition);

	// Same vehicle without the VTOL capability gets no transition request.
	request.is_vtol = false;

	ASSERT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
	EXPECT_EQ(plan.vtol_transition_action, mission_route::VtolTransitionAction::kNone);
}

// Returning to the initial route prefix restores the mode before its first transition.
TEST_F(MissionRoutePlannerTest, MissionAndRtlJoinInferModeBeforeFirstTransition)
{
	for (const uint8_t transition_target : {kVtolFw, kVtolMc}) {
		SCOPED_TRACE(static_cast<int>(transition_target));
		TestPlanner planner({
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
			makeVtolTransitionItem(transition_target),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
		}, {
			makeSafePointFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt),
		});
		const mission_route::Position vehicle_position =
			makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt);
		const bool is_fixed_wing = transition_target == kVtolFw;
		const auto expected_action = is_fixed_wing ? mission_route::VtolTransitionAction::kBackTransition :
					     mission_route::VtolTransitionAction::kFrontTransition;
		mission_route::MissionResumeRequest resume_request = makeMissionResumeRequest(vehicle_position, 1);
		resume_request.is_vtol = true;
		resume_request.is_fixed_wing = is_fixed_wing;
		mission_route::MissionResumePlan resume_plan{};

		ASSERT_EQ(planner.planMissionResumeJoin(resume_request, resume_plan), mission_route::FailureReason::kNone);
		ASSERT_TRUE(resume_plan.valid());
		EXPECT_EQ(resume_plan.first_mission_item_index, 1);
		EXPECT_EQ(resume_plan.vtol_transition_action, expected_action);

		mission_route::RtlRouteRequest rtl_request = makeRtlRouteRequest(vehicle_position, 1);
		rtl_request.is_vtol = true;
		rtl_request.is_fixed_wing = is_fixed_wing;
		mission_route::RtlRoutePlan rtl_plan{};

		ASSERT_EQ(planner.planRtlRoute(rtl_request, rtl_plan), mission_route::FailureReason::kNone);
		ASSERT_TRUE(rtl_plan.valid());
		EXPECT_FALSE(rtl_plan.fly_direct_to_goal);
		EXPECT_EQ(rtl_plan.first_mission_item_index, 1);
		EXPECT_EQ(rtl_plan.vtol_transition_action, expected_action);
	}
}

// VTOL takeoff defines the following segments as FW.
TEST_F(MissionRoutePlannerTest, MissionResumeAfterVtolTakeoffKeepsFw)
{
	std::vector<mission_item_s> mission = uturn_penalty_dataset::mission();
	mission[0].nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	TestPlanner planner(mission);
	mission_route::MissionResumeRequest request =
		makeMissionResumeRequest(uturn_penalty_dataset::vehiclePosition(), 1);
	request.is_vtol = true;
	request.is_fixed_wing = true;
	mission_route::MissionResumePlan plan{};

	ASSERT_EQ(planner.planMissionResumeJoin(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	EXPECT_EQ(plan.vtol_transition_action, mission_route::VtolTransitionAction::kNone);
}

// On the corner mission, MC picks the closest rally in reverse (rally 1).
TEST_F(MissionRoutePlannerTest, CornerMissionRallyOnCornerMC)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.103348739288705, 2.3235968076446945, 600.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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

// A rally with an invalid loop candidate still wins via its nominal projection (rally 3).
TEST_F(MissionRoutePlannerTest, CornerMissionBackNoTransitionMC)
{
	// Vehicle near a transition boundary.
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.102107841234414, 2.31680521490218, 650.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 7);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 3);
}

// Small end-of-mission segments must not hide valid rally points (rally 5).
TEST_F(MissionRoutePlannerTest, CornerMissionSmallSegmentFrontMC)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10361319095525, 2.3183349874167636, 510.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 13);
	request.velocity_north_m_s = corner_dataset::kVelDiag;
	request.velocity_east_m_s = corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.safe_point_index, 5);
}

// Reverse-flight corner case: rally 2, branch-off on [5-7].
TEST_F(MissionRoutePlannerTest, CornerMissionReverseCornerScenarioSelectsRally2OnSegment5To7)
{
	TestPlanner planner(corner_dataset::mission(), corner_dataset::safePoints());

	const mission_route::Position vehicle_position =
		makePositionAbsolute(46.10205080248656, 2.318838207366314, 650.f);
	mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 5);
	request.current_route_direction_reversed = true;
	request.velocity_north_m_s = -corner_dataset::kVelDiag;
	request.velocity_east_m_s = -corner_dataset::kVelDiag;
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

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

// ---- Plan fingerprint ----

static constexpr float kFingerprintHomeAlt{488.f};

enum class FingerprintMission : uint8_t {
	kSurvey,
	kVtolSurveyWithJump,
	kStackedLand,
	kSpeedCommands,
	kVtolTransitions,
	kNorthSouthSurvey,
};

// Lawnmower survey in rows of 12 waypoints, from a takeoff to a landing below the last waypoint.
static std::vector<mission_item_s> makeFingerprintMission(FingerprintMission shape, int position_count)
{
	const bool vtol = shape == FingerprintMission::kVtolSurveyWithJump || shape == FingerprintMission::kVtolTransitions;
	std::vector<mission_item_s> mission{makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kFingerprintHomeAlt + 30.f,
					    vtol ? NAV_CMD_VTOL_TAKEOFF : NAV_CMD_TAKEOFF, false)};
	int16_t jump_target = -1;

	if (shape == FingerprintMission::kVtolSurveyWithJump) {
		mission.push_back(makeVtolTransitionItem(kVtolFw));
	}

	for (int i = 0; i < position_count; ++i) {
		const int row = i / 12;
		const float north = 150.f * row + 40.f;
		const float east = 120.f * (row % 2 == 0 ? i % 12 : 11 - i % 12) + 30.f;
		const float alt = kFingerprintHomeAlt + 50.f + 5.f * (i % 7);

		if (shape == FingerprintMission::kVtolSurveyWithJump && i == position_count / 3) {
			jump_target = static_cast<int16_t>(mission.size());
		}

		mission.push_back(shape == FingerprintMission::kNorthSouthSurvey
				  ? makePositionItemFromOffset(kBaseLat, kBaseLon, east, north, alt)
				  : makePositionItemFromOffset(kBaseLat, kBaseLon, north, east, alt));

		if (shape == FingerprintMission::kSpeedCommands && i % 3 == 1) {
			mission_item_s speed_change{};
			speed_change.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
			mission.push_back(speed_change);
		}

		if (shape == FingerprintMission::kVtolTransitions && i % 20 == 10) {
			mission.push_back(makeVtolTransitionItem((i / 20) % 2 ? kVtolFw : kVtolMc));
		}

		if (shape == FingerprintMission::kVtolSurveyWithJump && i == position_count / 2) {
			mission.push_back(makeDoJump(jump_target, 2, 1));
		}
	}

	const mission_item_s last = mission.back();

	if (shape == FingerprintMission::kStackedLand) {
		mission.push_back(makePositionItem(last.lat, last.lon, kFingerprintHomeAlt + 20.f));
	}

	if (shape == FingerprintMission::kVtolSurveyWithJump) {
		mission.push_back(makeVtolTransitionItem(kVtolMc));
	}

	mission.push_back(makePositionItem(last.lat, last.lon, kFingerprintHomeAlt, vtol ? NAV_CMD_VTOL_LAND : NAV_CMD_LAND));
	mission.back().autocontinue = false;
	return mission;
}

static std::vector<mission_item_s> makeFingerprintRallyPoints(int count)
{
	std::vector<mission_item_s> rally_points;

	for (int i = 0; i < count; ++i) {
		rally_points.push_back(makeSafePointFromOffset(kBaseLat, kBaseLon, -300.f + 173.f * (i % 8),
				       -250.f + 211.f * (i / 8) + 37.f * (i % 3), kFingerprintHomeAlt + 10.f));
	}

	return rally_points;
}

// Vehicles on a grid around the survey, for several mission indices, vehicle types, directions and velocities.
static std::vector<mission_route::RtlRouteRequest> makeFingerprintRequests(const std::vector<mission_item_s> &mission)
{
	struct VehicleType {
		bool is_fixed_wing;
		bool in_transition_to_fw;
		bool is_vtol;
	};

	// MC, FW, VTOL in FW, VTOL in MC, VTOL in front transition.
	static constexpr VehicleType kVehicleTypes[] {
		{false, false, false}, {true, false, false}, {true, false, true}, {false, false, true}, {false, true, true}
	};
	static constexpr float kVelocitiesNorthEast[][2] {{NAN, NAN}, {12.f, 3.f}, {-9.f, -14.f}};

	const int32_t count = static_cast<int32_t>(mission.size());
	int32_t jump_index = -1;

	for (int32_t i = 0; i < count; ++i) {
		if (mission[i].nav_cmd == NAV_CMD_DO_JUMP) {
			jump_index = i;
		}
	}

	std::vector<mission_route::RtlRouteRequest> requests;
	mission_route::RtlRouteRequest request{};
	request.home_altitude_amsl = kFingerprintHomeAlt;
	request.acceptance_radius_m = 10.f;
	request.safe_point_projection_search_distance_m = 30.f;
	request.direct_goal_acceptance_radius_m = 30.f;
	request.altitude_acceptance_radius_m = 5.f;

	for (int grid_north = -1; grid_north <= 8; grid_north += 3) {
		for (int grid_east = -1; grid_east <= 11; grid_east += 3) {
			request.vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 97.f * grid_north + 11.f,
						   83.f * grid_east - 7.f, kFingerprintHomeAlt + 15.f + 9.f * ((grid_north + grid_east) % 5));

			for (const int32_t mission_index : {0, 1, count / 3, count / 2, count - 2, count - 1}) {
				request.mission_index = mission_index;

				for (const VehicleType &type : kVehicleTypes) {
					request.is_fixed_wing = type.is_fixed_wing;
					request.in_transition_to_fw = type.in_transition_to_fw;
					request.is_vtol = type.is_vtol;
					request.projection_search_distance_m = type.is_fixed_wing ? 150.f : 30.f;

					for (const bool reversed : {false, true}) {
						request.current_route_direction_reversed = reversed;

						for (const auto &velocity : kVelocitiesNorthEast) {
							request.velocity_north_m_s = velocity[0];
							request.velocity_east_m_s = velocity[1];
							request.active_jump_anchor = {};
							requests.push_back(request);

							// On the jump edge only from mid-mission, flying nominally.
							if (jump_index >= 0 && mission_index == count / 2 && !reversed) {
								request.active_jump_anchor = {jump_index};
								requests.push_back(request);
							}
						}
					}
				}
			}
		}
	}

	return requests;
}

// 64-bit FNV-1a.
static void hashText(uint64_t &hash, const char *text)
{
	for (; *text != '\0'; ++text) {
		hash = (hash ^ static_cast<uint8_t>(*text)) * 1099511628211ULL;
	}
}

// Hashes the plan as text; hex floats keep every bit.
static void hashPlan(uint64_t &hash, mission_route::FailureReason status, const mission_route::MissionResumePlan &plan)
{
	char text[256];
	snprintf(text, sizeof(text), "%d %d %d %d %d %d %a %a %a\n", static_cast<int>(status),
		 static_cast<int>(plan.first_mission_item_index), plan.direction_reversed, plan.use_current_altitude,
		 static_cast<int>(plan.active_jump_anchor.jump_item_index), static_cast<int>(plan.vtol_transition_action),
		 plan.join_position.lat, plan.join_position.lon, static_cast<double>(plan.join_position.alt));
	hashText(hash, text);
}

static void hashPlan(uint64_t &hash, mission_route::FailureReason status, const mission_route::RtlRoutePlan &plan)
{
	hashPlan(hash, status, static_cast<const mission_route::MissionResumePlan &>(plan));
	char text[256];
	snprintf(text, sizeof(text), "%d %d %d %d %a %a %a %a %a %a\n", static_cast<int>(plan.goal_type),
		 static_cast<int>(plan.safe_point_index), static_cast<int>(plan.branch_off_mission_item_index),
		 plan.fly_direct_to_goal, plan.goal_position.lat, plan.goal_position.lon,
		 static_cast<double>(plan.goal_position.alt), plan.branch_off_position.lat, plan.branch_off_position.lon,
		 static_cast<double>(plan.branch_off_position.alt));
	hashText(hash, text);
}

// Hashes every Return and join plan of a fixed sweep. Run it with --gtest_also_run_disabled_tests before and
// after a planner change on the same machine: equal "Plan fingerprint" hashes mean no plan changed.
TEST_F(MissionRoutePlannerTest, DISABLED_PlanFingerprint)
{
	static constexpr FingerprintMission kShapes[] {
		FingerprintMission::kSurvey, FingerprintMission::kVtolSurveyWithJump, FingerprintMission::kStackedLand,
		FingerprintMission::kSpeedCommands, FingerprintMission::kVtolTransitions, FingerprintMission::kNorthSouthSurvey
	};
	uint64_t hash = 14695981039346656037ULL; // FNV-1a offset basis
	unsigned plan_count = 0;

	for (const FingerprintMission shape : kShapes) {
		for (const int position_count : {3, 40}) {
			const std::vector<mission_item_s> mission = makeFingerprintMission(shape, position_count);
			const std::vector<mission_route::RtlRouteRequest> requests = makeFingerprintRequests(mission);

			for (const int rally_count : {0, 3, 33}) {
				TestPlanner planner(mission, makeFingerprintRallyPoints(rally_count));

				for (const mission_route::RtlRouteRequest &request : requests) {
					mission_route::RtlRoutePlan rtl_plan{};
					hashPlan(hash, planner.planRtlRoute(request, rtl_plan), rtl_plan);
					mission_route::MissionResumePlan join_plan{};
					hashPlan(hash, planner.planMissionResumeJoin(request, join_plan), join_plan);
					plan_count += 2;
				}
			}
		}
	}

	printf("Plan fingerprint: %u plans, hash %016" PRIx64 "\n", plan_count, hash);
}
