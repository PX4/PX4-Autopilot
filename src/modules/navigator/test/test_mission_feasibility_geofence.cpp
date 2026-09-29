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

/* EVENT
 * @skip-file
 */

#include <gtest/gtest.h>

#include "mission_feasibility_checker.h"
#include "support/geofence_test_helpers.h"
#include "support/mission_route_test_helpers.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <px4_platform_common/events.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/event.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/vehicle_status.h>

class MissionFeasibilityGeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kAltitude = 500.f;

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence(kAltitude - 100.f));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		_home_pub.publish(*_navigator.get_home_position());

		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		*_navigator.get_vstatus() = status;
		_status_pub.publish(status);
		ASSERT_TRUE(loadFence({}));
	}

	void TearDown() override { param_reset_all(); }

	mission_item_s waypoint(const matrix::Vector2f &offset) const
	{
		return navigator_test::makePositionItemFromOffset(_reference(0), _reference(1), offset(0), offset(1), kAltitude);
	}

	static mission_item_s changeSpeed()
	{
		mission_item_s item{};
		item.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
		item.params[0] = 1.f;
		item.params[1] = 5.f;
		return item;
	}

	FencePoints inclusionSquare() const
	{
		return polygon(true, {{-1000.f, -1000.f}, {1000.f, -1000.f}, {1000.f, 1000.f}, {-1000.f, 1000.f}});
	}

	void drainReports()
	{
		mavlink_log_s report{};
		event_s event{};

		while (_log_sub.update(&report)) {}

		while (_event_sub.update(&event)) {}
	}

	bool missionFeasible(const std::vector<mission_item_s> &items)
	{
		drainReports();

		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		mission_s mission{};
		mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		mission.count = static_cast<uint16_t>(items.size());
		MissionFeasibilityChecker checker(&_navigator, _dataman_client);
		return checker.checkMissionFeasible(mission);
	}

	bool logContains(const char *text)
	{
		mavlink_log_s report{};
		bool found = false;

		while (_log_sub.update(&report)) {
			found |= strstr(reinterpret_cast<const char *>(report.text), text) != nullptr;
		}

		return found;
	}

	::testing::AssertionResult waypointEvent(uint32_t id, int16_t expected_waypoint)
	{
		event_s event{};

		while (_event_sub.update(&event)) {
			if (event.id == id) {
				int16_t waypoint_index;
				memcpy(&waypoint_index, event.arguments, sizeof(waypoint_index));

				if (waypoint_index != expected_waypoint) {
					return ::testing::AssertionFailure() << "reported waypoint " << waypoint_index
					       << ", expected " << expected_waypoint;
				}

				return ::testing::AssertionSuccess();
			}
		}

		return ::testing::AssertionFailure() << "expected waypoint event not published";
	}

	void expectPathViolation(int16_t waypoint_index)
	{
		const std::string message = "Geofence breach on path to waypoint " + std::to_string(waypoint_index);
		EXPECT_TRUE(logContains(message.c_str()));
		EXPECT_TRUE(waypointEvent(events::ID("navigator_mis_geofence_path_violation"), waypoint_index));
	}

	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Subscription _log_sub{ORB_ID(mavlink_log)};
	uORB::Subscription _event_sub{ORB_ID(event)};
};

struct MissionLegCase {
	const char *name;
	matrix::Vector2f start;
	matrix::Vector2f end;
	bool feasible;
};

class MissionGeofenceLegTest : public MissionFeasibilityGeofenceTest,
	public ::testing::WithParamInterface<MissionLegCase> {};

TEST_P(MissionGeofenceLegTest, ChecksLegAcrossCommandItems)
{
	const MissionLegCase &test = GetParam();
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_EQ(missionFeasible({changeSpeed(), waypoint(test.start), changeSpeed(), waypoint(test.end)}), test.feasible);

	if (!test.feasible) {
		expectPathViolation(4);
	}
}

INSTANTIATE_TEST_SUITE_P(MissionLegs, MissionGeofenceLegTest, ::testing::Values(
				 MissionLegCase{"AcrossExclusionPolygon", {0.f, 100.f}, {0.f, 500.f}, false},
				 MissionLegCase{"ClearOfExclusionPolygon", {60.f, 100.f}, {60.f, 500.f}, true}),
			 [](const ::testing::TestParamInfo<MissionLegCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(MissionFeasibilityGeofenceTest, FirstPositionMustPassEveryInclusion)
{
	FencePoints points = inclusionSquare();
	const FencePoints smaller = polygon(true, {{-100.f, -100.f}, {100.f, -100.f}, {100.f, 100.f}, {-100.f, 100.f}});
	points.insert(points.end(), smaller.begin(), smaller.end());
	ASSERT_TRUE(loadFence(points));
	EXPECT_FALSE(missionFeasible({changeSpeed(), waypoint({0.f, 200.f})}));
	EXPECT_TRUE(waypointEvent(events::ID("navigator_mis_geofence_violation"), 2));
	EXPECT_TRUE(missionFeasible({waypoint({0.f, 0.f}), waypoint({50.f, 50.f})}));
}

TEST_F(MissionFeasibilityGeofenceTest, SinglePositionOnBoundaryIsRejected)
{
	const FencePoints points = exclusionSquare();
	ASSERT_TRUE(loadFence(points));
	mission_item_s item = waypoint({0.f, 0.f});
	item.lat = points.front().lat;
	item.lon = points.front().lon;
	ASSERT_TRUE(_fence.checkPointAgainstAllGeofences(item.lat, item.lon, item.altitude));
	EXPECT_FALSE(missionFeasible({item}));
	expectPathViolation(1);
}

TEST_F(MissionFeasibilityGeofenceTest, EmptyFenceAcceptsPositionsAndCommandOnlyMission)
{
	EXPECT_TRUE(missionFeasible({waypoint({0.f, 100.f}), waypoint({0.f, 5000.f})}));
	EXPECT_TRUE(missionFeasible({changeSpeed()}));
}

struct MissionBatchCase {
	const char *name;
	size_t safe_positions;
	bool final_leg_breaches;
};

class MissionGeofenceBatchTest : public MissionFeasibilityGeofenceTest,
	public ::testing::WithParamInterface<MissionBatchCase> {};

TEST_P(MissionGeofenceBatchTest, ChecksBatchesAndResetsForNextMission)
{
	const MissionBatchCase &test = GetParam();
	ASSERT_TRUE(loadFence(exclusionSquare()));
	std::vector<mission_item_s> items{changeSpeed()};

	for (size_t i = 0; i < test.safe_positions; ++i) {
		items.push_back(waypoint({0.f, 100.f}));
		items.push_back(changeSpeed());
	}

	items.push_back(waypoint({0.f, test.final_leg_breaches ? 500.f : 150.f}));
	EXPECT_EQ(missionFeasible(items), !test.final_leg_breaches);

	if (test.final_leg_breaches) {
		expectPathViolation(static_cast<int16_t>(items.size()));
	}

	// The next upload must not retain paths from the previous validation.
	EXPECT_TRUE(missionFeasible({waypoint({0.f, 150.f})}));
}

INSTANTIATE_TEST_SUITE_P(MissionBatches, MissionGeofenceBatchTest, ::testing::Values(
				 MissionBatchCase{"BreachAtFirstBatchEnd", Geofence::MAX_PATH_CHECKS - 1, true},
				 MissionBatchCase{"BreachAfterFirstBatch", Geofence::MAX_PATH_CHECKS, true},
				 MissionBatchCase{"ClearFullBatches", 2 * Geofence::MAX_PATH_CHECKS - 1, false},
				 MissionBatchCase{"ClearFinalRemainder", 2 * Geofence::MAX_PATH_CHECKS + 1, false}),
			 [](const ::testing::TestParamInfo<MissionBatchCase> &test_info)
{
	return test_info.param.name;
});

enum class MissionScalarLimit { Horizontal, Vertical, AltitudeBand };

struct MissionScalarCase {
	const char *name;
	MissionScalarLimit limit;
	float east;
	float altitude;
	bool relative;
	bool feasible;
};

class MissionGeofenceScalarTest : public MissionFeasibilityGeofenceTest,
	public ::testing::WithParamInterface<MissionScalarCase> {};

TEST_P(MissionGeofenceScalarTest, ChecksEveryPosition)
{
	const MissionScalarCase &test = GetParam();
	ASSERT_TRUE(loadFence(inclusionSquare()));

	if (test.limit == MissionScalarLimit::AltitudeBand) {
		GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);

	} else {
		const float maximum = test.limit == MissionScalarLimit::Horizontal ? 300.f : 150.f;
		ASSERT_EQ(param_set(param_find(test.limit == MissionScalarLimit::Horizontal ? "GF_MAX_HOR_DIST" : "GF_MAX_VER_DIST"),
				    &maximum), PX4_OK);
		_navigator.updateParams();
	}

	mission_item_s second = waypoint({0.f, test.east});
	second.altitude = test.altitude;
	second.altitude_is_relative = test.relative;
	EXPECT_EQ(missionFeasible({waypoint({0.f, 100.f}), changeSpeed(), second}), test.feasible);

	if (!test.feasible) {
		EXPECT_TRUE(waypointEvent(events::ID("navigator_mis_geofence_violation"), 3));
	}
}

INSTANTIATE_TEST_SUITE_P(MissionScalarLimits, MissionGeofenceScalarTest, ::testing::Values(
				 MissionScalarCase{"WithinHomeDistance", MissionScalarLimit::Horizontal, 200.f, 500.f, false, true},
				 MissionScalarCase{"BeyondHomeDistance", MissionScalarLimit::Horizontal, 400.f, 500.f, false, false},
				 MissionScalarCase{"WithinRelativeAltitude", MissionScalarLimit::Vertical, 200.f, 100.f, true, true},
				 MissionScalarCase{"BeyondRelativeAltitude", MissionScalarLimit::Vertical, 200.f, 200.f, true, false},
				 MissionScalarCase{"BelowAltitudeBand", MissionScalarLimit::AltitudeBand, 200.f, 440.f, false, false},
				 MissionScalarCase{"AboveAltitudeBand", MissionScalarLimit::AltitudeBand, 200.f, 560.f, false, false},
				 MissionScalarCase{"AtAltitudeBandMinimum", MissionScalarLimit::AltitudeBand, 200.f, 450.f, false, true},
				 MissionScalarCase{"AtAltitudeBandMaximum", MissionScalarLimit::AltitudeBand, 200.f, 550.f, false, true}),
			 [](const ::testing::TestParamInfo<MissionScalarCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(MissionFeasibilityGeofenceTest, EmptyFenceStillChecksHomeLimits)
{
	const float maximum = 300.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum), PX4_OK);
	_navigator.updateParams();
	EXPECT_FALSE(missionFeasible({waypoint({0.f, 100.f}), waypoint({0.f, 400.f})}));
	EXPECT_TRUE(waypointEvent(events::ID("navigator_mis_geofence_violation"), 2));
}

TEST_F(MissionFeasibilityGeofenceTest, EmptyFenceIgnoresStoredAltitudeBand)
{
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	mission_item_s second = waypoint({0.f, 200.f});
	second.altitude = 600.f;
	EXPECT_TRUE(missionFeasible({waypoint({0.f, 100.f}), second}));
}

enum class LaterMissionFailure { Altitude, NoHome };

class MissionGeofenceFailureOrderTest : public MissionFeasibilityGeofenceTest,
	public ::testing::WithParamInterface<LaterMissionFailure> {};

TEST_P(MissionGeofenceFailureOrderTest, EarlierPathBreachPrecedesLaterFailure)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	mission_item_s invalid = waypoint({100.f, 500.f});

	if (GetParam() == LaterMissionFailure::NoHome) {
		_navigator.get_home_position()->valid_hpos = false;
		_home_pub.publish(*_navigator.get_home_position());
		invalid.altitude_is_relative = true;
		invalid.altitude = 100.f;

	} else {
		const float maximum = 150.f;
		ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum), PX4_OK);
		_navigator.updateParams();
		invalid.altitude = 600.f;
	}

	// The first leg crosses the exclusion, but is still buffered when the third item fails.
	EXPECT_FALSE(missionFeasible({waypoint({0.f, 100.f}), waypoint({0.f, 500.f}), invalid}));
	expectPathViolation(2);
}

INSTANTIATE_TEST_SUITE_P(MissionFailureOrder, MissionGeofenceFailureOrderTest,
			 ::testing::Values(LaterMissionFailure::Altitude, LaterMissionFailure::NoHome),
			 [](const ::testing::TestParamInfo<LaterMissionFailure> &test_info)
{
	return test_info.param == LaterMissionFailure::NoHome ? "MissingHome" : "AltitudeLimit";
});

TEST_F(MissionFeasibilityGeofenceTest, RelativeAltitudeRequiresValidHome)
{
	_navigator.get_home_position()->valid_hpos = false;
	_home_pub.publish(*_navigator.get_home_position());
	mission_item_s relative = waypoint({0.f, 100.f});
	relative.altitude_is_relative = true;
	relative.altitude = 100.f;
	EXPECT_FALSE(missionFeasible({relative}));
	EXPECT_TRUE(logContains("Geofence requires valid home position"));
}

TEST_F(MissionFeasibilityGeofenceTest, MissingFenceCacheRejectsUpload)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofenceTestPeer::invalidateCache(_fence);
	EXPECT_FALSE(missionFeasible({waypoint({100.f, 100.f}), waypoint({100.f, 500.f})}));
	EXPECT_TRUE(logContains("geofence path check unavailable"));
}

TEST_F(MissionFeasibilityGeofenceTest, PendingFenceRefreshRejectsUpload)
{
	_fence.updateFence();
	EXPECT_FALSE(missionFeasible({waypoint({0.f, 100.f})}));
	EXPECT_TRUE(logContains("geofence path check unavailable"));
}

TEST_F(MissionFeasibilityGeofenceTest, UnreadableMissionStorageRejectsUpload)
{
	drainReports();
	mission_s mission{};
	// This key cannot hold a mission_item_s, so readSync fails without a timeout.
	mission.mission_dataman_id = DM_KEY_FENCE_POINTS_STATE;
	mission.count = 1;
	MissionFeasibilityChecker checker(&_navigator, _dataman_client);
	EXPECT_FALSE(checker.checkMissionFeasible(mission));
	EXPECT_TRUE(logContains("dataman read failed at item 0"));
	EXPECT_TRUE(waypointEvent(events::ID("navigator_mis_dm_read_fail"), 0));
}
