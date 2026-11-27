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
 * @file test_RTL_mission_safe_point_follow.cpp
 *
 * Return along the mission route (RTL_TYPE 7): the follower's steps, and when RTL plans.
 * The planner's choice of the goal is tested in test_mission_route_planner.cpp.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "navigator.h"
#include "rtl.h"
#include "rtl_mission_safe_point_follow.h"
#include "support/event_recorder.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mode_completed.h>
#include <uORB/topics/rtl_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <string>
#include <vector>

/* EVENT
 * @skip-file
 */

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kHomeAlt = 100.f;
static constexpr float kRouteAlt = 160.f;

static mission_item_s rallyPoint(float north_m, float east_m, float alt = kHomeAlt)
{
	return makeSafePointFromOffset(kBaseLat, kBaseLon, north_m, east_m, alt);
}

// Reads the state of a follower, also of the one RTL creates.
class RtlMissionSafePointFollowTestPeer : public RtlMissionSafePointFollow
{
public:
	using MissionBase::WorkItemType;

	static WorkItemType workItem(const RtlMissionSafePointFollow &follower) { return follower._work_item_type; }
	static const mission_item_s &missionItem(const RtlMissionSafePointFollow &follower) { return follower._mission_item; }
	static int32_t index(const RtlMissionSafePointFollow &follower) { return follower._mission.current_seq; }
	static uint32_t missionId(const RtlMissionSafePointFollow &follower) { return follower._mission.mission_id; }

	// LAND_DELAY counts real time once its point is reached.
	static void ageWaypointReached(RtlMissionSafePointFollow &follower, float seconds)
	{
		if (follower._time_wp_reached != 0) {
			follower._time_wp_reached -= static_cast<hrt_abstime>(seconds * 1e6f);
		}
	}
};

using FollowerPeer = RtlMissionSafePointFollowTestPeer;

class RTLTestPeer : public RTL
{
public:
	explicit RTLTestPeer(Navigator *navigator) : RTL(navigator) {}

	void refreshParametersForTest() { updateParams(); }

	// The inactive evaluation RTL runs every 2 s.
	void evaluateForTest()
	{
		_global_pos_sub.update();
		_vehicle_status_sub.update();
		_mission_sub.update();
		_home_pos_sub.update();
		setRtlTypeAndDestination();
	}

	// The retry interval without a route starts now, or has passed.
	void startRetryIntervalForTest() { _destination_check_time = hrt_absolute_time(); }
	void expireRetryIntervalForTest() { _destination_check_time = hrt_absolute_time() - 3_s; }

	RtlType rtlTypeForTest() const { return _rtl_type; }

	RtlMissionSafePointFollow &follower() { return *static_cast<RtlMissionSafePointFollow *>(_rtl_mission_type_handle); }
};

namespace
{

enum class Step : uint8_t { Climb, Join, Route, BranchOff, MoveToLand, Wait, Land, PrecisionLand };

struct ExpectedStep {
	Step step;
	float north_m;
	float east_m;
	float alt;
	int32_t index; ///< mission index while the step is flown
	bool reports_reached; ///< reaching it reports the index reached
};

const char *stepName(Step step)
{
	switch (step) {
	case Step::Climb: return "climb";

	case Step::Join: return "join";

	case Step::Route: return "route";

	case Step::BranchOff: return "branch-off";

	case Step::MoveToLand: return "move to land";

	case Step::Wait: return "wait";

	case Step::Land: return "land";

	case Step::PrecisionLand: return "precision land";
	}

	return "?";
}

} // namespace

class RtlRouteTest : public NavigatorDatamanTestBase
{
protected:
	using WorkItemType = FollowerPeer::WorkItemType;

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		publishVehicle(false);
		publishLocalPosition();
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kHomeAlt));
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kRouteAlt));

		// The planner checks its legs against the loaded geofence, here an empty one.
		mission_stats_entry_s fence_stats{};
		fence_stats.dataman_id = DM_KEY_FENCE_POINTS_0;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&fence_stats),
						      sizeof(fence_stats)));
		Geofence &fence = _navigator.get_geofence();
		fence.updateFence();
		const hrt_abstime start = hrt_absolute_time();

		while (!fence.isReadyForPathChecks() && hrt_elapsed_time(&start) < 1_s) {
			fence.run();
			px4_usleep(1000);
		}

		ASSERT_TRUE(fence.isReadyForPathChecks());
	}

	void TearDown() override
	{
		param_reset_all();
		param_control_autosave(true);
	}

	void setParam(const char *name, float value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	void setParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	// Takeoff at Home, a square of three waypoints 60 m above Home, and a landing between Home and the last one.
	// Mission holds 5 s at item 2.
	static std::vector<mission_item_s> squareMission()
	{
		std::vector<mission_item_s> items{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kRouteAlt, NAV_CMD_TAKEOFF),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kRouteAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kRouteAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kRouteAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kHomeAlt, NAV_CMD_LAND),
		};

		for (mission_item_s &item : items) {
			item.yaw = NAN;
		}

		items[2].time_inside = 5.f;
		return items;
	}

	// The mission and rally points in Dataman and in the route cache, and the mission checked by Mission.
	mission_s loadRoute(const std::vector<mission_item_s> &items, const std::vector<mission_item_s> &rally_points,
			    int32_t current_seq, uint32_t mission_id = 31, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		mission_s mission = writeRoute(items, rally_points, current_seq, mission_id, dataman_id);
		loadRouteCache(mission);
		markMissionChecked(mission);
		return mission;
	}

	// Only stores and publishes, as an upload does: the route cache and the mission check follow later.
	mission_s writeRoute(const std::vector<mission_item_s> &items, const std::vector<mission_item_s> &rally_points,
			     int32_t current_seq, uint32_t mission_id = 31, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			EXPECT_TRUE(_dataman_client.writeSync(dataman_id, i, reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		writeRallyPoints(rally_points, mission_id);

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.current_seq = current_seq;
		mission.count = items.size();
		mission.land_start_index = -1;
		mission.land_index = -1;

		for (size_t i = 0; i < items.size(); ++i) {
			if (mission_route::isLandingCmd(items[i].nav_cmd)) {
				mission.land_start_index = i;
				mission.land_index = i;
			}
		}

		mission.mission_id = mission_id;
		mission.safe_points_id = mission_id;
		mission.geofence_id = 0;
		mission.mission_dataman_id = dataman_id;
		mission.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
		mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
		publishMission(mission);
		return mission;
	}

	void writeRallyPoints(const std::vector<mission_item_s> &rally_points, uint32_t safe_points_id)
	{
		for (size_t i = 0; i < rally_points.size(); ++i) {
			mission_item_s item = rally_points[i];
			EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_0, i, reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		mission_stats_entry_s stats{};
		stats.num_items = rally_points.size();
		stats.opaque_id = safe_points_id;
		stats.dataman_id = DM_KEY_SAFE_POINTS_0;
		EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0, reinterpret_cast<uint8_t *>(&stats),
						      sizeof(stats)));
	}

	void loadRouteCache(const mission_s &mission)
	{
		MissionRouteCache &route_cache = _navigator.get_mission_route_cache();
		EXPECT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(route_cache, mission, [&] {
			return route_cache.missionItemsReady(mission) && route_cache.safePointsReady();
		})) << "the route cache did not load";
	}

	void markMissionChecked(const mission_s &mission)
	{
		mission_result_s &result = *_navigator.get_mission_result();
		result = {};
		result.valid = true;
		result.mission_id = mission.mission_id;
		result.geofence_id = mission.geofence_id;
	}

	void publishMission(const mission_s &mission) { _mission_pub.publish(mission); }

	void publishVehicle(bool landed, bool armed = true, uint8_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
			    bool is_vtol = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_type;
		status.is_vtol = is_vtol;
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		_vehicle_status_pub.publish(status);
		*_navigator.get_vstatus() = status;

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		_land_detected_pub.publish(land_detected);
		*_navigator.get_land_detected() = land_detected;
	}

	void publishGlobalPosition(const mission_route::Position &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_global_position_pub.publish(global_position);
		*_navigator.get_global_position() = global_position;
	}

	void publishLocalPosition()
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;
		local_position.z_valid = true;
		local_position.heading_good_for_control = true;
		_local_position_pub.publish(local_position);
		*_navigator.get_local_position() = local_position;
	}

	void publishHomePosition(const mission_route::Position &position)
	{
		home_position_s home{};
		home.timestamp = hrt_absolute_time();
		home.valid_hpos = true;
		home.valid_alt = true;
		home.lat = position.lat;
		home.lon = position.lon;
		home.alt = position.alt;
		_home_pub.publish(home);
		*_navigator.get_home_position() = home;
	}

	void moveTo(float north_m, float east_m, float alt)
	{
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, north_m, east_m, alt));
	}

	const position_setpoint_s &currentSetpoint() { return _navigator.get_position_setpoint_triplet()->current; }

	// Plans on the mission and starts the follower, as RTL does at activation.
	void start(RtlMissionSafePointFollow &follower, const mission_s &mission)
	{
		follower.run(false);
		follower.planRoute(mission);
		ASSERT_TRUE(follower.plan().valid());
		follower.run(true);
	}

	void expectStep(const ExpectedStep &expected, const RtlMissionSafePointFollow &follower, float land_delay = 0.f)
	{
		const mission_item_s &item = FollowerPeer::missionItem(follower);
		WorkItemType work_item{WorkItemType::WORK_ITEM_TYPE_DEFAULT};
		uint16_t nav_cmd{NAV_CMD_WAYPOINT};

		switch (expected.step) {
		case Step::Climb: work_item = WorkItemType::WORK_ITEM_TYPE_CLIMB; nav_cmd = NAV_CMD_LOITER_TO_ALT; break;

		case Step::Join: work_item = WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE; break;

		case Step::Route: break;

		case Step::BranchOff: work_item = WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF; break;

		case Step::MoveToLand: work_item = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND; break;

		case Step::Wait:
			work_item = WorkItemType::WORK_ITEM_TYPE_LAND_DELAY;
			nav_cmd = land_delay < 0.f ? NAV_CMD_LOITER_UNLIMITED : NAV_CMD_LOITER_TIME_LIMIT;
			break;

		case Step::Land: nav_cmd = NAV_CMD_LAND; break;

		case Step::PrecisionLand: work_item = WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND; nav_cmd = NAV_CMD_LAND; break;
		}

		EXPECT_EQ(FollowerPeer::workItem(follower), work_item);
		EXPECT_EQ(item.nav_cmd, nav_cmd);
		EXPECT_EQ(FollowerPeer::index(follower), expected.index);
		float north_m = NAN;
		float east_m = NAN;
		offsetOf(item.lat, item.lon, north_m, east_m);
		EXPECT_NEAR(north_m, expected.north_m, 1.f);
		EXPECT_NEAR(east_m, expected.east_m, 1.f);
		EXPECT_NEAR(item.altitude_is_relative ? item.altitude + kHomeAlt : item.altitude, expected.alt, 0.5f);

		// Route items are flown through, without their hold time.
		if (expected.step == Step::Route) {
			EXPECT_FLOAT_EQ(item.time_inside, 0.f);
		}
	}

	// Flies onto the current setpoint, or touches down for a landing, until the follower moves on.
	// run_cycle runs the follower, or RTL with it.
	template<typename RunCycle>
	void reachStep(RtlMissionSafePointFollow &follower, RunCycle run_cycle)
	{
		const WorkItemType work_item = FollowerPeer::workItem(follower);
		const int32_t index = FollowerPeer::index(follower);
		const uint16_t nav_cmd = FollowerPeer::missionItem(follower).nav_cmd;

		for (int cycle = 0; cycle < 10; ++cycle) {
			if (nav_cmd == NAV_CMD_LAND) {
				publishVehicle(true);

			} else {
				const position_setpoint_s &setpoint = currentSetpoint();
				publishGlobalPosition({setpoint.lat, setpoint.lon, setpoint.alt});
			}

			run_cycle();
			FollowerPeer::ageWaypointReached(follower, 10.f);

			if ((FollowerPeer::workItem(follower) != work_item) || (FollowerPeer::index(follower) != index)
			    || (FollowerPeer::missionItem(follower).nav_cmd != nav_cmd)) {
				return;
			}
		}
	}

	void reachStep(RtlMissionSafePointFollow &follower) { reachStep(follower, [&] { follower.run(true); }); }

	// Meters north and east of the base point.
	static void offsetOf(double lat, double lon, float &north_m, float &east_m)
	{
		get_vector_to_next_waypoint(kBaseLat, kBaseLon, lat, lon, &north_m, &east_m);
	}

	Navigator _navigator{};
	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
};

struct RouteCase {
	const char *test_name;
	float vehicle_north_m;
	float vehicle_east_m;
	float vehicle_alt;
	int32_t current_seq;
	std::vector<mission_item_s> rally_points;
	float land_delay;
	float descend_alt;
	std::vector<ExpectedStep> steps;
	int32_t precision_land{0};
};

class RtlRouteStepTest : public RtlRouteTest, public ::testing::WithParamInterface<RouteCase> {};

// WHY: the follower decides which steps the Return flies; MissionBase decides when each is reached.
// WHAT: each goal of the planner, with RTL_LAND_DELAY 0, 5 s and -1, flown to touchdown or to the hold.
TEST_P(RtlRouteStepTest, FliesTheStepsToTheGoal)
{
	const RouteCase &c = GetParam();
	setParam("RTL_LAND_DELAY", c.land_delay);
	setParam("RTL_DESCEND_ALT", c.descend_alt);
	setParam("RTL_PLD_MD", c.precision_land);
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(c.vehicle_north_m, c.vehicle_east_m, c.vehicle_alt);
	const mission_s mission = loadRoute(squareMission(), c.rally_points, c.current_seq);
	uORB::SubscriptionData<mode_completed_s> mode_completed_sub{ORB_ID(mode_completed)};
	(void)mode_completed_sub.update(); // a completion from an earlier test

	// WHEN: Return starts and the vehicle reaches each step
	start(follower, mission);
	int32_t reported = _navigator.get_mission_result()->seq_reached;

	for (size_t i = 0; i < c.steps.size(); ++i) {
		const ExpectedStep &expected = c.steps[i];
		SCOPED_TRACE(::testing::Message() << "step " << i << ": " << stepName(expected.step));

		// THEN: it flies the step, and reports the uploaded item reached only where one was reached
		expectStep(expected, follower, c.land_delay);
		const position_setpoint_triplet_s &triplet = *_navigator.get_position_setpoint_triplet();
		float north_m = NAN;
		float east_m = NAN;

		// The next setpoint is the next step: the branch-off point in place of its route item, toward a route item or
		// the branch-off point also at its altitude. The wait and the landing have none.
		if ((i + 1 < c.steps.size()) && (expected.step != Step::Wait) && (expected.step != Step::Land)
		    && (expected.step != Step::PrecisionLand)) {
			const ExpectedStep &following = c.steps[i + 1];
			offsetOf(triplet.next.lat, triplet.next.lon, north_m, east_m);
			EXPECT_TRUE(triplet.next.valid);
			EXPECT_NEAR(north_m, following.north_m, 1.f);
			EXPECT_NEAR(east_m, following.east_m, 1.f);

			if ((following.step == Step::Join) || (following.step == Step::Route) || (following.step == Step::BranchOff)) {
				EXPECT_NEAR(triplet.next.alt, following.alt, 0.5f);
			}
		}

		// On the route the previous setpoint is where the leg starts, so the vehicle follows the leg.
		if (((expected.step == Step::Route) || (expected.step == Step::BranchOff)) && (i > 0)
		    && ((c.steps[i - 1].step == Step::Join) || (c.steps[i - 1].step == Step::Route))) {
			offsetOf(triplet.previous.lat, triplet.previous.lon, north_m, east_m);
			EXPECT_TRUE(triplet.previous.valid);
			EXPECT_NEAR(north_m, c.steps[i - 1].north_m, 1.f);
			EXPECT_NEAR(east_m, c.steps[i - 1].east_m, 1.f);
		}

		if ((expected.step == Step::Wait) && (c.land_delay < 0.f)) {
			// An unlimited delay holds above the goal and never lands.
			reachStep(follower);
			expectStep(expected, follower, c.land_delay);
			EXPECT_FALSE(mode_completed_sub.update());
			return;
		}

		reachStep(follower);
		reported = expected.reports_reached ? expected.index : reported;
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, reported);
	}

	// Touchdown completes Return, with nothing left to estimate.
	EXPECT_EQ(FollowerPeer::missionItem(follower).nav_cmd, NAV_CMD_IDLE);
	EXPECT_FALSE(follower.calc_rtl_time_estimate().valid);
	ASSERT_TRUE(mode_completed_sub.update());
	EXPECT_EQ(mode_completed_sub.get().nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

static const RouteCase kRouteCases[] = {
	// Beside the second leg, flying to item 2: on to the rally point beside the third leg, which branches off at 3.
	{
		"ForwardToRallyPoint", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, 0.f, 30.f, {
			{Step::Join, 200.f, 60.f, kRouteAlt, 2, false},
			{Step::Route, 200.f, 200.f, kRouteAlt, 2, true},
			{Step::BranchOff, 100.f, 200.f, kRouteAlt, 3, false},
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}
	},
	{
		"ForwardToRallyPointWaits", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, 5.f, 30.f, {
			{Step::Join, 200.f, 60.f, kRouteAlt, 2, false},
			{Step::Route, 200.f, 200.f, kRouteAlt, 2, true},
			{Step::BranchOff, 100.f, 200.f, kRouteAlt, 3, false},
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}
	},
	// The wait never climbs: with RTL_DESCEND_ALT above the route, it waits at the route altitude.
	{
		"ForwardToRallyPointHolds", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, -1.f, 100.f, {
			{Step::Join, 200.f, 60.f, kRouteAlt, 2, false},
			{Step::Route, 200.f, 200.f, kRouteAlt, 2, true},
			{Step::BranchOff, 100.f, 200.f, kRouteAlt, 3, false},
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Wait, 100.f, 230.f, kRouteAlt, 3, false},
		}
	},
	// A rally point 20 m above the route: Return climbs to its altitude on the way to the branch-off point.
	{
		"RallyPointAboveTheRoute", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f, kRouteAlt + 20.f)}, 5.f, 30.f, {
			{Step::Join, 200.f, 60.f, kRouteAlt, 2, false},
			{Step::Route, 200.f, 200.f, kRouteAlt, 2, true},
			{Step::BranchOff, 100.f, 200.f, kRouteAlt + 20.f, 3, false},
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt + 20.f, 3, true},
			{Step::Wait, 100.f, 230.f, kRouteAlt + 20.f, 3, false},
			{Step::Land, 100.f, 230.f, kRouteAlt + 20.f, 3, true},
		}
	},
	// Joined on the leg where Return leaves the route: from the branch-in point on toward the branch-off point.
	{
		"JoinOnTheBranchOffLeg", 180.f, 190.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f)}, 0.f, 30.f, {
			{Step::Join, 180.f, 200.f, kRouteAlt, 3, false},
			{Step::BranchOff, 100.f, 200.f, kRouteAlt, 3, false},
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}
	},
	// Beside the third leg: back along the route to the rally point beside the first leg, left at item 0.
	{
		"ReverseToRallyPoint", 100.f, 210.f, kRouteAlt, 3, {rallyPoint(100.f, -30.f)}, 0.f, 30.f, {
			{Step::Join, 100.f, 200.f, kRouteAlt, 2, false},
			{Step::Route, 200.f, 200.f, kRouteAlt, 2, true},
			{Step::Route, 200.f, 0.f, kRouteAlt, 1, true},
			{Step::BranchOff, 100.f, 0.f, kRouteAlt, 0, false},
			{Step::MoveToLand, 100.f, -30.f, kRouteAlt, 0, true},
			{Step::Land, 100.f, -30.f, kHomeAlt, 0, true},
		}
	},
	// 20 m below the first leg without rally points: a climb in place, then back to the takeoff point at Home.
	{
		"ReverseToTakeoffFromBelow", 60.f, 10.f, kRouteAlt - 20.f, 1, {}, 5.f, 30.f, {
			{Step::Climb, 60.f, 10.f, kRouteAlt, 0, false},
			{Step::Join, 60.f, 0.f, kRouteAlt, 0, false},
			{Step::MoveToLand, 0.f, 0.f, kRouteAlt, 0, true},
			{Step::Wait, 0.f, 0.f, kHomeAlt + 30.f, 0, false},
			{Step::Land, 0.f, 0.f, kHomeAlt, 0, true},
		}
	},
	// Beside the third leg without rally points: on to the mission landing, which lands without the delay.
	{
		"MissionLanding", 50.f, 210.f, kRouteAlt, 3, {}, 5.f, 30.f, {
			{Step::Join, 50.f, 200.f, kRouteAlt, 3, false},
			{Step::Route, 0.f, 200.f, kRouteAlt, 3, true},
			{Step::MoveToLand, 0.f, 100.f, kRouteAlt, 4, true},
			{Step::Land, 0.f, 100.f, kHomeAlt, 4, true},
		}
	},
	// On the leg from the branch-off point to the rally point: straight to it. The plan still names item 3.
	{
		"DirectToRallyPoint", 101.f, 215.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f)}, 5.f, 30.f, {
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}
	},
	// Within NAV_ACC_RAD of a rally point beside the corner at item 2: it lands there, although its own leg ends at 2
	// and the rally point leaves the route from the next leg.
	{
		"DirectToRallyPointFromAnotherLeg", 201.f, 198.f, kRouteAlt, 2, {rallyPoint(195.f, 205.f)}, 0.f, 30.f, {
			{Step::Land, 195.f, 205.f, kHomeAlt, 2, true},
		}
	},
	// Already above the rally point: it waits there at once, without reporting an item reached.
	{
		"WaitsAtOnceAboveRallyPoint", 101.f, 226.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f)}, 5.f, 30.f, {
			{Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}
	},
	// With RTL_PLD_MD a precision landing follows the wait. Its touchdown completes Return, it does not wait again.
	{
		"PrecisionLandAfterTheWait", 101.f, 215.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f)}, 5.f, 30.f, {
			{Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, true},
			{Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false},
			{Step::PrecisionLand, 100.f, 230.f, kHomeAlt, 3, true},
			{Step::Land, 100.f, 230.f, kHomeAlt, 3, true},
		}, 1
	},
	// Above the mission landing, during its descent: lands there at once.
	{
		"DirectToMissionLanding", 0.f, 105.f, kHomeAlt + 20.f, 4, {}, 5.f, 30.f, {
			{Step::Land, 0.f, 100.f, kHomeAlt, 4, true},
		}
	},
};

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteStepTest,
	::testing::ValuesIn(kRouteCases),
	[](const ::testing::TestParamInfo<RouteCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: MissionBase restarts a finished mission after a disarm. Return flies its plan instead.
// WHAT: after a disarm while Return is inactive, a Return planned on the last mission item starts there.
TEST_F(RtlRouteTest, DisarmBeforeReturnKeepsThePlannedItem)
{
	RtlMissionSafePointFollow follower{&_navigator};
	// Above the mission landing during its descent: the plan starts at the landing, the last item.
	moveTo(0.f, 105.f, kHomeAlt + 20.f);
	const mission_s mission = loadRoute(squareMission(), {}, 4);
	start(follower, mission);
	follower.run(false);

	// GIVEN: a disarm while Return is inactive, then a new flight
	publishVehicle(false, false);
	follower.run(false);
	publishVehicle(false);

	// WHEN: Return starts again
	start(follower, mission);

	// THEN: it lands at the mission landing, not at the first mission item
	EXPECT_EQ(FollowerPeer::index(follower), 4);
	EXPECT_EQ(FollowerPeer::missionItem(follower).nav_cmd, NAV_CMD_LAND);
}

// WHY: RTL hands the follower its mission when it plans. MissionBase would restart the current item when the mission
// topic changes: a ground station selects an item, a geofence or a mission is uploaded.
// WHAT: during the wait above the rally point, none of them changes the step.
TEST_F(RtlRouteTest, StepIgnoresTheMissionTopic)
{
	setParam("RTL_LAND_DELAY", -1.f);
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(101.f, 215.f, kRouteAlt);
	const mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 3);
	start(follower, mission);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::workItem(follower), WorkItemType::WORK_ITEM_TYPE_LAND_DELAY);
	const position_setpoint_s hold = currentSetpoint();

	mission_s selected = mission;
	selected.current_seq = 1;
	mission_s fence_upload = mission;
	fence_upload.geofence_id = 7;
	mission_s mission_upload = mission;
	mission_upload.mission_id = 77;

	mission_s changes[] {selected, fence_upload, mission_upload};

	for (mission_s &changed : changes) {
		changed.timestamp = hrt_absolute_time();
		publishMission(changed);
		follower.run(true);

		EXPECT_EQ(FollowerPeer::workItem(follower), WorkItemType::WORK_ITEM_TYPE_LAND_DELAY);
		EXPECT_EQ(FollowerPeer::index(follower), 3);
		EXPECT_DOUBLE_EQ(currentSetpoint().lat, hold.lat);
		EXPECT_DOUBLE_EQ(currentSetpoint().lon, hold.lon);
		EXPECT_FLOAT_EQ(currentSetpoint().alt, hold.alt);
	}
}

// WHY: Return follows the route as geometry. Flying a loop again would lead away from the goal and use up its repeats.
// WHAT: after the last item of a loop it goes on to the item after the DO_JUMP, which keeps its repeat count.
TEST_F(RtlRouteTest, RouteSkipsLoopsWithoutCountingThem)
{
	// The square mission with a DO_JUMP after item 2 back to item 1, once.
	std::vector<mission_item_s> items = squareMission();
	items.insert(items.begin() + 3, makeDoJump(1, 1, 0));
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(200.f, 180.f, kRouteAlt);
	const mission_s mission = loadRoute(items, {}, 2);
	start(follower, mission);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::index(follower), 2);

	// WHEN: the vehicle reaches the last item of the loop
	reachStep(follower);

	// THEN: it flies to the item after the jump, without counting a repeat
	EXPECT_EQ(FollowerPeer::index(follower), 4);
	const mission_item_s *jump = _navigator.get_mission_route_cache().missionItem(3);
	ASSERT_NE(jump, nullptr);
	EXPECT_EQ(jump->do_jump_current_count, 0);
}

namespace
{

struct Leg {
	float horizontal_m;
	float vertical_m; ///< positive up
	float wait_s;
};

struct EstimateCase {
	const char *test_name;
	float vehicle_north_m;
	float vehicle_east_m;
	float vehicle_alt;
	int32_t current_seq;
	std::vector<mission_item_s> rally_points;
	float land_delay;
	std::vector<Leg> legs;
	float descend_alt{30.f};
};

} // namespace

// Each leg takes the longer of its horizontal and vertical time, as RtlTimeEstimator counts it.
static float expectedTime(const std::vector<Leg> &legs)
{
	float cruise_speed = NAN;
	float climb_rate = NAN;
	float descent_rate = NAN;
	param_get(param_find("MPC_XY_CRUISE"), &cruise_speed);
	param_get(param_find("MPC_Z_V_AUTO_UP"), &climb_rate);
	param_get(param_find("MPC_Z_V_AUTO_DN"), &descent_rate);
	float time_s = 0.f;

	for (const Leg &leg : legs) {
		const float vertical_s = leg.vertical_m > 0.f ? leg.vertical_m / climb_rate : -leg.vertical_m / descent_rate;
		time_s += math::max(leg.horizontal_m / cruise_speed, vertical_s) + leg.wait_s;
	}

	return time_s;
}

class RtlRouteEstimateTest : public RtlRouteTest, public ::testing::WithParamInterface<EstimateCase> {};

// WHY: the distance-aware return failsafe compares the battery with this estimate before Return starts.
// WHAT: the estimate of each plan counts the climb before the join, the route, the branch-off, the move above the goal,
// RTL_LAND_DELAY and the descent; an unlimited delay ends it at the hold.
TEST_P(RtlRouteEstimateTest, EstimatesThePlannedReturn)
{
	const EstimateCase &c = GetParam();
	setParam("RTL_LAND_DELAY", c.land_delay);
	setParam("RTL_DESCEND_ALT", c.descend_alt);
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(c.vehicle_north_m, c.vehicle_east_m, c.vehicle_alt);
	const mission_s mission = loadRoute(squareMission(), c.rally_points, c.current_seq);
	follower.run(false);
	follower.planRoute(mission);
	ASSERT_TRUE(follower.plan().valid());

	const rtl_time_estimate_s estimate = follower.calc_rtl_time_estimate();

	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, expectedTime(c.legs), 0.1f);
}

// The plans of the step table, with the legs flown from the vehicle: {horizontal, vertical, wait}.
static const EstimateCase kEstimateCases[] = {
	EstimateCase{"ForwardToRallyPoint", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, 0.f, {{140.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -60.f, 0.f}}},
	EstimateCase{"ForwardToRallyPointWaits", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, 5.f, {{140.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -30.f, 5.f}, {0.f, -30.f, 0.f}}},
	EstimateCase{"ForwardToRallyPointHolds", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, -1.f, {{140.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -30.f, 0.f}}},
	// With RTL_DESCEND_ALT above the route, the wait is at the route altitude.
	EstimateCase{"ForwardToRallyPointWaitsAtArrival", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f)}, 5.f, {{140.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 5.f}, {0.f, -60.f, 0.f}}, 100.f},
	EstimateCase{"ReverseToTakeoffFromBelow", 60.f, 10.f, kRouteAlt - 20.f, 1, {}, 5.f, {{0.f, 20.f, 0.f}, {10.f, 0.f, 0.f}, {60.f, 0.f, 0.f}, {0.f, -30.f, 5.f}, {0.f, -30.f, 0.f}}},
	EstimateCase{"MissionLanding", 50.f, 210.f, kRouteAlt, 3, {}, 5.f, {{10.f, 0.f, 0.f}, {50.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {0.f, -60.f, 0.f}}},
	EstimateCase{"DirectToRallyPoint", 101.f, 215.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f)}, 5.f, {{15.03f, 0.f, 0.f}, {0.f, -30.f, 5.f}, {0.f, -30.f, 0.f}}},
	// Above the route: the join leg descends on the way, no separate descent.
	EstimateCase{"JoinFromAbove", 220.f, 60.f, kRouteAlt + 20.f, 2, {rallyPoint(100.f, 230.f)}, 0.f, {{20.f, -20.f, 0.f}, {140.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -60.f, 0.f}}},
	// A rally point 20 m above the route: the climb on the way to the branch-off point.
	EstimateCase{"RallyPointAboveTheRoute", 200.f, 60.f, kRouteAlt, 2, {rallyPoint(100.f, 230.f, kRouteAlt + 20.f)}, 0.f, {{140.f, 0.f, 0.f}, {100.f, 20.f, 0.f}, {30.f, 0.f, 0.f}}},
	// Next to a rally point that is 20 m above the vehicle: the landing adds no climb.
	EstimateCase{"BelowARallyPointAboveGround", 101.f, 226.f, kRouteAlt, 3, {rallyPoint(100.f, 230.f, kRouteAlt + 20.f)}, 0.f, {{4.12f, 0.f, 0.f}}},
};

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteEstimateTest,
	::testing::ValuesIn(kEstimateCases),
	[](const ::testing::TestParamInfo<EstimateCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: during Return the estimate must shrink with the flown steps, or the failsafe would act on legs already flown.
// WHAT: on the way to the branch-off point, after 3 s of a 5 s wait above the rally point, and in the descent.
TEST_F(RtlRouteTest, EstimateCountsOnlyTheRemainingSteps)
{
	setParam("RTL_LAND_DELAY", 5.f);
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(200.f, 60.f, kRouteAlt);
	start(follower, loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2));
	reachStep(follower);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::workItem(follower), WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF);

	const float to_branch_off_s = expectedTime({{100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -30.f, 5.f}, {0.f, -30.f, 0.f}});
	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, to_branch_off_s, 0.1f);

	reachStep(follower);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::workItem(follower), WorkItemType::WORK_ITEM_TYPE_LAND_DELAY);
	moveTo(100.f, 230.f, kHomeAlt + 30.f);
	follower.run(true);
	FollowerPeer::ageWaypointReached(follower, 3.f);

	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, expectedTime({{0.f, -30.f, 2.f}}), 0.1f);

	reachStep(follower);
	ASSERT_EQ(FollowerPeer::missionItem(follower).nav_cmd, NAV_CMD_LAND);

	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, expectedTime({{0.f, -30.f, 0.f}}), 0.1f);
}

// WHY: during Return the estimate walks from the route item the vehicle flies toward, not from the start of the plan.
// WHAT: on a route flown backwards, past its first item: no join leg and no leg already flown.
TEST_F(RtlRouteTest, EstimateWalksFromTheCurrentRouteItem)
{
	setParam("RTL_LAND_DELAY", 0.f);
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(100.f, 210.f, kRouteAlt);
	start(follower, loadRoute(squareMission(), {rallyPoint(100.f, -30.f)}, 3));
	reachStep(follower);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::index(follower), 1);

	// Halfway along the leg to item 1.
	moveTo(200.f, 100.f, kRouteAlt);

	const float remaining_s = expectedTime({{100.f, 0.f, 0.f}, {100.f, 0.f, 0.f}, {30.f, 0.f, 0.f}, {0.f, -60.f, 0.f}});
	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, remaining_s, 0.1f);
}

// WHY: Return flies at the default cruise speed, which its time estimate assumes, and does not restart a survey camera.
// WHAT: a speed change and a camera trigger before the first route item are not sent again, also not past that item,
// and the first setpoint already has the default speed.
TEST_F(RtlRouteTest, ReturnReplaysNoMissionItem)
{
	// The square mission with a speed change to 2 m/s and a camera trigger every 10 m after item 1.
	std::vector<mission_item_s> items = squareMission();
	mission_item_s speed{};
	speed.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	speed.params[0] = 1.f;
	speed.params[1] = 2.f;
	speed.params[2] = -1.f;
	speed.autocontinue = true;
	mission_item_s camera{};
	camera.nav_cmd = NAV_CMD_DO_SET_CAM_TRIGG_DIST;
	camera.params[0] = 10.f;
	camera.autocontinue = true;
	items.insert(items.begin() + 2, {speed, camera});
	RtlMissionSafePointFollow follower{&_navigator};
	moveTo(100.f, 210.f, kRouteAlt);
	const mission_s mission = loadRoute(items, {}, 5);

	// GIVEN: the mission speed in force, and the commands Mission sent before
	_navigator.set_cruising_speed(2.f);
	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s vehicle_command{};

	while (vehicle_command_sub.update(&vehicle_command)) {}

	// WHEN: Return starts beside the last leg and flies past its first route item, item 5
	start(follower, mission);
	EXPECT_LT(currentSetpoint().cruising_speed, 0.f);
	reachStep(follower);
	reachStep(follower);
	ASSERT_EQ(FollowerPeer::index(follower), 6);
	follower.run(true);

	// THEN: neither command is sent, and the default speed stays
	bool replayed = false;

	while (vehicle_command_sub.update(&vehicle_command)) {
		replayed |= (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED)
			    || (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST);
	}

	EXPECT_FALSE(replayed);
	EXPECT_LT(_navigator.get_cruising_speed(), 0.f);
}

// Return with RTL_TYPE 7 on a multicopter.
class RtlRouteReturnTest : public RtlRouteTest
{
protected:
	void SetUp() override
	{
		RtlRouteTest::SetUp();
		setParam("RTL_TYPE", int32_t{7});
		_rtl.refreshParametersForTest();
	}

	const rtl_status_s &status()
	{
		_rtl_status_sub.update();
		return _rtl_status_sub.get();
	}

	void loadFence()
	{
		Geofence &fence = _navigator.get_geofence();
		const hrt_abstime start_time = hrt_absolute_time();

		while (!fence.isReadyForPathChecks() && hrt_elapsed_time(&start_time) < 1_s) {
			fence.run();
			px4_usleep(1000);
		}

		ASSERT_TRUE(fence.isReadyForPathChecks());
	}

	// The same items in a Dataman slot, published as a new upload.
	mission_s uploadCopy(const mission_s &mission, uint32_t mission_id, dm_item_t dataman_id)
	{
		const std::vector<mission_item_s> items = squareMission();

		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			EXPECT_TRUE(_dataman_client.writeSync(dataman_id, i, reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		mission_s copy = mission;
		copy.timestamp = hrt_absolute_time();
		copy.mission_id = mission_id;
		copy.mission_dataman_id = dataman_id;
		publishMission(copy);
		return copy;
	}

	// New rally points in Dataman, published with the mission.
	mission_s uploadRallyPoints(mission_s mission, uint32_t safe_points_id)
	{
		writeRallyPoints({rallyPoint(100.f, 230.f)}, safe_points_id);
		mission.timestamp = hrt_absolute_time();
		mission.safe_points_id = safe_points_id;
		publishMission(mission);
		return mission;
	}

	void reachStep() { RtlRouteTest::reachStep(_rtl.follower(), [&] { _rtl.run(true); }); }

	void expectHold()
	{
		EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_NONE);
		EXPECT_EQ(status().safe_point_index, UINT8_MAX);
		EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_LOITER_UNLIMITED);
	}

	RTLTestPeer _rtl{&_navigator};
	uORB::SubscriptionData<rtl_status_s> _rtl_status_sub{ORB_ID(rtl_status)};
};

namespace
{

struct ActivationCase {
	const char *test_name;
	float vehicle_north_m;
	float vehicle_east_m;
	int32_t current_seq;
	std::vector<mission_item_s> rally_points;
	ExpectedStep first_step;
	uint8_t safe_point_index;
	float mis_mc_seg_dist{30.f};
	float rtl_rp_seg_dist{30.f};
};

} // namespace

class RtlRouteActivationTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<ActivationCase> {};

// WHY: RTL decides which Return runs, the follower which steps it flies. It plans from Mission's current item with the
// search margin of the vehicle and the one of the rally points.
// WHAT: Return creates the follower, plans and flies the first step, and reports the route and its rally point.
TEST_P(RtlRouteActivationTest, ActivationFliesTheRoute)
{
	const ActivationCase &c = GetParam();
	setParam("MIS_MC_SEG_DIST", c.mis_mc_seg_dist);
	setParam("RTL_RP_SEG_DIST", c.rtl_rp_seg_dist);
	moveTo(c.vehicle_north_m, c.vehicle_east_m, kRouteAlt);
	loadRoute(squareMission(), c.rally_points, c.current_seq);

	_rtl.run(true);

	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT);
	EXPECT_EQ(status().safe_point_index, c.safe_point_index);
	expectStep(c.first_step, _rtl.follower());
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteActivationTest,
	::testing::Values(
		ActivationCase{"RouteToRallyPoint", 200.f, 60.f, 2, {rallyPoint(100.f, 230.f)}, {Step::Join, 200.f, 60.f, kRouteAlt, 2, false}, 0},
		ActivationCase{"DirectToRallyPoint", 101.f, 215.f, 3, {rallyPoint(100.f, 230.f)}, {Step::MoveToLand, 100.f, 230.f, kRouteAlt, 3, false}, 0},
		ActivationCase{"RouteToMissionLanding", 50.f, 210.f, 3, {}, {Step::Join, 50.f, 200.f, kRouteAlt, 3, false}, UINT8_MAX},
		// 30 m beside the first leg, which Mission flew, and 20 m beside the second: the first is within MIS_MC_SEG_DIST.
		ActivationCase{"VehicleMarginKeepsTheCurrentLeg", 180.f, 30.f, 1, {rallyPoint(100.f, 230.f)}, {Step::Join, 180.f, 0.f, kRouteAlt, 1, false}, 0, 50.f, 0.f},
		// The same place while Mission flew the second leg: it joins that one.
		ActivationCase{"CurrentItemPicksTheLeg", 180.f, 30.f, 2, {rallyPoint(100.f, 230.f)}, {Step::Join, 200.f, 30.f, kRouteAlt, 2, false}, 0, 50.f, 0.f},
		// A rally point 80 m from the third leg and 120 m from the first, next to the vehicle: within RTL_RP_SEG_DIST it
		// leaves the first leg, here at once.
		ActivationCase{"RallyMarginReachesANearLeg", 100.f, 10.f, 1, {rallyPoint(100.f, 120.f)}, {Step::MoveToLand, 100.f, 120.f, kRouteAlt, 1, false}, 0, 0.f, 50.f}),
	[](const ::testing::TestParamInfo<ActivationCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: without a route Return holds. It neither flies direct, nor tells an external mode executor that Return is done.
// WHAT: without a mission, Return holds where the vehicle is and reports why, without the mission alarm.
TEST_F(RtlRouteReturnTest, WithoutARouteHoldsWithoutCompletingReturn)
{
	moveTo(200.f, 60.f, kRouteAlt);
	mission_s no_mission{};
	no_mission.timestamp = hrt_absolute_time();
	publishMission(no_mission);
	navigator_test::EventRecorder route_events;
	navigator_test::EventRecorder mission_events;
	uORB::SubscriptionData<mode_completed_s> mode_completed_sub{ORB_ID(mode_completed)};
	(void)mode_completed_sub.update();

	_rtl.run(true);

	expectHold();
	const position_setpoint_s &hold = currentSetpoint();
	float north_m = NAN;
	float east_m = NAN;
	offsetOf(hold.lat, hold.lon, north_m, east_m);
	EXPECT_TRUE(hold.valid);
	EXPECT_NEAR(north_m, 200.f, 0.1f);
	EXPECT_NEAR(east_m, 60.f, 0.1f);
	EXPECT_NEAR(hold.alt, kRouteAlt, 0.1f);
	EXPECT_TRUE(route_events.sent(events::ID("rtl_route_unavailable")));
	EXPECT_FALSE(mission_events.sent(events::ID("mission_not_valid_loiter")));
	EXPECT_FALSE(mode_completed_sub.update());
}

namespace
{

enum class MissingInput : uint8_t { MissionCheck, CheckWithThisFence, MissionInRouteCache, RallyPointsLoaded, RallyPointsSeen, Geofence };

struct HoldCase {
	const char *test_name;
	MissingInput missing;
};

} // namespace

class RtlRouteHoldTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<HoldCase> {};

// WHY: RTL plans only on a checked mission with its items, rally points and fence loaded, and retries every 2 s.
// WHAT: Return holds while one input is missing, and flies the route at the first retry after it arrives.
TEST_P(RtlRouteHoldTest, HoldResumesOnceTheRouteCanBePlanned)
{
	// 20 m beside the route, so the join point is not reached in the cycle that plans it.
	moveTo(220.f, 60.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);

	// GIVEN: one input of the plan missing
	switch (GetParam().missing) {
	case MissingInput::MissionCheck:
		_navigator.get_mission_result()->valid = false;
		break;

	case MissingInput::CheckWithThisFence:
		mission.geofence_id = 5;
		publishMission(mission);
		break;

	case MissingInput::MissionInRouteCache:
		// The route cache still holds the mission before this upload.
		mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_1);
		markMissionChecked(mission);
		break;

	case MissingInput::RallyPointsLoaded:
		mission = uploadRallyPoints(mission, 33);
		_navigator.get_mission_route_cache().update(mission);
		break;

	case MissingInput::RallyPointsSeen:
		mission = uploadRallyPoints(mission, 33);
		break;

	case MissingInput::Geofence:
		_navigator.get_geofence().updateFence();
		break;
	}

	// WHEN: Return starts
	_rtl.run(true);

	// THEN: it holds, and tells while rally points load
	expectHold();
	EXPECT_EQ(status().is_evaluation_pending, GetParam().missing == MissingInput::RallyPointsLoaded);

	// WHEN: the input arrives and the retry is due
	switch (GetParam().missing) {
	case MissingInput::MissionCheck:
	case MissingInput::CheckWithThisFence:
		markMissionChecked(mission);
		break;

	case MissingInput::MissionInRouteCache:
	case MissingInput::RallyPointsLoaded:
	case MissingInput::RallyPointsSeen:
		loadRouteCache(mission);
		break;

	case MissingInput::Geofence:
		loadFence();
		break;
	}

	// Not before the retry is due.
	_rtl.startRetryIntervalForTest();
	_rtl.run(true);
	expectHold();

	navigator_test::EventRecorder events;
	_rtl.expireRetryIntervalForTest();
	_rtl.run(true);

	// THEN: it flies the route to the rally point
	EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT);
	EXPECT_EQ(status().safe_point_index, 0);
	EXPECT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(events.sent(events::ID("rtl_route_found")));
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteHoldTest,
	::testing::Values(
		HoldCase{"MissionNotChecked", MissingInput::MissionCheck},
		HoldCase{"MissionCheckedWithOtherFence", MissingInput::CheckWithThisFence},
		HoldCase{"MissionNotInRouteCache", MissingInput::MissionInRouteCache},
		HoldCase{"RallyPointsLoading", MissingInput::RallyPointsLoaded},
		HoldCase{"RallyPointsNotSeenByRouteCache", MissingInput::RallyPointsSeen},
		HoldCase{"GeofenceLoading", MissingInput::Geofence}),
	[](const ::testing::TestParamInfo<HoldCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: a hold that moved at each failed plan would drift with the vehicle.
// WHAT: neither a failed retry nor an upload that brings no route moves the hold point.
TEST_F(RtlRouteReturnTest, HoldKeepsItsPointUntilARouteIsFound)
{
	moveTo(200.f, 60.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_navigator.get_mission_result()->valid = false;
	_rtl.run(true);
	const position_setpoint_s hold = currentSetpoint();

	// GIVEN: the vehicle drifts from the hold point
	moveTo(210.f, 70.f, kRouteAlt - 2.f);

	// WHEN: a retry fails, then rally points arrive while the mission is still unchecked
	_rtl.expireRetryIntervalForTest();
	_rtl.run(true);
	mission = uploadRallyPoints(mission, 34);
	loadRouteCache(mission);
	_rtl.run(true);

	// THEN: it still holds at the first point
	expectHold();
	EXPECT_DOUBLE_EQ(currentSetpoint().lat, hold.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, hold.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, hold.alt);
}

// WHY: Return never takes off again on its own.
// WHAT: after touchdown during the hold, a route that becomes available is not planned.
TEST_F(RtlRouteReturnTest, HoldStopsRetryingOnceLanded)
{
	moveTo(200.f, 60.f, kRouteAlt);
	const mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_navigator.get_mission_result()->valid = false;
	_rtl.run(true);

	publishVehicle(true);
	markMissionChecked(mission);
	_rtl.expireRetryIntervalForTest();
	_rtl.run(true);

	EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_NONE);
	EXPECT_FALSE(_rtl.follower().plan().valid());
}

namespace
{

enum class Upload : uint8_t { Mission, SameMissionOtherSlot, RallyPoints, Geofence };

struct UploadCase {
	const char *test_name;
	Upload upload;
};

} // namespace

class RtlRouteUploadTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<UploadCase> {};

// WHY: the route is planned on one mission, rally points and fence. A new one makes the route outdated.
// WHAT: an upload while Return follows the route replans at once. It flies the new route, or holds while the new
// geofence is not yet checked against the mission.
TEST_P(RtlRouteUploadTest, UploadDuringTheRouteReplans)
{
	moveTo(200.f, 60.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	ASSERT_EQ(FollowerPeer::index(_rtl.follower()), 2);

	// WHEN: 20 m beside the route, an upload, loaded and checked unless it is a geofence
	moveTo(220.f, 100.f, kRouteAlt);

	switch (GetParam().upload) {
	case Upload::Mission:
		mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_0);
		break;

	case Upload::SameMissionOtherSlot:
		mission = uploadCopy(mission, 31, DM_KEY_WAYPOINTS_OFFBOARD_1);
		break;

	case Upload::RallyPoints:
		mission = uploadRallyPoints(mission, 33);
		break;

	case Upload::Geofence:
		mission.geofence_id = 5;
		publishMission(mission);
		break;
	}

	if (GetParam().upload != Upload::Geofence) {
		loadRouteCache(mission);
		markMissionChecked(mission);
	}

	navigator_test::EventRecorder events;
	_rtl.run(true);
	// "route found" only ends a hold.
	EXPECT_FALSE(events.sent(events::ID("rtl_route_found")));

	// THEN: Return starts again on the new route, or holds where the vehicle is
	if (GetParam().upload == Upload::Geofence) {
		expectHold();
		float north_m = NAN;
		float east_m = NAN;
		offsetOf(currentSetpoint().lat, currentSetpoint().lon, north_m, east_m);
		EXPECT_NEAR(north_m, 220.f, 0.1f);
		EXPECT_NEAR(east_m, 100.f, 0.1f);

	} else {
		EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT);
		EXPECT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		EXPECT_EQ(FollowerPeer::missionId(_rtl.follower()), mission.mission_id);
		EXPECT_TRUE(_rtl.follower().plannedOn(mission));
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteUploadTest,
	::testing::Values(
		UploadCase{"Mission", Upload::Mission},
		UploadCase{"SameMissionToOtherSlot", Upload::SameMissionOtherSlot},
		UploadCase{"RallyPoints", Upload::RallyPoints},
		UploadCase{"Geofence", Upload::Geofence}),
	[](const ::testing::TestParamInfo<UploadCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

namespace
{

struct LandingCase {
	const char *test_name;
	float vehicle_north_m;
	float vehicle_east_m;
	int32_t current_seq;
	std::vector<mission_item_s> rally_points;
	float land_delay;
	int32_t precision_land;
	float landing_yaw;
	FollowerPeer::WorkItemType step;
	uint16_t nav_cmd;
};

} // namespace

class RtlRouteLandingTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<LandingCase> {};

// WHY: from the move above the goal on, new rally points no longer change the Return: it lands where it planned.
// WHAT: new rally points during each landing step do not replan.
TEST_P(RtlRouteLandingTest, RallyPointUploadDuringTheLandingKeepsIt)
{
	const LandingCase &c = GetParam();
	setParam("RTL_LAND_DELAY", c.land_delay);
	setParam("RTL_PLD_MD", c.precision_land);
	std::vector<mission_item_s> items = squareMission();
	items[4].yaw = c.landing_yaw;
	moveTo(c.vehicle_north_m, c.vehicle_east_m, kRouteAlt);
	mission_s mission = loadRoute(items, c.rally_points, c.current_seq);
	_rtl.run(true);

	for (int step = 0; (step < 10) && !((FollowerPeer::workItem(_rtl.follower()) == c.step)
					    && (FollowerPeer::missionItem(_rtl.follower()).nav_cmd == c.nav_cmd)); ++step) {
		reachStep();
	}

	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), c.step);
	ASSERT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, c.nav_cmd);
	const int32_t index = FollowerPeer::index(_rtl.follower());

	mission = uploadRallyPoints(mission, 35);
	loadRouteCache(mission);
	_rtl.run(true);

	EXPECT_EQ(FollowerPeer::workItem(_rtl.follower()), c.step);
	EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, c.nav_cmd);
	EXPECT_EQ(FollowerPeer::index(_rtl.follower()), index);
	EXPECT_FALSE(_rtl.follower().plannedOn(mission));
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteLandingTest,
	::testing::Values(
		LandingCase{"MoveToRallyPoint", 200.f, 60.f, 2, {rallyPoint(100.f, 230.f)}, 0.f, 0, NAN, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND, NAV_CMD_WAYPOINT},
		LandingCase{"WaitAboveRallyPoint", 200.f, 60.f, 2, {rallyPoint(100.f, 230.f)}, 5.f, 0, NAN, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_LAND_DELAY, NAV_CMD_LOITER_TIME_LIMIT},
		LandingCase{"LandAtRallyPoint", 200.f, 60.f, 2, {rallyPoint(100.f, 230.f)}, 0.f, 0, NAN, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT, NAV_CMD_LAND},
		LandingCase{"PrecisionLandAtRallyPoint", 200.f, 60.f, 2, {rallyPoint(100.f, 230.f)}, 0.f, 1, NAN, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND, NAV_CMD_LAND},
		// A mission landing with a heading turns to it above the landing point, here east while the vehicle faces north.
		LandingCase{"AlignAboveMissionLanding", 50.f, 210.f, 3, {}, 0.f, 0, 1.5f, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND, NAV_CMD_WAYPOINT}),
	[](const ::testing::TestParamInfo<LandingCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: a Return that has its route keeps it. Replanning it every 2 s would start each time with a new join.
// WHAT: without an upload, the retry interval passing does not restart the route.
TEST_F(RtlRouteReturnTest, RouteIsNotReplannedWithoutAnUpload)
{
	moveTo(220.f, 60.f, kRouteAlt);
	loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	moveTo(220.f, 100.f, kRouteAlt);

	_rtl.expireRetryIntervalForTest();
	_rtl.run(true);

	EXPECT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(FollowerPeer::index(_rtl.follower()), 2);
}

namespace
{

enum class MissionChange : uint8_t { Upload, Clear };

struct GoalStepCase {
	const char *test_name;
	float land_delay;
	FollowerPeer::WorkItemType step;
	MissionChange change;
	ExpectedStep after; ///< the step after the replan, unless it holds
	bool holds;
};

} // namespace

class RtlRouteGoalStepTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<GoalStepCase> {};

// WHY: from the move above the goal on, Return does not replan, but a new or cleared mission makes the planned one
// invalid. Return ends the step, reports itself complete, then plans on the new mission (a documented limit).
// WHAT: a mission upload during the move or the wait, a mission clear during the wait: the step ends, Return reports
// completion once, then flies the new route from above the goal or holds without one.
TEST_P(RtlRouteGoalStepTest, MissionChangeEndsTheStepThenReplans)
{
	const GoalStepCase &c = GetParam();
	setParam("RTL_LAND_DELAY", c.land_delay);
	moveTo(101.f, 215.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 3);
	uORB::SubscriptionData<mode_completed_s> mode_completed_sub{ORB_ID(mode_completed)};
	(void)mode_completed_sub.update();
	_rtl.run(true);

	for (int step = 0; (step < 5) && (FollowerPeer::workItem(_rtl.follower()) != c.step); ++step) {
		reachStep();
	}

	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), c.step);

	// GIVEN: a new mission, loaded and checked, or a cleared one
	if (c.change == MissionChange::Upload) {
		mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_1);
		loadRouteCache(mission);
		markMissionChecked(mission);

	} else {
		mission_s cleared{};
		cleared.timestamp = hrt_absolute_time();
		cleared.land_start_index = -1;
		cleared.land_index = -1;
		cleared.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;
		publishMission(cleared);
		*_navigator.get_mission_result() = {};
	}

	navigator_test::EventRecorder events;
	_rtl.run(true);
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), c.step);
	ASSERT_EQ(FollowerPeer::missionId(_rtl.follower()), 31u);

	// WHEN: the step ends
	reachStep();

	// THEN: Return reports itself complete
	ASSERT_TRUE(mode_completed_sub.update());
	EXPECT_EQ(mode_completed_sub.get().nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	EXPECT_TRUE(_navigator.get_mission_result()->finished);

	// THEN: and in the next cycle flies the new route from above the goal, or holds there
	_rtl.run(true);

	if (c.holds) {
		expectHold();
		EXPECT_TRUE(events.sent(events::ID("rtl_route_unavailable")));

	} else {
		EXPECT_EQ(FollowerPeer::missionId(_rtl.follower()), 32u);
		EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT);
		expectStep(c.after, _rtl.follower(), c.land_delay);
	}

	EXPECT_FALSE(mode_completed_sub.update());
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteGoalStepTest,
	::testing::Values(
		GoalStepCase{"MissionUploadDuringTheMove", 0.f, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND, MissionChange::Upload, {Step::Land, 100.f, 230.f, kHomeAlt, 3, true}, false},
		GoalStepCase{"MissionUploadDuringTheWait", 5.f, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_LAND_DELAY, MissionChange::Upload, {Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false}, false},
		GoalStepCase{"MissionClearDuringTheWait", 5.f, FollowerPeer::WorkItemType::WORK_ITEM_TYPE_LAND_DELAY, MissionChange::Clear, {}, true}),
	[](const ::testing::TestParamInfo<GoalStepCase> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: Return never takes off again on its own, and a ground station may upload after the landing.
// WHAT: after touchdown, a new mission, loaded and checked, does not replan.
TEST_F(RtlRouteReturnTest, UploadAfterTouchdownDoesNotReplan)
{
	setParam("RTL_LAND_DELAY", 0.f);
	moveTo(101.f, 226.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 3);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_IDLE);
	navigator_test::EventRecorder events;

	mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_1);
	loadRouteCache(mission);
	markMissionChecked(mission);
	_rtl.run(true);

	EXPECT_EQ(FollowerPeer::missionId(_rtl.follower()), 31u);
	EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_IDLE);
	EXPECT_FALSE(events.sent(events::ID("rtl_route_unavailable")));
}

// WHY: Return never takes off. On the ground it stays idle, as the direct Return types do, without an alarm.
// WHAT: Return started on the ground, without a route and with one: idle, no message, Return not completed.
TEST_F(RtlRouteReturnTest, ReturnOnTheGroundStaysThere)
{
	publishVehicle(true);
	moveTo(0.f, 0.f, kHomeAlt);
	mission_s no_mission{};
	no_mission.timestamp = hrt_absolute_time();
	publishMission(no_mission);
	uORB::SubscriptionData<mode_completed_s> mode_completed_sub{ORB_ID(mode_completed)};
	(void)mode_completed_sub.update();
	navigator_test::EventRecorder events;

	_rtl.run(true);

	EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_IDLE);
	EXPECT_FALSE(events.sent(events::ID("rtl_route_unavailable")));

	// With a route
	_rtl.run(false);
	loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 0);
	_rtl.run(true);

	EXPECT_TRUE(_rtl.follower().plan().valid());
	EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_IDLE);
	EXPECT_FALSE(events.sent(events::ID("mission_not_valid_refuse")));
	EXPECT_FALSE(mode_completed_sub.update());
}

// WHY: the branch-off point is still on the route, so an upload there replans at once, as on the route.
// WHAT: a new mission during the branch-off step restarts Return on it.
TEST_F(RtlRouteReturnTest, UploadDuringTheBranchOffReplans)
{
	moveTo(200.f, 60.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_rtl.run(true);
	reachStep();
	reachStep();
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF);

	moveTo(150.f, 200.f, kRouteAlt);
	mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_1);
	loadRouteCache(mission);
	markMissionChecked(mission);
	_rtl.run(true);

	EXPECT_EQ(FollowerPeer::missionId(_rtl.follower()), 32u);
	EXPECT_TRUE(_rtl.follower().plannedOn(mission));
}

// WHY: RTL plans before the follower runs. Otherwise the follower would first advance on the old mission and publish it
// over the upload.
// WHAT: an upload in the cycle the vehicle reaches a route item: the mission topic keeps the upload.
TEST_F(RtlRouteReturnTest, UploadWhileReachingARouteItemKeepsTheUpload)
{
	moveTo(200.f, 60.f, kRouteAlt);
	mission_s mission = loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::index(_rtl.follower()), 2);

	// GIVEN: the vehicle at item 2, and a new mission with the same current item
	moveTo(200.f, 200.f, kRouteAlt);
	mission = uploadCopy(mission, 32, DM_KEY_WAYPOINTS_OFFBOARD_1);
	loadRouteCache(mission);
	markMissionChecked(mission);

	// WHEN: RTL runs
	_rtl.run(true);

	// THEN: the mission topic names the upload
	uORB::SubscriptionData<mission_s> mission_sub{ORB_ID(mission)};
	mission_sub.update();
	EXPECT_EQ(mission_sub.get().mission_id, 32u);
}

// WHY: a pause leaves the route. Return plans again from where the vehicle is when it continues.
// WHAT: paused on the route and moved 20 m aside, Return joins the route again beside the vehicle.
TEST_F(RtlRouteReturnTest, ResumeAfterAPauseJoinsTheRouteAgain)
{
	moveTo(200.f, 60.f, kRouteAlt);
	loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_DEFAULT);

	_rtl.run(false);
	moveTo(220.f, 120.f, kRouteAlt);
	_rtl.run(true);

	expectStep({Step::Join, 200.f, 120.f, kRouteAlt, 2, false}, _rtl.follower());
}

// WHY: a pause ends the wait. Return plans again from above the goal when it continues (a documented limit).
// WHAT: paused after 3 s of a 5 s wait, Return waits the whole delay again, then lands.
TEST_F(RtlRouteReturnTest, ResumeDuringTheWaitWaitsAgain)
{
	setParam("RTL_LAND_DELAY", 5.f);
	moveTo(101.f, 215.f, kRouteAlt);
	loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 3);
	_rtl.run(true);
	reachStep();
	ASSERT_EQ(FollowerPeer::workItem(_rtl.follower()), WorkItemType::WORK_ITEM_TYPE_LAND_DELAY);
	moveTo(100.f, 230.f, kHomeAlt + 30.f);
	_rtl.run(true);
	FollowerPeer::ageWaypointReached(_rtl.follower(), 3.f);

	_rtl.run(false);
	_rtl.run(true);

	expectStep({Step::Wait, 100.f, 230.f, kHomeAlt + 30.f, 3, false}, _rtl.follower(), 5.f);
	const float wait_and_landing_s = expectedTime({{0.f, 0.f, 5.f}, {0.f, -30.f, 0.f}});
	EXPECT_NEAR(_rtl.follower().calc_rtl_time_estimate().time_estimate, wait_and_landing_s, 0.1f);
	reachStep();
	EXPECT_EQ(FollowerPeer::missionItem(_rtl.follower()).nav_cmd, NAV_CMD_LAND);
}

namespace
{

struct VehicleCase {
	const char *test_name;
	uint8_t vehicle_type;
	bool is_vtol;
};

} // namespace

class RtlRouteVehicleTest : public RtlRouteReturnTest, public ::testing::WithParamInterface<VehicleCase> {};

// WHY: the route Return is for multicopters only. Other vehicles keep the selection of main for an unknown type.
// WHAT: RTL_TYPE 7 selects as RTL_TYPE 0: here the closer rally point, flown directly.
TEST_P(RtlRouteVehicleTest, OtherVehiclesReturnAsWithType0)
{
	moveTo(200.f, 60.f, kRouteAlt);
	loadRoute(squareMission(), {rallyPoint(100.f, 230.f)}, 2);
	publishVehicle(false, true, GetParam().vehicle_type, GetParam().is_vtol);

	for (const int32_t rtl_type : {0, 7}) {
		SCOPED_TRACE(::testing::Message() << "RTL_TYPE " << rtl_type);
		setParam("RTL_TYPE", rtl_type);
		_rtl.refreshParametersForTest();
		_rtl.evaluateForTest();

		EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
		EXPECT_EQ(status().rtl_type, rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT);
		EXPECT_EQ(status().safe_point_index, 0);
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RtlRouteVehicleTest,
	::testing::Values(
		VehicleCase{"FixedWing", vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false},
		VehicleCase{"VtolInHover", vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true},
		VehicleCase{"VtolInFixedWing", vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true},
		VehicleCase{"Rover", vehicle_status_s::VEHICLE_TYPE_ROVER, false}),
	[](const ::testing::TestParamInfo<VehicleCase> &param_info)
{
	return std::string(param_info.param.test_name);
});
