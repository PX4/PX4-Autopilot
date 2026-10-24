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
 * Tests for the route-following Return executor (RTL_TYPE 7).
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "navigator.h"
#include "mission_route_planner.h"
#include "rtl_mission_safe_point_follow.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mission.h>
#include <uORB/topics/mode_completed.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include <tuple>
#include <vector>

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

/** Test peer with vector-backed mission items and access to the executor state. */
class RtlMissionSafePointFollowTestPeer : public RtlMissionSafePointFollow
{
public:
	using Stage = RtlMissionSafePointFollow::Stage;
	using VtolTransitionAction = mission_route::VtolTransitionAction;
	using WorkItemType = MissionBase::WorkItemType;

	explicit RtlMissionSafePointFollowTestPeer(Navigator *navigator = nullptr) : RtlMissionSafePointFollow(navigator, mission_s{}) {}

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _use_real_route_cache ? RtlMissionSafePointFollow::loadMissionItemFromCache(index, mission_item)
		       : _mission_store.loadItem(index, mission_item);
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		_mission = {};
		_mission.count = static_cast<int32_t>(items.size());
		_mission.current_seq = 0;
		_state = {};
		_arrival.reset();
		_plan = {};
		resetJoinRouteState();
		resetVtolTransition();
		_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
	}

	void configurePlanForTest(const mission_route::RtlRoutePlan &plan, int32_t land_index = -1)
	{
		_mission.land_index = land_index;
		configureRoute(plan, {});
	}

	const mission_item_s &currentMissionItemForTest() const { return _mission_item; }

	bool joiningRouteForTest() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE
		       || _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN;
	}

	void useRealRouteCacheForTest() { _use_real_route_cache = true; }
	bool reloadCurrentMissionItemForTest() { return loadCurrentMissionItem(); }
	void reloadAndPublishMissionItemsForTest() { set_mission_items(); }

	void setStageForTest(Stage stage) { _state.stage = stage; }
	Stage stageForTest() const { return _state.stage; }

	RtlArrival::Phase arrivalPhaseForTest() const { return _arrival.phase(); }

	void setArrivalPhaseForTest(RtlArrival::Phase phase)
	{
		_state.stage = Stage::Arrival;
		_arrival.start(phase);
	}

	void setCurrentSequenceForTest(int32_t index) { _mission.current_seq = index; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	uint32_t missionIdForTest() const { return _mission.mission_id; }

	void prepareActiveMissionForTest(uint32_t mission_id, int32_t current_seq)
	{
		_mission.timestamp = hrt_absolute_time();
		_mission.mission_id = mission_id;
		_mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		_mission.current_seq = current_seq;
		_is_current_planned_mission_item_valid = true;
		_mission_type = MissionType::MISSION_TYPE_MISSION;
		_mission_store.loadItem(current_seq, _mission_item);
		_navigator->get_mission_result()->valid = true;
		_navigator->get_mission_result()->mission_id = mission_id;
	}

	void publishMissionUpdateForTest(uint32_t mission_id, int32_t current_seq)
	{
		mission_s mission = _mission;
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = mission_id;
		mission.current_seq = current_seq;
		_mission_pub_for_test.publish(mission);
	}

	void setDisarmedSinceActivationForTest()
	{
		_mission_has_been_activated = true;
		_system_disarmed_while_inactive = true;
	}

	void setSafePointSelectionForTest(bool direction_reversed, int32_t branch_off_index)
	{
		_plan = {};
		_plan.goal_type = mission_route::GoalType::kSafePoint;
		_plan.safe_point_index = 0;
		_plan.direction_reversed = direction_reversed;
		_plan.first_mission_item_index = branch_off_index;
		_plan.branch_off_mission_item_index = branch_off_index;
	}

	void setSafePointGeometryForTest(const mission_route::Position &branch_off_projection,
					 const mission_route::Position &goal_position)
	{
		_plan.branch_off_position = branch_off_projection;
		_plan.goal_position = goal_position;
	}

	void setAdvanceAfterTransitionForTest(bool advance) { _state.advance_route_after_transition = advance; }
	bool transitionActiveForTest() const { return vtolTransitionActive(); }
	WorkItemType workItemForTest() const { return _work_item_type; }

	void setJoinTransitionForTest() { setupJoinRoute(_plan.join_position, false, VtolTransitionAction::kFrontTransition); }

	void setVtolFailureForTest(bool failed)
	{
		vtol_vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.fixed_wing_system_failure = failed;
		uORB::Publication<vtol_vehicle_status_s> publisher{ORB_ID(vtol_vehicle_status)};
		publisher.publish(status);
		_vtol_status_sub.update();
	}

	void setVehicleStatusForTest(bool is_vtol, bool is_fixed_wing, bool in_transition_to_fw, bool in_back_transition = false)
	{
		vehicle_status_s status{};
		status.is_vtol = is_vtol;
		status.vehicle_type = is_fixed_wing
				      ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				      : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.in_transition_to_fw = in_transition_to_fw;
		status.in_transition_mode = in_transition_to_fw || in_back_transition;
		status.timestamp = hrt_absolute_time();
		_vehicle_status_pub.publish(status);
		_vehicle_status_sub.update();

		if (_navigator != nullptr) {
			*_navigator->get_vstatus() = status;
		}
	}

	void setLandedForTest(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.landed = landed;
		land_detected.timestamp = hrt_absolute_time();
		_land_detected_pub.publish(land_detected);
		_land_detected_sub.update();

		if (_navigator != nullptr) {
			*_navigator->get_land_detected() = land_detected;
		}
	}

	void setGlobalPositionForTest(const mission_route::Position &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_global_position_pub.publish(global_position);
		_global_pos_sub.update();
		*_navigator->get_global_position() = global_position;
	}

	void setGoalLandApproachForTest(const loiter_point_s &land_approach) { _goal_land_approach = land_approach; }
	bool advanceStageForTest() { return setNextMissionItem(); }

	void completeWorkItemForTest()
	{
		advance_mission();
		set_mission_items();
	}

	bool reportsReachedItemForTest() const { return shouldReportMissionItemReached(); }

	void normalizeRouteMissionItemForTest(mission_item_s &mission_item) const { normalizeRouteMissionItem(mission_item); }
	void publishActiveMissionItemsForTest() { setActiveMissionItems(); }
	void setCurrentMissionItemForTest(const mission_item_s &mission_item) { _mission_item = mission_item; }
	bool missionItemReachedForTest() { return is_mission_item_reached_or_completed(); }

	void setArrivalParametersForTest(float land_delay, float descend_alt, float loiter_radius)
	{
		_param_rtl_land_delay.set(land_delay);
		_param_rtl_descend_alt.set(descend_alt);
		_param_rtl_loiter_rad.set(loiter_radius);
	}

	void setPrecisionLandingForTest(int32_t precision) { _param_rtl_pld_md.set(precision); }
	void setForceVtolForTest(bool force) { _param_nav_force_vt.set(force); }

	void ageWaypointReachedForTest(float elapsed_seconds)
	{
		_time_wp_reached -= static_cast<hrt_abstime>(elapsed_seconds * 1_s);
	}

private:
	bool _use_real_route_cache{false};
	navigator_test::VectorMissionItemStore _mission_store{};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<mission_s> _mission_pub_for_test{ORB_ID(mission)};
};

/** Executor without a Navigator, for stage transitions that publish no setpoints. */
class RtlMissionSafePointFollowStageTest : public NavigatorDatamanTestBase
{
protected:
	RtlMissionSafePointFollowTestPeer executor{};

	void SetUp() override
	{
		executor.setVehicleStatusForTest(false, false, false);
		executor.setLandedForTest(false);
		executor.setVtolFailureForTest(false);
	}
};

/** Executor with a Navigator, for tests that publish setpoints, commands or estimates. */
class RtlMissionSafePointFollowNavigatorTest : public NavigatorDatamanTestBase
{
protected:
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer follower{&navigator};

	void SetUp() override
	{
		follower.setVehicleStatusForTest(false, false, false);
		follower.setLandedForTest(false);
		follower.setVtolFailureForTest(false);
	}

	float flightTime(float horizontal_m, float descent_m, bool fixed_wing = false)
	{
		float cruise_speed = NAN;
		float descend_speed = NAN;
		EXPECT_EQ(param_get(param_find(fixed_wing ? "FW_AIRSPD_TRIM" : "MPC_XY_CRUISE"), &cruise_speed), PX4_OK);
		EXPECT_EQ(param_get(param_find(fixed_wing ? "FW_T_SINK_R_SP" : "MPC_Z_V_AUTO_DN"), &descend_speed), PX4_OK);
		return horizontal_m / cruise_speed + descent_m / descend_speed;
	}
};

class RtlMissionSafePointFollowArrivalTest : public RtlMissionSafePointFollowNavigatorTest,
	public ::testing::WithParamInterface<std::tuple<bool, float>> {};

// A completed transition resumes route following without advancing its target.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesFollowRoute)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setAdvanceAfterTransitionForTest(false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
}

// A transition on the final route segment hands over to BranchOff, not back to FollowRoute.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesBranchOffWhenTargetIsBranchOff)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(false, 2);
	executor.setAdvanceAfterTransitionForTest(false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
}

// Reverse FT holds at the reached waypoint and points towards the next reverse target.
class RtlReverseFrontTransitionTest : public RtlMissionSafePointFollowNavigatorTest, public ::testing::WithParamInterface<bool> {};

TEST_P(RtlReverseFrontTransitionTest, WaitsForBackTransitionAndHeadingBeforeCommand)
{
	const std::vector<mission_item_s> items{
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};
	follower.loadTestMission(items);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.prepareActiveMissionForTest(65, 2);
	follower.setCurrentMissionItemForTest(items[2]);
	follower.setSafePointSelectionForTest(true, -1);
	follower.setVehicleStatusForTest(true, false, false, GetParam());
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest({items[2].lat, items[2].lon, items[2].altitude});
	navigator.get_local_position()->heading_good_for_control = true;
	navigator.get_local_position()->heading = M_PI_2_F;
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	ASSERT_TRUE(follower.advanceStageForTest());
	follower.on_active();

	if (GetParam()) {
		EXPECT_EQ(follower.workItemForTest(), RtlMissionSafePointFollowTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
		EXPECT_FALSE(follower.missionItemReachedForTest());
		EXPECT_FALSE(command_sub.updated());
		EXPECT_FALSE(follower.reportsReachedItemForTest());
		follower.setVehicleStatusForTest(true, false, false);
		ASSERT_TRUE(follower.missionItemReachedForTest());
		follower.on_active();
	}

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_DOUBLE_EQ(triplet.current.lat, items[2].lat);
	const float expected_yaw = get_bearing_to_next_waypoint(items[2].lat, items[2].lon, items[0].lat, items[0].lon);
	EXPECT_NEAR(triplet.current.yaw, expected_yaw, 1e-4f);
	navigator.get_local_position()->heading_good_for_control = true;
	navigator.get_local_position()->heading = matrix::wrap_pi(expected_yaw + M_PI_2_F);
	EXPECT_FALSE(follower.missionItemReachedForTest());
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(follower.currentSequenceForTest(), 2);
	EXPECT_FALSE(follower.reportsReachedItemForTest());

	navigator.get_local_position()->heading = expected_yaw;
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_FALSE(follower.reportsReachedItemForTest());
	EXPECT_NEAR(triplet.current.yaw, expected_yaw, 1e-4f);

	follower.setVehicleStatusForTest(true, false, true);
	EXPECT_FALSE(follower.missionItemReachedForTest());
	follower.on_active();
	EXPECT_FALSE(command_sub.updated());
	follower.setVehicleStatusForTest(true, true, false);
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.on_active();
	EXPECT_EQ(follower.currentSequenceForTest(), 0);
}

INSTANTIATE_TEST_SUITE_P(WithAndWithoutBackTransition, RtlReverseFrontTransitionTest, ::testing::Bool());

// Completing a reverse route transition advances current_seq right away, no re-reach needed.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteTransitionCompletionAdvancesToPreviousTarget)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, -1);
	executor.setAdvanceAfterTransitionForTest(true);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
}

// Reaching the virtual branch-off waypoint starts the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffStartsArrivalMove)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(executor.arrivalPhaseForTest(), RtlArrival::Phase::Move);
}

// Touchdown at a rally point completes Return, as direct RTL does.
TEST_F(RtlMissionSafePointFollowNavigatorTest, SyntheticGoalTouchdownCompletesReturn)
{
	follower.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	follower.prepareActiveMissionForTest(44, 1);
	follower.setSafePointSelectionForTest(false, 1);
	follower.setSafePointGeometryForTest({kBaseLat + 0.001, kBaseLon, kAlt}, {kBaseLat + 0.002, kBaseLon, kAlt - 50.f});
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest({kBaseLat + 0.002, kBaseLon, kAlt - 50.f});
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	follower.publishActiveMissionItemsForTest();
	ASSERT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LAND);

	uORB::Subscription mode_completed_sub{ORB_ID(mode_completed)};
	mode_completed_s completed{};

	while (mode_completed_sub.update(&completed)) {}

	follower.on_active();
	EXPECT_FALSE(mode_completed_sub.updated());

	follower.setLandedForTest(true);
	follower.on_active();
	ASSERT_TRUE(mode_completed_sub.update(&completed));
	EXPECT_EQ(completed.nav_state, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
}

// The virtual branch-off and the synthetic goal items never report an uploaded item as reached.
TEST_F(RtlMissionSafePointFollowStageTest, OnlyUploadedTargetsAreReportedReached)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setSafePointSelectionForTest(false, 1);

	using Stage = RtlMissionSafePointFollowTestPeer::Stage;
	const std::pair<Stage, bool> cases[] {
		{Stage::FollowRoute, true},
		{Stage::TransitionDuringRoute, false},
		{Stage::BranchOff, false},
		{Stage::Arrival, false},
		{Stage::LandAtGoal, false},
	};

	for (const auto &[stage, reported] : cases) {
		SCOPED_TRACE(static_cast<int>(stage));
		executor.setStageForTest(stage);
		EXPECT_EQ(executor.reportsReachedItemForTest(), reported);
	}

	// An uploaded mission landing is reported once it completes.
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;
	executor.configurePlanForTest(plan, 1);
	executor.setStageForTest(Stage::LandAtGoal);
	EXPECT_TRUE(executor.reportsReachedItemForTest());
}

// With a valid goal approach, branch-off starts the arrival descent.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffStartsArrivalDescentWithValidApproach)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});

	loiter_point_s goal_land_approach{};
	goal_land_approach.lat = kBaseLat + 0.0005;
	goal_land_approach.lon = kBaseLon + 0.0002;
	goal_land_approach.height_m = kAlt + 20.f;
	goal_land_approach.loiter_radius_m = 60.f;
	executor.setGoalLandApproachForTest(goal_land_approach);
	executor.setVehicleStatusForTest(true, true, false);
	executor.setSafePointSelectionForTest(false, 1);
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(executor.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, FollowRouteIgnoresExternalCurrentSequenceUpdate)
{
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	follower.loadTestMission(items);
	follower.prepareActiveMissionForTest(10, 0);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.publishActiveMissionItemsForTest();
	const position_setpoint_s expected_setpoint = navigator.get_position_setpoint_triplet()->current;

	follower.publishMissionUpdateForTest(10, 1);
	follower.on_active();

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(follower.missionIdForTest(), 10u);
	EXPECT_EQ(follower.currentSequenceForTest(), 0);
	const position_setpoint_s &setpoint = navigator.get_position_setpoint_triplet()->current;
	EXPECT_DOUBLE_EQ(setpoint.lat, expected_setpoint.lat);
	EXPECT_DOUBLE_EQ(setpoint.lon, expected_setpoint.lon);
	EXPECT_FLOAT_EQ(setpoint.alt, expected_setpoint.alt);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, RouteOnlyActivationDoesNotReplayMissionActions)
{
	mission_item_s speed{};
	speed.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	speed.params[1] = 12.f;
	mission_item_s camera{};
	camera.nav_cmd = NAV_CMD_IMAGE_START_CAPTURE;
	mission_item_s gimbal{};
	gimbal.nav_cmd = NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
	const std::vector<mission_item_s> items{
		speed,
		camera,
		gimbal,
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};
	follower.loadTestMission(items);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.goal_position = {kBaseLat, kBaseLon, kAlt};
	plan.join_position = {kBaseLat - 0.0001, kBaseLon, kAlt};
	plan.first_mission_item_index = 3;
	plan.branch_off_mission_item_index = 3;
	plan.branch_off_position = plan.goal_position;
	follower.configurePlanForTest(plan);
	follower.prepareActiveMissionForTest(11, 3);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);

	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (vehicle_command_sub.update(&command)) {}

	follower.on_activation();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	ASSERT_EQ(follower.currentSequenceForTest(), 3);
	// Camera and gimbal replays run in on_active().
	follower.on_active();

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED);
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE);
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
	}
}

class RtlMissionSafePointFollowGoalUpdateTest : public RtlMissionSafePointFollowNavigatorTest,
	public ::testing::WithParamInterface<RtlMissionSafePointFollowTestPeer::Stage> {};

TEST_P(RtlMissionSafePointFollowGoalUpdateTest, IgnoresReplacementMission)
{
	const auto stage = GetParam();
	const bool landing = stage == RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal;
	follower.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	follower.prepareActiveMissionForTest(20, 0);

	if (landing) {
		follower.setStageForTest(stage);

	} else {
		follower.setArrivalPhaseForTest(RtlArrival::Phase::Descend);
	}

	follower.setVehicleStatusForTest(!landing, false, false);
	follower.setLandedForTest(false);

	const mission_route::Position branch_off{kBaseLat, kBaseLon, kAlt};
	const mission_route::Position goal{kBaseLat + 0.002, kBaseLon + 0.001, kAlt - 10.f};
	follower.setSafePointSelectionForTest(false, 0);
	follower.setSafePointGeometryForTest(branch_off, goal);

	if (!landing) {
		loiter_point_s approach{};
		approach.lat = kBaseLat + 0.001;
		approach.lon = kBaseLon + 0.001;
		approach.height_m = kAlt + 20.f;
		approach.loiter_radius_m = 60.f;
		follower.setGoalLandApproachForTest(approach);
	}

	follower.setGlobalPositionForTest(landing ? goal : mission_route::Position{kBaseLat - 0.01, kBaseLon, kAlt});
	follower.publishActiveMissionItemsForTest();
	const position_setpoint_s expected_current = navigator.get_position_setpoint_triplet()->current;
	const position_setpoint_s expected_next = navigator.get_position_setpoint_triplet()->next;
	ASSERT_TRUE(expected_current.valid);
	ASSERT_EQ(expected_current.type, landing ? position_setpoint_s::SETPOINT_TYPE_LAND : position_setpoint_s::SETPOINT_TYPE_LOITER);

	// The replacement first arrives after this arrival or landing stage has started.
	follower.publishMissionUpdateForTest(21, 0);
	follower.on_active();

	EXPECT_EQ(follower.stageForTest(), stage);

	if (!landing) {
		EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
	}

	EXPECT_EQ(follower.missionIdForTest(), 20u);
	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	EXPECT_EQ(triplet->current.valid, expected_current.valid);
	EXPECT_EQ(triplet->current.type, expected_current.type);
	EXPECT_DOUBLE_EQ(triplet->current.lat, expected_current.lat);
	EXPECT_DOUBLE_EQ(triplet->current.lon, expected_current.lon);
	EXPECT_FLOAT_EQ(triplet->current.alt, expected_current.alt);
	EXPECT_EQ(triplet->next.valid, expected_next.valid);

	if (expected_next.valid) {
		EXPECT_DOUBLE_EQ(triplet->next.lat, expected_next.lat);
		EXPECT_DOUBLE_EQ(triplet->next.lon, expected_next.lon);
		EXPECT_FLOAT_EQ(triplet->next.alt, expected_next.alt);
	}
}

INSTANTIATE_TEST_SUITE_P(ArrivalAndLand, RtlMissionSafePointFollowGoalUpdateTest,
			 ::testing::Values(RtlMissionSafePointFollowTestPeer::Stage::Arrival,
					 RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal));

// Advancing onto the branch-off index stays in FollowRoute so a pending transition can still run.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);
	executor.setSafePointSelectionForTest(false, 2);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 2);
}

// Same in reverse: the branch-off anchor keeps FollowRoute active for a possible transition.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, 1);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
}

// When the branch-off anchor becomes the target, the virtual waypoint is published in the same pass.
TEST_F(RtlMissionSafePointFollowNavigatorTest, ForwardBranchOffAnchorPublishesVirtualBranchOffImmediately)
{
	std::vector<mission_item_s> items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	const auto branch_off_projection = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt);
	const auto safe_point_goal = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 60.f, kAlt);

	follower.loadTestMission(items);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.setCurrentSequenceForTest(2);
	follower.setCurrentMissionItemForTest(items[2]);
	follower.setSafePointSelectionForTest(false, 2);
	follower.setSafePointGeometryForTest(branch_off_projection, safe_point_goal);

	vehicle_global_position_s global_position{};
	global_position.lat = items[1].lat;
	global_position.lon = items[1].lon;
	global_position.alt = items[1].altitude;
	*navigator.get_global_position() = global_position;

	follower.publishActiveMissionItemsForTest();

	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet->current.valid);
	ASSERT_TRUE(triplet->next.valid);
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	EXPECT_NEAR(triplet->current.lat, branch_off_projection.lat, 1e-9);
	EXPECT_NEAR(triplet->current.lon, branch_off_projection.lon, 1e-9);
	EXPECT_NEAR(triplet->current.alt, branch_off_projection.alt, 1e-3f);
	EXPECT_NEAR(triplet->next.lat, safe_point_goal.lat, 1e-9);
	EXPECT_NEAR(triplet->next.lon, safe_point_goal.lon, 1e-9);
	EXPECT_NEAR(triplet->next.alt, safe_point_goal.alt, 1e-3f);
	EXPECT_FALSE(follower.missionItemReachedForTest());
}

// Forward traversal exhausted at the last route item: start the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionStartsArrivalMove)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(executor.arrivalPhaseForTest(), RtlArrival::Phase::Move);
}

// Reverse traversal exhausted at the first route item: start the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteExhaustionStartsArrivalMove)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(0);
	executor.setSafePointSelectionForTest(true, 0);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(executor.arrivalPhaseForTest(), RtlArrival::Phase::Move);
}

// A front transition flies the fixed-wing arrival item, and no transition takes the zero-delay landing shortcut.
TEST_F(RtlMissionSafePointFollowNavigatorTest, TransitionArrivalUsesFixedWingMoveAndDescendsFirst)
{
	follower.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	follower.setSafePointSelectionForTest(false, 0);
	follower.setSafePointGeometryForTest({kBaseLat, kBaseLon, kAlt}, {kBaseLat + 0.001, kBaseLon, kAlt - 50.f});
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest({kBaseLat, kBaseLon, kAlt});

	for (const bool front_transition : {true, false}) {
		SCOPED_TRACE(front_transition);
		follower.setVehicleStatusForTest(true, false, front_transition, !front_transition);
		follower.setArrivalPhaseForTest(RtlArrival::Phase::Move);
		follower.publishActiveMissionItemsForTest();
		EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd,
			  front_transition ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_WAYPOINT);
		ASSERT_TRUE(follower.advanceStageForTest());
		EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
	}

	follower.setVehicleStatusForTest(true, false, false);
	follower.setArrivalPhaseForTest(RtlArrival::Phase::Move);
	ASSERT_TRUE(follower.advanceStageForTest());
	EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Land);
}

// Endpoint handlers own takeoff and landing.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesEndpointCommands)
{
	for (const uint16_t command : {NAV_CMD_TAKEOFF, NAV_CMD_VTOL_TAKEOFF, NAV_CMD_LAND, NAV_CMD_VTOL_LAND}) {
		SCOPED_TRACE(command);
		mission_item_s item = makePositionItem(kBaseLat, kBaseLon, kAlt, command);
		item.autocontinue = false;
		item.time_inside = 9.f;

		executor.normalizeRouteMissionItemForTest(item);

		EXPECT_EQ(item.nav_cmd, command);
		EXPECT_FLOAT_EQ(item.time_inside, 9.f);
		EXPECT_FALSE(item.autocontinue);
	}
}

// Route altitude changes retain loiter geometry while clearing mission holds.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesLoiterToAltitude)
{
	mission_item_s loiter_item = makePositionItem(kBaseLat, kBaseLon, kAlt + 20.f, NAV_CMD_LOITER_TO_ALT);
	loiter_item.autocontinue = false;
	loiter_item.time_inside = 8.f;

	executor.normalizeRouteMissionItemForTest(loiter_item);

	EXPECT_EQ(loiter_item.nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_TRUE(loiter_item.autocontinue);
	EXPECT_FLOAT_EQ(loiter_item.time_inside, 0.f);
}

TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemClearsIntermediateHolds)
{
	for (const uint16_t command : {NAV_CMD_WAYPOINT, NAV_CMD_LOITER_TIME_LIMIT, NAV_CMD_LOITER_UNLIMITED}) {
		mission_item_s hold_item = makePositionItem(kBaseLat, kBaseLon, kAlt, command);
		hold_item.autocontinue = false;
		hold_item.time_inside = 8.f;

		executor.normalizeRouteMissionItemForTest(hold_item);

		EXPECT_EQ(hold_item.nav_cmd, NAV_CMD_WAYPOINT);
		EXPECT_TRUE(hold_item.autocontinue);
		EXPECT_FLOAT_EQ(hold_item.time_inside, 0.f);
	}
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, RouteLoiterChangesAltitudeOnlyAfterHorizontalArrival)
{
	mission_item_s loiter = makePositionItem(kBaseLat, kBaseLon, kAlt + 30.f, NAV_CMD_LOITER_TO_ALT);
	loiter.loiter_radius = 60.f;
	loiter.autocontinue = false;
	loiter.time_inside = 8.f;
	follower.loadTestMission({loiter, makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt)});
	follower.prepareActiveMissionForTest(46, 0);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.setVehicleStatusForTest(false, true, false);
	follower.setLandedForTest(false);
	const float transit_altitude = kAlt + 90.f;
	follower.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, transit_altitude));
	follower.publishActiveMissionItemsForTest();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(triplet.current.alt, transit_altitude);
	EXPECT_FALSE(follower.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(triplet.current.alt, transit_altitude);

	// Enter the orbit at the transit altitude before accepting the descent.
	follower.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -loiter.loiter_radius, 0.f, transit_altitude));
	EXPECT_FALSE(follower.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(triplet.current.alt, loiter.altitude);
	EXPECT_FALSE(follower.missionItemReachedForTest());

	follower.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -loiter.loiter_radius, 0.f, loiter.altitude));
	EXPECT_TRUE(follower.missionItemReachedForTest());
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, InactiveForecastUsesPlanStartWithoutChangingExecution)
{
	// The planned start differs from the stored cursor; forecasting must use it without activating RTL.
	follower.loadTestMission({
		makePositionItemFromOffset(kBaseLat, kBaseLon, -1000.f, 0.f, kAlt + 100.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 100.f),
	});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt);
	plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, -100.f, 0.f, kAlt + 100.f);
	plan.first_mission_item_index = 1;
	plan.branch_off_mission_item_index = 2;
	plan.branch_off_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 100.f);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setGlobalPositionForTest(makePositionFromOffset(kBaseLat, kBaseLon, -100.f, -100.f, kAlt + 100.f));
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	// This belongs to the currently active mode, not the hypothetical Return.
	auto &triplet = *navigator.get_position_setpoint_triplet();
	triplet = {}; // Finite values allow exact member comparisons without depending on NaNs or padding.
	triplet.current.valid = true;
	triplet.current.alt = kAlt + 200.f;
	const auto triplet_before = triplet;
	const auto setpoint_fields = [](const position_setpoint_s & sp) {
		return std::tie(sp.timestamp, sp.valid, sp.type, sp.vx, sp.vy, sp.vz, sp.lat, sp.lon, sp.alt, sp.yaw,
				sp.loiter_radius, sp.loiter_minor_radius, sp.loiter_direction_counter_clockwise,
				sp.loiter_orientation, sp.loiter_pattern, sp.acceptance_radius, sp.alt_acceptance_radius,
				sp.course, sp.cruising_speed, sp.gliding_enabled, sp.cruising_throttle);
	};
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	for (int forecast = 0; forecast < 2; ++forecast) {
		const auto estimate = follower.calc_rtl_time_estimate();
		ASSERT_TRUE(estimate.valid);
		// Vehicle -> join (100), first target (100), branch-off (200), goal (100), descent (100).
		EXPECT_NEAR(estimate.time_estimate, flightTime(500.f, 100.f), 0.02f);
		EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Idle);
		EXPECT_EQ(follower.currentSequenceForTest(), 0);
		EXPECT_FALSE(follower.joiningRouteForTest());
		EXPECT_EQ(triplet.timestamp, triplet_before.timestamp);
		EXPECT_EQ(setpoint_fields(triplet.previous), setpoint_fields(triplet_before.previous));
		EXPECT_EQ(setpoint_fields(triplet.current), setpoint_fields(triplet_before.current));
		EXPECT_EQ(setpoint_fields(triplet.next), setpoint_fields(triplet_before.next));
		EXPECT_FALSE(command_sub.updated());
	}

	// During RTL, estimate from actual progress instead of replaying the initial join.
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.setCurrentSequenceForTest(2);
	follower.setGlobalPositionForTest(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f));
	const auto active_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(active_estimate.valid);
	EXPECT_NEAR(active_estimate.time_estimate, flightTime(300.f, 100.f), 0.02f);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, InactiveReverseForecastHonorsCurrentAltitudeJoin)
{
	// Neither the projected join altitude nor the uploaded takeoff height should add a climb here.
	follower.loadTestMission({
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 300.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 100.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 100.f),
	});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	plan.goal_position = {kBaseLat, kBaseLon, kAlt};
	plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 300.f);
	plan.first_mission_item_index = 1;
	plan.direction_reversed = true;
	plan.use_current_altitude = true;
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setGlobalPositionForTest(makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt + 100.f));
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	const auto estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, flightTime(300.f, 100.f), 0.02f);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, PrefixLegExecutionUsesModeBeforeFirstTransition)
{
	for (bool prefix_fixed_wing : {false, true}) {
		for (bool reverse : {false, true}) {
			SCOPED_TRACE(::testing::Message() << "prefix_fixed_wing=" << prefix_fixed_wing << ", reverse=" << reverse);
			const std::vector<mission_item_s> items{
				makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
				makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
				makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
				makeVtolTransitionItem(prefix_fixed_wing ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
						       : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
			};
			follower.loadTestMission(items);
			const int32_t current_index = reverse ? 2 : 1;
			const int32_t vehicle_index = reverse ? 2 : 0;
			follower.prepareActiveMissionForTest(73, current_index);
			follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
			follower.setSafePointSelectionForTest(reverse, -1);
			follower.setVtolFailureForTest(false);
			follower.setVehicleStatusForTest(true, !prefix_fixed_wing, false);
			follower.setGlobalPositionForTest({items[vehicle_index].lat, items[vehicle_index].lon, kAlt});
			uORB::Subscription command_sub{ORB_ID(vehicle_command)};
			vehicle_command_s command{};

			while (command_sub.update(&command)) {}

			// Forward transitions precede a target; reverse transitions follow the reached target.
			if (reverse) {
				ASSERT_TRUE(follower.advanceStageForTest());

			} else {
				follower.publishActiveMissionItemsForTest();
			}

			ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
			ASSERT_TRUE(follower.transitionActiveForTest());
			EXPECT_EQ(follower.currentSequenceForTest(), current_index);

			if (prefix_fixed_wing) {
				EXPECT_FALSE(command_sub.updated());
				const float expected_yaw = get_bearing_to_next_waypoint(items[vehicle_index].lat, items[vehicle_index].lon,
							   items[1].lat, items[1].lon);
				EXPECT_NEAR(navigator.get_position_setpoint_triplet()->current.yaw, expected_yaw, 1e-4f);
				navigator.get_local_position()->heading_good_for_control = true;
				navigator.get_local_position()->heading = expected_yaw;
				follower.on_active();
			}

			ASSERT_TRUE(command_sub.update(&command));
			EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
			EXPECT_FLOAT_EQ(command.param1, prefix_fixed_wing ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
					: vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
		}
	}
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, PrefixLegForecastUsesModeBeforeFirstTransition)
{
	for (bool prefix_fixed_wing : {false, true}) {
		for (bool reverse : {false, true}) {
			SCOPED_TRACE(::testing::Message() << "prefix_fixed_wing=" << prefix_fixed_wing << ", reverse=" << reverse);
			follower.loadTestMission({
				makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f),
				makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 100.f),
				makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 100.f),
				makeVtolTransitionItem(prefix_fixed_wing ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
						       : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
			});
			mission_route::RtlRoutePlan plan{};
			plan.goal_type = mission_route::GoalType::kSafePoint;
			plan.safe_point_index = 0;
			plan.direction_reversed = reverse;
			plan.first_mission_item_index = reverse ? 2 : 0;
			plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, reverse ? 200.f : 0.f, 0.f, kAlt + 100.f);
			plan.branch_off_mission_item_index = reverse ? 0 : 2;
			plan.branch_off_position = makePositionFromOffset(kBaseLat, kBaseLon, reverse ? 50.f : 150.f, 0.f, kAlt + 100.f);
			plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, reverse ? 50.f : 150.f, 100.f, kAlt);
			follower.configurePlanForTest(plan);
			follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
			follower.setCurrentSequenceForTest(1);
			follower.setVehicleStatusForTest(true, !prefix_fixed_wing, false);
			follower.setGlobalPositionForTest(plan.join_position);
			follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
			const auto estimate = follower.calc_rtl_time_estimate();
			ASSERT_TRUE(estimate.valid);
			// Both directions cover 150 m of route and 100 m to the goal, before any transition command.
			const float expected_time = prefix_fixed_wing
						    ? flightTime(250.f, 70.f, true) + flightTime(0.f, 30.f)
						    : flightTime(250.f, 100.f);
			EXPECT_NEAR(estimate.time_estimate, expected_time, 0.03f);
		}
	}
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, VtolForecastUsesEachRouteLegModeAndMcFinalDescent)
{
	follower.loadTestMission({
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 100.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 100.f),
	});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.first_mission_item_index = 0;
	plan.join_position = {kBaseLat, kBaseLon, kAlt + 100.f};
	plan.branch_off_mission_item_index = 4;
	plan.branch_off_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt + 100.f);
	plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 100.f, kAlt);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(true, false, false);
	follower.setGlobalPositionForTest(plan.join_position);
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	const auto forward_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(forward_estimate.valid);
	EXPECT_NEAR(forward_estimate.time_estimate, flightTime(100.f, 0.f, true) + flightTime(150.f, 100.f), 0.03f);
	follower.setVtolFailureForTest(true);
	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, flightTime(250.f, 100.f), 0.03f);
	follower.setVtolFailureForTest(false);

	plan.direction_reversed = true;
	plan.first_mission_item_index = 4;
	plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 100.f);
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 100.f);
	plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 100.f, kAlt);
	follower.configurePlanForTest(plan);
	follower.setGlobalPositionForTest(plan.join_position);
	const auto reverse_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(reverse_estimate.valid);
	EXPECT_NEAR(reverse_estimate.time_estimate, flightTime(150.f, 30.f) + flightTime(150.f, 70.f, true), 0.03f);
	follower.setVtolFailureForTest(true);
	EXPECT_NEAR(follower.calc_rtl_time_estimate().time_estimate, flightTime(300.f, 100.f), 0.03f);
	follower.setVtolFailureForTest(false);

	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	follower.setVehicleStatusForTest(true, true, false);
	follower.setGlobalPositionForTest({plan.goal_position.lat, plan.goal_position.lon, kAlt + 100.f});
	const auto landing_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(landing_estimate.valid);
	EXPECT_NEAR(landing_estimate.time_estimate, flightTime(0.f, 100.f), 0.03f);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, ActivationAfterDisarmKeepsRouteTargetOnLastItem)
{
	// A later flight's Return must not restart the uploaded mission at item 0.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.goal_position = {kBaseLat, kBaseLon, kAlt - 30.f};
	plan.join_position = {items[2].lat - 0.0001, items[2].lon, kAlt};
	plan.first_mission_item_index = 2;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = {items[0].lat, items[0].lon, kAlt};
	plan.direction_reversed = true;
	follower.setGlobalPositionForTest({items[2].lat - 0.0002, items[2].lon, kAlt});
	follower.loadTestMission(items);
	follower.configurePlanForTest(plan);
	follower.prepareActiveMissionForTest(72, plan.first_mission_item_index);
	follower.setDisarmedSinceActivationForTest();

	follower.on_activation();

	EXPECT_EQ(follower.currentSequenceForTest(), plan.first_mission_item_index);
	EXPECT_TRUE(follower.joiningRouteForTest());
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, FwFailureReleasesPendingTransitionsAndKeepsRouteTargets)
{
	using Stage = RtlMissionSafePointFollowTestPeer::Stage;
	using WorkItem = RtlMissionSafePointFollowTestPeer::WorkItemType;
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.goal_position = {kBaseLat, kBaseLon, kAlt - 30.f};
	plan.join_position = {kBaseLat + 0.0001, kBaseLon, kAlt};
	plan.first_mission_item_index = 4;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = {items[0].lat, items[0].lon, kAlt};
	plan.vtol_transition_action = mission_route::VtolTransitionAction::kFrontTransition;
	follower.setVehicleStatusForTest(true, false, false);
	follower.setGlobalPositionForTest({kBaseLat, kBaseLon, kAlt});
	follower.setVtolFailureForTest(true);
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	// A failure already latched at activation must not arm the planner's requested FT.
	follower.loadTestMission(items);
	follower.configurePlanForTest(plan);
	follower.prepareActiveMissionForTest(70, plan.first_mission_item_index);
	follower.on_activation();
	EXPECT_TRUE(follower.joiningRouteForTest());
	EXPECT_FALSE(command_sub.updated());
	follower.setGlobalPositionForTest(plan.join_position);
	follower.on_active();
	EXPECT_FALSE(follower.joiningRouteForTest());
	EXPECT_EQ(follower.currentSequenceForTest(), plan.first_mission_item_index);
	EXPECT_FALSE(command_sub.updated());
	follower.setGlobalPositionForTest({kBaseLat, kBaseLon, kAlt});

	// Failure may arrive while flying to the join, aligning, or waiting for FW.
	for (WorkItem work : {
		     WorkItem::WORK_ITEM_TYPE_JOIN_ROUTE,
		     WorkItem::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION,
		     WorkItem::WORK_ITEM_TYPE_ALIGN_HEADING, WorkItem::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN
	     }) {
		SCOPED_TRACE(static_cast<int>(work));
		follower.loadTestMission(items);
		follower.configurePlanForTest(plan);
		follower.prepareActiveMissionForTest(71, 2);
		follower.setStageForTest(Stage::FollowRoute);
		follower.setVtolFailureForTest(false);
		follower.setVehicleStatusForTest(true, false, false,
						 work == WorkItem::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
		follower.setGlobalPositionForTest(work == WorkItem::WORK_ITEM_TYPE_JOIN_ROUTE
						  ? mission_route::Position{kBaseLat, kBaseLon, kAlt} : plan.join_position);
		follower.setJoinTransitionForTest();
		follower.publishActiveMissionItemsForTest();

		if (work != WorkItem::WORK_ITEM_TYPE_JOIN_ROUTE) {
			follower.on_active();
		}

		if (work == WorkItem::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN) {
			navigator.get_local_position()->heading_good_for_control = true;
			navigator.get_local_position()->heading = navigator.get_position_setpoint_triplet()->current.yaw;
			follower.on_active();
		}

		ASSERT_EQ(follower.workItemForTest(), work);
		follower.setVtolFailureForTest(true);

		while (command_sub.update(&command)) {}

		follower.on_active();
		EXPECT_EQ(follower.currentSequenceForTest(), 2);
		EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
		EXPECT_FALSE(command_sub.updated());
		EXPECT_DOUBLE_EQ(follower.currentMissionItemForTest().lat,
				 work == WorkItem::WORK_ITEM_TYPE_JOIN_ROUTE ? plan.join_position.lat : items[2].lat);
	}

	// Reverse transitions follow a reached waypoint; forward transitions precede their target.
	for (bool reverse : {false, true}) {
		for (WorkItem work : {
			     WorkItem::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION,
			     WorkItem::WORK_ITEM_TYPE_ALIGN_HEADING, WorkItem::WORK_ITEM_TYPE_DEFAULT
		     }) {
			SCOPED_TRACE(::testing::Message() << "reverse=" << reverse << ", work=" << static_cast<int>(work));
			follower.loadTestMission(items);
			plan.direction_reversed = reverse;
			plan.branch_off_mission_item_index = reverse ? 0 : 4;
			follower.configurePlanForTest(plan);
			follower.prepareActiveMissionForTest(72, 2);
			follower.setStageForTest(Stage::FollowRoute);
			follower.setVtolFailureForTest(false);
			follower.setVehicleStatusForTest(true, false, false, work == WorkItem::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
			follower.setGlobalPositionForTest({items[2].lat, items[2].lon, kAlt});

			if (reverse) {
				ASSERT_TRUE(follower.advanceStageForTest());

			} else {
				follower.publishActiveMissionItemsForTest();
			}

			if (work == WorkItem::WORK_ITEM_TYPE_DEFAULT) {
				navigator.get_local_position()->heading_good_for_control = true;
				navigator.get_local_position()->heading = navigator.get_position_setpoint_triplet()->current.yaw;
				follower.on_active();
			}

			ASSERT_EQ(follower.workItemForTest(), work);
			follower.setVtolFailureForTest(true);

			while (command_sub.update(&command)) {}

			follower.on_active();
			EXPECT_EQ(follower.currentSequenceForTest(), reverse ? 0 : 2);
			EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
			EXPECT_FALSE(follower.transitionActiveForTest());
			EXPECT_FALSE(command_sub.updated());
		}
	}

	follower.setVtolFailureForTest(false);
}

class RtlMissionSafePointFollowLoiterEstimateTest : public RtlMissionSafePointFollowNavigatorTest,
	public ::testing::WithParamInterface<bool>
{
};

TEST_P(RtlMissionSafePointFollowLoiterEstimateTest, LoiterApproachAndAltitudeChangeAreSequential)
{
	const bool fixed_wing = GetParam();
	mission_item_s loiter = makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 30.f);
	loiter.nav_cmd = NAV_CMD_LOITER_TO_ALT;
	loiter.loiter_radius = 60.f;
	follower.loadTestMission({loiter, makeLandItem(loiter.lat, loiter.lon, kAlt)});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.goal_position = {loiter.lat, loiter.lon, kAlt};
	plan.join_position = {kBaseLat, kBaseLon, kAlt + 90.f};
	plan.first_mission_item_index = 0;
	follower.configurePlanForTest(plan, 1);
	follower.prepareActiveMissionForTest(61, 0);
	follower.setVehicleStatusForTest(false, fixed_wing, false);
	follower.setGlobalPositionForTest(plan.join_position);
	const auto inactive_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(inactive_estimate.valid);
	EXPECT_NEAR(inactive_estimate.time_estimate, flightTime(300.f, 90.f, fixed_wing), 0.02f);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.publishActiveMissionItemsForTest();
	const auto approach_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(approach_estimate.valid);
	EXPECT_NEAR(approach_estimate.time_estimate, flightTime(300.f, 90.f, fixed_wing), 0.02f);

	// Once inside the loiter, its altitude setpoint changes; estimate only the remaining descent.
	follower.setGlobalPositionForTest({loiter.lat, loiter.lon, kAlt + 90.f});
	EXPECT_FALSE(follower.missionItemReachedForTest());
	follower.setGlobalPositionForTest({loiter.lat, loiter.lon, kAlt + 50.f});
	const auto descent_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(descent_estimate.valid);
	EXPECT_NEAR(descent_estimate.time_estimate, flightTime(0.f, 50.f, fixed_wing), 0.02f);
}

INSTANTIATE_TEST_SUITE_P(McAndFw, RtlMissionSafePointFollowLoiterEstimateTest, ::testing::Bool());

TEST_F(RtlMissionSafePointFollowNavigatorTest, UploadedMcLandingIncludesApproachAndRemainingFinalDescent)
{
	const mission_item_s landing = makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt);
	follower.loadTestMission({landing});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.goal_position = {landing.lat, landing.lon, landing.altitude};
	plan.join_position = {kBaseLat, kBaseLon, kAlt + 90.f};
	plan.first_mission_item_index = 0;
	plan.fly_direct_to_goal = true;
	follower.configurePlanForTest(plan, 0);
	follower.prepareActiveMissionForTest(62, 0);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(plan.join_position);
	// Uploaded LAND does not acquire the synthetic goal's indefinite hold.
	follower.setArrivalParametersForTest(-1.f, 30.f, 60.f);
	const auto inactive_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(inactive_estimate.valid);
	EXPECT_NEAR(inactive_estimate.time_estimate, flightTime(300.f, 90.f), 0.02f);

	follower.on_activation();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	const auto approach_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(approach_estimate.valid);
	EXPECT_NEAR(approach_estimate.time_estimate, inactive_estimate.time_estimate, 0.02f);
	follower.setGlobalPositionForTest({landing.lat, landing.lon, kAlt + 90.f});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	follower.setGlobalPositionForTest({landing.lat, landing.lon, kAlt + 20.f});
	const auto landing_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(landing_estimate.valid);
	EXPECT_NEAR(landing_estimate.time_estimate, flightTime(0.f, 20.f), 0.02f);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, ForcedVtolLandingAtPlainLandDescendsAsMc)
{
	// With NAV_FORCE_VT a fixed-wing VTOL flies to a plain LAND, back-transitions and descends as MC.
	const mission_item_s landing = makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt);
	follower.loadTestMission({makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f), landing});
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.goal_position = {landing.lat, landing.lon, landing.altitude};
	plan.join_position = {kBaseLat, kBaseLon, kAlt + 100.f};
	plan.first_mission_item_index = 1;
	follower.configurePlanForTest(plan, 1);
	follower.setVehicleStatusForTest(true, true, false);
	follower.setGlobalPositionForTest(plan.join_position);

	follower.setForceVtolForTest(false);
	const auto glide_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(glide_estimate.valid);
	// One leg descends while it cruises: the longer of the two counts.
	EXPECT_NEAR(glide_estimate.time_estimate, math::max(flightTime(300.f, 0.f, true), flightTime(0.f, 100.f, true)), 0.03f);

	follower.setForceVtolForTest(true);
	const auto vertical_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(vertical_estimate.valid);
	EXPECT_NEAR(vertical_estimate.time_estimate, flightTime(300.f, 0.f, true) + flightTime(0.f, 100.f), 0.03f);
}

TEST_P(RtlMissionSafePointFollowArrivalTest, SyntheticGoalHonorsArrivalDescentAndLandingDelay)
{
	// Cover MC and FW with negative, zero and positive delay; takeoff goals take the same arrival path.
	const bool fixed_wing = std::get<0>(GetParam());
	const float land_delay = std::get<1>(GetParam());
	constexpr float descend_altitude = 30.f;
	constexpr float loiter_radius = -70.f;
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 90.f);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 0;
	plan.fly_direct_to_goal = true;
	plan.safe_point_index = 0;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = vehicle_position;

	ASSERT_TRUE(plan.valid());
	follower.loadTestMission({makeTakeoffItem(goal.lat, goal.lon, goal.alt + descend_altitude)});
	follower.prepareActiveMissionForTest(47, 0);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, fixed_wing, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setArrivalParametersForTest(land_delay, descend_altitude, loiter_radius);
	const auto inactive_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(inactive_estimate.valid);
	follower.on_activation();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, fixed_wing ? position_setpoint_s::SETPOINT_TYPE_LOITER :
		  position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, goal.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, goal.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	EXPECT_FALSE(follower.missionItemReachedForTest());
	const rtl_time_estimate_s arrival_estimate = follower.calc_rtl_time_estimate();
	ASSERT_TRUE(arrival_estimate.valid);
	EXPECT_NEAR(arrival_estimate.time_estimate, inactive_estimate.time_estimate, 0.01f);
	EXPECT_GT(arrival_estimate.time_estimate, 0.f);
	follower.setGlobalPositionForTest({vehicle_position.lat, vehicle_position.lon, NAN});
	EXPECT_FALSE(follower.calc_rtl_time_estimate().valid);

	follower.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();

	if (fixed_wing || fabsf(land_delay) > FLT_EPSILON) {
		ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
		ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
		EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
		EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
		EXPECT_FLOAT_EQ(triplet.current.loiter_radius, fabsf(loiter_radius));
		EXPECT_TRUE(triplet.current.loiter_direction_counter_clockwise);
		EXPECT_FALSE(follower.missionItemReachedForTest());
		EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt + descend_altitude);
		EXPECT_FALSE(follower.missionItemReachedForTest());

		follower.setGlobalPositionForTest({goal.lat, goal.lon, goal.alt + descend_altitude});
		ASSERT_TRUE(follower.missionItemReachedForTest());
		ASSERT_TRUE(follower.advanceStageForTest());
		follower.publishActiveMissionItemsForTest();
		ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
		ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
		EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
		EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt + descend_altitude);
		const rtl_time_estimate_s hold_estimate = follower.calc_rtl_time_estimate();
		ASSERT_TRUE(hold_estimate.valid);

		if (land_delay < 0.f) {
			EXPECT_FLOAT_EQ(hold_estimate.time_estimate, 0.f);
			EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_UNLIMITED);
			EXPECT_FALSE(follower.currentMissionItemForTest().autocontinue);
			EXPECT_FALSE(follower.missionItemReachedForTest());
			EXPECT_FALSE(triplet.next.valid);
			return;
		}

		EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
		EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().time_inside, land_delay);
		follower.setArrivalParametersForTest(0.f, descend_altitude, loiter_radius);
		const rtl_time_estimate_s without_wait = follower.calc_rtl_time_estimate();
		ASSERT_TRUE(without_wait.valid);
		EXPECT_NEAR(hold_estimate.time_estimate - without_wait.time_estimate, land_delay, 1e-4f);
		follower.setArrivalParametersForTest(land_delay, descend_altitude, loiter_radius);

		if (land_delay > 0.f) {
			EXPECT_FALSE(follower.missionItemReachedForTest());
			follower.ageWaypointReachedForTest(1.f);
			const auto elapsed_hold_estimate = follower.calc_rtl_time_estimate();
			ASSERT_TRUE(elapsed_hold_estimate.valid);
			EXPECT_NEAR(hold_estimate.time_estimate - elapsed_hold_estimate.time_estimate, 1.f, 0.02f);
			follower.ageWaypointReachedForTest(land_delay + 1.f);
		}

		ASSERT_TRUE(follower.missionItemReachedForTest());
		ASSERT_TRUE(follower.advanceStageForTest());
		follower.publishActiveMissionItemsForTest();
	}

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_TRUE(follower.calc_rtl_time_estimate().valid);
	EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt);
	EXPECT_FALSE(follower.missionItemReachedForTest());
}

INSTANTIATE_TEST_SUITE_P(McAndFw, RtlMissionSafePointFollowArrivalTest,
			 ::testing::Combine(::testing::Bool(), ::testing::Values(-1.f, 0.f, 5.f)));

TEST_F(RtlMissionSafePointFollowNavigatorTest, SyntheticGoalDoesNotClimbAboveFrozenArrivalAltitude)
{
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 10.f);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 0;
	plan.fly_direct_to_goal = true;
	follower.loadTestMission({makeTakeoffItem(goal.lat, goal.lon, goal.alt + 50.f)});
	follower.prepareActiveMissionForTest(48, 0);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, true, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setArrivalParametersForTest(5.f, 30.f, 60.f);
	follower.on_activation();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	// Republishing while en route preserves the arrival altitude captured at handoff.
	follower.setGlobalPositionForTest({vehicle_position.lat, vehicle_position.lon, vehicle_position.alt - 5.f});
	follower.publishActiveMissionItemsForTest();
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);

	follower.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
	EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().altitude, vehicle_position.alt);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, ReverseTakeoffEndpointPreviewsAndUsesArrivalAltitude)
{
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 50.f);
	follower.loadTestMission({
		makeTakeoffItem(goal.lat, goal.lon, goal.alt + 200.f),
		makePositionItem(vehicle_position.lat, vehicle_position.lon, vehicle_position.alt),
	});
	follower.prepareActiveMissionForTest(51, 1);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 1;
	plan.direction_reversed = true;
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	follower.publishActiveMissionItemsForTest();
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.next.valid);
	EXPECT_EQ(triplet.next.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_FLOAT_EQ(triplet.next.alt, vehicle_position.alt);

	ASSERT_TRUE(follower.advanceStageForTest());
	follower.reloadAndPublishMissionItemsForTest();
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);
	EXPECT_DOUBLE_EQ(triplet.current.lat, goal.lat);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
}

// Exercise the public planner output through activation: a direct takeoff fallback
// must land at home altitude without first climbing to the stacked route waypoints.
TEST_F(RtlMissionSafePointFollowNavigatorTest, DirectStackedTakeoffPlanArrivesWithoutClimbingOrJoiningRoute)
{
	const std::vector<mission_item_s> items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 25.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
	};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 2.f, 0.5f, kAlt + 10.f);
	const auto provider = makeRouteProvider(items);
	MissionRoutePlanner planner{provider};
	mission_route::RtlRouteRequest request{};
	request.vehicle_position = vehicle_position;
	request.mission_index = 0;
	request.home_altitude_amsl = kAlt;
	request.projection_search_distance_m = 60.f;
	request.safe_point_projection_search_distance_m = 60.f;
	request.acceptance_radius_m = 10.f;
	request.direct_goal_acceptance_radius_m = 10.f;
	request.altitude_acceptance_radius_m = 10.f;
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	ASSERT_TRUE(plan.fly_direct_to_goal);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);

	follower.loadTestMission(items);
	follower.prepareActiveMissionForTest(41, 0);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	follower.on_activation();

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);
	EXPECT_FALSE(follower.joiningRouteForTest());
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, items[0].lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, items[0].lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);

	// With no requested hold, reaching the destination permits immediate landing.
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(triplet.current.alt, kAlt);
}

// Direct endpoint returns also preserve the uploaded LAND command when the active
// mission item has been replaced by a synthetic landing helper.
TEST_F(RtlMissionSafePointFollowNavigatorTest, DirectStackedLandingPlanPreservesUploadedLandingCommand)
{
	std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 50.f),
	};
	items[3].land_precision = 2;
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 98.f, 0.5f, kAlt + 10.f);
	const auto provider = makeRouteProvider(items);
	MissionRoutePlanner planner{provider};
	mission_route::RtlRouteRequest request{};
	request.vehicle_position = vehicle_position;
	request.mission_index = 3;
	request.mission_land_index = 3;
	request.home_altitude_amsl = kAlt;
	request.projection_search_distance_m = 60.f;
	request.safe_point_projection_search_distance_m = 60.f;
	request.acceptance_radius_m = 10.f;
	request.direct_goal_acceptance_radius_m = 10.f;
	request.altitude_acceptance_radius_m = 10.f;
	mission_route::RtlRoutePlan plan{};
	ASSERT_EQ(planner.planRtlRoute(request, plan), mission_route::FailureReason::kNone);
	ASSERT_TRUE(plan.valid());
	ASSERT_TRUE(plan.fly_direct_to_goal);
	ASSERT_EQ(plan.goal_type, mission_route::GoalType::kMissionLand);

	follower.loadTestMission(items);
	follower.prepareActiveMissionForTest(42, 3);
	follower.configurePlanForTest(plan, 3);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setArrivalParametersForTest(-1.f, 30.f, 60.f);
	follower.setPrecisionLandingForTest(0);
	follower.on_activation();

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_FALSE(follower.joiningRouteForTest());
	follower.setCurrentMissionItemForTest(items[2]);
	follower.publishActiveMissionItemsForTest();
	const auto &landing_item = follower.currentMissionItemForTest();
	EXPECT_EQ(landing_item.nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(landing_item.land_precision, 2);
	EXPECT_FLOAT_EQ(landing_item.altitude, items[3].altitude);
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(triplet.current.alt, items[3].altitude);
}

// A fixed-wing VTOL at an uploaded LAND goal back-transitions first, as Mission does with NAV_FORCE_VT.
TEST_F(RtlMissionSafePointFollowNavigatorTest, ForcedVtolLandingAtMissionLandGoalBackTransitionsFirst)
{
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 50.f),
	};
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.first_mission_item_index = 2;
	plan.fly_direct_to_goal = true;
	plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 50.f);
	ASSERT_TRUE(plan.valid());

	follower.loadTestMission(items);
	follower.prepareActiveMissionForTest(43, 2);
	follower.configurePlanForTest(plan, 2);
	follower.setVehicleStatusForTest(true, true, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
	follower.on_activation();

	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(follower.currentMissionItemForTest().vtol_back_transition);
	EXPECT_NE(navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
}

/** Executor with a Navigator built with NAV_FORCE_VT disabled. */
class RtlMissionSafePointFollowNoForcedVtolTest : public NavigatorDatamanTestBase
{
protected:
	// Set before the Navigator reads it, restored after the Navigator is destroyed.
	struct ForcedVtolDisabled {
		ForcedVtolDisabled()
		{
			const int32_t disabled = 0;
			param_set_no_notification(param_find("NAV_FORCE_VT"), &disabled);
		}

		~ForcedVtolDisabled() { param_reset_no_notification(param_find("NAV_FORCE_VT")); }
	} forced_vtol_disabled{};

	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer follower{&navigator};

	void SetUp() override
	{
		follower.setVehicleStatusForTest(true, true, false);
		follower.setLandedForTest(false);
		follower.setVtolFailureForTest(false);
	}
};

// Without NAV_FORCE_VT a fixed-wing VTOL lands a plain LAND goal as fixed-wing, unless
// the mission back-transitions before it: the route skips that transition.
TEST_F(RtlMissionSafePointFollowNoForcedVtolTest, PlainLandGoalAfterMissionBackTransitionLandsVertically)
{
	for (const bool back_transition : {false, true}) {
		SCOPED_TRACE(back_transition);
		const std::vector<mission_item_s> items{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			back_transition ? makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC)
			: makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 50.f),
		};
		mission_route::RtlRoutePlan plan{};
		plan.goal_type = mission_route::GoalType::kMissionLand;
		plan.first_mission_item_index = 2;
		plan.fly_direct_to_goal = true;
		plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
		plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 50.f);
		ASSERT_TRUE(plan.valid());

		follower.loadTestMission(items);
		follower.prepareActiveMissionForTest(44, 2);
		follower.configurePlanForTest(plan, 2);
		follower.setGlobalPositionForTest(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
		follower.on_activation();

		ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
		EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, back_transition ? NAV_CMD_WAYPOINT : NAV_CMD_LAND);
		EXPECT_EQ(static_cast<bool>(follower.currentMissionItemForTest().vtol_back_transition), back_transition);
	}
}

// The first target can be reached over a jump edge whose VTOL state differs
// from the ordinary segment entering the same target index.
TEST_F(RtlMissionSafePointFollowNavigatorTest, FirstRouteTargetPreservesPlannerJumpTransitionDecision)
{
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
		makeDoJump(2, 2),
	};
	mission_route::RtlRoutePlan plan{};
	plan.join_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt);
	plan.first_mission_item_index = 2;
	plan.active_jump_anchor.jump_item_index = 6;
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.safe_point_index = 0;
	plan.goal_position = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 50.f, kAlt);
	plan.branch_off_position = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt);
	plan.branch_off_mission_item_index = 5;
	// The jump segment is FW; a FW vehicle needs no transition at its join.
	plan.vtol_transition_action = mission_route::VtolTransitionAction::kNone;
	ASSERT_TRUE(plan.valid());

	follower.loadTestMission(items);
	follower.prepareActiveMissionForTest(43, 2);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(true, true, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(plan.join_position);
	// Simulate the completed join, now flying to the plan's first route target.
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (vehicle_command_sub.update(&command)) {}

	follower.publishActiveMissionItemsForTest();

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(follower.activeJumpAnchor().jump_item_index, 6);

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	}

	ASSERT_TRUE(follower.advanceStageForTest());
	EXPECT_EQ(follower.currentSequenceForTest(), 3);
	EXPECT_TRUE(follower.activeJumpAnchor().empty());
}

// RTL treats DO_JUMP as route geometry and advances past it. Only an edge
// actually flown can be carried into the next planner request as an anchor.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardGeometryTraversalDoesNotCreateJumpAnchor)
{
	executor.loadTestMission({
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeDoJump(0, 2),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	});
	executor.setSafePointSelectionForTest(false, 3);
	executor.setCurrentSequenceForTest(1);
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);

	ASSERT_TRUE(executor.advanceStageForTest());

	EXPECT_EQ(executor.currentSequenceForTest(), 3);
	EXPECT_TRUE(executor.activeJumpAnchor().empty());
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, FrontTransitionArrivalBackTransitionsBeforePrecisionLanding)
{
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto approach_position = makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kAlt + 30.f);
	follower.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt + 100.f)});
	follower.prepareActiveMissionForTest(51, 0);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.goal_position = goal;
	plan.join_position = approach_position;
	plan.first_mission_item_index = 0;
	plan.safe_point_index = 0;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = approach_position;
	plan.direction_reversed = true;
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(true, false, false);
	follower.setLandedForTest(false);
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	follower.setPrecisionLandingForTest(2);
	loiter_point_s approach{};
	approach.lat = approach_position.lat;
	approach.lon = approach_position.lon;
	approach.height_m = approach_position.alt;
	approach.loiter_radius_m = 60.f;
	follower.setGoalLandApproachForTest(approach);

	// A route entered in MC can reach its landing approach in FW.
	follower.setVehicleStatusForTest(true, false, true);
	follower.setGlobalPositionForTest(approach_position);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
	EXPECT_DOUBLE_EQ(follower.currentMissionItemForTest().lat, approach.lat);
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.publishActiveMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	ASSERT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(follower.currentMissionItemForTest().vtol_back_transition);
	EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().altitude, approach.height_m);

	// Landing helpers remain usable after the mission cache is replaced.
	follower.useRealRouteCacheForTest();
	navigator.get_mission_result()->valid = false;
	follower.setGlobalPositionForTest({goal.lat, goal.lon, approach.height_m});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.completeWorkItemForTest();
	ASSERT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().params[0], vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	follower.setVehicleStatusForTest(true, false, false, true);
	EXPECT_FALSE(follower.missionItemReachedForTest());
	follower.setVehicleStatusForTest(true, false, false);
	ASSERT_TRUE(follower.missionItemReachedForTest());

	// Recover drift in MC at the held altitude before descending.
	follower.setGlobalPositionForTest(approach_position);
	follower.completeWorkItemForTest();
	ASSERT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().altitude, approach.height_m);
	follower.setGlobalPositionForTest({goal.lat, goal.lon, approach.height_m});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.completeWorkItemForTest();
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_LAND);
	EXPECT_EQ(follower.currentMissionItemForTest().land_precision, 2);
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(follower.currentSequenceForTest(), 0);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, CommittedLandingReloadsUploadedGoalWithoutMissionCache)
{
	mission_item_s landing = makeLandItem(kBaseLat, kBaseLon, kAlt - 30.f);
	landing.land_precision = 2;
	follower.loadTestMission({landing});
	follower.prepareActiveMissionForTest(44, 0);
	mission_route::RtlRoutePlan plan{};
	plan.join_position = {kBaseLat, kBaseLon, kAlt};
	plan.first_mission_item_index = 0;
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.goal_position = {landing.lat, landing.lon, landing.altitude};
	plan.fly_direct_to_goal = true;
	follower.configurePlanForTest(plan, 0);
	follower.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(plan.goal_position);

	// Use the production loader with no matching cached mission, as after replacement.
	follower.useRealRouteCacheForTest();
	follower.setCurrentMissionItemForTest(makePositionItem(kBaseLat, kBaseLon, kAlt));
	ASSERT_TRUE(follower.reloadCurrentMissionItemForTest());
	// A newly uploaded mission may also fail its own feasibility check.
	navigator.get_mission_result()->valid = false;
	follower.reloadAndPublishMissionItemsForTest();

	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(follower.currentMissionItemForTest().land_precision, 2);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, landing.altitude);
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, CommittedArrivalKeepsGoalAndPrecisionLandingWithoutMissionCache)
{
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 90.f);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 0;
	plan.safe_point_index = 0;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = vehicle_position;
	plan.fly_direct_to_goal = true;
	follower.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	follower.prepareActiveMissionForTest(49, 0);
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(vehicle_position);
	follower.setArrivalParametersForTest(5.f, 30.f, 60.f);
	follower.setPrecisionLandingForTest(2);
	follower.on_activation();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);
	ASSERT_TRUE(follower.isLanding());

	// The replacement mission has no matching cache and fails its own feasibility check.
	follower.useRealRouteCacheForTest();
	follower.publishMissionUpdateForTest(50, 0);
	navigator.get_mission_result()->valid = false;
	follower.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, vehicle_position.alt);

	follower.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Descend);
	ASSERT_TRUE(follower.isLanding());
	EXPECT_FALSE(follower.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt + 30.f);

	follower.setGlobalPositionForTest({goal.lat, goal.lon, goal.alt + 30.f});
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
	ASSERT_TRUE(follower.isLanding());
	EXPECT_FALSE(follower.missionItemReachedForTest());
	follower.ageWaypointReachedForTest(6.f);
	ASSERT_TRUE(follower.missionItemReachedForTest());
	ASSERT_TRUE(follower.advanceStageForTest());
	follower.reloadAndPublishMissionItemsForTest();

	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(follower.missionIdForTest(), 49u);
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(follower.currentMissionItemForTest().land_precision, 2);
	EXPECT_DOUBLE_EQ(navigator.get_position_setpoint_triplet()->current.lat, goal.lat);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt);
	EXPECT_TRUE(navigator.get_precland()->is_activated());
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, ReactivationRestartsArrivalAltitudeAndHold)
{
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto initial_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 90.f);
	follower.loadTestMission({makePositionItem(goal.lat, goal.lon, initial_position.alt)});
	follower.prepareActiveMissionForTest(52, 0);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kSafePoint;
	plan.goal_position = goal;
	plan.join_position = initial_position;
	plan.first_mission_item_index = 0;
	plan.safe_point_index = 0;
	plan.branch_off_mission_item_index = 0;
	plan.branch_off_position = initial_position;
	plan.fly_direct_to_goal = true;
	follower.configurePlanForTest(plan);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(initial_position);
	follower.setArrivalParametersForTest(5.f, 30.f, 60.f);
	follower.run(true);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);

	const auto fly_to_hold = [&]() {
		// Move, enter the descent loiter, and reach its altitude before starting the hold.
		for (unsigned step = 0; step < 4u && follower.arrivalPhaseForTest() != RtlArrival::Phase::Hold; ++step) {
			const position_setpoint_s target = navigator.get_position_setpoint_triplet()->current;
			ASSERT_TRUE(target.valid);
			follower.setGlobalPositionForTest({target.lat, target.lon, target.alt});
			follower.run(true);
		}

		ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
		EXPECT_FALSE(follower.missionItemReachedForTest());
	};
	fly_to_hold();
	follower.ageWaypointReachedForTest(6.f);
	ASSERT_TRUE(follower.missionItemReachedForTest());
	follower.run(false);
	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Idle);
	EXPECT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Land);

	// Reactivation starts a new arrival below both the previous altitude and configured descent height.
	const auto new_position = makePositionFromOffset(kBaseLat, kBaseLon, -150.f, 0.f, kAlt + 10.f);
	follower.setGlobalPositionForTest(new_position);
	follower.run(true);
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Move);
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	EXPECT_DOUBLE_EQ(triplet.current.lat, goal.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, goal.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, new_position.alt);
	fly_to_hold();
	EXPECT_FLOAT_EQ(triplet.current.alt, new_position.alt);
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FLOAT_EQ(follower.currentMissionItemForTest().time_inside, 5.f);
	EXPECT_FALSE(follower.missionItemReachedForTest());
}

TEST_F(RtlMissionSafePointFollowNavigatorTest, CommittedApproachHandsOffToGoalWithoutMissionCache)
{
	follower.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	follower.prepareActiveMissionForTest(45, 0);
	follower.setSafePointSelectionForTest(false, 0);
	const mission_route::Position goal{kBaseLat + 0.002, kBaseLon, kAlt - 20.f};
	follower.setSafePointGeometryForTest({kBaseLat, kBaseLon, kAlt}, goal);
	loiter_point_s approach{};
	approach.lat = kBaseLat + 0.001;
	approach.lon = kBaseLon;
	approach.height_m = kAlt;
	approach.loiter_radius_m = 60.f;
	follower.setGoalLandApproachForTest(approach);
	follower.setArrivalPhaseForTest(RtlArrival::Phase::Descend);
	follower.setVehicleStatusForTest(false, false, false);
	follower.setLandedForTest(false);
	follower.setGlobalPositionForTest(goal);
	follower.setArrivalParametersForTest(0.f, 30.f, 60.f);
	follower.useRealRouteCacheForTest();

	ASSERT_TRUE(follower.reloadCurrentMissionItemForTest());
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	ASSERT_TRUE(follower.advanceStageForTest());
	ASSERT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Arrival);
	ASSERT_EQ(follower.arrivalPhaseForTest(), RtlArrival::Phase::Hold);
	ASSERT_TRUE(follower.reloadCurrentMissionItemForTest());
	EXPECT_EQ(follower.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	ASSERT_TRUE(follower.advanceStageForTest());
	ASSERT_TRUE(follower.reloadCurrentMissionItemForTest());
	follower.publishActiveMissionItemsForTest();

	EXPECT_EQ(follower.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_DOUBLE_EQ(navigator.get_position_setpoint_triplet()->current.lat, goal.lat);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt);
}
