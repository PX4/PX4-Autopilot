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
 * Unit tests for the lightweight RtlMissionSafePointFollow stage machine.
 * Focuses on setNextMissionItem() stage transitions without constructing a
 * full Navigator stack.
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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include <cstring>
#include <tuple>
#include <vector>

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

/**
 * Lightweight peer for the RTL stage machine.
 *
 * Overrides mission-item loading with vector-backed storage and exposes only
 * the minimal internal state needed to exercise setNextMissionItem().
 */
class RtlMissionSafePointFollowTestPeer : public RtlMissionSafePointFollow
{
public:
	using Stage = RtlMissionSafePointFollow::Stage;
	using VtolTransitionAction = mission_route::VtolTransitionAction;

	explicit RtlMissionSafePointFollowTestPeer(Navigator *navigator = nullptr)
		: RtlMissionSafePointFollow(navigator, mission_s{})
	{
	}

	~RtlMissionSafePointFollowTestPeer() override = default;

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		if (_use_real_route_cache) {
			return RtlMissionSafePointFollow::loadMissionItemFromCache(index, mission_item);
		}

		if (index < 0 || index >= static_cast<int32_t>(_items.size())) {
			return false;
		}

		mission_item = _items[static_cast<size_t>(index)];
		return true;
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_items = items;
		_mission = {};
		_mission.count = static_cast<int32_t>(items.size());
		_mission.current_seq = 0;
		_state = {};
		_plan = {};
	}

	void configurePlanForTest(const mission_route::RtlRoutePlan &plan, int32_t land_index = -1)
	{
		_mission.land_index = land_index;
		RouteSafePointConfig config{};
		config.plan = plan;
		configureRouteSafePoint(config);
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

	void setStageForTest(Stage stage)
	{
		_state.stage = stage;
	}

	Stage stageForTest() const
	{
		return _state.stage;
	}

	void setCurrentSequenceForTest(int32_t index)
	{
		_mission.current_seq = index;
	}

	int32_t currentSequenceForTest() const
	{
		return _mission.current_seq;
	}

	uint32_t missionIdForTest() const
	{
		return _mission.mission_id;
	}

	void prepareActiveMissionForTest(uint32_t mission_id, int32_t current_seq)
	{
		_mission.timestamp = hrt_absolute_time();
		_mission.mission_id = mission_id;
		_mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		_mission.current_seq = current_seq;
		_is_current_planned_mission_item_valid = true;
		_mission_type = MissionType::MISSION_TYPE_MISSION;
		_mission_item = _items[static_cast<size_t>(current_seq)];
		_navigator->get_mission_result()->valid = true;
	}

	void publishMissionUpdateForTest(uint32_t mission_id, int32_t current_seq)
	{
		mission_s mission = _mission;
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = mission_id;
		mission.current_seq = current_seq;
		_mission_pub_for_test.publish(mission);
	}

	void runActiveCycleForTest()
	{
		MissionBase::on_active();
	}

	void runBaseActivationForTest()
	{
		MissionBase::on_activation();
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

	void setTransitionTargetIndexForTest(int32_t index)
	{
		_state.transition_target_index = index;
	}

	void setTransitionStateForTest(VtolTransitionAction action, bool command_sent, bool advance_route_after_transition)
	{
		_state.transition_action = action;
		_state.transition_command_sent = command_sent;
		_state.advance_route_after_transition = advance_route_after_transition;
	}

	void setVehicleStatusForTest(bool is_vtol, bool is_fixed_wing, bool in_transition_to_fw)
	{
		vehicle_status_s status{};
		status.is_vtol = is_vtol;
		status.vehicle_type = is_fixed_wing
				      ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				      : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.in_transition_to_fw = in_transition_to_fw;
		status.in_transition_mode = in_transition_to_fw;
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

	void setGoalLandApproachForTest(const loiter_point_s &land_approach)
	{
		_goal_land_approach = land_approach;
	}

	int32_t transitionTargetIndexForTest() const
	{
		return _state.transition_target_index;
	}

	VtolTransitionAction transitionActionForTest() const
	{
		return _state.transition_action;
	}

	bool transitionCommandSentForTest() const
	{
		return _state.transition_command_sent;
	}

	bool advanceRouteAfterTransitionForTest() const
	{
		return _state.advance_route_after_transition;
	}

	bool advanceStageForTest()
	{
		return setNextMissionItem();
	}

	void normalizeRouteMissionItemForTest(mission_item_s &mission_item) const
	{
		normalizeRouteMissionItem(mission_item);
	}

	void publishActiveMissionItemsForTest()
	{
		setActiveMissionItems();
	}

	void resetExecutorProgressForTest()
	{
		resetExecutorProgress();
	}

	void setCurrentMissionItemForTest(const mission_item_s &mission_item)
	{
		_mission_item = mission_item;
	}

	bool missionItemReachedForTest()
	{
		return is_mission_item_reached_or_completed();
	}

	void setArrivalParametersForTest(float land_delay, float descend_alt, float loiter_radius)
	{
		_param_rtl_land_delay.set(land_delay);
		_param_rtl_descend_alt.set(descend_alt);
		_param_rtl_loiter_rad.set(loiter_radius);
	}

	void setPrecisionLandingForTest(int32_t precision) { _param_rtl_pld_md.set(precision); }

	void ageWaypointReachedForTest(float elapsed_seconds)
	{
		_time_wp_reached -= static_cast<hrt_abstime>(elapsed_seconds * 1_s);
	}

private:
	bool _use_real_route_cache{false};
	std::vector<mission_item_s> _items;
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<mission_s> _mission_pub_for_test{ORB_ID(mission)};
};

/**
 * @brief Fixture for lightweight RTL mission-safe-point-follow stage transitions.
 */
class RtlMissionSafePointFollowStageTest : public NavigatorDatamanTestBase
{
protected:
	RtlMissionSafePointFollowTestPeer executor{};

	void SetUp() override
	{
		executor.loadTestMission({});
		executor.setVehicleStatusForTest(false, false, false);
		executor.setLandedForTest(false);
	}
};

// TransitionDuringRoute is one-shot: the next advance returns to FollowRoute and clears the target.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesFollowRoute)
{
	// GIVEN: An executor paused in the transition stage with a remembered target index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setTransitionTargetIndexForTest(1);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::kFrontTransition, true,
					   false);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The transition stage completes and route following resumes.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
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
	executor.setTransitionTargetIndexForTest(2);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::kFrontTransition, true,
					   false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// Reverse: reaching a waypoint with an attached transition arms it and holds current_seq.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseReachedWaypointArmsTransitionBeforeAdvancing)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                                   // idx 0: WP1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),                 // idx 1: attached to WP1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),                                   // idx 2: WP2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),                 // idx 3: attached to WP2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt + 10.f),                                   // idx 4: WP3
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, -1);
	executor.setVehicleStatusForTest(true, false, false);
	executor.setLandedForTest(false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 2);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), 2);
	EXPECT_EQ(executor.transitionActionForTest(), RtlMissionSafePointFollowTestPeer::VtolTransitionAction::kFrontTransition);
	EXPECT_FALSE(executor.transitionCommandSentForTest());
	EXPECT_TRUE(executor.advanceRouteAfterTransitionForTest());
}

// A reverse post-reach transition tracks the next reverse target, not the waypoint already reached.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteTransitionPublishesPreviousTargetDuringTransition)
{
	Navigator navigator;
	RtlMissionSafePointFollowTestPeer executor_with_nav(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                                   // idx 0: WP1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),                 // idx 1: attached to WP1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),                                   // idx 2: WP2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),                 // idx 3: attached to WP2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt + 10.f),                                   // idx 4: WP3
	};

	executor_with_nav.loadTestMission(items);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor_with_nav.setCurrentSequenceForTest(2);
	executor_with_nav.setCurrentMissionItemForTest(items[2]);
	executor_with_nav.setSafePointSelectionForTest(true, -1);
	executor_with_nav.setTransitionTargetIndexForTest(2);
	executor_with_nav.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::kFrontTransition,
			false, true);
	executor_with_nav.setVehicleStatusForTest(true, false, false);
	executor_with_nav.setLandedForTest(false);

	vehicle_global_position_s global_position{};
	global_position.lat = items[2].lat;
	global_position.lon = items[2].lon;
	global_position.alt = items[2].altitude;
	*navigator.get_global_position() = global_position;

	executor_with_nav.publishActiveMissionItemsForTest();

	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet->current.valid);
	EXPECT_NEAR(triplet->current.lat, items[0].lat, 1e-9);
	EXPECT_NEAR(triplet->current.lon, items[0].lon, 1e-9);
	EXPECT_NEAR(triplet->current.alt, items[0].altitude, 1e-3f);
	EXPECT_EQ(triplet->current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
}

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
	executor.setTransitionTargetIndexForTest(2);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::kFrontTransition, true,
					   true);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// Reaching the virtual branch-off waypoint starts the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToMoveToGoal)
{
	// GIVEN: An executor that has already reached the branch-off waypoint.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor starts the destination arrival stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
}

// With a valid goal approach, BranchOff moves to ApproachAtGoal instead of landing directly.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToApproachAtGoalWhenGoalApproachValid)
{
	// GIVEN: An executor that has reached the branch-off waypoint for a safe point with a chosen approach.
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
	executor.setSafePointSelectionForTest(false, 1);
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor commits to the goal-approach stage before landing.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
}

// A completed goal-approach loiter starts the configured destination hold.
TEST_F(RtlMissionSafePointFollowStageTest, ApproachAtGoalTransitionsToHoldAtGoal)
{
	// GIVEN: An executor already flying the selected safe-point landing approach.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor leaves the approach stage and enters the destination hold stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::HoldAtGoal);
}

TEST_F(RtlMissionSafePointFollowStageTest, FollowRouteIgnoresExternalCurrentSequenceUpdate)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	executor_with_nav.loadTestMission(items);
	executor_with_nav.prepareActiveMissionForTest(10, 0);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.publishActiveMissionItemsForTest();
	const position_setpoint_s expected_setpoint = navigator.get_position_setpoint_triplet()->current;

	executor_with_nav.publishMissionUpdateForTest(10, 1);
	executor_with_nav.runActiveCycleForTest();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor_with_nav.missionIdForTest(), 10u);
	EXPECT_EQ(executor_with_nav.currentSequenceForTest(), 0);
	const position_setpoint_s &setpoint = navigator.get_position_setpoint_triplet()->current;
	EXPECT_DOUBLE_EQ(setpoint.lat, expected_setpoint.lat);
	EXPECT_DOUBLE_EQ(setpoint.lon, expected_setpoint.lon);
	EXPECT_FLOAT_EQ(setpoint.alt, expected_setpoint.alt);
}

TEST_F(RtlMissionSafePointFollowStageTest, RouteOnlyActivationDoesNotReplayMissionActions)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
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
	executor_with_nav.loadTestMission(items);
	executor_with_nav.prepareActiveMissionForTest(11, 3);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::Idle);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);

	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (vehicle_command_sub.update(&command)) {}

	executor_with_nav.runBaseActivationForTest();

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED);
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE);
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
	}
}

TEST_F(RtlMissionSafePointFollowStageTest, ApproachAtGoalIgnoresReplacementMission)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	executor_with_nav.prepareActiveMissionForTest(20, 0);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	executor_with_nav.setVehicleStatusForTest(true, false, false);
	executor_with_nav.setLandedForTest(false);

	const mission_route::Position branch_off{kBaseLat, kBaseLon, kAlt};
	const mission_route::Position goal{kBaseLat + 0.002, kBaseLon + 0.001, kAlt - 10.f};
	executor_with_nav.setSafePointSelectionForTest(false, 0);
	executor_with_nav.setSafePointGeometryForTest(branch_off, goal);
	loiter_point_s approach{};
	approach.lat = kBaseLat + 0.001;
	approach.lon = kBaseLon + 0.001;
	approach.height_m = kAlt + 20.f;
	approach.loiter_radius_m = 60.f;
	executor_with_nav.setGoalLandApproachForTest(approach);
	executor_with_nav.setGlobalPositionForTest({kBaseLat - 0.01, kBaseLon, kAlt});
	executor_with_nav.publishActiveMissionItemsForTest();
	const position_setpoint_s expected_current = navigator.get_position_setpoint_triplet()->current;
	const position_setpoint_s expected_next = navigator.get_position_setpoint_triplet()->next;

	executor_with_nav.publishMissionUpdateForTest(21, 0);
	executor_with_nav.runActiveCycleForTest();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	EXPECT_EQ(executor_with_nav.missionIdForTest(), 20u);
	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	EXPECT_DOUBLE_EQ(triplet->current.lat, expected_current.lat);
	EXPECT_DOUBLE_EQ(triplet->current.lon, expected_current.lon);
	EXPECT_FLOAT_EQ(triplet->current.alt, expected_current.alt);
	EXPECT_DOUBLE_EQ(triplet->next.lat, expected_next.lat);
	EXPECT_DOUBLE_EQ(triplet->next.lon, expected_next.lon);
	EXPECT_FLOAT_EQ(triplet->next.alt, expected_next.alt);
}

TEST_F(RtlMissionSafePointFollowStageTest, LandAtGoalIgnoresReplacementMission)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	executor_with_nav.prepareActiveMissionForTest(30, 0);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);

	const mission_route::Position goal{kBaseLat + 0.002, kBaseLon + 0.001, kAlt - 10.f};
	executor_with_nav.setSafePointSelectionForTest(false, 0);
	executor_with_nav.setSafePointGeometryForTest({kBaseLat, kBaseLon, kAlt}, goal);
	executor_with_nav.setGlobalPositionForTest(goal);
	executor_with_nav.publishActiveMissionItemsForTest();
	const position_setpoint_s expected_setpoint = navigator.get_position_setpoint_triplet()->current;

	executor_with_nav.publishMissionUpdateForTest(31, 0);
	executor_with_nav.runActiveCycleForTest();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(executor_with_nav.missionIdForTest(), 30u);
	const position_setpoint_s &setpoint = navigator.get_position_setpoint_triplet()->current;
	EXPECT_DOUBLE_EQ(setpoint.lat, expected_setpoint.lat);
	EXPECT_DOUBLE_EQ(setpoint.lon, expected_setpoint.lon);
	EXPECT_FLOAT_EQ(setpoint.alt, expected_setpoint.alt);
	EXPECT_EQ(setpoint.type, expected_setpoint.type);
}

// Advancing onto the branch-off index stays in FollowRoute so a pending transition can still run.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	// GIVEN: A forward route with the next position item equal to the cached branch-off index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);
	executor.setSafePointSelectionForTest(false, 2);

	// WHEN: setNextMissionItem advances along the nominal route.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor advances to the branch-off anchor but defers the BranchOff stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 2);
}

// Same in reverse: the branch-off anchor keeps FollowRoute active for a possible transition.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	// GIVEN: A reverse route whose previous position item is the cached branch-off index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, 1);

	// WHEN: setNextMissionItem advances along the reverse route.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor reaches the branch-off anchor but defers the BranchOff stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
}

// When the branch-off anchor becomes the target, the virtual waypoint is published in the same pass.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardBranchOffAnchorPublishesVirtualBranchOffImmediately)
{
	Navigator navigator;
	RtlMissionSafePointFollowTestPeer executor_with_nav(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	const auto branch_off_projection = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt);
	const auto safe_point_goal = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 60.f, kAlt);

	executor_with_nav.loadTestMission(items);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor_with_nav.setCurrentSequenceForTest(2);
	executor_with_nav.setCurrentMissionItemForTest(items[2]);
	executor_with_nav.setSafePointSelectionForTest(false, 2);
	executor_with_nav.setSafePointGeometryForTest(branch_off_projection, safe_point_goal);

	vehicle_global_position_s global_position{};
	global_position.lat = items[1].lat;
	global_position.lon = items[1].lon;
	global_position.alt = items[1].altitude;
	*navigator.get_global_position() = global_position;

	executor_with_nav.publishActiveMissionItemsForTest();

	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet->current.valid);
	ASSERT_TRUE(triplet->next.valid);
	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	EXPECT_NEAR(triplet->current.lat, branch_off_projection.lat, 1e-9);
	EXPECT_NEAR(triplet->current.lon, branch_off_projection.lon, 1e-9);
	EXPECT_NEAR(triplet->current.alt, branch_off_projection.alt, 1e-3f);
	EXPECT_NEAR(triplet->next.lat, safe_point_goal.lat, 1e-9);
	EXPECT_NEAR(triplet->next.lon, safe_point_goal.lon, 1e-9);
	EXPECT_NEAR(triplet->next.alt, safe_point_goal.alt, 1e-3f);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
}

// Same in reverse: the virtual branch-off waypoint replaces the raw mission waypoint immediately.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseBranchOffAnchorPublishesVirtualBranchOffImmediately)
{
	Navigator navigator;
	RtlMissionSafePointFollowTestPeer executor_with_nav(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	const auto branch_off_projection = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt);
	const auto safe_point_goal = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, -60.f, kAlt);

	executor_with_nav.loadTestMission(items);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor_with_nav.setCurrentSequenceForTest(1);
	executor_with_nav.setCurrentMissionItemForTest(items[1]);
	executor_with_nav.setSafePointSelectionForTest(true, 1);
	executor_with_nav.setSafePointGeometryForTest(branch_off_projection, safe_point_goal);

	vehicle_global_position_s global_position{};
	global_position.lat = items[2].lat;
	global_position.lon = items[2].lon;
	global_position.alt = items[2].altitude;
	*navigator.get_global_position() = global_position;

	executor_with_nav.publishActiveMissionItemsForTest();

	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet->current.valid);
	ASSERT_TRUE(triplet->next.valid);
	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	EXPECT_NEAR(triplet->current.lat, branch_off_projection.lat, 1e-9);
	EXPECT_NEAR(triplet->current.lon, branch_off_projection.lon, 1e-9);
	EXPECT_NEAR(triplet->current.alt, branch_off_projection.alt, 1e-3f);
	EXPECT_NEAR(triplet->next.lat, safe_point_goal.lat, 1e-9);
	EXPECT_NEAR(triplet->next.lon, safe_point_goal.lon, 1e-9);
	EXPECT_NEAR(triplet->next.alt, safe_point_goal.alt, 1e-3f);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
}

// Forward traversal exhausted at the last route item: start the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionTransitionsToMoveToGoal)
{
	// GIVEN: A forward route whose current sequence is already the final position item.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);

	// WHEN: setNextMissionItem tries to advance beyond the route end.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the destination arrival stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
}

// Route exhausted with a goal approach set: fly the approach first, FollowRoute moves to ApproachAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionTransitionsToApproachAtGoalWhenGoalApproachValid)
{
	// GIVEN: A forward route whose current sequence is already the last position item, plus a chosen goal approach.
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
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);
	executor.setSafePointSelectionForTest(false, -1);

	// WHEN: setNextMissionItem tries to advance beyond the route end.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor continues with the approach stage instead of going straight to land.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
}

// Reverse traversal exhausted at the first route item: start the destination arrival sequence.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteExhaustionTransitionsToMoveToGoal)
{
	// GIVEN: A reverse route whose current sequence is already the first position item.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(0);
	executor.setSafePointSelectionForTest(true, 0);

	// WHEN: setNextMissionItem tries to advance past the reverse route start.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the destination arrival stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
}

// Resetting executor progress clears the stage and the remembered transition target.
TEST_F(RtlMissionSafePointFollowStageTest, ResetExecutorProgressClearsStageAndTransitionTarget)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	executor.setTransitionTargetIndexForTest(1);

	executor.resetExecutorProgressForTest();

	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Idle);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// Takeoff commands carry their own altitude semantics: normalizeRouteMissionItem keeps them unchanged.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesTakeoffCommand)
{
	mission_item_s takeoff_item = makeTakeoffItem(kBaseLat, kBaseLon, kAlt + 30.f);
	takeoff_item.time_inside = 12.f;

	executor.normalizeRouteMissionItemForTest(takeoff_item);

	EXPECT_EQ(takeoff_item.nav_cmd, NAV_CMD_TAKEOFF);
	EXPECT_FLOAT_EQ(takeoff_item.time_inside, 12.f);
	EXPECT_FALSE(takeoff_item.autocontinue);
}

// Endpoint fallback needs the real land command: normalizeRouteMissionItem keeps NAV_CMD_LAND.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesLandingCommand)
{
	mission_item_s landing_item = makeLandItem(kBaseLat, kBaseLon, kAlt - 5.f);
	landing_item.time_inside = 9.f;

	executor.normalizeRouteMissionItemForTest(landing_item);

	EXPECT_EQ(landing_item.nav_cmd, NAV_CMD_LAND);
	EXPECT_FLOAT_EQ(landing_item.time_inside, 9.f);
	EXPECT_FALSE(landing_item.autocontinue);
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

TEST_F(RtlMissionSafePointFollowStageTest, RouteLoiterChangesAltitudeOnlyAfterHorizontalArrival)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	mission_item_s loiter = makePositionItem(kBaseLat, kBaseLon, kAlt + 30.f, NAV_CMD_LOITER_TO_ALT);
	loiter.loiter_radius = 60.f;
	loiter.autocontinue = false;
	loiter.time_inside = 8.f;
	executor_with_nav.loadTestMission({loiter, makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt)});
	executor_with_nav.prepareActiveMissionForTest(46, 0);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor_with_nav.setVehicleStatusForTest(false, true, false);
	executor_with_nav.setLandedForTest(false);
	const float transit_altitude = kAlt + 90.f;
	executor_with_nav.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, transit_altitude));
	executor_with_nav.publishActiveMissionItemsForTest();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(triplet.current.alt, transit_altitude);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(triplet.current.alt, transit_altitude);

	// Enter the orbit at the transit altitude before accepting the descent.
	executor_with_nav.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -loiter.loiter_radius, 0.f, transit_altitude));
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(triplet.current.alt, loiter.altitude);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());

	executor_with_nav.setGlobalPositionForTest(
		makePositionFromOffset(kBaseLat, kBaseLon, -loiter.loiter_radius, 0.f, loiter.altitude));
	EXPECT_TRUE(executor_with_nav.missionItemReachedForTest());
}

class RtlMissionSafePointFollowArrivalTest : public RtlMissionSafePointFollowStageTest,
	public ::testing::WithParamInterface<std::tuple<mission_route::GoalType, bool, float>>
{
};

class RtlMissionSafePointFollowEstimateTest : public RtlMissionSafePointFollowStageTest
{
protected:
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer follower{&navigator};

	float flightTime(float horizontal_m, float descent_m, bool fixed_wing = false)
	{
		float cruise_speed = NAN;
		float descend_speed = NAN;
		EXPECT_EQ(param_get(param_find(fixed_wing ? "FW_AIRSPD_TRIM" : "MPC_XY_CRUISE"), &cruise_speed), PX4_OK);
		EXPECT_EQ(param_get(param_find(fixed_wing ? "FW_T_SINK_R_SP" : "MPC_Z_V_AUTO_DN"), &descend_speed), PX4_OK);
		return horizontal_m / cruise_speed + descent_m / descend_speed;
	}
};

TEST_F(RtlMissionSafePointFollowEstimateTest, InactiveForecastUsesPlanStartWithoutChangingExecution)
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
	triplet.current.valid = true;
	triplet.current.alt = kAlt + 200.f;
	const auto triplet_before = triplet;
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
		EXPECT_EQ(memcmp(&triplet, &triplet_before, sizeof(triplet)), 0);
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

TEST_F(RtlMissionSafePointFollowEstimateTest, InactiveReverseForecastHonorsCurrentAltitudeJoin)
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

class RtlMissionSafePointFollowLoiterEstimateTest : public RtlMissionSafePointFollowEstimateTest,
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

TEST_F(RtlMissionSafePointFollowEstimateTest, UploadedMcLandingIncludesApproachAndRemainingFinalDescent)
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

TEST_P(RtlMissionSafePointFollowArrivalTest, SyntheticGoalHonorsArrivalDescentAndLandingDelay)
{
	// Rally and takeoff goals share arrival policy; cover MC/FW with negative, zero and positive delay.
	const auto goal_type = std::get<0>(GetParam());
	const bool fixed_wing = std::get<1>(GetParam());
	const float land_delay = std::get<2>(GetParam());
	constexpr float descend_altitude = 30.f;
	constexpr float loiter_radius = -70.f;
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 90.f);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = goal_type;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 0;
	plan.fly_direct_to_goal = true;

	if (goal_type == mission_route::GoalType::kSafePoint) {
		plan.safe_point_index = 0;
		plan.branch_off_mission_item_index = 0;
		plan.branch_off_position = vehicle_position;
	}

	ASSERT_TRUE(plan.valid());
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission({makeTakeoffItem(goal.lat, goal.lon, goal.alt + descend_altitude)});
	executor_with_nav.prepareActiveMissionForTest(47, 0);
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(false, fixed_wing, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setArrivalParametersForTest(land_delay, descend_altitude, loiter_radius);
	const auto inactive_estimate = executor_with_nav.calc_rtl_time_estimate();
	ASSERT_TRUE(inactive_estimate.valid);
	executor_with_nav.on_activation();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, fixed_wing ? position_setpoint_s::SETPOINT_TYPE_LOITER :
		  position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, goal.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, goal.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
	const rtl_time_estimate_s arrival_estimate = executor_with_nav.calc_rtl_time_estimate();
	ASSERT_TRUE(arrival_estimate.valid);
	EXPECT_NEAR(arrival_estimate.time_estimate, inactive_estimate.time_estimate, 0.01f);
	EXPECT_GT(arrival_estimate.time_estimate, 0.f);
	executor_with_nav.setGlobalPositionForTest({vehicle_position.lat, vehicle_position.lon, NAN});
	EXPECT_FALSE(executor_with_nav.calc_rtl_time_estimate().valid);

	executor_with_nav.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.publishActiveMissionItemsForTest();

	if (fixed_wing || fabsf(land_delay) > FLT_EPSILON) {
		ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
		EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
		EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
		EXPECT_FLOAT_EQ(triplet.current.loiter_radius, fabsf(loiter_radius));
		EXPECT_TRUE(triplet.current.loiter_direction_counter_clockwise);
		EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
		EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt + descend_altitude);
		EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());

		executor_with_nav.setGlobalPositionForTest({goal.lat, goal.lon, goal.alt + descend_altitude});
		ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
		ASSERT_TRUE(executor_with_nav.advanceStageForTest());
		executor_with_nav.publishActiveMissionItemsForTest();
		ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::HoldAtGoal);
		EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
		EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt + descend_altitude);
		const rtl_time_estimate_s hold_estimate = executor_with_nav.calc_rtl_time_estimate();
		ASSERT_TRUE(hold_estimate.valid);

		if (land_delay < 0.f) {
			EXPECT_FLOAT_EQ(hold_estimate.time_estimate, 0.f);
			EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_UNLIMITED);
			EXPECT_FALSE(executor_with_nav.currentMissionItemForTest().autocontinue);
			EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
			EXPECT_FALSE(triplet.next.valid);
			return;
		}

		EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
		EXPECT_FLOAT_EQ(executor_with_nav.currentMissionItemForTest().time_inside, land_delay);
		executor_with_nav.setArrivalParametersForTest(0.f, descend_altitude, loiter_radius);
		const rtl_time_estimate_s without_wait = executor_with_nav.calc_rtl_time_estimate();
		ASSERT_TRUE(without_wait.valid);
		EXPECT_NEAR(hold_estimate.time_estimate - without_wait.time_estimate, land_delay, 1e-4f);
		executor_with_nav.setArrivalParametersForTest(land_delay, descend_altitude, loiter_radius);

		if (land_delay > 0.f) {
			EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
			executor_with_nav.ageWaypointReachedForTest(1.f);
			const auto elapsed_hold_estimate = executor_with_nav.calc_rtl_time_estimate();
			ASSERT_TRUE(elapsed_hold_estimate.valid);
			EXPECT_NEAR(hold_estimate.time_estimate - elapsed_hold_estimate.time_estimate, 1.f, 0.02f);
			executor_with_nav.ageWaypointReachedForTest(land_delay + 1.f);
		}

		ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
		ASSERT_TRUE(executor_with_nav.advanceStageForTest());
		executor_with_nav.publishActiveMissionItemsForTest();
	}

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_TRUE(executor_with_nav.calc_rtl_time_estimate().valid);
	EXPECT_FLOAT_EQ(triplet.current.alt, goal.alt);
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
}

INSTANTIATE_TEST_SUITE_P(SafePointAndTakeoff, RtlMissionSafePointFollowArrivalTest,
			 ::testing::Combine(::testing::Values(mission_route::GoalType::kSafePoint,
					 mission_route::GoalType::kMissionTakeoff),
					 ::testing::Bool(), ::testing::Values(-1.f, 0.f, 5.f)));

TEST_F(RtlMissionSafePointFollowStageTest, SyntheticGoalDoesNotClimbAboveFrozenArrivalAltitude)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, -300.f, 0.f, kAlt + 10.f);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 0;
	plan.fly_direct_to_goal = true;
	executor_with_nav.loadTestMission({makeTakeoffItem(goal.lat, goal.lon, goal.alt + 50.f)});
	executor_with_nav.prepareActiveMissionForTest(48, 0);
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(false, true, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setArrivalParametersForTest(5.f, 30.f, 60.f);
	executor_with_nav.on_activation();

	const auto &triplet = *navigator.get_position_setpoint_triplet();
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	// Republishing while en route preserves the arrival altitude captured at handoff.
	executor_with_nav.setGlobalPositionForTest({vehicle_position.lat, vehicle_position.lon, vehicle_position.alt - 5.f});
	executor_with_nav.publishActiveMissionItemsForTest();
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);

	executor_with_nav.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.publishActiveMissionItemsForTest();
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	EXPECT_FLOAT_EQ(executor_with_nav.currentMissionItemForTest().altitude, vehicle_position.alt);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.publishActiveMissionItemsForTest();
	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::HoldAtGoal);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
}

TEST_F(RtlMissionSafePointFollowStageTest, ReverseTakeoffEndpointPreviewsAndUsesArrivalAltitude)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	const mission_route::Position goal{kBaseLat, kBaseLon, kAlt};
	const auto vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 50.f);
	executor_with_nav.loadTestMission({
		makeTakeoffItem(goal.lat, goal.lon, goal.alt + 200.f),
		makePositionItem(vehicle_position.lat, vehicle_position.lon, vehicle_position.alt),
	});
	executor_with_nav.prepareActiveMissionForTest(51, 1);
	mission_route::RtlRoutePlan plan{};
	plan.goal_type = mission_route::GoalType::kMissionTakeoff;
	plan.goal_position = goal;
	plan.join_position = vehicle_position;
	plan.first_mission_item_index = 1;
	plan.direction_reversed = true;
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor_with_nav.publishActiveMissionItemsForTest();
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.next.valid);
	EXPECT_EQ(triplet.next.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_FLOAT_EQ(triplet.next.alt, vehicle_position.alt);

	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.reloadAndPublishMissionItemsForTest();
	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
	EXPECT_DOUBLE_EQ(triplet.current.lat, goal.lat);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);
}

// Exercise the public planner output through activation: a direct takeoff fallback
// must land at home altitude without first climbing to the stacked route waypoints.
TEST_F(RtlMissionSafePointFollowStageTest, DirectStackedTakeoffPlanArrivesWithoutClimbingOrJoiningRoute)
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

	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission(items);
	executor_with_nav.prepareActiveMissionForTest(41, 0);
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setArrivalParametersForTest(0.f, 30.f, 60.f);
	executor_with_nav.on_activation();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
	EXPECT_FALSE(executor_with_nav.joiningRouteForTest());
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, items[0].lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, items[0].lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, vehicle_position.alt);

	// With no requested hold, reaching the destination permits immediate landing.
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.publishActiveMissionItemsForTest();
	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(triplet.current.alt, kAlt);
}

// Direct endpoint returns also preserve the uploaded LAND command when the active
// mission item has been replaced by a synthetic landing helper.
TEST_F(RtlMissionSafePointFollowStageTest, DirectStackedLandingPlanPreservesUploadedLandingCommand)
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

	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission(items);
	executor_with_nav.prepareActiveMissionForTest(42, 3);
	executor_with_nav.configurePlanForTest(plan, 3);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setArrivalParametersForTest(-1.f, 30.f, 60.f);
	executor_with_nav.setPrecisionLandingForTest(0);
	executor_with_nav.on_activation();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_FALSE(executor_with_nav.joiningRouteForTest());
	executor_with_nav.setCurrentMissionItemForTest(items[2]);
	executor_with_nav.publishActiveMissionItemsForTest();
	const auto &landing_item = executor_with_nav.currentMissionItemForTest();
	EXPECT_EQ(landing_item.nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(landing_item.land_precision, 2);
	EXPECT_FLOAT_EQ(landing_item.altitude, items[3].altitude);
	const auto &triplet = *navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(triplet.current.alt, items[3].altitude);
}

// The first target can be reached over a jump edge whose VTOL state differs
// from the ordinary segment entering the same target index.
TEST_F(RtlMissionSafePointFollowStageTest, FirstRouteTargetPreservesPlannerJumpTransitionDecision)
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

	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission(items);
	executor_with_nav.prepareActiveMissionForTest(43, 2);
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(true, true, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(plan.join_position);
	// Simulate the completed join, now flying to the plan's first route target.
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (vehicle_command_sub.update(&command)) {}

	executor_with_nav.publishActiveMissionItemsForTest();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor_with_nav.activeJumpAnchor().jump_item_index, 6);

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	}

	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	EXPECT_EQ(executor_with_nav.currentSequenceForTest(), 3);
	EXPECT_TRUE(executor_with_nav.activeJumpAnchor().empty());
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

TEST_F(RtlMissionSafePointFollowStageTest, CommittedLandingReloadsUploadedGoalWithoutMissionCache)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	mission_item_s landing = makeLandItem(kBaseLat, kBaseLon, kAlt - 30.f);
	landing.land_precision = 2;
	executor_with_nav.loadTestMission({landing});
	executor_with_nav.prepareActiveMissionForTest(44, 0);
	mission_route::RtlRoutePlan plan{};
	plan.join_position = {kBaseLat, kBaseLon, kAlt};
	plan.first_mission_item_index = 0;
	plan.goal_type = mission_route::GoalType::kMissionLand;
	plan.goal_position = {landing.lat, landing.lon, landing.altitude};
	plan.fly_direct_to_goal = true;
	executor_with_nav.configurePlanForTest(plan, 0);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(plan.goal_position);

	// Use the production loader with no matching cached mission, as after replacement.
	executor_with_nav.useRealRouteCacheForTest();
	executor_with_nav.setCurrentMissionItemForTest(makePositionItem(kBaseLat, kBaseLon, kAlt));
	ASSERT_TRUE(executor_with_nav.reloadCurrentMissionItemForTest());
	// A newly uploaded mission may also fail its own feasibility check.
	navigator.get_mission_result()->valid = false;
	executor_with_nav.reloadAndPublishMissionItemsForTest();

	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().land_precision, 2);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, landing.altitude);
}

TEST_F(RtlMissionSafePointFollowStageTest, CommittedArrivalKeepsGoalAndPrecisionLandingWithoutMissionCache)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
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
	executor_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	executor_with_nav.prepareActiveMissionForTest(49, 0);
	executor_with_nav.configurePlanForTest(plan);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(vehicle_position);
	executor_with_nav.setArrivalParametersForTest(5.f, 30.f, 60.f);
	executor_with_nav.setPrecisionLandingForTest(2);
	executor_with_nav.on_activation();
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::MoveToGoal);
	ASSERT_TRUE(executor_with_nav.isLanding());

	// The replacement mission has no matching cache and fails its own feasibility check.
	executor_with_nav.useRealRouteCacheForTest();
	executor_with_nav.publishMissionUpdateForTest(50, 0);
	navigator.get_mission_result()->valid = false;
	executor_with_nav.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, vehicle_position.alt);

	executor_with_nav.setGlobalPositionForTest({goal.lat, goal.lon, vehicle_position.alt});
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	ASSERT_TRUE(executor_with_nav.isLanding());
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt + 30.f);

	executor_with_nav.setGlobalPositionForTest({goal.lat, goal.lon, goal.alt + 30.f});
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.reloadAndPublishMissionItemsForTest();
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::HoldAtGoal);
	ASSERT_TRUE(executor_with_nav.isLanding());
	EXPECT_FALSE(executor_with_nav.missionItemReachedForTest());
	executor_with_nav.ageWaypointReachedForTest(6.f);
	ASSERT_TRUE(executor_with_nav.missionItemReachedForTest());
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	executor_with_nav.reloadAndPublishMissionItemsForTest();

	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(executor_with_nav.missionIdForTest(), 49u);
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().land_precision, 2);
	EXPECT_DOUBLE_EQ(navigator.get_position_setpoint_triplet()->current.lat, goal.lat);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt);
	EXPECT_TRUE(navigator.get_precland()->is_activated());
}

TEST_F(RtlMissionSafePointFollowStageTest, CommittedApproachHandsOffToGoalWithoutMissionCache)
{
	Navigator navigator{};
	RtlMissionSafePointFollowTestPeer executor_with_nav{&navigator};
	executor_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	executor_with_nav.prepareActiveMissionForTest(45, 0);
	executor_with_nav.setSafePointSelectionForTest(false, 0);
	const mission_route::Position goal{kBaseLat + 0.002, kBaseLon, kAlt - 20.f};
	executor_with_nav.setSafePointGeometryForTest({kBaseLat, kBaseLon, kAlt}, goal);
	loiter_point_s approach{};
	approach.lat = kBaseLat + 0.001;
	approach.lon = kBaseLon;
	approach.height_m = kAlt;
	approach.loiter_radius_m = 60.f;
	executor_with_nav.setGoalLandApproachForTest(approach);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	executor_with_nav.setVehicleStatusForTest(false, false, false);
	executor_with_nav.setLandedForTest(false);
	executor_with_nav.setGlobalPositionForTest(goal);
	executor_with_nav.setArrivalParametersForTest(0.f, 30.f, 60.f);
	executor_with_nav.useRealRouteCacheForTest();

	ASSERT_TRUE(executor_with_nav.reloadCurrentMissionItemForTest());
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	ASSERT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::HoldAtGoal);
	ASSERT_TRUE(executor_with_nav.reloadCurrentMissionItemForTest());
	EXPECT_EQ(executor_with_nav.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	ASSERT_TRUE(executor_with_nav.advanceStageForTest());
	ASSERT_TRUE(executor_with_nav.reloadCurrentMissionItemForTest());
	executor_with_nav.publishActiveMissionItemsForTest();

	EXPECT_EQ(executor_with_nav.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_EQ(navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_DOUBLE_EQ(navigator.get_position_setpoint_triplet()->current.lat, goal.lat);
	EXPECT_FLOAT_EQ(navigator.get_position_setpoint_triplet()->current.alt, goal.alt);
}
