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
		_plan.selection.found = true;
		_plan.selection.safe_point_found = true;
		_plan.selection.goal_type = mission_route::GoalType::kSafePoint;
		_plan.selection.path.direction_reversed = direction_reversed;
		_plan.selection.path.first_item_index = branch_off_index;
		_plan.selection.branch_off_segment.start.idx = direction_reversed ? branch_off_index : branch_off_index - 1;
		_plan.selection.branch_off_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
		_plan.selection.branch_off_segment.end.idx = direction_reversed ? branch_off_index + 1 : branch_off_index;
		_plan.selection.branch_off_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	}

	void setSafePointGeometryForTest(const mission_route::Position &branch_off_projection,
					 const mission_route::Position &goal_position)
	{
		_plan.selection.branch_off_projection = branch_off_projection;
		_plan.selection.safe_point_position = goal_position;
		_plan.selection.goal_position = goal_position;
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
	}

	void setLandedForTest(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.landed = landed;
		land_detected.timestamp = hrt_absolute_time();
		_land_detected_pub.publish(land_detected);
		_land_detected_sub.update();
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

private:
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

// Reaching the virtual branch-off waypoint moves BranchOff to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToLandAtGoal)
{
	// GIVEN: An executor that has already reached the branch-off waypoint.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor commits to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
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

// A completed goal-approach loiter moves ApproachAtGoal to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ApproachAtGoalTransitionsToLandAtGoal)
{
	// GIVEN: An executor already flying the selected safe-point landing approach.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor leaves the approach stage and enters the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
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

// Forward traversal exhausted at the last route item: FollowRoute moves to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionTransitionsToLandAtGoal)
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

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
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

// Reverse traversal exhausted at the first route item: FollowRoute moves to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteExhaustionTransitionsToLandAtGoal)
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

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
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

// RTL must not stop at intermediate loiters: NAV_CMD_LOITER_TO_ALT is flattened to a plain waypoint.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemFlattensLoiterCommand)
{
	mission_item_s loiter_item = makePositionItem(kBaseLat, kBaseLon, kAlt + 20.f, NAV_CMD_LOITER_TO_ALT);
	loiter_item.autocontinue = false;
	loiter_item.time_inside = 8.f;

	executor.normalizeRouteMissionItemForTest(loiter_item);

	EXPECT_EQ(loiter_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(loiter_item.autocontinue);
	EXPECT_FLOAT_EQ(loiter_item.time_inside, 0.f);
}
