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
 * @file test_mission_base_vtol.cpp
 *
 * Unit tests for MissionBase VTOL transition detection and traversal methods:
 *   - getVtolStateAtMissionIndex()
 *   - vtolTransitionActionForTarget()
 *   - findNextPositionIndexNoJump()
 *   - findPreviousPositionIndexNoJump()
 *
 * These tests verify the centralized transition logic that was previously
 * duplicated in MissionRoutePlanner. The executor (RtlMissionSafePointFollow)
 * now calls these MissionBase methods directly instead of going through
 * the planner.
 *
 * Uses a MissionBaseTestPeer subclass that overrides loadMissionItemFromCache
 * with a vector-backed implementation (same pattern as VectorProvider in
 * the planner tests), and publishes vehicle_status via uORB to control
 * the VTOL state seen by vtolTransitionActionForTarget.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "mission_base.h"
#include "navigator.h"
#include "navigation.h"
#include "test_RTL_helpers.h"

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/Publication.hpp>

#include <algorithm>
#include <vector>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

/**
 * Minimal MissionBase subclass for testing vtolTransitionActionForTarget,
 * findNextPositionIndexNoJump, and findPreviousPositionIndexNoJump.
 *
 * Overrides loadMissionItemFromCache with a vector-backed implementation
 * so no dataman server is required. Pure virtual methods are stubbed out.
 * Vehicle status is published via uORB to control the VTOL state.
 *
 * Nullptr safety: passing nullptr for Navigator is safe because:
 *   - NavigatorMode constructor calls base on_inactivation()/on_inactive() which are no-ops
 *     (virtual dispatch during construction resolves to the base class, not the override)
 *   - ModuleParams(nullptr) explicitly null-checks before dereferencing the parent pointer
 *   - Tests must NOT call on_activation(), on_inactivation(), or on_active() which
 *     dereference _navigator (e.g. _navigator->disable_camera_trigger())
 */
class MissionBaseTestPeer : public MissionBase
{
public:
	explicit MissionBaseTestPeer(Navigator *navigator = nullptr)
		: MissionBase(navigator, 64, 0)
	{
	}

	~MissionBaseTestPeer() override = default;

	// Stub out pure virtual methods — not exercised by the transition tests.
	void setActiveMissionItems() override {}
	bool setNextMissionItem() override { return false; }

	/**
	 * @brief Override loadMissionItemFromCache with vector-backed storage.
	 *
	 * Bypasses DatamanCache entirely so no dataman server is needed.
	 */
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		if (std::find(_load_failure_indices.begin(), _load_failure_indices.end(), index)
		    != _load_failure_indices.end()) {
			return false;
		}

		if (index < 0 || index >= static_cast<int32_t>(_items.size())) {
			return false;
		}

		mission_item = _items[static_cast<size_t>(index)];
		return true;
	}

	/**
	 * @brief Load a mission into the vector-backed store for testing.
	 */
	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_items = items;
		_mission.count = static_cast<int32_t>(items.size());
		_mission.current_seq = 0;
		clearLoadFailures();
	}

	/**
	 * @brief Publish a vehicle_status message to configure the VTOL state.
	 */
	void setVehicleStatus(bool is_vtol, bool is_fixed_wing, bool in_transition_to_fw)
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

		// Force the subscription to pick up the published data.
		_vehicle_status_sub.update();
	}

	void setLoadFailureIndices(std::initializer_list<int32_t> indices)
	{
		_load_failure_indices.assign(indices.begin(), indices.end());
	}

	void clearLoadFailures()
	{
		_load_failure_indices.clear();
	}

	void setMissionUploadVtolState(uint8_t vtol_state)
	{
		_vtol_state_on_mission_upload = vtol_state;
	}

	// Expose protected methods for testing.
	using MissionBase::vtolTransitionActionForTarget;
	using MissionBase::getVtolStateAtMissionIndex;
	using MissionBase::VtolTransitionAction;
	using MissionBase::WorkItemType;
	using MissionBase::setupJoinRoute;
	using MissionBase::findNextPositionIndexNoJump;
	using MissionBase::findPreviousPositionIndexNoJump;
	using MissionBase::getNextPositionItems;
	using MissionBase::getPreviousPositionItems;
	using MissionBase::updateLastFlownLoopSegmentForNominalAdvance;
	using MissionBase::computeFrontTransitionAlignmentYaw;
	using MissionBase::checkMissionRestart;

	void setCurrentSequence(int32_t index)
	{
		_mission.current_seq = index;
	}

	int32_t currentSequenceForTest() const
	{
		return _mission.current_seq;
	}

	MissionRoutePlanner::JoinContext routeJoinContextForTest() const
	{
		return _route_join_context;
	}

	VtolTransitionAction joinTransitionActionForTest() const
	{
		return _route_join_context.transition_action;
	}

	WorkItemType workItemTypeForTest() const
	{
		return _work_item_type;
	}

	void setMissionRestartState(bool mission_has_been_activated, bool system_disarmed_while_inactive,
				    int inactivation_index = -1)
	{
		_mission_has_been_activated = mission_has_been_activated;
		_system_disarmed_while_inactive = system_disarmed_while_inactive;
		_inactivation_index = inactivation_index;
	}

private:
	std::vector<mission_item_s> _items;
	std::vector<int32_t> _load_failure_indices;
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
};

/**
 * @brief Base fixture for MissionBase helper tests that need PX4 runtime services.
 */
class MissionBaseVtolTest : public NavigatorDatamanTestBase
{
protected:
	MissionBaseTestPeer mission_base{};

	void SetUp() override
	{
		// Reset state between tests to prevent leakage from previous runs.
		mission_base.loadTestMission({});
		mission_base.setVehicleStatus(false, false, false);
		mission_base.setMissionUploadVtolState(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	}
};

/** @brief Covers VTOL-state reconstruction from mission transition commands. */
class MissionBaseVtolStateScanTest : public MissionBaseVtolTest {};
/** @brief Covers front-transition alignment yaw selection near join-route targets. */
class MissionBaseFrontTransitionAlignmentTest : public MissionBaseVtolTest {};
/** @brief Covers transition-action selection before entering VTOL route segments. */
class MissionBaseTransitionActionTest : public MissionBaseVtolTest {};
/** @brief Covers join-route setup and execution-side join-context corrections. */
class MissionBaseJoinRouteTest : public MissionBaseVtolTest {};
/** @brief Covers mission traversal helpers that skip non-position control items. */
class MissionBaseTraversalTest : public MissionBaseVtolTest {};
/** @brief Covers active loop-segment tracking used by replanning. */
class MissionBaseLoopTrackingTest : public MissionBaseVtolTest {};
/** @brief Covers mission restart behavior after inactive finished missions. */
class MissionBaseRestartBehaviorTest : public MissionBaseVtolTest {};

static float wrappedAngleError(float a, float b)
{
	return atan2f(sinf(a - b), cosf(a - b));
}

static constexpr float kYawAlignmentTolerance = 1e-4f;

// WHY: A mission without explicit VTOL transition commands should default to MC state
//      at every index, because getVtolStateAtMissionIndex scans backward for the last
//      DO_VTOL_TRANSITION and returns MC when none is found.
// WHAT: Two plain waypoints → both report VEHICLE_VTOL_STATE_MC.
TEST_F(MissionBaseVtolStateScanTest, DefaultStateIsMC)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(1),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// WHY: Missions uploaded while the VTOL is already in FW mode must preserve that state when
//      no explicit DO_VTOL_TRANSITION exists before the queried anchor.
// WHAT: Two plain waypoints with upload state forced to FW -> both report VEHICLE_VTOL_STATE_FW.
TEST_F(MissionBaseVtolStateScanTest, DefaultStateCanStartInFw)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setMissionUploadVtolState(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(1),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

// WHY: After completing JOIN_ROUTE, a front-transition should normally align with the
//      currently targeted waypoint so route following starts on the correct leg.
// WHAT: A target outside acceptance radius keeps the alignment on the current target.
TEST_F(MissionBaseFrontTransitionAlignmentTest, FrontTransitionAlignmentUsesCurrentTargetByDefault)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};
	items[1].acceptance_radius = 20.f;
	items[2].acceptance_radius = 20.f;

	mission_base_peer.loadTestMission(items);
	mission_base_peer.setCurrentSequence(1);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0003;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt;
	*navigator.get_global_position() = global_position;

	const float yaw = mission_base_peer.computeFrontTransitionAlignmentYaw(1, false);
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[1].lat, items[1].lon);

	ASSERT_TRUE(PX4_ISFINITE(yaw));
	EXPECT_NEAR(wrappedAngleError(yaw, expected_yaw), 0.f, kYawAlignmentTolerance);
}

// WHY: Once the current target is already reached at the end of JOIN_ROUTE, a front-transition
//      should point at the next route position instead of re-aiming at the already-accepted waypoint.
// WHAT: Being within the current target acceptance radius aligns to the next position item.
TEST_F(MissionBaseFrontTransitionAlignmentTest, FrontTransitionAlignmentUsesNextTargetInsideAcceptanceRadius)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon + 0.001, kAlt),
	};
	items[1].acceptance_radius = 40.f;
	items[2].acceptance_radius = 40.f;

	mission_base_peer.loadTestMission(items);
	mission_base_peer.setCurrentSequence(1);

	vehicle_global_position_s global_position{};
	global_position.lat = items[1].lat;
	global_position.lon = items[1].lon;
	global_position.alt = kAlt;
	*navigator.get_global_position() = global_position;

	const float yaw = mission_base_peer.computeFrontTransitionAlignmentYaw(1, false);
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[2].lat, items[2].lon);

	ASSERT_TRUE(PX4_ISFINITE(yaw));
	EXPECT_NEAR(wrappedAngleError(yaw, expected_yaw), 0.f, kYawAlignmentTolerance);
}

// WHY: On reverse traversal, "next" means the previous position-bearing mission item.
// WHAT: When the current reverse target is already reached, alignment points behind it.
TEST_F(MissionBaseFrontTransitionAlignmentTest, FrontTransitionAlignmentUsesPreviousTargetWhenDirectionReversed)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon + 0.001, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon + 0.001, kAlt),
	};
	items[1].acceptance_radius = 40.f;
	items[2].acceptance_radius = 40.f;

	mission_base_peer.loadTestMission(items);
	mission_base_peer.setCurrentSequence(2);

	vehicle_global_position_s global_position{};
	global_position.lat = items[2].lat;
	global_position.lon = items[2].lon;
	global_position.alt = kAlt;
	*navigator.get_global_position() = global_position;

	const float yaw = mission_base_peer.computeFrontTransitionAlignmentYaw(2, true);
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[1].lat, items[1].lon);

	ASSERT_TRUE(PX4_ISFINITE(yaw));
	EXPECT_NEAR(wrappedAngleError(yaw, expected_yaw), 0.f, kYawAlignmentTolerance);
}

// WHY: If the current target is already reached but there is no next position-bearing mission item,
//      PX4 should keep the front-transition aligned with the current target instead of clearing it.
// WHAT: Being inside the current target acceptance radius at the route end still aligns to the current target.
TEST_F(MissionBaseFrontTransitionAlignmentTest, FrontTransitionAlignmentFallsBackToCurrentTargetWhenNoNextItem)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};
	items[1].acceptance_radius = 40.f;

	mission_base_peer.loadTestMission(items);
	mission_base_peer.setCurrentSequence(1);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.00095;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt;
	*navigator.get_global_position() = global_position;

	const float yaw = mission_base_peer.computeFrontTransitionAlignmentYaw(1, false);
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[1].lat, items[1].lon);

	ASSERT_TRUE(PX4_ISFINITE(yaw));
	EXPECT_NEAR(wrappedAngleError(yaw, expected_yaw), 0.f, kYawAlignmentTolerance);
}

// WHY: getVtolStateAtMissionIndex must detect a DO_VTOL_TRANSITION to FW and report
//      FW state at and after the transition index, while items before remain MC.
// WHAT: [WP, VTOL_FW, WP] → idx 0 is MC, idx 1 and 2 are FW.
TEST_F(MissionBaseVtolStateScanTest, FwTransitionDetectedAtAnchor)
{
	//   idx: 0=WP, 1=VTOL_FW, 2=WP
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	// WHEN/THEN: Before the transition → MC; at/after → FW.
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(1),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(2),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

// WHY: When multiple transitions exist (MC→FW→MC), getVtolStateAtMissionIndex must
//      return the state established by the *most recent* transition at or before the
//      queried index, not just the first one found.
// WHAT: [WP, VTOL_FW, WP, VTOL_MC, WP] → idx 0 MC, idx 2 FW, idx 4 MC.
TEST_F(MissionBaseVtolStateScanTest, MultipleTransitionsReturnsLatest)
{
	//   idx: 0=WP, 1=VTOL_FW, 2=WP, 3=VTOL_MC, 4=WP
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(2),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.getVtolStateAtMissionIndex(4),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// WHY: Invalid DO_VTOL_TRANSITION values must be ignored instead of inventing a bogus vehicle state.
// WHAT: An unsupported transition value leaves the queried state in MC.
TEST_F(MissionBaseVtolStateScanTest, InvalidTransitionValueIsIgnored)
{
	// GIVEN: A mission containing a DO_VTOL_TRANSITION with an unsupported state value.
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(42),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	// WHEN: getVtolStateAtMissionIndex scans past the invalid transition item.
	const uint8_t state_at_transition = mission_base.getVtolStateAtMissionIndex(1);
	const uint8_t state_after_transition = mission_base.getVtolStateAtMissionIndex(2);

	// THEN: The invalid transition is ignored and the state remains MC.
	EXPECT_EQ(state_at_transition, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(state_after_transition, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// WHY: Cache read failures during backward VTOL-state scans must degrade to a safe default.
// WHAT: A failed read before the transition causes getVtolStateAtMissionIndex to return MC.
TEST_F(MissionBaseVtolStateScanTest, CacheReadFailureDuringStateScanFallsBackToMc)
{
	// GIVEN: A mission whose VTOL transition item cannot be loaded from the cache.
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setLoadFailureIndices({1});

	// WHEN: getVtolStateAtMissionIndex scans backward across the unreadable transition item.
	const uint8_t state = mission_base.getVtolStateAtMissionIndex(2);

	// THEN: The method falls back to MC instead of exposing stale or undefined state.
	EXPECT_EQ(state, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// WHY: vtolTransitionActionForTarget must return None for non-VTOL vehicles regardless
//      of mission content or direction, because transition commands are meaningless for
//      multicopters or fixed-wing-only aircraft. This covers both traversal directions in one test
//      so a regression in either path is caught without keeping a separate parameterized fixture.
// WHAT: Non-VTOL vehicle with VTOL_FW in mission → None for both forward and reverse traversal.
TEST_F(MissionBaseTransitionActionTest, NonVtolReturnsNoneInBothDirections)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setVehicleStatus(false, false, false); // non-VTOL MC

	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(2, false),
		  MissionBaseTestPeer::VtolTransitionAction::None);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::None);
}

// WHY: When the executor flies in reverse, the "anchor" for the target segment is the
//      next position item *after* the target. If that anchor is in an MC zone and the
//      vehicle is currently in FW mode, a BackTransition is required so the VTOL can land
//      or hover at the MC-zone waypoint.
// WHAT: FW vehicle, target 2 reversed → anchor in MC zone → BackTransition.
TEST_F(MissionBaseTransitionActionTest, BackTransitionDetectedInReverse)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setVehicleStatus(true, true, false); // VTOL in FW mode

	// Target index 2 reversed: anchor is idx 3+ → VTOL_MC → MC zone.
	// FW vehicle needs BackTransition.
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::BackTransition);
}

// WHY: When the executor flies in reverse and the anchor lands in an FW zone while the
//      vehicle is in MC mode, a FrontTransition is required so the VTOL can cruise in
//      fixed-wing mode along the FW segment.
// WHAT: MC vehicle, target 1 reversed → anchor in FW zone → FrontTransition.
TEST_F(MissionBaseTransitionActionTest, FrontTransitionDetectedInReverse)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setVehicleStatus(true, false, false); // VTOL in MC mode

	// Target index 1 reversed: anchor is idx 2+ → finds WP at idx 3,
	// walk back from 3 finds VTOL_FW at idx 2 → FW zone.
	// MC vehicle needs FrontTransition.
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(1, true),
		  MissionBaseTestPeer::VtolTransitionAction::FrontTransition);
}

// WHY: Mission and RTL both arm JOIN_ROUTE from planner output, so the helper that takes
//      JoinContext + Path must be the single owner of transition selection and stored join state.
// WHAT: Forward path into an FW segment from MC mode -> setupJoinRoute stores FrontTransition
//       in both the caller context and the armed join state, leaves skip-altitude unset,
//       preserves the join projection,
//       and arms WORK_ITEM_TYPE_JOIN_ROUTE.
TEST_F(MissionBaseJoinRouteTest, SetupJoinRouteFromPathUsesSharedTransitionLogic)
{
	// GIVEN: A VTOL mission whose target waypoint lies in an FW segment while the vehicle is in MC mode.
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setVehicleStatus(true, false, false);

	MissionRoutePlanner::JoinContext join_context{};
	join_context.projection = {kBaseLat + 0.0005, kBaseLon, kAlt + 10.f};
	join_context.direction_reversed = false;

	MissionRoutePlanner::Path path{};
	path.direction_reversed = false;
	path.first_item_index = 2;
	path.first_item_cmd = NAV_CMD_WAYPOINT;
	path.dist = 50.f;

	Navigator navigator;
	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0002;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt + 3.f;
	*navigator.get_global_position() = global_position;

	// WHEN: The shared helper arms the virtual branch-in from planner output.
	mission_base.setupJoinRoute(join_context, path, global_position.alt);

	// THEN: The helper stores the same transition action in the caller context and armed state.
	EXPECT_EQ(join_context.transition_action, MissionBaseTestPeer::VtolTransitionAction::FrontTransition);
	EXPECT_FALSE(join_context.skip_altitude_requirement);
	EXPECT_EQ(mission_base.joinTransitionActionForTest(), MissionBaseTestPeer::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission_base.workItemTypeForTest(), MissionBaseTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_DOUBLE_EQ(mission_base.routeJoinContextForTest().projection.lat, join_context.projection.lat);
	EXPECT_DOUBLE_EQ(mission_base.routeJoinContextForTest().projection.lon, join_context.projection.lon);
	EXPECT_FLOAT_EQ(mission_base.routeJoinContextForTest().projection.alt, join_context.projection.alt);
	EXPECT_EQ(mission_base.routeJoinContextForTest().direction_reversed, join_context.direction_reversed);
	EXPECT_EQ(mission_base.routeJoinContextForTest().skip_altitude_requirement,
		  join_context.skip_altitude_requirement);
}

// WHY: Landing joins that are already inside the target acceptance radius should skip the
//      planned join altitude in MissionBase, because that is an execution-side correction
//      rather than planner geometry.
// WHAT: Near a landing target, setupJoinRoute marks skip_altitude_requirement and updates
//       the join altitude to the vehicle's current altitude.
TEST_F(MissionBaseJoinRouteTest, SetupJoinRouteAppliesSkipAltitudeRequirementNearLand)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);
	mission_base_peer.setVehicleStatus(false, false, false);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0005;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt - 6.f;
	*navigator.get_global_position() = global_position;

	MissionRoutePlanner::JoinContext join_context{};
	join_context.projection = {kBaseLat + 0.0005, kBaseLon, kAlt - 10.f};
	join_context.direction_reversed = false;

	MissionRoutePlanner::Path path{};
	path.direction_reversed = false;
	path.in_first_item_acc_rad = true;
	path.first_item_index = 2;
	path.first_item_cmd = NAV_CMD_LAND;
	path.dist = 8.f;

	mission_base_peer.setupJoinRoute(join_context, path, global_position.alt);

	EXPECT_TRUE(join_context.skip_altitude_requirement);
	EXPECT_FLOAT_EQ(join_context.projection.alt, global_position.alt);
	EXPECT_EQ(join_context.transition_action, MissionBaseTestPeer::VtolTransitionAction::None);
	EXPECT_TRUE(mission_base_peer.routeJoinContextForTest().skip_altitude_requirement);
	EXPECT_FLOAT_EQ(mission_base_peer.routeJoinContextForTest().projection.alt, global_position.alt);
}

// WHY: RTL can pre-seed skip_altitude_requirement for takeoff-endpoint fallbacks before
//      MissionBase arms JOIN_ROUTE, so setupJoinRoute must preserve that hint and apply
//      the same altitude correction even when the path itself is not a landing command.
// WHAT: A caller-provided skip-altitude hint updates the join altitude to the vehicle altitude
//       and survives setupJoinRoute with no VTOL transition required.
TEST_F(MissionBaseJoinRouteTest, SetupJoinRouteHonorsCallerSkipAltitudeHint)
{
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);
	mission_base_peer.setVehicleStatus(false, false, false);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0002;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt + 3.f;
	*navigator.get_global_position() = global_position;

	MissionRoutePlanner::JoinContext join_context{};
	join_context.projection = {kBaseLat + 0.0002, kBaseLon, kAlt + 30.f};
	join_context.direction_reversed = true;
	join_context.skip_altitude_requirement = true;

	MissionRoutePlanner::Path path{};
	path.direction_reversed = true;
	path.in_first_item_acc_rad = false;
	path.first_item_index = 0;
	path.first_item_cmd = NAV_CMD_TAKEOFF;
	path.dist = 4.f;

	mission_base_peer.setupJoinRoute(join_context, path, global_position.alt);

	EXPECT_TRUE(join_context.skip_altitude_requirement);
	EXPECT_FLOAT_EQ(join_context.projection.alt, global_position.alt);
	EXPECT_EQ(join_context.transition_action, MissionBaseTestPeer::VtolTransitionAction::None);
	EXPECT_TRUE(mission_base_peer.routeJoinContextForTest().skip_altitude_requirement);
	EXPECT_FLOAT_EQ(mission_base_peer.routeJoinContextForTest().projection.alt, global_position.alt);
}

// WHY: The executor walks the route forward and must know, for each target waypoint,
//      whether a transition is needed *before* commanding that segment. This test
//      exercises the nominal forward case with a realistic takeoff→FW cruise→MC landing
//      mission, verifying both MC→FW (FrontTransition) and FW→MC (BackTransition) cases.
// WHAT: Full VTOL mission forward: MC vehicle sees FrontTransition in FW zone, FW vehicle
//       sees BackTransition in MC zone, and matching zones return None.
TEST_F(MissionBaseTransitionActionTest, MidRouteTransitionsDetectedNominal)
{
	//   idx: 0=Takeoff, 1=WP, 2=VTOL_FW, 3=WP, 4=WP, 5=VTOL_MC, 6=WP, 7=Land
	std::vector<mission_item_s> items = {
		makeTakeoffItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.005, kBaseLon, kAlt + 50.f),
		makePositionItem(kBaseLat + 0.008, kBaseLon, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.010, kBaseLon, kAlt + 30.f),
		makeLandItem(kBaseLat + 0.012, kBaseLon, kAlt - 10.f),
	};

	mission_base.loadTestMission(items);

	// MC vehicle: FW zones require FrontTransition, MC zones are None.
	mission_base.setVehicleStatus(true, false, false); // VTOL MC
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(3, false),
		  MissionBaseTestPeer::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(4, false),
		  MissionBaseTestPeer::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(6, false),
		  MissionBaseTestPeer::VtolTransitionAction::None);

	// FW vehicle: MC zones require BackTransition, FW zones are None.
	mission_base.setVehicleStatus(true, true, false); // VTOL FW
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(3, false),
		  MissionBaseTestPeer::VtolTransitionAction::None);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(4, false),
		  MissionBaseTestPeer::VtolTransitionAction::None);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(6, false),
		  MissionBaseTestPeer::VtolTransitionAction::BackTransition);
}

// WHY: The RTL executor may walk the route in reverse (direction_reversed=true). In that
//      case the anchor for each target is found by looking *forward* in mission index
//      space (i.e. the segment end in reverse). This test ensures the anchor lookup and
//      VTOL state detection work correctly for multiple reverse targets on the same mission.
// WHAT: Same full VTOL mission walked in reverse: FW vehicle sees BackTransition when
//       anchor is in MC zone, MC vehicle sees FrontTransition when anchor is in FW zone.
TEST_F(MissionBaseTransitionActionTest, MidRouteTransitionsDetectedReverse)
{
	std::vector<mission_item_s> items = {
		makeTakeoffItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.005, kBaseLon, kAlt + 50.f),
		makePositionItem(kBaseLat + 0.008, kBaseLon, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.010, kBaseLon, kAlt + 30.f),
		makeLandItem(kBaseLat + 0.012, kBaseLon, kAlt - 10.f),
	};

	mission_base.loadTestMission(items);

	// FW vehicle flying in reverse:
	// Target 4 reversed: anchor idx 5+ → VTOL_MC → MC zone → BackTransition.
	mission_base.setVehicleStatus(true, true, false); // VTOL FW
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(4, true),
		  MissionBaseTestPeer::VtolTransitionAction::BackTransition);

	// MC vehicle flying in reverse:
	// Target 1 reversed: anchor idx 2+ → finds VTOL_FW → FW zone → FrontTransition.
	mission_base.setVehicleStatus(true, false, false); // VTOL MC
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(1, true),
		  MissionBaseTestPeer::VtolTransitionAction::FrontTransition);

	// MC vehicle, target 6 reversed: anchor idx 7 → land item → getVtolState walks
	// back and finds VTOL_MC at idx 5 → MC zone → None.
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(6, true),
		  MissionBaseTestPeer::VtolTransitionAction::None);
}

// WHY: A mission with alternating FW/MC zones exercises the boundary conditions of the
//      anchor-lookup logic. The method must correctly identify which zone each target
//      falls into even when transitions are closely spaced, and return the right action
//      for both FW and MC vehicles in both forward and reverse directions.
// WHAT: 4-transition mission [MC, FW, FW, MC, MC, FW, FW, MC] with mixed vehicle states
//       and directions → correct FrontTransition / BackTransition / None for each case.
TEST_F(MissionBaseTransitionActionTest, MultiTransitionMissionDetectsCorrectAction)
{
	//   [WP, VTOL_FW, WP, WP, VTOL_MC, WP, VTOL_FW, WP, VTOL_MC, WP]
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                    // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),                           // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),                           // idx 3
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 4
		makePositionItem(kBaseLat + 0.003, kBaseLon, kAlt),                           // idx 5
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 6
		makePositionItem(kBaseLat + 0.004, kBaseLon, kAlt),                           // idx 7
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 8
		makePositionItem(kBaseLat + 0.005, kBaseLon, kAlt),                           // idx 9
	};

	mission_base.loadTestMission(items);

	// FW vehicle: target 2 (FW zone) reversed → None.
	mission_base.setVehicleStatus(true, true, false);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::None);

	// MC vehicle: target 2 (FW zone) reversed → FrontTransition.
	mission_base.setVehicleStatus(true, false, false);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::FrontTransition);

	// FW vehicle: target 7 (FW zone after VTOL_FW@6) reversed →
	//   anchor is idx 8+ → VTOL_MC → MC zone → BackTransition.
	mission_base.setVehicleStatus(true, true, false);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(7, true),
		  MissionBaseTestPeer::VtolTransitionAction::BackTransition);

	// MC vehicle: target 7 reversed → MC zone → None.
	mission_base.setVehicleStatus(true, false, false);
	EXPECT_EQ(mission_base.vtolTransitionActionForTarget(7, true),
		  MissionBaseTestPeer::VtolTransitionAction::None);
}

// WHY: findNextPositionIndexNoJump walks forward through the mission to find the next
//      position-bearing item. Non-position items (like DO_VTOL_TRANSITION) must be
//      skipped. This is the simplest case with no DO_JUMP items.
// WHAT: Starting from a VTOL transition item, the function skips it and returns the
//       next waypoint.
TEST_F(MissionBaseTraversalTest, FindNextSkipsNonPositionItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                    // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),                           // idx 2
	};

	mission_base.loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base.findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 2);
}

// WHY: findNextPositionIndexNoJump must NOT skip DO_JUMP items — it treats them as
//      non-position items and continues past them. This is critical for the RTL executor
//      which needs to find the next physical waypoint without following jump control flow.
// WHAT: [WP, DO_JUMP, WP, WP] — starting from idx 1 (DO_JUMP), returns idx 2 (next WP).
TEST_F(MissionBaseTraversalTest, FindNextSkipsDoJumpItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),              // idx 0
		makeDoJump(0, 3),                                // idx 1: DO_JUMP
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),     // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),     // idx 3
	};

	mission_base.loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base.findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 2);
}

// WHY: When multiple non-position items (DO_JUMP + VTOL transition) are stacked
//      consecutively, findNextPositionIndexNoJump must skip all of them and return the
//      first position item that follows.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] — starting from idx 1, skips both and returns idx 3.
TEST_F(MissionBaseTraversalTest, FindNextSkipsConsecutiveNonPositionItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                    // idx 0
		makeDoJump(0, 5),                                                      // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),                           // idx 3
	};

	mission_base.loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base.findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 3);
}

// WHY: When no position item exists after the start index, findNextPositionIndexNoJump
//      must return false so callers know there is no valid forward target.
// WHAT: [WP, DO_JUMP] — starting from idx 1, no position item follows → returns false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseAtEnd)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),  // idx 0
		makeDoJump(0, 3),                    // idx 1
	};

	mission_base.loadTestMission(items);

	int32_t next = -1;
	EXPECT_FALSE(mission_base.findNextPositionIndexNoJump(1, next));
}

// WHY: Route traversal must stop cleanly when the next cache read fails.
// WHAT: A cache failure on the next position item makes findNextPositionIndexNoJump return false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseOnCacheReadFailure)
{
	// GIVEN: A mission whose next position item cannot be loaded.
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeDoJump(0, 3),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setLoadFailureIndices({2});

	int32_t next = -1;

	// WHEN: findNextPositionIndexNoJump advances past the DO_JUMP item.
	const bool found = mission_base.findNextPositionIndexNoJump(1, next);

	// THEN: The unreadable next position item causes a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(next, -1);
}

// WHY: findPreviousPositionIndexNoJump walks backward and must explicitly skip DO_JUMP
//      items (it checks nav_cmd == NAV_CMD_DO_JUMP and continues). Without this, the
//      function would stop at a DO_JUMP item which has no position data.
// WHAT: [WP, WP, DO_JUMP, WP] — starting from idx 3, skips DO_JUMP at idx 2, returns idx 1.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsDoJumpItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),              // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),     // idx 1
		makeDoJump(0, 3),                                // idx 2: DO_JUMP
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),     // idx 3
	};

	mission_base.loadTestMission(items);

	int32_t prev = -1;
	EXPECT_TRUE(mission_base.findPreviousPositionIndexNoJump(3, prev));
	EXPECT_EQ(prev, 1);
}

// WHY: When multiple DO_JUMP items are stacked before the start index,
//      findPreviousPositionIndexNoJump must skip all of them.
// WHAT: [WP, DO_JUMP, DO_JUMP, WP] — starting from idx 3, skips both jumps, returns idx 0.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsConsecutiveDoJumps)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),              // idx 0
		makeDoJump(0, 3),                                // idx 1
		makeDoJump(0, 2),                                // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),     // idx 3
	};

	mission_base.loadTestMission(items);

	int32_t prev = -1;
	EXPECT_TRUE(mission_base.findPreviousPositionIndexNoJump(3, prev));
	EXPECT_EQ(prev, 0);
}

// WHY: When there is no position item before the start index (only DO_JUMPs or nothing),
//      findPreviousPositionIndexNoJump must return false.
// WHAT: [DO_JUMP, WP] — starting from idx 1, idx 0 is DO_JUMP → returns false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseWhenOnlyJumpsBefore)
{
	std::vector<mission_item_s> items = {
		makeDoJump(0, 3),                            // idx 0: DO_JUMP
		makePositionItem(kBaseLat, kBaseLon, kAlt),          // idx 1
	};

	mission_base.loadTestMission(items);

	int32_t prev = -1;
	EXPECT_FALSE(mission_base.findPreviousPositionIndexNoJump(1, prev));
}

// WHY: getNextPositionItems is used by Mission and mission-based RTL flows, so it must
//      follow active DO_JUMP control flow rather than treating jumps as geometry-only.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 2 returns WP0 then WP1.
TEST_F(MissionBaseTraversalTest, GetNextPositionItemsFollowsActiveDoJump)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	int32_t next_items[2] = {-1, -1};
	size_t num_found_items = 0;
	mission_base.getNextPositionItems(2, next_items, num_found_items, 2u);

	ASSERT_EQ(num_found_items, 2u);
	EXPECT_EQ(next_items[0], 0);
	EXPECT_EQ(next_items[1], 1);
}

// WHY: getPreviousPositionItems is used in both mission base and rtl mission fast reverse.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 returns WP0 as the previous item.
TEST_F(MissionBaseTraversalTest, GetPreviousPositionItemsFollowsActiveDoJump)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);

	int32_t previous_items[1] = {-1};
	size_t num_found_items = 0;
	mission_base.getPreviousPositionItems(3, previous_items, num_found_items, 1u);

	ASSERT_EQ(num_found_items, 1u);
	EXPECT_EQ(previous_items[0], 0);
}

// WHY: Projection-based replans must remember which active DO_JUMP edge the vehicle was flying
//      before advancing, otherwise rejoin logic near loops can snap to the wrong segment.
// WHAT: [WP0, WP1, WP2, DO_JUMP->0 repeat=3 current=1, WP3] at current_seq=2 tracks loop edge 2->0.
TEST_F(MissionBaseLoopTrackingTest, TracksActiveLoopSegmentBeforeNominalAdvance)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
		makeDoJump(0, 3, 1),
		makePositionItem(kBaseLat + 0.003, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setCurrentSequence(2);

	MissionRoutePlanner::Segment segment{};
	mission_base.updateLastFlownLoopSegmentForNominalAdvance(segment);

	EXPECT_TRUE(segment.valid());
	EXPECT_TRUE(segment.is_loop);
	EXPECT_EQ(segment.start.idx, 2);
	EXPECT_EQ(segment.start.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_EQ(segment.end.idx, 0);
	EXPECT_EQ(segment.end.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_EQ(segment.loops_remaining, 2);
}

// WHY: When there is no active loop ahead, the cached loop edge must be cleared so later replans
//      do not keep biasing candidate selection toward a stale jump segment.
// WHAT: Plain waypoint mission leaves the returned segment invalid.
TEST_F(MissionBaseLoopTrackingTest, ClearsLoopSegmentWhenNoActiveJumpAhead)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setCurrentSequence(1);

	MissionRoutePlanner::Segment segment{};
	segment.start.idx = 7;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = 2;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	segment.is_loop = true;
	segment.loops_remaining = 5;

	mission_base.updateLastFlownLoopSegmentForNominalAdvance(segment);

	EXPECT_FALSE(segment.valid());
	EXPECT_FALSE(segment.is_loop);
	EXPECT_EQ(segment.start.idx, -1);
	EXPECT_EQ(segment.end.idx, -1);
	EXPECT_EQ(segment.loops_remaining, 0);
}

// WHY: Malformed DO_JUMP targets must clear any cached loop edge instead of leaving stale state behind.
// WHAT: An invalid jump target makes updateLastFlownLoopSegmentForNominalAdvance return an invalid segment.
TEST_F(MissionBaseLoopTrackingTest, InvalidDoJumpTargetClearsLoopSegment)
{
	// GIVEN: A mission with a DO_JUMP whose target index does not resolve to a valid position item.
	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(10, 3, 1),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base.loadTestMission(items);
	mission_base.setCurrentSequence(1);

	MissionRoutePlanner::Segment segment{};
	segment.start.idx = 7;
	segment.end.idx = 2;
	segment.is_loop = true;
	segment.loops_remaining = 4;

	// WHEN: The helper tries to derive the active loop segment from the malformed jump.
	mission_base.updateLastFlownLoopSegmentForNominalAdvance(segment);

	// THEN: The cached loop segment is cleared instead of keeping stale routing state.
	EXPECT_FALSE(segment.valid());
	EXPECT_FALSE(segment.is_loop);
	EXPECT_EQ(segment.start.idx, -1);
	EXPECT_EQ(segment.end.idx, -1);
	EXPECT_EQ(segment.loops_remaining, 0);
}

// WHY: RTL Type 6 can finish through a synthetic route-follow/landing pipeline without advancing
//      current_seq to the final uploaded mission item, so restart-on-activation must also honor
//      mission_result.finished instead of only checking the raw sequence index.
// WHAT: A disarmed reactivation with finished=true while current_seq still points mid-mission
//       resets the mission back to seq 0 and clears the finished flag.
TEST_F(MissionBaseRestartBehaviorTest, CheckMissionRestartResetsFinishedMissionWithoutLastSequence)
{
	// GIVEN: A valid 3-item mission, current_seq still at the middle waypoint, and a mode that
	//        already declared the mission finished before deactivation.
	Navigator navigator;
	MissionBaseTestPeer mission_base_peer(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	};

	mission_base_peer.loadTestMission(items);
	mission_base_peer.setCurrentSequence(1);
	mission_base_peer.setMissionRestartState(true, true, 1);
	navigator.get_mission_result()->valid = true;
	navigator.get_mission_result()->finished = true;

	// WHEN: checkMissionRestart runs on activation.
	mission_base_peer.checkMissionRestart();

	// THEN: Mission execution restarts from the beginning and the finished latch is cleared.
	EXPECT_EQ(mission_base_peer.currentSequenceForTest(), 0);
	EXPECT_FALSE(navigator.get_mission_result()->finished);
}
