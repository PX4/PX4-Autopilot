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
 * duplicated in RtlRoutePlanner. The executor (RtlMissionSafePointFollow)
 * now calls these MissionBase methods directly instead of going through
 * the planner.
 *
 * Uses a TestMissionBase subclass that overrides loadMissionItemFromCache
 * with a vector-backed implementation (same pattern as VectorProvider in
 * the planner tests), and publishes vehicle_status via uORB to control
 * the VTOL state seen by vtolTransitionActionForTarget.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "mission_base.h"
#include "navigation.h"

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/Publication.hpp>

#include <vector>

// ============================================================================
// Test subclass of MissionBase
// ============================================================================

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
class TestMissionBase : public MissionBase
{
public:
	TestMissionBase()
		: MissionBase(nullptr, 64, 0)
	{
	}

	~TestMissionBase() override = default;

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

	// Expose protected methods for testing.
	using MissionBase::vtolTransitionActionForTarget;
	using MissionBase::getVtolStateAtMissionIndex;
	using MissionBase::VtolTransitionAction;
	using MissionBase::findNextPositionIndexNoJump;
	using MissionBase::findPreviousPositionIndexNoJump;

private:
	std::vector<mission_item_s> _items;
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
};

// ============================================================================
// Mission item factory helpers (mirrors test_RTL_helpers.h)
// ============================================================================

static mission_item_s makePositionItem(double lat, double lon, float alt,
				       uint16_t nav_cmd = NAV_CMD_WAYPOINT)
{
	mission_item_s item{};
	item.lat = lat;
	item.lon = lon;
	item.altitude = alt;
	item.nav_cmd = nav_cmd;
	item.frame = NAV_FRAME_GLOBAL;
	item.altitude_is_relative = false;
	item.autocontinue = true;
	return item;
}

static mission_item_s makeVtolTransitionItem(uint8_t target_state)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(target_state);
	return item;
}

static mission_item_s makeTakeoffItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_TAKEOFF);
	item.autocontinue = false;
	return item;
}

static mission_item_s makeLandItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_LAND);
	item.autocontinue = false;
	return item;
}

static mission_item_s makeDoJump(int16_t jump_target_index, uint16_t repeat_count,
				 uint16_t current_count = 0)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_JUMP;
	item.do_jump_mission_index = jump_target_index;
	item.do_jump_repeat_count = repeat_count;
	item.do_jump_current_count = current_count;
	return item;
}

// ============================================================================
// Test fixture
// ============================================================================

class MissionBaseVtolTest : public ::testing::Test
{
protected:
	static TestMissionBase *mission_base;

	static void SetUpTestSuite()   { mission_base = new TestMissionBase(); }
	static void TearDownTestSuite() { delete mission_base; mission_base = nullptr; }

	void SetUp() override
	{
		// Reset state between tests to prevent leakage from previous runs.
		mission_base->loadTestMission({});
		mission_base->setVehicleStatus(false, false, false);
	}

	static constexpr double kLat = 47.397742;
	static constexpr double kLon = 8.545594;
	static constexpr float kAlt = 500.f;
};

TestMissionBase *MissionBaseVtolTest::mission_base = nullptr;

// ============================================================================
// getVtolStateAtMissionIndex tests
// ============================================================================

// WHY: A mission without explicit VTOL transition commands should default to MC state
//      at every index, because getVtolStateAtMissionIndex scans backward for the last
//      DO_VTOL_TRANSITION and returns MC when none is found.
// WHAT: Two plain waypoints → both report VEHICLE_VTOL_STATE_MC.
TEST_F(MissionBaseVtolTest, DefaultStateIsMC)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	};

	mission_base->loadTestMission(items);

	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(1),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// WHY: getVtolStateAtMissionIndex must detect a DO_VTOL_TRANSITION to FW and report
//      FW state at and after the transition index, while items before remain MC.
// WHAT: [WP, VTOL_FW, WP] → idx 0 is MC, idx 1 and 2 are FW.
TEST_F(MissionBaseVtolTest, FwTransitionDetectedAtAnchor)
{
	//   idx: 0=WP, 1=VTOL_FW, 2=WP
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	};

	mission_base->loadTestMission(items);

	// WHEN/THEN: Before the transition → MC; at/after → FW.
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(1),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(2),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

// WHY: When multiple transitions exist (MC→FW→MC), getVtolStateAtMissionIndex must
//      return the state established by the *most recent* transition at or before the
//      queried index, not just the first one found.
// WHAT: [WP, VTOL_FW, WP, VTOL_MC, WP] → idx 0 MC, idx 2 FW, idx 4 MC.
TEST_F(MissionBaseVtolTest, MultipleTransitionsReturnsLatest)
{
	//   idx: 0=WP, 1=VTOL_FW, 2=WP, 3=VTOL_MC, 4=WP
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.001, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kLat + 0.002, kLon, kAlt),
	};

	mission_base->loadTestMission(items);

	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(0),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(2),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base->getVtolStateAtMissionIndex(4),
		  vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

// ============================================================================
// vtolTransitionActionForTarget tests
// ============================================================================

// ---------------------------------------------------------------------------
// Parameterized fixture for direction-independent tests (forward / reverse)
// ---------------------------------------------------------------------------

class MissionBaseVtolDirectionTest : public ::testing::TestWithParam<bool>
{
protected:
	static TestMissionBase *mission_base;

	static void SetUpTestSuite()   { mission_base = new TestMissionBase(); }
	static void TearDownTestSuite() { delete mission_base; mission_base = nullptr; }

	void SetUp() override
	{
		mission_base->loadTestMission({});
		mission_base->setVehicleStatus(false, false, false);
	}

	bool reversed() const { return GetParam(); }

	static constexpr double kLat = 47.397742;
	static constexpr double kLon = 8.545594;
	static constexpr float kAlt = 500.f;
};

TestMissionBase *MissionBaseVtolDirectionTest::mission_base = nullptr;

// WHY: vtolTransitionActionForTarget must return None for non-VTOL vehicles regardless
//      of mission content or direction, because transition commands are meaningless for
//      multicopters or fixed-wing-only aircraft. Each direction is evaluated independently
//      so a regression in one direction is caught even if the other passes.
// WHAT: Non-VTOL vehicle with VTOL_FW in mission → None for the parameterized direction.
TEST_P(MissionBaseVtolDirectionTest, NonVtolAlwaysReturnsNone)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	};

	mission_base->loadTestMission(items);
	mission_base->setVehicleStatus(false, false, false); // non-VTOL MC

	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(2, reversed()),
		  TestMissionBase::VtolTransitionAction::None);
}

INSTANTIATE_TEST_SUITE_P(Direction, MissionBaseVtolDirectionTest,
			 ::testing::Values(false, true),
			 [](const ::testing::TestParamInfo<bool> &param_info)
{
	return param_info.param ? "Reverse" : "Forward";
});

// WHY: When the executor flies in reverse, the "anchor" for the target segment is the
//      next position item *after* the target. If that anchor is in an MC zone and the
//      vehicle is currently in FW mode, a BackTransition is required so the VTOL can land
//      or hover at the MC-zone waypoint.
// WHAT: FW vehicle, target 2 reversed → anchor in MC zone → BackTransition.
TEST_F(MissionBaseVtolTest, BackTransitionDetectedInReverse)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.001, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kLat + 0.002, kLon, kAlt),
	};

	mission_base->loadTestMission(items);
	mission_base->setVehicleStatus(true, true, false); // VTOL in FW mode

	// Target index 2 reversed: anchor is idx 3+ → VTOL_MC → MC zone.
	// FW vehicle needs BackTransition.
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(2, true),
		  TestMissionBase::VtolTransitionAction::BackTransition);
}

// WHY: When the executor flies in reverse and the anchor lands in an FW zone while the
//      vehicle is in MC mode, a FrontTransition is required so the VTOL can cruise in
//      fixed-wing mode along the FW segment.
// WHAT: MC vehicle, target 1 reversed → anchor in FW zone → FrontTransition.
TEST_F(MissionBaseVtolTest, FrontTransitionDetectedInReverse)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.002, kLon, kAlt),
	};

	mission_base->loadTestMission(items);
	mission_base->setVehicleStatus(true, false, false); // VTOL in MC mode

	// Target index 1 reversed: anchor is idx 2+ → finds WP at idx 3,
	// walk back from 3 finds VTOL_FW at idx 2 → FW zone.
	// MC vehicle needs FrontTransition.
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(1, true),
		  TestMissionBase::VtolTransitionAction::FrontTransition);
}

// WHY: The executor walks the route forward and must know, for each target waypoint,
//      whether a transition is needed *before* commanding that segment. This test
//      exercises the nominal forward case with a realistic takeoff→FW cruise→MC landing
//      mission, verifying both MC→FW (FrontTransition) and FW→MC (BackTransition) cases.
// WHAT: Full VTOL mission forward: MC vehicle sees FrontTransition in FW zone, FW vehicle
//       sees BackTransition in MC zone, and matching zones return None.
TEST_F(MissionBaseVtolTest, MidRouteTransitionsDetectedNominal)
{
	//   idx: 0=Takeoff, 1=WP, 2=VTOL_FW, 3=WP, 4=WP, 5=VTOL_MC, 6=WP, 7=Land
	std::vector<mission_item_s> items = {
		makeTakeoffItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.005, kLon, kAlt + 50.f),
		makePositionItem(kLat + 0.008, kLon, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kLat + 0.010, kLon, kAlt + 30.f),
		makeLandItem(kLat + 0.012, kLon, kAlt - 10.f),
	};

	mission_base->loadTestMission(items);

	// MC vehicle: FW zones require FrontTransition, MC zones are None.
	mission_base->setVehicleStatus(true, false, false); // VTOL MC
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(3, false),
		  TestMissionBase::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(4, false),
		  TestMissionBase::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(6, false),
		  TestMissionBase::VtolTransitionAction::None);

	// FW vehicle: MC zones require BackTransition, FW zones are None.
	mission_base->setVehicleStatus(true, true, false); // VTOL FW
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(3, false),
		  TestMissionBase::VtolTransitionAction::None);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(4, false),
		  TestMissionBase::VtolTransitionAction::None);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(6, false),
		  TestMissionBase::VtolTransitionAction::BackTransition);
}

// WHY: The RTL executor may walk the route in reverse (direction_reversed=true). In that
//      case the anchor for each target is found by looking *forward* in mission index
//      space (i.e. the segment end in reverse). This test ensures the anchor lookup and
//      VTOL state detection work correctly for multiple reverse targets on the same mission.
// WHAT: Same full VTOL mission walked in reverse: FW vehicle sees BackTransition when
//       anchor is in MC zone, MC vehicle sees FrontTransition when anchor is in FW zone.
TEST_F(MissionBaseVtolTest, MidRouteTransitionsDetectedReverse)
{
	std::vector<mission_item_s> items = {
		makeTakeoffItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kLat + 0.005, kLon, kAlt + 50.f),
		makePositionItem(kLat + 0.008, kLon, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kLat + 0.010, kLon, kAlt + 30.f),
		makeLandItem(kLat + 0.012, kLon, kAlt - 10.f),
	};

	mission_base->loadTestMission(items);

	// FW vehicle flying in reverse:
	// Target 4 reversed: anchor idx 5+ → VTOL_MC → MC zone → BackTransition.
	mission_base->setVehicleStatus(true, true, false); // VTOL FW
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(4, true),
		  TestMissionBase::VtolTransitionAction::BackTransition);

	// MC vehicle flying in reverse:
	// Target 1 reversed: anchor idx 2+ → finds VTOL_FW → FW zone → FrontTransition.
	mission_base->setVehicleStatus(true, false, false); // VTOL MC
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(1, true),
		  TestMissionBase::VtolTransitionAction::FrontTransition);

	// MC vehicle, target 6 reversed: anchor idx 7 → land item → getVtolState walks
	// back and finds VTOL_MC at idx 5 → MC zone → None.
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(6, true),
		  TestMissionBase::VtolTransitionAction::None);
}

// WHY: A mission with alternating FW/MC zones exercises the boundary conditions of the
//      anchor-lookup logic. The method must correctly identify which zone each target
//      falls into even when transitions are closely spaced, and return the right action
//      for both FW and MC vehicles in both forward and reverse directions.
// WHAT: 4-transition mission [MC, FW, FW, MC, MC, FW, FW, MC] with mixed vehicle states
//       and directions → correct FrontTransition / BackTransition / None for each case.
TEST_F(MissionBaseVtolTest, MultiTransitionMissionDetectsCorrectAction)
{
	//   [WP, VTOL_FW, WP, WP, VTOL_MC, WP, VTOL_FW, WP, VTOL_MC, WP]
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),                                    // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 1
		makePositionItem(kLat + 0.001, kLon, kAlt),                           // idx 2
		makePositionItem(kLat + 0.002, kLon, kAlt),                           // idx 3
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 4
		makePositionItem(kLat + 0.003, kLon, kAlt),                           // idx 5
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 6
		makePositionItem(kLat + 0.004, kLon, kAlt),                           // idx 7
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 8
		makePositionItem(kLat + 0.005, kLon, kAlt),                           // idx 9
	};

	mission_base->loadTestMission(items);

	// FW vehicle: target 2 (FW zone) reversed → None.
	mission_base->setVehicleStatus(true, true, false);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(2, true),
		  TestMissionBase::VtolTransitionAction::None);

	// MC vehicle: target 2 (FW zone) reversed → FrontTransition.
	mission_base->setVehicleStatus(true, false, false);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(2, true),
		  TestMissionBase::VtolTransitionAction::FrontTransition);

	// FW vehicle: target 7 (FW zone after VTOL_FW@6) reversed →
	//   anchor is idx 8+ → VTOL_MC → MC zone → BackTransition.
	mission_base->setVehicleStatus(true, true, false);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(7, true),
		  TestMissionBase::VtolTransitionAction::BackTransition);

	// MC vehicle: target 7 reversed → MC zone → None.
	mission_base->setVehicleStatus(true, false, false);
	EXPECT_EQ(mission_base->vtolTransitionActionForTarget(7, true),
		  TestMissionBase::VtolTransitionAction::None);
}

// ============================================================================
// findNextPositionIndexNoJump / findPreviousPositionIndexNoJump tests
// ============================================================================

// WHY: findNextPositionIndexNoJump walks forward through the mission to find the next
//      position-bearing item. Non-position items (like DO_VTOL_TRANSITION) must be
//      skipped. This is the simplest case with no DO_JUMP items.
// WHAT: Starting from a VTOL transition item, the function skips it and returns the
//       next waypoint.
TEST_F(MissionBaseVtolTest, FindNextSkipsNonPositionItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),                                    // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 1
		makePositionItem(kLat + 0.001, kLon, kAlt),                           // idx 2
	};

	mission_base->loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base->findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 2);
}

// WHY: findNextPositionIndexNoJump must NOT skip DO_JUMP items — it treats them as
//      non-position items and continues past them. This is critical for the RTL executor
//      which needs to find the next physical waypoint without following jump control flow.
// WHAT: [WP, DO_JUMP, WP, WP] — starting from idx 1 (DO_JUMP), returns idx 2 (next WP).
TEST_F(MissionBaseVtolTest, FindNextSkipsDoJumpItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),              // idx 0
		makeDoJump(0, 3),                                // idx 1: DO_JUMP
		makePositionItem(kLat + 0.001, kLon, kAlt),     // idx 2
		makePositionItem(kLat + 0.002, kLon, kAlt),     // idx 3
	};

	mission_base->loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base->findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 2);
}

// WHY: When multiple non-position items (DO_JUMP + VTOL transition) are stacked
//      consecutively, findNextPositionIndexNoJump must skip all of them and return the
//      first position item that follows.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] — starting from idx 1, skips both and returns idx 3.
TEST_F(MissionBaseVtolTest, FindNextSkipsConsecutiveNonPositionItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),                                    // idx 0
		makeDoJump(0, 5),                                                      // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 2
		makePositionItem(kLat + 0.001, kLon, kAlt),                           // idx 3
	};

	mission_base->loadTestMission(items);

	int32_t next = -1;
	EXPECT_TRUE(mission_base->findNextPositionIndexNoJump(1, next));
	EXPECT_EQ(next, 3);
}

// WHY: When no position item exists after the start index, findNextPositionIndexNoJump
//      must return false so callers know there is no valid forward target.
// WHAT: [WP, DO_JUMP] — starting from idx 1, no position item follows → returns false.
TEST_F(MissionBaseVtolTest, FindNextReturnsFalseAtEnd)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),  // idx 0
		makeDoJump(0, 3),                    // idx 1
	};

	mission_base->loadTestMission(items);

	int32_t next = -1;
	EXPECT_FALSE(mission_base->findNextPositionIndexNoJump(1, next));
}

// WHY: findPreviousPositionIndexNoJump walks backward and must explicitly skip DO_JUMP
//      items (it checks nav_cmd == NAV_CMD_DO_JUMP and continues). Without this, the
//      function would stop at a DO_JUMP item which has no position data.
// WHAT: [WP, WP, DO_JUMP, WP] — starting from idx 3, skips DO_JUMP at idx 2, returns idx 1.
TEST_F(MissionBaseVtolTest, FindPreviousSkipsDoJumpItems)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),              // idx 0
		makePositionItem(kLat + 0.001, kLon, kAlt),     // idx 1
		makeDoJump(0, 3),                                // idx 2: DO_JUMP
		makePositionItem(kLat + 0.002, kLon, kAlt),     // idx 3
	};

	mission_base->loadTestMission(items);

	int32_t prev = -1;
	EXPECT_TRUE(mission_base->findPreviousPositionIndexNoJump(3, prev));
	EXPECT_EQ(prev, 1);
}

// WHY: When multiple DO_JUMP items are stacked before the start index,
//      findPreviousPositionIndexNoJump must skip all of them.
// WHAT: [WP, DO_JUMP, DO_JUMP, WP] — starting from idx 3, skips both jumps, returns idx 0.
TEST_F(MissionBaseVtolTest, FindPreviousSkipsConsecutiveDoJumps)
{
	std::vector<mission_item_s> items = {
		makePositionItem(kLat, kLon, kAlt),              // idx 0
		makeDoJump(0, 3),                                // idx 1
		makeDoJump(0, 2),                                // idx 2
		makePositionItem(kLat + 0.001, kLon, kAlt),     // idx 3
	};

	mission_base->loadTestMission(items);

	int32_t prev = -1;
	EXPECT_TRUE(mission_base->findPreviousPositionIndexNoJump(3, prev));
	EXPECT_EQ(prev, 0);
}

// WHY: When there is no position item before the start index (only DO_JUMPs or nothing),
//      findPreviousPositionIndexNoJump must return false.
// WHAT: [DO_JUMP, WP] — starting from idx 1, idx 0 is DO_JUMP → returns false.
TEST_F(MissionBaseVtolTest, FindPreviousReturnsFalseWhenOnlyJumpsBefore)
{
	std::vector<mission_item_s> items = {
		makeDoJump(0, 3),                            // idx 0: DO_JUMP
		makePositionItem(kLat, kLon, kAlt),          // idx 1
	};

	mission_base->loadTestMission(items);

	int32_t prev = -1;
	EXPECT_FALSE(mission_base->findPreviousPositionIndexNoJump(1, prev));
}
