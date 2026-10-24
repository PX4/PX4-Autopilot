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
 * @file test_mission_base.cpp
 *
 * MissionBase position traversal tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "mission_base.h"
#include "navigator.h"
#include "support/geofence_test_helpers.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

#include <initializer_list>
#include <vector>

#include <cstring>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>

class MissionBaseTestPeer : public MissionBase
{
public:
	explicit MissionBaseTestPeer(Navigator *navigator = nullptr) : MissionBase(navigator, 8, 0) {}

	void setActiveMissionItems() override { ++mission_items_set_count; }
	bool setNextMissionItem() override { return false; }
	unsigned mission_items_set_count{0};

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

	// Reject injected failures before writing dataman, then mirror only successful writes.
	bool writeMissionItemToCache(int32_t index, mission_item_s &mission_item) override
	{
		if (!_mission_store.canWriteItem(index)
		    || !MissionBase::writeMissionItemToCache(index, mission_item)) {
			return false;
		}

		return _mission_store.writeItem(index, mission_item);
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
		_mission.current_seq = 0;
	}

	void loadTestMission(const std::vector<mission_item_s> &items, const mission_s &mission)
	{
		loadTestMission(items);
		_mission = mission;
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
		_mission.current_seq = 0;
	}

	void setLoadFailureIndices(std::initializer_list<int32_t> indices)
	{
		_mission_store.setLoadFailureIndices(indices);
	}

	void clearLoadFailures()
	{
		_mission_store.clearLoadFailures();
	}

	void setWriteFailureIndices(std::initializer_list<int32_t> indices)
	{
		_mission_store.setWriteFailureIndices(indices);
	}

	void setCurrentSequence(int32_t current_seq)
	{
		_mission.current_seq = current_seq;
	}

	int32_t currentSequence() const
	{
		return _mission.current_seq;
	}

	bool checkMission(bool forced = false)
	{
		check_mission_valid(forced);
		return missionChecked();
	}

	bool missionChecked() const { return _mission_check_state == MissionCheckState::Complete; }

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void setVehicleStatus(bool is_vtol, bool fixed_wing)
	{
		vehicle_status_s status{};
		status.is_vtol = is_vtol;
		status.vehicle_type = fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				      : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.timestamp = hrt_absolute_time();
		_vehicle_status_pub.publish(status);
		_vehicle_status_sub.update();

		if (_navigator) {
			*_navigator->get_vstatus() = status;
		}
	}

	bool vtolStatesForMission(std::vector<uint8_t> &states)
	{
		states.resize(_mission.count);
		return getVtolStatesForMission(states.data(), static_cast<int32_t>(states.size()));
	}

	uint8_t vtolStateAt(int32_t anchor_index)
	{
		return getVtolStateAtMissionIndex(anchor_index);
	}

	VtolTransitionAction transitionForTarget(int32_t target_index, bool direction_reversed)
	{
		return vtolTransitionActionForTarget(target_index, direction_reversed);
	}

	VtolTransitionAction transitionAfterReverseTarget(int32_t target_index)
	{
		return vtolTransitionActionAfterReachingReverseTarget(target_index);
	}

	void updateJumpAnchorForNextNominalAdvance(mission_route::ActiveJumpAnchor &anchor)
	{
		_active_jump_anchor = anchor;
		updateActiveJumpAnchorForNominalAdvance();
		anchor = _active_jump_anchor;
	}

	bool runJoinWorkItem()
	{
		if (vtolTransitionActive()) {
			if (!updateVtolTransition()) {
				return true;
			}

			resetVtolTransition();
		}

		loadCurrentMissionItem();
		return handleJoinRouteWorkItems();
	}

	void setJoinWaypointReached(bool reached)
	{
		_waypoint_position_reached = reached;
		_waypoint_yaw_reached = reached;
	}

	void processMissionSourceChange(bool items_changed = true)
	{
		onMissionUpdate(items_changed);
	}

	bool joinWorkItemActive() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE;
	}

	bool transitionAfterJoinActive() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN;
	}

	bool alignmentActive() const { return _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING; }

	bool alignmentAfterJoinActive() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING
		       && _vtol_transition.completion_work == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN;
	}

	const RouteJoinContext &joinContext() const
	{
		return _route_join_context;
	}

	const mission_item_s &currentMissionItem() const
	{
		return _mission_item;
	}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	using MissionBase::findNextPositionIndex;
	using MissionBase::findPreviousPositionIndex;
	using MissionBase::getNonJumpItem;
	using MissionBase::getNextPositionItems;
	using MissionBase::getPreviousPositionItems;
	using MissionBase::goToNextPositionItem;
	using MissionBase::goToPreviousPositionItem;
	using MissionBase::MissionTraversalType;
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	using MissionBase::VtolTransitionAction;
	using MissionBase::RouteJoinContext;
	using MissionBase::setupJoinRoute;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
	using MissionBase::resetMissionJumpCounter;

private:
	navigator_test::VectorMissionItemStore _mission_store{};
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	uORB::Publication<vehicle_status_s> _vehicle_status_pub {ORB_ID(vehicle_status)};
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
};

class IgnoreDoJumpMissionBaseTestPeer : public MissionBaseTestPeer
{
protected:
	MissionTraversalType traversalType() const override
	{
		return MissionTraversalType::IgnoreDoJump;
	}
};

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kAlt = 100.f;

class MissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
	MissionBaseTestPeer mission_base{};
};

class IgnoreDoJumpMissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
	IgnoreDoJumpMissionBaseTestPeer mission_base{};
};

class MissionBaseTestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

class MissionBaseFeasibilityTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_FENCE_POINTS_0));

		home_position_s *home = _navigator.get_home_position();
		home->timestamp = hrt_absolute_time();
		home->lat = kBaseLat;
		home->lon = kBaseLon;
		home->alt = 0.f;
		home->valid_hpos = true;
		home->valid_alt = true;
		_home_pub.publish(*home);
		_navigator.get_land_detected()->landed = true;
		vehicle_global_position_s *global_position = _navigator.get_global_position();
		global_position->timestamp = home->timestamp;
		global_position->lat = home->lat;
		global_position->lon = home->lon;
		global_position->alt = home->alt;

		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		*_navigator.get_vstatus() = status;
		_status_pub.publish(status);

		mission_fence_point_s circle{};
		circle.nav_cmd = NAV_CMD_FENCE_CIRCLE_INCLUSION;
		circle.frame = NAV_FRAME_GLOBAL;
		circle.lat = kBaseLat;
		circle.lon = kBaseLon;
		circle.circle_radius = 100.f;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_0, 0,
						      reinterpret_cast<uint8_t *>(&circle), sizeof(circle)));
		mission_stats_entry_s stats{};
		stats.dataman_id = DM_KEY_FENCE_POINTS_0;
		stats.opaque_id = 1;
		stats.num_items = 1;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
		_navigator.get_geofence().updateFence();
		ASSERT_TRUE(finishFenceUpdate());

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = 1;
		mission.geofence_id = stats.opaque_id;
		mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		_mission_base.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)}, mission);
		mission.count = 1;
		_mission_pub.publish(mission);
		ASSERT_TRUE(writeWaypoint(0.f));
	}

	void TearDown() override { param_reset_all(); }

	bool finishFenceUpdate()
	{
		Geofence &fence = _navigator.get_geofence();
		const hrt_abstime start = hrt_absolute_time();

		while (!fence.isReadyForPathChecks() && hrt_elapsed_time(&start) < 1_s) {
			fence.run();
			px4_usleep(1000);
		}

		return fence.isReadyForPathChecks();
	}

	bool writeWaypoint(float north_m)
	{
		mission_item_s item = makePositionItem(kBaseLat, kBaseLon, kAlt);
		add_vector_to_global_position(kBaseLat, kBaseLon, north_m, 0.f, &item.lat, &item.lon);
		return _dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
						 reinterpret_cast<uint8_t *>(&item), sizeof(item));
	}

	DatamanClient _dataman_client{};
	MissionBaseTestNavigator _navigator{};
	MissionBaseTestPeer _mission_base{&_navigator};
	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
};

TEST_F(MissionBaseFeasibilityTest, DeferredCheckPreservesVerdictUntilRetry)
{
	ASSERT_TRUE(_mission_base.checkMission(true));
	const mission_result_s checked_result = *_navigator.get_mission_result();
	ASSERT_TRUE(checked_result.valid);

	// Change storage without changing IDs to verify that the pending check really runs.
	ASSERT_TRUE(writeWaypoint(200.f));
	_navigator.get_geofence().updateFence();
	EXPECT_FALSE(_mission_base.checkMission(true));
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, checked_result.mission_id);
	EXPECT_EQ(_navigator.get_mission_result()->geofence_id, checked_result.geofence_id);
	EXPECT_EQ(_navigator.get_mission_result()->home_position_counter, checked_result.home_position_counter);

	ASSERT_TRUE(finishFenceUpdate());
	EXPECT_TRUE(_mission_base.checkMission());
	ASSERT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());

	// An unforced request can reuse a completed rejection when the inputs have not changed.
	ASSERT_TRUE(writeWaypoint(0.f));
	EXPECT_TRUE(_mission_base.checkMission());
	EXPECT_FALSE(_navigator.get_mission_result()->valid);

	_navigator.get_geofence().updateFence();
	EXPECT_FALSE(_mission_base.checkMission(true));
	EXPECT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());
	ASSERT_TRUE(finishFenceUpdate());
	EXPECT_TRUE(_mission_base.checkMission());
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
}

TEST_F(MissionBaseFeasibilityTest, SpentFenceRetriesLetTheCheckReject)
{
	_mission_base.on_inactive();
	ASSERT_TRUE(_navigator.get_mission_result()->valid);
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = _navigator.get_mission_result()->mission_id;
	mission.geofence_id = _navigator.get_mission_result()->geofence_id + 1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 1;
	_mission_pub.publish(mission);

	Geofence &fence = _navigator.get_geofence();
	fence.updateFence();

	for (unsigned attempt = 0; attempt <= GeofenceTestPeer::maxLoadRetries(); ++attempt) {
		SCOPED_TRACE(attempt);
		// The verdict is kept while a retry is still pending.
		_mission_base.on_inactive();
		EXPECT_TRUE(_navigator.get_mission_result()->valid);
		EXPECT_FALSE(_mission_base.missionChecked());
		GeofenceTestPeer::expireRetryDelay(fence);
		fence.run();
		fence.run();
		ASSERT_TRUE(GeofenceTestPeer::failPendingRead(fence));
		fence.run();
		fence.run();
	}

	// Once the fence gives up, the deferred check runs and rejects the mission.
	ASSERT_FALSE(fence.isFenceUpdatePending());
	// A fresh checker must also reject the unusable fence, even with matching cached IDs.
	MissionBaseTestPeer fresh_mission{&_navigator};
	mission_s unchanged = mission;
	unchanged.geofence_id = _navigator.get_mission_result()->geofence_id;
	fresh_mission.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)}, unchanged);
	EXPECT_TRUE(fresh_mission.checkMission());
	EXPECT_FALSE(_navigator.get_mission_result()->valid);

	_mission_base.on_inactive();
	EXPECT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
}

TEST_F(MissionBaseFeasibilityTest, SecondCheckerReusesVerdictAfterFenceReload)
{
	// Mission and an RTL mission handle are two checkers sharing one mission_result.
	MissionBaseTestPeer second{&_navigator};
	_mission_base.on_inactive();
	second.on_inactive();
	ASSERT_TRUE(_navigator.get_mission_result()->valid);

	mission_stats_entry_s stats{};
	stats.dataman_id = DM_KEY_FENCE_POINTS_0;
	stats.opaque_id = _navigator.get_mission_result()->geofence_id + 1;
	stats.num_items = 1;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
					      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = _navigator.get_mission_result()->mission_id;
	mission.geofence_id = stats.opaque_id;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 1;
	_mission_pub.publish(mission);
	_navigator.get_geofence().updateFence();
	_mission_base.on_inactive();
	second.on_inactive();
	ASSERT_TRUE(finishFenceUpdate());

	_mission_base.on_inactive();
	ASSERT_TRUE(_navigator.get_mission_result()->valid);

	// Change storage without changing IDs: only a redundant second check would see it.
	ASSERT_TRUE(writeWaypoint(200.f));
	second.on_inactive();
	EXPECT_TRUE(second.missionChecked());
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
}

TEST_F(MissionBaseFeasibilityTest, NewMissionCannotReuseVerdictWhileFenceLoads)
{
	_mission_base.on_inactive();
	ASSERT_TRUE(_navigator.get_mission_result()->valid);
	ASSERT_TRUE(_mission_base.missionChecked());

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = _navigator.get_mission_result()->mission_id + 1;
	mission.geofence_id = _navigator.get_mission_result()->geofence_id;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 1;
	_mission_pub.publish(mission);
	_navigator.get_geofence().updateFence();
	_mission_base.on_inactive();
	EXPECT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());

	ASSERT_TRUE(finishFenceUpdate());
	_mission_base.on_inactive();
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, mission.mission_id);
}

TEST_F(MissionBaseFeasibilityTest, NewMissionIsCheckedOnceHomeReturns)
{
	_mission_base.on_inactive();
	ASSERT_TRUE(_mission_base.missionChecked());

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = _navigator.get_mission_result()->mission_id + 1;
	mission.geofence_id = _navigator.get_mission_result()->geofence_id;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 1;
	_mission_pub.publish(mission);
	_navigator.get_home_position()->valid_hpos = false;
	_mission_base.on_inactive();
	EXPECT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());

	// The check request must survive until Home is back.
	_navigator.get_home_position()->valid_hpos = true;
	_mission_base.on_inactive();
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, mission.mission_id);
}

TEST_F(MissionBaseFeasibilityTest, DeferredActiveCheckKeepsCurrentMissionItems)
{
	_mission_base.on_inactive();
	ASSERT_TRUE(_navigator.get_mission_result()->valid);
	_mission_base.on_activation();
	ASSERT_EQ(_mission_base.mission_items_set_count, 1u);

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = _navigator.get_mission_result()->mission_id;
	mission.geofence_id = _navigator.get_mission_result()->geofence_id + 1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.count = 1;
	_mission_pub.publish(mission);
	_navigator.get_geofence().updateFence();

	_mission_base.on_active();
	_mission_base.on_active();
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());
	EXPECT_EQ(_mission_base.mission_items_set_count, 1u);
}

class MissionActivationTestPeer : public Mission
{
public:
	using Mission::Mission;
	bool missionChecked() const { return _mission_check_state == MissionCheckState::Complete; }
	void setActiveMissionItems() override { ++mission_items_set_count; }
	unsigned mission_items_set_count{0};
};

class MissionActivationFeasibilityTest : public MissionBaseFeasibilityTest,
	public ::testing::WithParamInterface<bool> {};

TEST_P(MissionActivationFeasibilityTest, ForcedActivationCheckRetriesAfterFenceLoad)
{
	MissionActivationTestPeer mission{&_navigator};
	mission.run(false);
	ASSERT_TRUE(mission.missionChecked());
	ASSERT_TRUE(_navigator.get_mission_result()->valid);

	if (GetParam()) {
		// Parameter changes need the forced check: mission, fence and Home IDs remain unchanged.
		const float maximum_altitude = kAlt / 2.f;
		ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum_altitude), PX4_OK);
		_navigator.updateParams();
	}

	_navigator.get_geofence().updateFence();
	mission.run(true); // Calls the real Mission::on_activation(), including its forced check.
	EXPECT_FALSE(mission.missionChecked());
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_EQ(mission.mission_items_set_count, 1u);

	// Process another fence publication while loading, then return to the original fence.
	mission_s updated{};
	updated.timestamp = hrt_absolute_time();
	updated.mission_id = _navigator.get_mission_result()->mission_id;
	updated.geofence_id = _navigator.get_mission_result()->geofence_id + 1;
	updated.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	updated.count = 1;
	_mission_pub.publish(updated);
	mission.run(true);
	EXPECT_FALSE(mission.missionChecked());
	EXPECT_EQ(mission.mission_items_set_count, 1u);

	ASSERT_TRUE(finishFenceUpdate());
	// The final publication is consumed after loading: matching cached IDs must not erase
	// the deferred forced check when there is no longer a pending load to defer it again.
	updated.timestamp = hrt_absolute_time();
	updated.geofence_id = _navigator.get_mission_result()->geofence_id;
	_mission_pub.publish(updated);
	mission.run(true);
	EXPECT_TRUE(mission.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->valid, !GetParam());
	EXPECT_EQ(_navigator.get_mission_result()->finished, GetParam());
	EXPECT_EQ(mission.mission_items_set_count, GetParam() ? 1u : 2u);
}

INSTANTIATE_TEST_SUITE_P(ForcedActivation, MissionActivationFeasibilityTest, ::testing::Bool(),
			 [](const ::testing::TestParamInfo<bool> &test_info)
{
	return test_info.param ? "RejectsChangedAltitudeLimit" : "AcceptsValidMission";
});

enum class ReplacementReadiness { Ready, FenceLoading, HomeUnavailable, PositionUnavailable, FenceStatusMismatch };

struct MissionReplacementCase {
	const char *name;
	float north;
	ReplacementReadiness readiness;
	bool accepted;
};

class MissionReplacementFeasibilityTest : public MissionBaseFeasibilityTest,
	public ::testing::WithParamInterface<MissionReplacementCase>
{
protected:
	void SetUp() override
	{
		MissionBaseFeasibilityTest::SetUp();
		// Already airborne at mission altitude, so no climb item obscures the commanded waypoint.
		_navigator.get_global_position()->alt = kAlt;
		_global_position_pub.publish(*_navigator.get_global_position());
		_navigator.get_land_detected()->landed = false;
		_land_detected_pub.publish(*_navigator.get_land_detected());
	}

	void TearDown() override
	{
		_navigator.get_land_detected()->landed = true;
		_land_detected_pub.publish(*_navigator.get_land_detected());
		MissionBaseFeasibilityTest::TearDown();
	}

	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
};

TEST_P(MissionReplacementFeasibilityTest, CommandsReplacementOnlyAfterValidation)
{
	const MissionReplacementCase &test = GetParam();
	Mission mission{&_navigator};
	mission.run(false);
	mission.run(true); // run() also marks the mode active, unlike calling on_activation() directly.
	ASSERT_TRUE(_navigator.get_mission_result()->valid);
	ASSERT_EQ(_navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);

	mission_item_s replacement = makePositionItem(kBaseLat, kBaseLon, kAlt);
	add_vector_to_global_position(kBaseLat, kBaseLon, test.north, 0.f, &replacement.lat, &replacement.lon);
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_1, 0,
					      reinterpret_cast<uint8_t *>(&replacement), sizeof(replacement)));
	mission_s uploaded{};
	uploaded.timestamp = hrt_absolute_time();
	uploaded.mission_id = _navigator.get_mission_result()->mission_id + 1;
	uploaded.geofence_id = _navigator.get_mission_result()->geofence_id;
	uploaded.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;
	uploaded.count = 1;

	switch (test.readiness) {
	case ReplacementReadiness::Ready:
		break;

	case ReplacementReadiness::FenceLoading:
		_navigator.get_geofence().updateFence();
		break;

	case ReplacementReadiness::HomeUnavailable:
		_navigator.get_home_position()->valid_hpos = false;
		break;

	case ReplacementReadiness::PositionUnavailable:
		_navigator.get_global_position()->timestamp = 0;
		break;

	case ReplacementReadiness::FenceStatusMismatch:
		++uploaded.geofence_id;
		break;
	}

	_mission_pub.publish(uploaded);

	for (unsigned cycle = 0; cycle < 2; ++cycle) {
		SCOPED_TRACE(cycle);
		mission.run(true);
		EXPECT_EQ(_navigator.get_mission_result()->valid, test.accepted);
		const position_setpoint_s &target = _navigator.get_position_setpoint_triplet()->current;
		ASSERT_TRUE(target.valid);
		EXPECT_EQ(target.type, test.accepted ? position_setpoint_s::SETPOINT_TYPE_POSITION : position_setpoint_s::SETPOINT_TYPE_LOITER);
		EXPECT_DOUBLE_EQ(target.lat, test.accepted ? replacement.lat : kBaseLat);
		EXPECT_DOUBLE_EQ(target.lon, test.accepted ? replacement.lon : kBaseLon);
	}
}

INSTANTIATE_TEST_SUITE_P(ReplacementSetpoints, MissionReplacementFeasibilityTest, ::testing::Values(
				 MissionReplacementCase{"ValidMission", 25.f, ReplacementReadiness::Ready, true},
				 MissionReplacementCase{"BreachingMission", 200.f, ReplacementReadiness::Ready, false},
				 MissionReplacementCase{"FenceLoading", 200.f, ReplacementReadiness::FenceLoading, false},
				 MissionReplacementCase{"HomeUnavailable", 200.f, ReplacementReadiness::HomeUnavailable, false},
				 MissionReplacementCase{"PositionUnavailable", 200.f, ReplacementReadiness::PositionUnavailable, false},
				 MissionReplacementCase{"FenceStatusMismatch", 200.f, ReplacementReadiness::FenceStatusMismatch, false}),
			 [](const ::testing::TestParamInfo<MissionReplacementCase> &test_info)
{
	return test_info.param.name;
});

enum class MissionFeasibilityInput { Mission, Geofence, Home };

struct ChangedMissionInput {
	const char *name;
	MissionFeasibilityInput input;
	bool keeps_verdict;
};

class MissionBaseChangedInputTest : public MissionBaseFeasibilityTest,
	public ::testing::WithParamInterface<ChangedMissionInput> {};

TEST_P(MissionBaseChangedInputTest, DeferredCheckInvalidatesOnlyAnUncheckedMission)
{
	_mission_base.checkMission(true);
	const mission_result_s checked_result = *_navigator.get_mission_result();
	ASSERT_TRUE(checked_result.valid);

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = checked_result.mission_id;
	mission.geofence_id = checked_result.geofence_id;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;

	switch (GetParam().input) {
	case MissionFeasibilityInput::Mission:
		++mission.mission_id;
		break;

	case MissionFeasibilityInput::Geofence:
		++mission.geofence_id;
		{
			mission_stats_entry_s stats{};
			stats.dataman_id = DM_KEY_FENCE_POINTS_0;
			stats.opaque_id = mission.geofence_id;
			stats.num_items = 1;
			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
							      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
		}
		break;

	case MissionFeasibilityInput::Home:
		++_navigator.get_home_position()->update_count;
		break;
	}

	_mission_base.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)}, mission);
	_navigator.get_geofence().updateFence();
	_mission_base.checkMission(true);
	// A fence or Home change keeps the checked mission flyable until the recheck.
	EXPECT_EQ(_navigator.get_mission_result()->valid, GetParam().keeps_verdict);
	EXPECT_FALSE(_mission_base.missionChecked());
	// No result has been computed for the changed inputs yet.
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, checked_result.mission_id);
	EXPECT_EQ(_navigator.get_mission_result()->geofence_id, checked_result.geofence_id);
	EXPECT_EQ(_navigator.get_mission_result()->home_position_counter, checked_result.home_position_counter);

	ASSERT_TRUE(finishFenceUpdate());
	_mission_base.checkMission();
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, mission.mission_id);
	EXPECT_EQ(_navigator.get_mission_result()->geofence_id, mission.geofence_id);
	EXPECT_EQ(_navigator.get_mission_result()->home_position_counter, _navigator.get_home_position()->update_count);
}

INSTANTIATE_TEST_SUITE_P(ChangedInputs, MissionBaseChangedInputTest, ::testing::Values(
				 ChangedMissionInput{"Mission", MissionFeasibilityInput::Mission, false},
				 ChangedMissionInput{"Geofence", MissionFeasibilityInput::Geofence, true},
				 ChangedMissionInput{"Home", MissionFeasibilityInput::Home, true}),
			 [](const ::testing::TestParamInfo<ChangedMissionInput> &test_info)
{
	return test_info.param.name;
});

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
class MissionBaseRouteCacheSyncTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		_navigator.get_mission_route_cache().invalidate();
	}

	void TearDown() override
	{
		_navigator.get_mission_route_cache().invalidate();
	}

	DatamanClient _dataman_client{};
	Navigator _navigator{};
	MissionBaseTestPeer _mission_base{&_navigator};
};

TEST_F(MissionBaseRouteCacheSyncTest, DoJumpWritesKeepRouteCacheCurrent)
{
	const std::vector<mission_item_s> items{
		makeDoJump(1, 2),
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = 1;
	mission.count = static_cast<uint16_t>(items.size());
	mission.current_seq = 0;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_mission_base.loadTestMission(items, mission);

	for (size_t i = 0; i < items.size(); ++i) {
		mission_item_s item = items[i];
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, static_cast<uint32_t>(i),
						      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
	}

	MissionRouteCache &route_cache = _navigator.get_mission_route_cache();
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(route_cache, mission,
			[&] { return route_cache.missionItemsReady(mission); }));

	int32_t mission_index = 0;
	mission_item_s mission_item{};
	ASSERT_EQ(_mission_base.getNonJumpItem(mission_index, mission_item,
					       MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow, true), PX4_OK);

	mission_item_s cached_item{};
	ASSERT_TRUE(route_cache.loadMissionItem(mission, 0, cached_item));
	EXPECT_EQ(cached_item.do_jump_current_count, 1);

	mission_item_s stored_item{};
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					     reinterpret_cast<uint8_t *>(&stored_item), sizeof(stored_item)));
	EXPECT_EQ(stored_item.do_jump_current_count, 1);

	_mission_base.resetMissionJumpCounter();

	ASSERT_TRUE(route_cache.loadMissionItem(mission, 0, cached_item));
	EXPECT_EQ(cached_item.do_jump_current_count, 0);
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					     reinterpret_cast<uint8_t *>(&stored_item), sizeof(stored_item)));
	EXPECT_EQ(stored_item.do_jump_current_count, 0);
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

// WHY: getNonJumpItem is used to find the next mission item.
// WHAT: A non-DO_JUMP item is returned unchanged.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemReturnsCurrentNonJumpItem)
{
	// GIVEN: A mission that starts with a normal position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 0), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: The helper loads the current item directly.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: It returns the same item and leaves the index unchanged.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat);
	EXPECT_DOUBLE_EQ(mission_item.lon, kBaseLon);
	EXPECT_FLOAT_EQ(mission_item.altitude, kAlt);
}

// WHY: getNonJumpItem() must follow active DO_JUMP targets.
// WHAT: [DO_JUMP->2, WP1, WP2] starting from idx 0 returns idx 2.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemFollowsActiveForwardDoJump)
{
	// GIVEN: A forward DO_JUMP that points to a later position item.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: Mission-control traversal resolves the jump without writing counters.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The helper follows the jump and returns the target item.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 2);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat + 0.001);
	EXPECT_DOUBLE_EQ(mission_item.lon, kBaseLon);
	EXPECT_FLOAT_EQ(mission_item.altitude, kAlt);
}

// WHY: Once a DO_JUMP has already used all repeats, callers should move on to the next item.
// WHAT: [WP0, DO_JUMP->0 done, WP2] starting from idx 1 returns idx 2.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemSkipsDoJumpAfterLastRepeat)
{
	// GIVEN: A DO_JUMP whose repeat count is already reached.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 1), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};

	// WHEN: The helper resolves the jump while traversing forward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The jump is skipped and the next non-jump item is returned.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 2);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat + 0.001);
}

// WHY: Reverse traversal that ignores DO_JUMP must step backward instead of following control flow.
// WHAT: [WP0, DO_JUMP->2, WP2] starting from idx 1 returns idx 0.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemSkipsDoJumpBackwardWhenIgnoringJumps)
{
	// GIVEN: An active forward DO_JUMP with a valid non-jump item before it.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(2, 1, 0), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};

	// WHEN: The helper resolves the jump while moving backward in geometry-only mode.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump,
			false, true);

	// THEN: The DO_JUMP is skipped and the previous non-jump item is returned.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat);
}

// WHY: Bad jump targets must return an error.
// WHAT: A DO_JUMP that points beyond the mission bounds returns PX4_ERROR.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemReturnsErrorForOutOfBoundsDoJumpTarget)
{
	// GIVEN: A mission with a DO_JUMP that points outside the mission.
	mission_base.loadTestMission({
		makeDoJump(3, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: The helper tries to resolve that jump.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The helper returns an error.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 0);
}

// Fixture with a real Navigator so the storage failure path is observable, it
// publishes to mavlink_log through the navigator instead of a null pointer.
class MissionBasePastBoundsTraversalTest : public NavigatorDatamanTestBase
{
protected:
	bool storageErrorPublished()
	{
		mavlink_log_s report;

		while (_mavlink_log_sub.update(&report)) {
			if (strstr(reinterpret_cast<const char *>(report.text), "could not be read") != nullptr) {
				return true;
			}
		}

		return false;
	}

	Navigator _navigator{};
	MissionBaseTestPeer mission_base{&_navigator};
	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
};

// Fixture with a real Navigator and a mission bound to a dataman area, so DO_JUMP counters are
// written for real and a failure injected in the store is the only thing that stops them.
class MissionBaseJumpCounterWriteTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		_navigator.get_mission_result()->item_do_jump_changed = false;

		// only messages published by this test count
		mavlink_log_s report;

		while (_mavlink_log_sub.update(&report)) {}
	}

	// [WP0, WP1, DO_JUMP->0 with two repeats left, WP3]
	void loadJumpMission()
	{
		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = 1;
		mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		mission.land_start_index = -1;
		mission.land_index = -1;
		mission_base.loadTestMission({
			makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
			makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
			makeDoJump(0, 2, 0), // idx 2
			makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
		}, mission);
	}

	uint16_t storedJumpCount()
	{
		mission_item_s item{};
		EXPECT_TRUE(mission_base.loadMissionItemFromCache(2, item));
		return item.do_jump_current_count;
	}

	bool jumpMessagePublished()
	{
		mavlink_log_s report;

		while (_mavlink_log_sub.update(&report)) {
			if (strstr(reinterpret_cast<const char *>(report.text), "DO JUMP could not be saved") != nullptr) {
				return true;
			}
		}

		return false;
	}

	DatamanClient _dataman_client{};
	Navigator _navigator{};
	MissionBaseTestPeer mission_base{&_navigator};
	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
};

// WHY: A jump that is counted is the normal case and must keep working as before.
// WHAT: [WP0, WP1, DO_JUMP->0 (2 left), WP3] entered at the DO_JUMP returns idx 0 and stores count 1.
TEST_F(MissionBaseJumpCounterWriteTest, GetNonJumpItemFollowsDoJumpWhenCounterIsStored)
{
	loadJumpMission();

	int32_t mission_index = 2;
	mission_item_s mission_item{};

	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			true, false);

	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(storedJumpCount(), 1);
	EXPECT_TRUE(_navigator.get_mission_result()->item_do_jump_changed);
	EXPECT_EQ(_navigator.get_mission_result()->item_do_jump_remaining, 1);
	EXPECT_FALSE(jumpMessagePublished());
}

// WHY: If the counter cannot be stored, following the jump would repeat it on every pass,
// since the stored count never advances. The mission has to continue past the jump.
// WHAT: With the write of idx 2 failing, entering at the DO_JUMP returns idx 3, the stored
// count stays 0, nothing is reported as a jump, and the operator gets the message.
TEST_F(MissionBaseJumpCounterWriteTest, GetNonJumpItemSkipsDoJumpWhenCounterCannotBeStored)
{
	loadJumpMission();
	mission_base.setWriteFailureIndices({2});

	int32_t mission_index = 2;
	mission_item_s mission_item{};

	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			true, false);

	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 3);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat + 0.002);
	EXPECT_EQ(storedJumpCount(), 0);
	EXPECT_FALSE(_navigator.get_mission_result()->item_do_jump_changed);
	EXPECT_TRUE(jumpMessagePublished());
}

// WHY: Backward traversal has to step the same way past a jump it cannot count.
// WHAT: With the write of idx 2 failing, entering at the DO_JUMP backward returns idx 1.
TEST_F(MissionBaseJumpCounterWriteTest, GetNonJumpItemStepsBackPastDoJumpWhenCounterCannotBeStored)
{
	loadJumpMission();
	mission_base.setWriteFailureIndices({2});

	int32_t mission_index = 2;
	mission_item_s mission_item{};

	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			true, true);

	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 1);
	EXPECT_EQ(storedJumpCount(), 0);
}

// WHY: Resolving a set-current index follows a jump without consuming a repetition, so a
// storage failure does not apply to it and it keeps following the jump.
// WHAT: With the write of idx 2 failing and write_jumps false, entering at the DO_JUMP returns idx 0.
TEST_F(MissionBaseJumpCounterWriteTest, GetNonJumpItemStillFollowsDoJumpWithoutWritingCounters)
{
	loadJumpMission();
	mission_base.setWriteFailureIndices({2});

	int32_t mission_index = 2;
	mission_item_s mission_item{};

	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(storedJumpCount(), 0);
	EXPECT_FALSE(jumpMessagePublished());
}

// WHY: Walking off the end of the mission while skipping an exhausted DO_JUMP is the
// normal end of a mission, not a storage failure.
// WHAT: [WP0, DO_JUMP->0 done] entered at the DO_JUMP returns PX4_ERROR without
// publishing a storage error.
TEST_F(MissionBasePastBoundsTraversalTest, GetNonJumpItemReturnsErrorPastMissionEnd)
{
	// GIVEN: A mission whose last item is a DO_JUMP with no repeats left.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 1), // idx 1
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};
	(void)storageErrorPublished(); // drain anything already queued

	// WHEN: The helper skips the exhausted jump while traversing forward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: It reports no further item, exactly like an out of range entry index,
	// and no "could not be read" error is published.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 1);
	EXPECT_FALSE(storageErrorPublished());
}

// WHY: The same walk moving backward can step in front of the first item.
// WHAT: [DO_JUMP->1 done, WP1] entered at the DO_JUMP backward returns PX4_ERROR
// without publishing a storage error.
TEST_F(MissionBasePastBoundsTraversalTest, GetNonJumpItemReturnsErrorPastMissionStart)
{
	// GIVEN: A mission that starts with a DO_JUMP with no repeats left.
	mission_base.loadTestMission({
		makeDoJump(1, 1, 1), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};
	(void)storageErrorPublished(); // drain anything already queued

	// WHEN: The helper skips the exhausted jump while traversing backward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, true);

	// THEN: It reports no further item, exactly like an out of range entry index,
	// and no "could not be read" error is published.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 0);
	EXPECT_FALSE(storageErrorPublished());
}

// WHY: Geometry-only position traversal must skip non-position mission items.
// WHAT: Starting from a VTOL transition item, the helper skips it and returns the next position item.
TEST_F(MissionBaseTraversalTest, FindNextSkipsNonPositionItems)
{
	// GIVEN: A position item, a non-position VTOL transition, and then another position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal searches forward from the non-position item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The next position item is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Geometry-only position traversal must skip non-position mission items.
// WHAT: Starting from a position item after a VTOL transition, the helper skips it and returns the previous position item.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsNonPositionItems)
{
	// GIVEN: A position item, a non-position VTOL transition, and then another position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches backward from the non-position item.
	const bool found = mission_base.findPreviousPositionIndex(2, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The previous position item is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 0);
}

// WHY: Geometry-only traversal must skip DO_JUMP items.
// WHAT: [WP, DO_JUMP, WP, WP] starting from idx 1 returns idx 2.
TEST_F(MissionBaseTraversalTest, FindNextSkipsDoJumpItems)
{
	// GIVEN: A mission where a DO_JUMP sits between two position items.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal starts at the DO_JUMP item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The first position item after the jump is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Geometry-only traversal must skip DO_JUMP items.
// WHAT: [WP, WP, DO_JUMP, WP] starting from idx 3 returns idx 1.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsDoJumpItems)
{
	// GIVEN: A mission where a DO_JUMP sits between two position items.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 3), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal starts at the DO_JUMP item.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The first position item before the jump is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 1);
}

// WHY: Consecutive non-position items must be skipped.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] starting from idx 1 returns idx 3.
TEST_F(MissionBaseTraversalTest, FindNextSkipsConsecutiveNonPositionItems)
{
	// GIVEN: Consecutive non-position items before the next position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 5), // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal walks forward through the control items.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: It returns the first following position item.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 3);
}

// WHY: Consecutive non-position items must be skipped in reverse.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsConsecutiveNonPositionItems)
{
	// GIVEN: Consecutive non-position items before the previous position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal walks backward through the control items.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: It returns the first previous position item.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 0);
}

// WHY: Callers need a failure when no later position item exists.
// WHAT: [WP, DO_JUMP] starting from idx 1 returns false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseAtEnd)
{
	// GIVEN: A mission with no position item after the starting index.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal searches past the last item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no next position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Callers need a failure when no earlier position item exists.
// WHAT: [DO_JUMP, WP] starting from idx 1 returns false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseAtStart)
{
	// GIVEN: A mission with no position item before the starting index.
	mission_base.loadTestMission({
		makeDoJump(0, 3), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches before the first position item.
	const bool found = mission_base.findPreviousPositionIndex(1, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no previous position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(previous_index, -1);
}

// WHY: Traversal should fail cleanly when a mission item cannot be loaded.
// WHAT: A cache failure on the next position item makes findNextPositionIndex() return false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseOnCacheReadFailure)
{
	// GIVEN: A mission where the next position item cannot be loaded.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setLoadFailureIndices({2});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal advances past the DO_JUMP item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The unreadable position item produces a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Traversal should fail cleanly when a mission item cannot be loaded.
// WHAT: A cache failure on the previous position item makes findPreviousPositionIndex() return false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseOnCacheReadFailure)
{
	// GIVEN: A mission where the previous position item cannot be loaded.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setLoadFailureIndices({0});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal moves backward past the DO_JUMP item.
	const bool found = mission_base.findPreviousPositionIndex(2, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The unreadable position item produces a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(previous_index, -1);
}

// WHY: findNextPositionIndex must use MissionTraversalType
// WHAT: [DO_JUMP->2, WP1, WP2] starting from idx 0 resolves to idx 2 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindNextSupportsBothTraversalSemantics)
{
	// GIVEN: A jump whose target is a later position item.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t next_follow{-1};
	int32_t next_geometry{-1};

	// WHEN: The same lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findNextPositionIndex(0, next_follow,
				  MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findNextPositionIndex(0, next_geometry,
				    MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(next_follow, 2);
	EXPECT_EQ(next_geometry, 1);
}

// WHY: findPreviousPositionIndex must use MissionTraversalType
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 resolves to idx 0 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindPreviousSupportsBothTraversalSemantics)
{
	// GIVEN: A jump whose target is an earlier position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_follow{-1};
	int32_t previous_geometry{-1};

	// WHEN: The same lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findPreviousPositionIndex(3, previous_follow,
				  MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findPreviousPositionIndex(3, previous_geometry,
				    MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(previous_follow, 0);
	EXPECT_EQ(previous_geometry, 1);
}

// WHY: The refactor must not change the legacy mission-control behavior.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 should still land on idx 2.
TEST_F(MissionBaseTraversalTest, GoToNextPositionItemFollowsMissionControlFlow)
{
	// GIVEN: A mission whose first item is an active DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToNextPositionItem(MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);

	// THEN: Traversal follows the jump target exactly as before.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 2);
}

// WHY: The backward wrapper must also preserve the legacy mission-control behavior.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 should still land on idx 0.
TEST_F(MissionBaseTraversalTest, GoToPreviousPositionItemFollowsMissionControlFlow)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToPreviousPositionItem(MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);

	// THEN: Traversal follows the active jump exactly as before.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 0);
}

// WHY: Existing mission execution relies on getNextPositionItems() following active DO_JUMP
//      control flow by default.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 2 returns idx 0 then idx 1.
TEST_F(MissionBaseTraversalTest, GetNextPositionItemsFollowsActiveDoJump)
{
	// GIVEN: A mission with an active jump loop.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t next_items[2] = {-1, -1};
	size_t num_found_items = 0;

	// WHEN: The multi-item helper walks forward with default traversal semantics.
	mission_base.getNextPositionItems(2, next_items, num_found_items, 2u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 2u);
	EXPECT_EQ(next_items[0], 0);
	EXPECT_EQ(next_items[1], 1);
}

// WHY: Reverse mission flows rely on getPreviousPositionItems() following active DO_JUMP.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, GetPreviousPositionItemsFollowsActiveDoJump)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_items[1] = {-1};
	size_t num_found_items = 0;

	// WHEN: The multi-item helper walks backward with default traversal semantics.
	mission_base.getPreviousPositionItems(3, previous_items, num_found_items, 1u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 1u);
	EXPECT_EQ(previous_items[0], 0);
}

// WHY: Mission-based Return configures position traversal to skip DO_JUMP loops consistently.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 lands on idx 1 with the configured traversal.
TEST_F(IgnoreDoJumpMissionBaseTraversalTest, ConfiguredTraversalSkipsDoJumpForGoToNextPositionItem)
{
	// GIVEN: A mission whose first item is an active DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The mode advances using its configured traversal policy.
	const int ret = mission_base.goToNextPositionItem();

	// THEN: The DO_JUMP loop is skipped and the geometric next waypoint is selected.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

// WHY: Reverse mission-path Return must skip DO_JUMP loops for backward progression too.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 lands on idx 1 with the configured traversal.
TEST_F(IgnoreDoJumpMissionBaseTraversalTest, ConfiguredTraversalSkipsDoJumpForGoToPreviousPositionItem)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The mode advances backward using its configured traversal policy.
	const int ret = mission_base.goToPreviousPositionItem();

	// THEN: The DO_JUMP loop is skipped and the geometric previous waypoint is selected.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
// MissionBase logic shared by the route join and the route RTL.
class MissionBaseRouteTest : public NavigatorDatamanTestBase
{
protected:
	// Reset the vehicle_status left by a previous test.
	void SetUp() override
	{
		mission_base.setVehicleStatus(false, false);
	}

	MissionBaseTestPeer mission_base{};
};

TEST_F(MissionBaseRouteTest, PlainWaypointMissionDoesNotInventATransition)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	EXPECT_EQ(mission_base.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);

	for (const bool fixed_wing : {false, true}) {
		mission_base.setVehicleStatus(true, fixed_wing);
		EXPECT_EQ(mission_base.transitionForTarget(1, false), MissionBaseTestPeer::VtolTransitionAction::kNone);
		EXPECT_EQ(mission_base.transitionForTarget(0, true), MissionBaseTestPeer::VtolTransitionAction::kNone);
	}
}

TEST_F(MissionBaseRouteTest, VtolLookaheadResolvesPrefixAndPreservesPrecedingMode)
{
	for (const uint8_t target : {vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC}) {
		const bool next_is_front_transition = target == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
		const uint8_t prefix_mode = next_is_front_transition ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
					    : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
		mission_base.loadTestMission({
			makePositionItem(kBaseLat, kBaseLon, kAlt),
			makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
			makeVtolTransitionItem(target),
			makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
			makeVtolTransitionItem(target),
			makePositionItem(kBaseLat + 0.003, kBaseLon, kAlt),
		});
		EXPECT_EQ(mission_base.vtolStateAt(1), prefix_mode);
		// Between repeated transitions, the previous command wins over lookahead.
		EXPECT_EQ(mission_base.vtolStateAt(3), target);
		mission_base.setVehicleStatus(true, next_is_front_transition);
		const auto action = next_is_front_transition ? MissionBaseTestPeer::VtolTransitionAction::kBackTransition
				    : MissionBaseTestPeer::VtolTransitionAction::kFrontTransition;
		EXPECT_EQ(mission_base.transitionForTarget(1, false), action);
		EXPECT_EQ(mission_base.transitionForTarget(0, true), action);
		EXPECT_EQ(mission_base.transitionForTarget(3, false), MissionBaseTestPeer::VtolTransitionAction::kNone);
	}
}

// The two-pass table used by the route time estimate gives the per-index scan result everywhere.
TEST_F(MissionBaseRouteTest, VtolStateTableMatchesPerIndexScan)
{
	const auto with_command = [](mission_item_s item, uint16_t nav_cmd) {
		item.nav_cmd = nav_cmd;
		return item;
	};
	const mission_item_s waypoint = makePositionItem(kBaseLat, kBaseLon, kAlt);
	const mission_item_s land = with_command(waypoint, NAV_CMD_LAND);
	const mission_item_s takeoff = with_command(waypoint, NAV_CMD_TAKEOFF);
	const mission_item_s vtol_takeoff = with_command(waypoint, NAV_CMD_VTOL_TAKEOFF);
	const mission_item_s front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const mission_item_s back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	mission_item_s invalid_transition = front_transition;
	invalid_transition.params[0] = 7.f;

	const std::vector<std::vector<mission_item_s>> missions{
		{waypoint, waypoint, front_transition, waypoint},
		{waypoint, land, front_transition, waypoint},
		{waypoint, invalid_transition, waypoint, back_transition, waypoint},
		{waypoint, takeoff, front_transition, waypoint},
		{vtol_takeoff, waypoint, back_transition, waypoint, land},
		{waypoint, waypoint, makeDoJump(0, 2), front_transition, waypoint},
	};

	for (size_t m = 0; m < missions.size(); ++m) {
		mission_base.loadTestMission(missions[m]);
		std::vector<uint8_t> states;
		ASSERT_TRUE(mission_base.vtolStatesForMission(states)) << "mission " << m;

		for (int32_t index = 0; index < static_cast<int32_t>(states.size()); ++index) {
			EXPECT_EQ(states[index], mission_base.vtolStateAt(index)) << "mission " << m << " index " << index;
		}
	}

	// A load failure leaves the per-target scan in charge.
	mission_base.setLoadFailureIndices({1});
	std::vector<uint8_t> states;
	EXPECT_FALSE(mission_base.vtolStatesForMission(states));
	mission_base.clearLoadFailures();
}

TEST_F(MissionBaseRouteTest, VtolLookaheadIgnoresJumpRepeats)
{
	for (const uint8_t target : {vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC}) {
		for (const uint16_t repeats_used : {0, 2}) {
			const auto jump = makeDoJump(0, 2, repeats_used);
			mission_base.loadTestMission({
				makePositionItem(kBaseLat, kBaseLon, kAlt),
				makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
				jump,
				makeVtolTransitionItem(target),
				makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
			});
			const bool next_is_front_transition = target == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
			mission_base.setVehicleStatus(true, next_is_front_transition);
			const auto action = next_is_front_transition ? MissionBaseTestPeer::VtolTransitionAction::kBackTransition
					    : MissionBaseTestPeer::VtolTransitionAction::kFrontTransition;
			EXPECT_EQ(mission_base.transitionForTarget(1, false), action);
			EXPECT_EQ(mission_base.transitionForTarget(0, true), action);
			mission_item_s after{};
			ASSERT_TRUE(mission_base.loadMissionItemFromCache(2, after));
			EXPECT_EQ(after.do_jump_current_count, repeats_used);
			EXPECT_EQ(after.do_jump_repeat_count, jump.do_jump_repeat_count);
			EXPECT_EQ(after.do_jump_mission_index, jump.do_jump_mission_index);
		}
	}
}

TEST_F(MissionBaseRouteTest, VtolLookaheadRequiresReadableMissionSection)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
	});
	mission_base.setLoadFailureIndices({2});
	EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);
	mission_base.setLoadFailureIndices({0});
	EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);
	mission_base.clearLoadFailures();
	EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

TEST_F(MissionBaseRouteTest, VtolLookaheadDoesNotCrossLandingAtTarget)
{
	for (const uint16_t command : {NAV_CMD_LAND, NAV_CMD_VTOL_LAND}) {
		auto landing = makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt);
		landing.nav_cmd = command;
		mission_base.loadTestMission({
			makePositionItem(kBaseLat, kBaseLon, kAlt),
			landing,
			makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		});
		EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);
		mission_base.setVehicleStatus(true, true);
		EXPECT_EQ(mission_base.transitionForTarget(1, false), MissionBaseTestPeer::VtolTransitionAction::kNone);
	}
}

TEST_F(MissionBaseRouteTest, VtolTakeoffDefinesFollowingSegmentsAsFixedWing)
{
	mission_item_s vtol_takeoff = makePositionItem(kBaseLat, kBaseLon, kAlt);
	vtol_takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	mission_base.loadTestMission({
		vtol_takeoff,
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});

	EXPECT_EQ(mission_base.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(3), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionForTarget(1, false),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
	EXPECT_EQ(mission_base.transitionForTarget(0, true),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
}

TEST_F(MissionBaseRouteTest, VtolStateAndActionsFollowMissionSegments)
{
	// Reverse travel uses the mode of the segment being entered.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});

	// Before the first front transition, lookahead infers MC.
	EXPECT_EQ(mission_base.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base.vtolStateAt(2), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(4), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionForTarget(2, false),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	EXPECT_EQ(mission_base.transitionForTarget(0, true),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	mission_base.setVehicleStatus(true, true);
	EXPECT_EQ(mission_base.transitionForTarget(4, false),
		  MissionBaseTestPeer::VtolTransitionAction::kBackTransition);
	EXPECT_EQ(mission_base.transitionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::kBackTransition);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionAfterReverseTarget(2),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	mission_base.setVehicleStatus(false, false);
	EXPECT_EQ(mission_base.transitionForTarget(2, false),
		  MissionBaseTestPeer::VtolTransitionAction::kNone);
}

TEST_F(MissionBaseRouteTest, ReverseTransitionRestoresNextLegWithoutAnAttachedCommand)
{
	// A jump can arrive in a different mode even when this waypoint has no transition attached.
	for (bool fixed_wing_leg : {false, true}) {
		mission_base.loadTestMission({
			makePositionItem(kBaseLat, kBaseLon, kAlt),
			makeVtolTransitionItem(fixed_wing_leg ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
					       : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
			makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
			makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
		});
		mission_base.setVehicleStatus(true, !fixed_wing_leg);
		EXPECT_EQ(mission_base.transitionAfterReverseTarget(2), fixed_wing_leg
			  ? MissionBaseTestPeer::VtolTransitionAction::kFrontTransition
			  : MissionBaseTestPeer::VtolTransitionAction::kBackTransition);
		EXPECT_EQ(mission_base.transitionAfterReverseTarget(0), MissionBaseTestPeer::VtolTransitionAction::kNone);
		EXPECT_EQ(mission_base.transitionAfterReverseTarget(4), MissionBaseTestPeer::VtolTransitionAction::kNone);
	}
}

TEST_F(MissionBaseRouteTest, ReverseTransitionUsesLastModeBeforeReachedWaypoint)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionAfterReverseTarget(4), MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
	mission_base.setVehicleStatus(true, true);
	EXPECT_EQ(mission_base.transitionAfterReverseTarget(4), MissionBaseTestPeer::VtolTransitionAction::kNone);
}

TEST_F(MissionBaseRouteTest, ReverseFirstPositionHasNoFurtherLegAfterLeadingActions)
{
	mission_base.loadTestMission({
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	mission_base.setVehicleStatus(true, false);
	// Index 1 is the first position: arrival owns the next flight-mode decision.
	EXPECT_EQ(mission_base.transitionAfterReverseTarget(1), MissionBaseTestPeer::VtolTransitionAction::kNone);
}

TEST_F(MissionBaseRouteTest, JoinRouteRunsWaypointTransitionAndResumeFlow)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	const std::vector<mission_item_s> items{
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
	};
	mission_base_with_nav.loadTestMission(items);
	mission_base_with_nav.setCurrentSequence(2);
	mission_base_with_nav.setVehicleStatus(true, false);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0004;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt + 5.f;
	*navigator.get_global_position() = global_position;

	const mission_route::Position join{kBaseLat + 0.0005, kBaseLon, kAlt + 10.f};
	mission_base_with_nav.setupJoinRoute(join, true, MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
	ASSERT_TRUE(mission_base_with_nav.joinWorkItemActive());

	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	const position_setpoint_s &join_setpoint = navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(join_setpoint.valid);
	EXPECT_DOUBLE_EQ(join_setpoint.lat, join.lat);
	EXPECT_DOUBLE_EQ(join_setpoint.lon, join.lon);
	EXPECT_FLOAT_EQ(join_setpoint.alt, global_position.alt);

	mission_base_with_nav.setJoinWaypointReached(true);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	ASSERT_TRUE(mission_base_with_nav.alignmentAfterJoinActive());
	EXPECT_EQ(mission_base_with_nav.currentMissionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(mission_base_with_nav.currentMissionItem().force_heading);
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[2].lat, items[2].lon);
	EXPECT_NEAR(join_setpoint.yaw, expected_yaw, 1e-4f);

	mission_base_with_nav.setJoinWaypointReached(true);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	ASSERT_TRUE(mission_base_with_nav.transitionAfterJoinActive());
	EXPECT_EQ(mission_base_with_nav.currentMissionItem().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(mission_base_with_nav.currentMissionItem().params[0],
			static_cast<float>(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW));
	EXPECT_NEAR(join_setpoint.yaw, expected_yaw, 1e-4f);

	mission_base_with_nav.setVehicleStatus(true, true);
	EXPECT_FALSE(mission_base_with_nav.runJoinWorkItem());
	EXPECT_FALSE(mission_base_with_nav.joinWorkItemActive());
	EXPECT_FALSE(mission_base_with_nav.alignmentAfterJoinActive());
	EXPECT_FALSE(mission_base_with_nav.transitionAfterJoinActive());
	EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
}

TEST_F(MissionBaseRouteTest, JoinRouteBackTransitionDoesNotWaitForHeading)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	mission_base_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	mission_base_with_nav.setVehicleStatus(true, true);

	mission_base_with_nav.setupJoinRoute({kBaseLat, kBaseLon, kAlt}, false,
					     MissionBaseTestPeer::VtolTransitionAction::kBackTransition);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	EXPECT_TRUE(mission_base_with_nav.currentMissionItem().vtol_back_transition);

	mission_base_with_nav.setJoinWaypointReached(true);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	EXPECT_TRUE(mission_base_with_nav.transitionAfterJoinActive());
	EXPECT_FALSE(mission_base_with_nav.alignmentAfterJoinActive());
	EXPECT_EQ(mission_base_with_nav.currentMissionItem().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(mission_base_with_nav.currentMissionItem().params[0],
			static_cast<float>(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC));
}

TEST_F(MissionBaseRouteTest, MissionSourceChangeClearsJoinHeadingAlignment)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	mission_base_with_nav.loadTestMission({makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt)});
	mission_base_with_nav.setVehicleStatus(true, false);
	navigator.get_global_position()->lat = kBaseLat;
	navigator.get_global_position()->lon = kBaseLon;
	navigator.get_global_position()->alt = kAlt;

	mission_base_with_nav.setupJoinRoute({kBaseLat, kBaseLon, kAlt}, false,
					     MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	mission_base_with_nav.setJoinWaypointReached(true);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	ASSERT_TRUE(mission_base_with_nav.alignmentAfterJoinActive());

	mission_base_with_nav.processMissionSourceChange();
	EXPECT_FALSE(mission_base_with_nav.alignmentAfterJoinActive());
	EXPECT_FALSE(mission_base_with_nav.alignmentActive());
	EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
}

// A replacement mission and an external sequence change both drop the join.
TEST_F(MissionBaseRouteTest, MissionUpdateClearsJoin)
{
	for (const bool items_changed : {true, false}) {
		SCOPED_TRACE(items_changed);
		Navigator navigator{};
		MissionBaseTestPeer mission_base_with_nav{&navigator};
		mission_base_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
		mission_base_with_nav.setupJoinRoute({kBaseLat, kBaseLon, kAlt}, false,
						     MissionBaseTestPeer::VtolTransitionAction::kNone);
		ASSERT_TRUE(mission_base_with_nav.joinWorkItemActive());

		mission_base_with_nav.processMissionSourceChange(items_changed);

		EXPECT_FALSE(mission_base_with_nav.joinWorkItemActive());
		EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
	}
}

TEST_F(MissionBaseRouteTest, NextNominalAdvanceCapturesActiveLoopEdge)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(0, 3, 1),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base.setCurrentSequence(1);

	mission_route::ActiveJumpAnchor anchor{};
	mission_base.updateJumpAnchorForNextNominalAdvance(anchor);
	ASSERT_TRUE(anchor.valid());
	EXPECT_EQ(anchor.jump_item_index, 2);
}

TEST_F(MissionBaseRouteTest, InvalidLoopTargetClearsPreviousLoopSegment)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(10, 3, 1),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base.setCurrentSequence(1);

	mission_route::ActiveJumpAnchor anchor{2};
	ASSERT_TRUE(anchor.valid());

	mission_base.updateJumpAnchorForNextNominalAdvance(anchor);

	EXPECT_TRUE(anchor.empty());
}

TEST_F(MissionBaseRouteTest, JumpAnchorSurvivesCommandsBeforeTheJumpTargetPosition)
{
	mission_base.loadTestMission({
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(0, 3, 1),
	});
	mission_base.setCurrentSequence(0);
	mission_route::ActiveJumpAnchor anchor{3};

	mission_base.updateJumpAnchorForNextNominalAdvance(anchor);

	EXPECT_EQ(anchor.jump_item_index, 3);

	mission_base.setCurrentSequence(1);
	mission_base.updateJumpAnchorForNextNominalAdvance(anchor);

	EXPECT_TRUE(anchor.empty());
}

TEST_F(MissionBaseRouteTest, ExhaustedJumpDoesNotBecomeAnActiveAnchor)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeDoJump(0, 1, 1),
	});
	mission_route::ActiveJumpAnchor anchor{1};

	mission_base.updateJumpAnchorForNextNominalAdvance(anchor);

	EXPECT_TRUE(anchor.empty());
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
