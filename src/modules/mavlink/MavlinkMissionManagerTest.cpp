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
 * Tests how MavlinkMissionManager instances keep the shared active-mission
 * state in sync through the `mission` topic.
 *
 * All Mavlink instances share one static copy of the active mission, and each
 * instance adopts `mission` publications that differ from it in
 * check_active_mission(). If an instance can copy a publication that is older
 * than the shared state (e.g. while another instance has stored a new upload
 * but not yet published it), it rolls the new mission back to the previous one.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/mission.h>

#include "mavlink_main.h"
#include "mavlink_mission.h"

static constexpr uint32_t OLD_MISSION_ID = 0;
static constexpr uint32_t NEW_MISSION_ID = 3846942581;

class MavlinkMissionManagerTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		// Skip the one-time dataman load in the constructor: no dataman runs here.
		MavlinkMissionManager::_dataman_init = true;

		// Normally created by Mavlink::start(); the Mavlink constructor needs it.
		if (!Mavlink::_event_buffer) {
			Mavlink::_event_buffer = new events::EventBuffer();
			ASSERT_EQ(Mavlink::_event_buffer->init(), 0);
		}

		_mavlink = new Mavlink();
	}

	static void TearDownTestSuite()
	{
		delete _mavlink;
		_mavlink = nullptr;
		delete Mavlink::_event_buffer;
		Mavlink::_event_buffer = nullptr;
	}

	void SetUp() override
	{
		_instance_a = new MavlinkMissionManager(*_mavlink);
		_instance_b = new MavlinkMissionManager(*_mavlink);

		// Start both instances from an agreed state: the old (empty) mission.
		storeMission(*_instance_a, OLD_MISSION_ID, 0);
		_instance_a->check_active_mission();
		_instance_b->check_active_mission();
	}

	void TearDown() override
	{
		delete _instance_a;
		delete _instance_b;
	}

	// Store a mission upload the way handle_mission_item() does once all items arrived.
	static void storeMission(MavlinkMissionManager &instance, uint32_t mission_id, uint16_t count)
	{
		instance.update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, count, 0, mission_id, false);
	}

	static uint32_t sharedMissionId()
	{
		pthread_mutex_lock(&MavlinkMissionManager::_shared_state_mutex);
		const uint32_t mission_id = MavlinkMissionManager::_crc32[MAV_MISSION_TYPE_MISSION];
		pthread_mutex_unlock(&MavlinkMissionManager::_shared_state_mutex);
		return mission_id;
	}

	static uint16_t sharedMissionCount()
	{
		pthread_mutex_lock(&MavlinkMissionManager::_shared_state_mutex);
		const uint16_t count = MavlinkMissionManager::_count[MAV_MISSION_TYPE_MISSION];
		pthread_mutex_unlock(&MavlinkMissionManager::_shared_state_mutex);
		return count;
	}

	// A `mission` message matching the shared state, except for the mission itself.
	static mission_s missionMessage(uint32_t mission_id, uint16_t count)
	{
		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_dataman_id = MavlinkMissionManager::_mission_dataman_id;
		mission.fence_dataman_id = MavlinkMissionManager::_fence_dataman_id;
		mission.safepoint_dataman_id = MavlinkMissionManager::_safepoint_dataman_id;
		mission.geofence_id = MavlinkMissionManager::_crc32[MAV_MISSION_TYPE_FENCE];
		mission.safe_points_id = MavlinkMissionManager::_crc32[MAV_MISSION_TYPE_RALLY];
		mission.mission_id = mission_id;
		mission.count = count;
		return mission;
	}

	static Mavlink *_mavlink;
	MavlinkMissionManager *_instance_a{nullptr};
	MavlinkMissionManager *_instance_b{nullptr};
};

Mavlink *MavlinkMissionManagerTest::_mavlink = nullptr;

TEST_F(MavlinkMissionManagerTest, ConcurrentCheckDoesNotRollBackStoredMission)
{
	// GIVEN: instance B checks for mission changes continuously on its own
	// thread, like a second Mavlink receiver
	std::atomic<bool> done{false};
	std::thread instance_b_receiver([&]() {
		while (!done.load()) {
			_instance_b->check_active_mission();
		}
	});

	// WHEN: instance A stores a sequence of new uploads
	static constexpr uint32_t UPLOADS = 20000;
	uint32_t rollbacks = 0;

	for (uint32_t mission_id = 1; mission_id <= UPLOADS; mission_id++) {
		storeMission(*_instance_a, mission_id, 1);

		// THEN: instance B never rolls the shared state back to an older upload.
		// Only instance A stores missions, so anything but the latest upload is
		// an outdated value that B adopted.
		if (sharedMissionId() != mission_id) {
			rollbacks++;
		}
	}

	done.store(true);
	instance_b_receiver.join();

	EXPECT_EQ(rollbacks, 0u);
	EXPECT_EQ(sharedMissionId(), UPLOADS);
}

TEST_F(MavlinkMissionManagerTest, MissionPublishedByAnotherModuleIsAdopted)
{
	// GIVEN: the shared state holds the old mission
	ASSERT_EQ(sharedMissionId(), OLD_MISSION_ID);

	// WHEN: another module (e.g. navigator) publishes a different mission
	uORB::Publication<mission_s> mission_pub{ORB_ID(mission)};
	mission_pub.publish(missionMessage(NEW_MISSION_ID, 1));
	_instance_b->check_active_mission();

	// THEN: it is adopted
	EXPECT_EQ(sharedMissionId(), NEW_MISSION_ID);
	EXPECT_EQ(sharedMissionCount(), 1);
}
