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
 * @file test_rtl_direct_mission_land.cpp
 *
 * Mission-landing Return walks the landing pattern geometrically.
 */

#include <gtest/gtest.h>

#include <vector>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include "navigator.h"
#include "rtl_direct_mission_land.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

namespace
{
constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;

class RtlDirectMissionLandTestPeer : public RtlDirectMissionLand
{
public:
	RtlDirectMissionLandTestPeer(Navigator *navigator, const mission_s &mission) :
		RtlDirectMissionLand(navigator, mission) {}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
	}

protected:
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

private:
	navigator_test::VectorMissionItemStore _mission_store{};
};

class RtlDirectMissionLandTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}

	void TearDown() override
	{
		for (orb_advert_t *pub : {&_vehicle_status_pub, &_global_position_pub, &_land_detected_pub, &_mission_pub}) {
			if (*pub != nullptr) {
				orb_unadvertise(*pub);
				*pub = nullptr;
			}
		}

		param_control_autosave(true);
	}

	template<typename T>
	static void publish(orb_advert_t &pub, const orb_metadata *meta, const T &data)
	{
		if (pub == nullptr) {
			pub = orb_advertise(meta, &data);

		} else {
			orb_publish(meta, pub, &data);
		}
	}

	void publishFlyingMulticopterAt(double lat, double lon, float alt)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		publish(_vehicle_status_pub, ORB_ID(vehicle_status), status);
		*_navigator.get_vstatus() = status;

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		publish(_land_detected_pub, ORB_ID(vehicle_land_detected), land_detected);
		*_navigator.get_land_detected() = land_detected;

		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = lat;
		global_position.lon = lon;
		global_position.alt = alt;
		publish(_global_position_pub, ORB_ID(vehicle_global_position), global_position);
		*_navigator.get_global_position() = global_position;
	}

	Navigator _navigator{};
	DatamanClient _dataman_client{};
	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _global_position_pub{nullptr};
	orb_advert_t _land_detected_pub{nullptr};
	orb_advert_t _mission_pub{nullptr};
};

// WHY: the return time estimate runs every 2 s while the mode is inactive and the landing is flown with the same traversal.
// WHAT: a DO_JUMP inside the landing pattern must not make either loop forever.
TEST_F(RtlDirectMissionLandTest, TimeEstimateIgnoresDoJumpInLandingPattern)
{
	// GIVEN: a flying multicopter and a landing pattern whose DO_JUMP repeats the approach once
	publishFlyingMulticopterAt(kBaseLat, kBaseLon, kAlt);
	_navigator.get_mission_result()->valid = true;

	mission_item_s land_start{};
	land_start.nav_cmd = NAV_CMD_DO_LAND_START;
	land_start.autocontinue = true;
	const mission_item_s first = navigator_test::makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt + 30.f);
	const mission_item_s second = navigator_test::makePositionItem(kBaseLat + 0.003, kBaseLon, kAlt + 30.f);
	const mission_item_s land = navigator_test::makePositionItem(kBaseLat + 0.003, kBaseLon, kAlt, NAV_CMD_LAND);
	std::vector<mission_item_s> items{land_start, first, second, navigator_test::makeDoJump(1, 1), land};

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.count = items.size();
	mission.land_start_index = 0;
	mission.land_index = 4;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	for (size_t i = 0; i < items.size(); ++i) {
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_1, i,
						      reinterpret_cast<uint8_t *>(&items[i]), sizeof(items[i])));
	}

	publish(_mission_pub, ORB_ID(mission), mission);
	RtlDirectMissionLandTestPeer rtl{&_navigator, mission};
	rtl.loadTestMission(items);
	rtl.run(false);

	// WHEN: the return time is estimated while the mode is inactive
	const rtl_time_estimate_s estimate = rtl.calc_rtl_time_estimate();

	// THEN: the estimate reaches the land item instead of looping on the DO_JUMP
	EXPECT_TRUE(estimate.valid);
	EXPECT_TRUE(PX4_ISFINITE(estimate.time_estimate));
	EXPECT_GT(estimate.time_estimate, 0.f);
}

} // namespace
