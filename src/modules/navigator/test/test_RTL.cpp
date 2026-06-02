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
 * @file test_RTL.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/wind.h>

#include "navigator.h"
#include "rtl.h"
#include "mission_route_types.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"

namespace
{

constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr float kApproachRadius = 50.f;

mission_item_s makeSafePointItem(double lat, double lon, float altitude, NAV_FRAME frame,
				 NAV_CMD nav_cmd = NAV_CMD_RALLY_POINT)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.frame = frame;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
	return item;
}

mission_item_s makeLandApproachItem(double lat, double lon, float altitude, float loiter_radius_m,
				    NAV_FRAME frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
	item.frame = frame;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
	item.altitude_is_relative = (frame == NAV_FRAME_GLOBAL_RELATIVE_ALT)
				    || (frame == NAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
	item.loiter_radius = loiter_radius_m;
	return item;
}

PositionYawSetpoint makePositionYawSetpointFromOffset(double base_lat, double base_lon, float north_m, float east_m,
		float alt)
{
	MapProjection ref{base_lat, base_lon};
	PositionYawSetpoint position{};
	ref.reproject(north_m, east_m, position.lat, position.lon);
	position.alt = alt;
	position.yaw = NAN;
	return position;
}

loiter_point_s makeLoiterPoint(const PositionYawSetpoint &position, float loiter_radius_m = kApproachRadius)
{
	loiter_point_s loiter_point{};
	loiter_point.lat = position.lat;
	loiter_point.lon = position.lon;
	loiter_point.height_m = position.alt;
	loiter_point.loiter_radius_m = loiter_radius_m;
	return loiter_point;
}

uint8_t countValidApproaches(const land_approaches_s &vtol_land_approaches)
{
	uint8_t count = 0;

	for (uint8_t i = 0; i < land_approaches_s::num_approaches_max; ++i) {
		if (vtol_land_approaches.approaches[i].isValid()) {
			++count;
		}
	}

	return count;
}

void expectLoiterPointNear(const loiter_point_s &actual, const PositionYawSetpoint &expected,
			   float loiter_radius_m = kApproachRadius)
{
	ASSERT_TRUE(actual.isValid());
	EXPECT_NEAR(actual.lat, expected.lat, 1e-9);
	EXPECT_NEAR(actual.lon, expected.lon, 1e-9);
	EXPECT_NEAR(actual.height_m, expected.alt, 0.01f);
	EXPECT_NEAR(actual.loiter_radius_m, loiter_radius_m, 0.01f);
}

struct ExtractValidSafePointPositionCase {
	const char *test_name;
	mission_item_s item;
	float home_altitude_amsl;
	bool expected_valid;
	double expected_lat;
	double expected_lon;
	float expected_alt;
};

struct ApproachGeometry {
	PositionYawSetpoint land;
	PositionYawSetpoint north;
	PositionYawSetpoint south;
};

struct VehicleStateCase {
	const char *test_name;
	bool is_vtol;
	uint8_t vehicle_type;
	bool expect_valid;
};

struct ReadFailureCase {
	const char *test_name;
	int32_t failure_index;
	bool expected_found;
	uint8_t expected_count;
};

} // namespace

class RTLTestPeer : public RTL
{
public:
	explicit RTLTestPeer(Navigator *navigator) : RTL(navigator) {}

	bool extractValidSafePointPositionForTest(const mission_item_s &safe_point_item, float home_altitude_amsl,
			PositionYawSetpoint &position) const
	{
		mission_route::Position safe_point_position{};

		if (!mission_route::extractSafePointPosition(safe_point_item, home_altitude_amsl, safe_point_position)) {
			return false;
		}

		return mission_route::copyPositionToYawSetpoint(safe_point_position, position);
	}

	loiter_point_s chooseBestLandingApproachForTest(const land_approaches_s &vtol_land_approaches)
	{
		_wind_sub.update();
		return chooseBestLandingApproach(vtol_land_approaches);
	}

	loiter_point_s selectLandingApproachForTest(const PositionYawSetpoint &destination)
	{
		_home_pos_sub.update();
		_vehicle_status_sub.update();
		_wind_sub.update();
		return selectLandingApproach(destination);
	}
};

class RTLTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};
	RTLTestPeer _rtl{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_SAFE_POINTS_0));

		mission_stats_entry_s empty_stats{};
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&empty_stats), sizeof(empty_stats)));

		_navigator.get_mission_route_cache()->invalidate();

		publishHomePosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		publishWind(0.f, 0.f);
	}

	void TearDown() override
	{
		_navigator.get_mission_route_cache()->invalidate();

		if (_home_pub != nullptr) {
			orb_unadvertise(_home_pub);
			_home_pub = nullptr;
		}

		if (_vehicle_status_pub != nullptr) {
			orb_unadvertise(_vehicle_status_pub);
			_vehicle_status_pub = nullptr;
		}

		if (_wind_pub != nullptr) {
			orb_unadvertise(_wind_pub);
			_wind_pub = nullptr;
		}

		param_control_autosave(true);
	}

	void loadSafePointsIntoRouteCache(const std::vector<mission_item_s> &items)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s copy = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_0, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&copy), sizeof(copy)));
		}

		mission_stats_entry_s stats{};
		stats.num_items = static_cast<uint16_t>(items.size());
		stats.opaque_id = ++_safe_points_opaque_id;
		stats.dataman_id = DM_KEY_SAFE_POINTS_0;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.safe_points_id = ++_safe_points_id;
		mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;

		MissionRouteCache *mission_route_cache = _navigator.get_mission_route_cache();
		mission_route_cache->invalidate();
		ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(*mission_route_cache, mission,
				[&] { return mission_route_cache->safePointsReady(); }))
				<< "test safe points did not load";
	}

	void publishHomePosition(const PositionYawSetpoint &position)
	{
		home_position_s home{};
		home.timestamp = hrt_absolute_time();
		home.lat = position.lat;
		home.lon = position.lon;
		home.alt = position.alt;
		home.valid_hpos = true;
		home.valid_alt = true;

		if (_home_pub == nullptr) {
			_home_pub = orb_advertise(ORB_ID(home_position), &home);

		} else {
			orb_publish(ORB_ID(home_position), _home_pub, &home);
		}
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;

		if (_vehicle_status_pub == nullptr) {
			_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

		} else {
			orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &status);
		}
	}

	void publishWind(float windspeed_north, float windspeed_east)
	{
		wind_s wind{};
		wind.timestamp = hrt_absolute_time();
		wind.windspeed_north = windspeed_north;
		wind.windspeed_east = windspeed_east;

		if (_wind_pub == nullptr) {
			_wind_pub = orb_advertise(ORB_ID(wind), &wind);

		} else {
			orb_publish(ORB_ID(wind), _wind_pub, &wind);
		}
	}

	ApproachGeometry makeApproachGeometry() const
	{
		return ApproachGeometry{
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 150.f, 100.f, kAlt + 20.f),
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 100.f, kAlt + 20.f),
		};
	}

	orb_advert_t _home_pub{nullptr};
	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	DatamanClient _dataman_client{};
	uint32_t _safe_points_id{0};
	uint32_t _safe_points_opaque_id{0};
};

// WHY: No land point means no usable approach bearing.
// WHAT: The chooser should return an invalid loiter.
TEST_F(RTLTest, ChooseBestLandingApproachRequiresLandLocation)
{
	// GIVEN: A valid loiter and no land point.
	publishWind(1.f, 0.f);

	const PositionYawSetpoint north_approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.approaches[0] = makeLoiterPoint(north_approach);

	// WHEN: The chooser runs.
	const loiter_point_s selected_approach = _rtl.chooseBestLandingApproachForTest(vtol_land_approaches);

	// THEN: It returns no approach.
	EXPECT_FALSE(selected_approach.isValid());
}

// WHY: Approach bearing is measured from the land point.
// WHAT: Home should not affect the choice.
TEST_F(RTLTest, ChooseBestLandingApproachUsesLandLocationAsBearingOrigin)
{
	// GIVEN: Two approaches on opposite sides of the land point and a 60 degree wind.
	publishWind(1.f, std::sqrt(3.0f));

	const ApproachGeometry geometry = makeApproachGeometry();
	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.land_location_lat_lon(0) = geometry.land.lat;
	vtol_land_approaches.land_location_lat_lon(1) = geometry.land.lon;
	vtol_land_approaches.approaches[0] = makeLoiterPoint(geometry.north);
	vtol_land_approaches.approaches[1] = makeLoiterPoint(geometry.south);

	// WHEN: The chooser evaluates the block.
	const loiter_point_s selected_approach = _rtl.chooseBestLandingApproachForTest(vtol_land_approaches);

	// THEN: The north approach is selected.
	expectLoiterPointNear(selected_approach, geometry.north);
}

class SelectLandingApproachVehicleStateTest :
	public RTLTest,
	public ::testing::WithParamInterface<VehicleStateCase>
{
};

// WHY: Only VTOL in FW mode should use an approach loiter.
// WHAT: All other vehicle states should reject it.
TEST_P(SelectLandingApproachVehicleStateTest, SelectLandingApproachHonorsVehicleState)
{
	const VehicleStateCase &test_case = GetParam();

	// GIVEN: One valid approach block, a 60 degree wind, and one vehicle state.
	const ApproachGeometry geometry = makeApproachGeometry();
	loadSafePointsIntoRouteCache({
		makeSafePointItem(geometry.land.lat, geometry.land.lon, geometry.land.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(geometry.north.lat, geometry.north.lon, geometry.north.alt, kApproachRadius),
		makeLandApproachItem(geometry.south.lat, geometry.south.lon, geometry.south.alt, kApproachRadius),
	});
	publishWind(1.f, std::sqrt(3.0f));
	publishVehicleStatus(test_case.is_vtol, test_case.vehicle_type);

	// WHEN: selectLandingApproach evaluates the destination.
	const loiter_point_s selected_approach = _rtl.selectLandingApproachForTest(geometry.land);

	// THEN: Only VTOL FW gets the selected approach.
	if (test_case.expect_valid) {
		expectLoiterPointNear(selected_approach, geometry.north);

	} else {
		EXPECT_FALSE(selected_approach.isValid());
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	SelectLandingApproachVehicleStateTest,
	::testing::Values(
		VehicleStateCase{"VtolRotaryWing", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false},
		VehicleStateCase{"NonVtolFixedWing", false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false},
		VehicleStateCase{"VtolFixedWing", true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true}),
	[](const ::testing::TestParamInfo<SelectLandingApproachVehicleStateTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: Each rally point owns the loiters that follow it.
// WHAT: Scanning should stop at the next rally point.
TEST_F(RTLTest, ScanVtolLandApproachBlockStopsAtNextRallyPoint)
{
	// GIVEN: One block with two loiters, then a new rally point.
	const PositionYawSetpoint land_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	const PositionYawSetpoint loiter_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 50.f, kAlt + 30.f);
	const PositionYawSetpoint land_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_3 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 40.f);

	VectorProvider provider({}, {
		makeSafePointItem(land_1.lat, land_1.lon, land_1.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_1.lat, loiter_1.lon, loiter_1.alt, kApproachRadius),
		makeLandApproachItem(loiter_2.lat, loiter_2.lon, loiter_2.alt, kApproachRadius),
		makeSafePointItem(land_2.lat, land_2.lon, land_2.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_3.lat, loiter_3.lon, loiter_3.alt, kApproachRadius),
	});

	// WHEN: The first block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: Only the first two loiters are returned.
	ASSERT_TRUE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 2);
	expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	expectLoiterPointNear(scanned_block.approaches[1], loiter_2);
	EXPECT_FALSE(scanned_block.approaches[2].isValid());
}

// WHY: The result array has a fixed size.
// WHAT: Extra loiters should be ignored once it is full.
TEST_F(RTLTest, ScanVtolLandApproachBlockCapsApproachCount)
{
	// GIVEN: More valid loiters than the block can hold.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	std::vector<mission_item_s> mission_items{
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
	};

	PositionYawSetpoint last_included{};

	for (uint8_t i = 0; i < land_approaches_s::num_approaches_max + 2; ++i) {
		const PositionYawSetpoint approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 25.f * (i + 1), 0.f,
						     kAlt + 10.f + i);

		if (i == land_approaches_s::num_approaches_max - 1) {
			last_included = approach;
		}

		mission_items.push_back(makeLandApproachItem(approach.lat, approach.lon, approach.alt, kApproachRadius));
	}

	VectorProvider provider({}, mission_items);

	// WHEN: The block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: The result stops at the hard limit.
	ASSERT_TRUE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), land_approaches_s::num_approaches_max);
	expectLoiterPointNear(scanned_block.approaches[0], makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 25.f, 0.f, kAlt + 10.f));
	expectLoiterPointNear(scanned_block.approaches[land_approaches_s::num_approaches_max - 1], last_included);
}

// WHY: A rally point can own an empty block.
// WHAT: Another rally point right after it should keep the block empty.
TEST_F(RTLTest, ScanVtolLandApproachBlockHandlesEmptyBlockBeforeNextRallyPoint)
{
	// GIVEN: A rally point followed immediately by another rally point.
	const PositionYawSetpoint land_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint land_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const PositionYawSetpoint loiter = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt + 20.f);

	VectorProvider provider({}, {
		makeSafePointItem(land_1.lat, land_1.lon, land_1.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(land_2.lat, land_2.lon, land_2.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter.lat, loiter.lon, loiter.alt, kApproachRadius),
	});

	// WHEN: The first block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: It stays empty.
	EXPECT_FALSE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

// WHY: End-of-mission is the other empty-block case.
// WHAT: A final rally point should also return zero approaches.
TEST_F(RTLTest, ScanVtolLandApproachBlockHandlesEmptyBlockAtMissionEnd)
{
	// GIVEN: A mission that ends with a rally point.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	VectorProvider provider({}, {
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: Its block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: It is empty.
	EXPECT_FALSE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

/**
 * @brief Read-failure cases while scanning a safe-point approach block.
 */
class ScanVtolLandApproachBlockReadFailureTest :
	public RTLTest,
	public ::testing::WithParamInterface<ReadFailureCase>
{
};

// WHY: Read failures should not leave partial results behind.
// WHAT: Scanning should stop cleanly on a broken item.
TEST_P(ScanVtolLandApproachBlockReadFailureTest, ScanVtolLandApproachBlockHandlesReadFailures)
{
	const ReadFailureCase &test_case = GetParam();

	// GIVEN: One rally point with two loiters.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	const PositionYawSetpoint loiter_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 30.f);
	const std::vector<mission_item_s> mission_items{
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_1.lat, loiter_1.lon, loiter_1.alt, kApproachRadius),
		makeLandApproachItem(loiter_2.lat, loiter_2.lon, loiter_2.alt, kApproachRadius),
	};

	VectorProvider provider(std::vector<mission_item_s> {}, mission_items, {}, {test_case.failure_index});

	// WHEN: The block scan hits a read failure.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: The scan stops without inventing extra approaches.
	EXPECT_EQ(found_approach, test_case.expected_found);
	EXPECT_EQ(countValidApproaches(scanned_block), test_case.expected_count);

	if (test_case.expected_count > 0) {
		expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	}

	EXPECT_FALSE(scanned_block.approaches[1].isValid());
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	ScanVtolLandApproachBlockReadFailureTest,
	::testing::Values(
		ReadFailureCase{"FirstLoiter", 1, false, 0},
		ReadFailureCase{"SecondLoiter", 2, true, 1}),
	[](const ::testing::TestParamInfo<ScanVtolLandApproachBlockReadFailureTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

class FindAssociatedSafePointTest : public ::testing::Test {};

// WHY: Association is distance-limited.
// WHAT: A rally point outside the 10 m window should be skipped.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexRejectsFarSafePoints)
{
	// GIVEN: One rally point outside the threshold and one inside.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint outside_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 10.25f, 0.f, kAlt);
	const PositionYawSetpoint inside_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 9.75f, kAlt);

	VectorProvider provider({}, {
		makeSafePointItem(outside_safe_point.lat, outside_safe_point.lon, outside_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(inside_safe_point.lat, inside_safe_point.lon, inside_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The nearby rally point is selected.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), inside_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), inside_safe_point.lon, 1e-9);
}

// WHY: The first valid nearby rally point owns the block.
// WHAT: A later match should not replace it.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexReturnsFirstMatch)
{
	// GIVEN: Two nearby valid rally points.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint first_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 4.f, 0.f, kAlt);
	const PositionYawSetpoint second_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	VectorProvider provider({}, {
		makeSafePointItem(first_safe_point.lat, first_safe_point.lon, first_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(second_safe_point.lat, second_safe_point.lon, second_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The first match is returned.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), first_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), first_safe_point.lon, 1e-9);
}

// WHY: Association reads can fail too.
// WHAT: A failed load should stop the search and return no match.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexHandlesReadFailure)
{
	// GIVEN: A matching rally point hidden behind a failed read.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint skipped_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 4.f, 0.f, kAlt);
	const PositionYawSetpoint later_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(skipped_safe_point.lat, skipped_safe_point.lon, skipped_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(later_safe_point.lat, later_safe_point.lon, later_safe_point.alt, NAV_FRAME_GLOBAL),
	};
	VectorProvider provider(std::vector<mission_item_s> {}, safe_points, {}, {0});

	// WHEN: The association lookup hits the failed read.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The search stops and reports no match.
	EXPECT_FALSE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
}

// WHY: An invalid rally point should not block the next one.
// WHAT: Association should skip bad entries and keep scanning.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexSkipsInvalidRallyPoints)
{
	// GIVEN: One invalid nearby rally point followed by a valid nearby rally point.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint valid_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	VectorProvider provider({}, {
		makeSafePointItem(91.0, kBaseLon, kAlt, NAV_FRAME_GLOBAL),
		makeSafePointItem(valid_safe_point.lat, valid_safe_point.lon, valid_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The valid rally point is returned.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), valid_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), valid_safe_point.lon, 1e-9);
}

class ExtractValidSafePointPositionTest :
	public RTLTest,
	public ::testing::WithParamInterface<ExtractValidSafePointPositionCase>
{
};

// WHY: Safe-point parsing should fail fast on bad input.
// WHAT: Valid frames pass; bad commands, frames and coordinates do not.
TEST_P(ExtractValidSafePointPositionTest, ExtractValidSafePointPositionValidatesInput)
{
	const ExtractValidSafePointPositionCase &test_case = GetParam();
	PositionYawSetpoint extracted_position{};

	// GIVEN: One safe-point item.
	// WHEN: The parser runs.
	const bool is_valid = _rtl.extractValidSafePointPositionForTest(test_case.item, test_case.home_altitude_amsl,
			      extracted_position);

	// THEN: Only valid items are accepted.
	EXPECT_EQ(is_valid, test_case.expected_valid);

	if (test_case.expected_valid) {
		EXPECT_NEAR(extracted_position.lat, test_case.expected_lat, 1e-9);
		EXPECT_NEAR(extracted_position.lon, test_case.expected_lon, 1e-9);
		EXPECT_NEAR(extracted_position.alt, test_case.expected_alt, 0.01f);
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	ExtractValidSafePointPositionTest,
	::testing::Values(
ExtractValidSafePointPositionCase{
	"GlobalAbsoluteRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	510.f,
},
ExtractValidSafePointPositionCase{
	"GlobalIntRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL_INT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	510.f,
},
ExtractValidSafePointPositionCase{
	"GlobalRelativeRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	kAlt + 25.f,
},
ExtractValidSafePointPositionCase{
	"GlobalRelativeIntRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	kAlt + 25.f,
},
ExtractValidSafePointPositionCase{
	"RelativeRallyPointWithoutHomeAltitude",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	NAN,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"UnsupportedFrame",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_MISSION),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NonRallyCommand",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL, NAV_CMD_WAYPOINT),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NanLatitude",
	makeSafePointItem(NAN, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NullIsland",
	makeSafePointItem(0.0, 0.0, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"LatitudeOutOfRange",
	makeSafePointItem(91.0, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
},
ExtractValidSafePointPositionCase{
	"LongitudeOutOfRange",
	makeSafePointItem(kBaseLat, 181.0, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	NAN,
	NAN,
	NAN,
}),
[](const ::testing::TestParamInfo<ExtractValidSafePointPositionTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: Approach altitude can be absolute or relative.
// WHAT: Relative altitude should add home altitude; absolute altitude should not.
TEST_F(RTLTest, MakeVtolLandApproachPointConvertsRelativeAndAbsoluteAltitude)
{
	// GIVEN: Absolute and relative loiter items in both MAVLink frame variants.
	const PositionYawSetpoint absolute_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, 530.f);
	const PositionYawSetpoint relative_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 40.f, NAN);

	const mission_item_s absolute_item = makeLandApproachItem(absolute_position.lat, absolute_position.lon,
					     absolute_position.alt, kApproachRadius);
	const mission_item_s absolute_int_item = makeLandApproachItem(absolute_position.lat, absolute_position.lon,
			absolute_position.alt, kApproachRadius, NAV_FRAME_GLOBAL_INT);
	const mission_item_s relative_item = makeLandApproachItem(relative_position.lat, relative_position.lon, 25.f,
					     kApproachRadius, NAV_FRAME_GLOBAL_RELATIVE_ALT);
	const mission_item_s relative_int_item = makeLandApproachItem(relative_position.lat, relative_position.lon, 25.f,
			kApproachRadius, NAV_FRAME_GLOBAL_RELATIVE_ALT_INT);

	VectorProvider provider({}, {
		makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL),
		absolute_item,
		absolute_int_item,
		relative_item,
		relative_int_item,
	});

	// WHEN: The mission items are converted while reading the approach block.
	land_approaches_s vtol_land_approaches{};
	provider.scanVtolLandApproachBlockForTest(0, kAlt, &vtol_land_approaches);
	const loiter_point_s absolute_point = vtol_land_approaches.approaches[0];
	const loiter_point_s absolute_int_point = vtol_land_approaches.approaches[1];
	const loiter_point_s relative_point = vtol_land_approaches.approaches[2];
	const loiter_point_s relative_int_point = vtol_land_approaches.approaches[3];

	// THEN: The AMSL altitude is resolved correctly.
	ASSERT_TRUE(absolute_point.isValid());
	ASSERT_TRUE(absolute_int_point.isValid());
	ASSERT_TRUE(relative_point.isValid());
	ASSERT_TRUE(relative_int_point.isValid());
	EXPECT_NEAR(absolute_point.height_m, 530.f, 0.01f);
	EXPECT_NEAR(absolute_int_point.height_m, 530.f, 0.01f);
	EXPECT_NEAR(relative_point.height_m, kAlt + 25.f, 0.01f);
	EXPECT_NEAR(relative_int_point.height_m, kAlt + 25.f, 0.01f);
	EXPECT_NEAR(absolute_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(absolute_int_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(relative_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(relative_int_point.loiter_radius_m, kApproachRadius, 0.01f);
}
