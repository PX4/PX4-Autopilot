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
 * @file test_prec_takeoff_integration.cpp
 * Precision takeoff integration tests through Navigator modes.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"
#include "takeoff.h"

#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
#include "vtol_takeoff.h"
#endif // CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/prec_takeoff_status.h>
#include <uORB/uORBManager.hpp>

#include <memory>

class PrecTakeoffIntegrationTest : public ::testing::Test
{
protected:
	static constexpr double kRefLat = 47.397742;
	static constexpr double kRefLon = 8.545594;
	static constexpr float kGroundAltitude = 500.f;
	static constexpr float kTakeoffAltitude = 520.f;

	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
		(void)navigatorDatamanRuntime();
	}

	void SetUp() override
	{
		_param = param_find("MIS_TKO_PREC");
		ASSERT_NE(_param, PARAM_INVALID);
		ASSERT_EQ(param_get(_param, &_param_backup), 0);

		prec_takeoff_status_s status{};

		while (_status_sub.update(&status)) {}
	}

	void TearDown() override
	{
		_navigator.reset();
		param_set_no_notification(_param, &_param_backup);
	}

	void initializeNavigator(bool precision_enabled = true, bool is_vtol = false)
	{
		const int32_t enabled = precision_enabled ? 1 : 0;
		ASSERT_EQ(param_set_no_notification(_param, &enabled), 0);
		_navigator = std::make_unique<Navigator>();
		ASSERT_EQ(_navigator->get_prec_takeoff()->enabled(), precision_enabled);

		auto &global_pos = *_navigator->get_global_position();
		global_pos.timestamp = hrt_absolute_time();
		global_pos.lat = kRefLat;
		global_pos.lon = kRefLon;
		global_pos.alt = kGroundAltitude;

		auto &local_pos = *_navigator->get_local_position();
		local_pos.xy_global = true;
		local_pos.ref_lat = kRefLat;
		local_pos.ref_lon = kRefLon;
		local_pos.ref_timestamp = 1;
		local_pos.heading_good_for_control = true;

		_navigator->get_home_position()->alt = kGroundAltitude;
		_navigator->get_land_detected()->landed = true;
		_navigator->get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_navigator->get_vstatus()->is_vtol = is_vtol;
	}

	void setTakeoffRequest()
	{
		auto &request = _navigator->get_takeoff_triplet()->current;
		request.valid = true;
		request.lat = kRefLat;
		request.lon = kRefLon;
		request.alt = kTakeoffAltitude;
		request.yaw = NAN;
	}

	void publishTarget(float x_abs, float y_abs)
	{
		landing_target_pose_s target{};
		target.timestamp = hrt_absolute_time();
		target.abs_pos_valid = true;
		target.x_abs = x_abs;
		target.y_abs = y_abs;
		ASSERT_TRUE(_target_pub.publish(target));
	}

	matrix::Vector2f setpointLocal() const
	{
		const auto &sp = _navigator->get_position_setpoint_triplet()->current;
		MapProjection ref{kRefLat, kRefLon};
		return ref.project(sp.lat, sp.lon);
	}

	void expectStatus(uint8_t state, bool adjusted)
	{
		// Navigator publishes after running the active mode each cycle.
		_navigator->get_prec_takeoff()->publish_status();
		prec_takeoff_status_s status{};
		ASSERT_TRUE(_status_sub.update(&status));
		EXPECT_EQ(status.state, state);
		EXPECT_EQ(status.setpoint_adjusted, adjusted);
	}

	std::unique_ptr<Navigator> _navigator;
	uORB::Publication<landing_target_pose_s> _target_pub{ORB_ID(landing_target_pose)};
	uORB::Subscription _status_sub{ORB_ID(prec_takeoff_status)};
	param_t _param{PARAM_INVALID};
	int32_t _param_backup{0};
};

TEST_F(PrecTakeoffIntegrationTest, DisabledPrecisionTakeoffIgnoresFreshTarget)
{
	// GIVEN: A normal takeoff with the feature disabled and a valid target available
	ASSERT_NO_FATAL_FAILURE(initializeNavigator(false));
	Takeoff takeoff{_navigator.get()};
	takeoff.initialize();
	setTakeoffRequest();
	takeoff.on_activation();

	auto &sp = _navigator->get_position_setpoint_triplet()->current;
	ASSERT_TRUE(sp.valid);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);

	// WHEN: The actual takeoff mode checks progress during the climb
	_navigator->get_land_detected()->landed = false;
	publishTarget(3.f, 4.f);
	takeoff.on_active();

	// THEN: The mode does not invoke precision corrections or request target tracking
	EXPECT_DOUBLE_EQ(sp.lat, kRefLat);
	EXPECT_DOUBLE_EQ(sp.lon, kRefLon);
	EXPECT_FLOAT_EQ(sp.alt, kTakeoffAltitude);
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	EXPECT_FALSE(_navigator->get_mission_result()->finished);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_STOPPED, false);
}

TEST_F(PrecTakeoffIntegrationTest, MulticopterLoiterKeepsCorrectedTakeoffPosition)
{
	// GIVEN: A multicopter climbing toward a target 3 m north and 4 m east
	ASSERT_NO_FATAL_FAILURE(initializeNavigator());
	Takeoff takeoff{_navigator.get()};
	takeoff.initialize();
	setTakeoffRequest();
	takeoff.on_activation();

	auto &sp = _navigator->get_position_setpoint_triplet()->current;
	ASSERT_TRUE(sp.valid);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	_navigator->get_land_detected()->landed = false;
	publishTarget(3.f, 4.f);
	takeoff.on_active();

	const matrix::Vector2f corrected_local = setpointLocal();
	ASSERT_NEAR(corrected_local(0), 3.f, 0.01f);
	ASSERT_NEAR(corrected_local(1), 4.f, 0.01f);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	ASSERT_FLOAT_EQ(sp.alt, kTakeoffAltitude);
	const position_setpoint_s corrected_sp = sp;
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING, true);

	// WHEN: Climb completes while the vehicle still lags 5 m behind the target in XY
	_navigator->get_global_position()->alt = kTakeoffAltitude;
	takeoff.on_active();

	// THEN: Loiter holds the corrected setpoint, rather than the current vehicle position
	ASSERT_TRUE(_navigator->get_mission_result()->finished);
	ASSERT_TRUE(sp.valid);
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_DOUBLE_EQ(sp.lat, corrected_sp.lat);
	EXPECT_DOUBLE_EQ(sp.lon, corrected_sp.lon);
	EXPECT_FLOAT_EQ(sp.alt, corrected_sp.alt);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_DONE, true);

	// Subsequent target motion cannot move the completed takeoff's loiter setpoint.
	publishTarget(6.f, 8.f);
	takeoff.on_active();
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_DOUBLE_EQ(sp.lat, corrected_sp.lat);
	EXPECT_DOUBLE_EQ(sp.lon, corrected_sp.lon);
	EXPECT_FLOAT_EQ(sp.alt, corrected_sp.alt);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_STOPPED, false);
}

#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
TEST_F(PrecTakeoffIntegrationTest, VtolClimbHandoffKeepsCorrectedTakeoffPosition)
{
	// GIVEN: A VTOL taking off from the local origin with precision takeoff enabled
	ASSERT_NO_FATAL_FAILURE(initializeNavigator(true, true));
	VtolTakeoff takeoff{_navigator.get()};
	takeoff.initialize();
	takeoff.setTransitionAltitudeAbsolute(kTakeoffAltitude);
	takeoff.setTransitionDirection(NAN);
	MapProjection ref{kRefLat, kRefLon};
	matrix::Vector2d loiter_location;
	ref.reproject(100.f, 100.f, loiter_location(0), loiter_location(1));
	takeoff.setLoiterLocation(loiter_location);
	takeoff.on_activation();

	auto &sp = _navigator->get_position_setpoint_triplet()->current;
	ASSERT_TRUE(sp.valid);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	ASSERT_DOUBLE_EQ(sp.lat, kRefLat);
	ASSERT_DOUBLE_EQ(sp.lon, kRefLon);

	// WHEN: A target corrects the climb setpoint by 3 m north and 4 m east
	_navigator->get_land_detected()->landed = false;
	publishTarget(3.f, 4.f);
	takeoff.on_active();

	const position_setpoint_s corrected_sp = sp;
	const matrix::Vector2f corrected_local = setpointLocal();
	ASSERT_NEAR(corrected_local(0), 3.f, 0.01f);
	ASSERT_NEAR(corrected_local(1), 4.f, 0.01f);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	ASSERT_FLOAT_EQ(sp.alt, kTakeoffAltitude);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING, true);

	// WHEN: Climb completes with the vehicle still at the original liftoff XY position
	_navigator->get_global_position()->alt = kTakeoffAltitude;
	takeoff.on_active();

	// THEN: Rebuilding the setpoint for ALIGN_HEADING preserves the precision correction
	ASSERT_TRUE(sp.valid);
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(sp.lat, corrected_sp.lat);
	EXPECT_DOUBLE_EQ(sp.lon, corrected_sp.lon);
	EXPECT_FLOAT_EQ(sp.alt, corrected_sp.alt);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_DONE, true);
}
#endif // CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

TEST_F(PrecTakeoffIntegrationTest, MissionTakeoffCorrectionPreservesPlannedDestination)
{
	// GIVEN: A stored VTOL mission with a forward-flight destination away from the takeoff pad
	ASSERT_NO_FATAL_FAILURE(initializeNavigator(true, true));
	auto &home = *_navigator->get_home_position();
	home.timestamp = hrt_absolute_time();
	home.lat = kRefLat;
	home.lon = kRefLon;
	home.valid_hpos = true;
	home.valid_alt = true;
	auto &vehicle_status = *_navigator->get_vstatus();
	vehicle_status.timestamp = hrt_absolute_time();
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	uORB::Publication<home_position_s> home_pub{ORB_ID(home_position)};
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_global_position_s> global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s> land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<geofence_status_s> geofence_status_pub{ORB_ID(geofence_status)};
	uORB::Publication<mission_s> mission_pub{ORB_ID(mission)};
	ASSERT_TRUE(home_pub.publish(home));
	ASSERT_TRUE(vehicle_status_pub.publish(vehicle_status));
	ASSERT_TRUE(global_pos_pub.publish(*_navigator->get_global_position()));
	ASSERT_TRUE(land_detected_pub.publish(*_navigator->get_land_detected()));
	geofence_status_s geofence_status{};
	geofence_status.timestamp = hrt_absolute_time();
	geofence_status.status = geofence_status_s::GF_STATUS_READY;
	ASSERT_TRUE(geofence_status_pub.publish(geofence_status));

	mission_item_s planned_takeoff{};
	planned_takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	planned_takeoff.frame = NAV_FRAME_GLOBAL;
	planned_takeoff.altitude = kTakeoffAltitude;
	planned_takeoff.autocontinue = true;
	planned_takeoff.yaw = NAN;
	MapProjection ref{kRefLat, kRefLon};
	ref.reproject(100.f, 50.f, planned_takeoff.lat, planned_takeoff.lon);
	mission_item_s planned_land = planned_takeoff;
	planned_land.nav_cmd = NAV_CMD_VTOL_LAND;
	planned_land.altitude = kGroundAltitude;

	DatamanClient dataman;
	ASSERT_TRUE(dataman.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
	ASSERT_TRUE(dataman.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
				      reinterpret_cast<uint8_t *>(&planned_takeoff), sizeof(planned_takeoff)));
	ASSERT_TRUE(dataman.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 1,
				      reinterpret_cast<uint8_t *>(&planned_land), sizeof(planned_land)));
	mission_s plan{};
	plan.timestamp = hrt_absolute_time();
	plan.mission_id = 1;
	plan.count = 2;
	plan.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	plan.land_start_index = -1;
	plan.land_index = 1;
	ASSERT_TRUE(mission_pub.publish(plan));

	Mission mission{_navigator.get()};
	mission.initialize();
	mission.on_inactive();
	mission.on_activation();
	ASSERT_TRUE(_navigator->get_mission_result()->valid);
	auto &sp = _navigator->get_position_setpoint_triplet()->current;
	ASSERT_TRUE(sp.valid);
	ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);

	// WHEN: Precision takeoff corrects the climb, then the mission proceeds to forward flight
	_navigator->get_land_detected()->landed = false;
	ASSERT_TRUE(land_detected_pub.publish(*_navigator->get_land_detected()));
	publishTarget(3.f, 4.f);
	mission.on_active();
	const matrix::Vector2f corrected_local = setpointLocal();
	ASSERT_NEAR(corrected_local(0), 3.f, 0.01f);
	ASSERT_NEAR(corrected_local(1), 4.f, 0.01f);
	expectStatus(prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING, true);

	auto &global_pos = *_navigator->get_global_position();
	global_pos.lat = sp.lat;
	global_pos.lon = sp.lon;
	global_pos.alt = kTakeoffAltitude;
	ASSERT_TRUE(global_pos_pub.publish(global_pos));
	mission.on_active();
	_navigator->get_local_position()->heading = sp.yaw;
	mission.on_active();
	vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	ASSERT_TRUE(vehicle_status_pub.publish(vehicle_status));
	mission.on_active();

	// THEN: The next flight destination is the planned waypoint, independent of the pad correction
	ASSERT_TRUE(sp.valid);
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(sp.lat, planned_takeoff.lat);
	EXPECT_DOUBLE_EQ(sp.lon, planned_takeoff.lon);
	EXPECT_FLOAT_EQ(sp.alt, planned_takeoff.altitude);
	EXPECT_FALSE(_navigator->get_mission_result()->finished);

	mission_item_s stored_takeoff{};
	ASSERT_TRUE(dataman.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
				     reinterpret_cast<uint8_t *>(&stored_takeoff), sizeof(stored_takeoff)));
	EXPECT_DOUBLE_EQ(stored_takeoff.lat, planned_takeoff.lat);
	EXPECT_DOUBLE_EQ(stored_takeoff.lon, planned_takeoff.lon);
	EXPECT_FLOAT_EQ(stored_takeoff.altitude, planned_takeoff.altitude);
}
