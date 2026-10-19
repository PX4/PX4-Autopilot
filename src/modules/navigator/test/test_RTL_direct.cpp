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
 * @file test_RTL_direct.cpp
 * Regression coverage for the observable direct Return arrival sequence.
 */

#include <gtest/gtest.h>

#include "navigator.h"
#include "rtl_direct.h"
#include "support/navigator_dataman_test.h"

#include <lib/geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

#include <cfloat>
#include <memory>
#include <vector>

class RtlDirectTestPeer : public RtlDirect
{
public:
	explicit RtlDirectTestPeer(Navigator *navigator) : RtlDirect(navigator) {}

	const mission_item_s &missionItem() const { return _mission_item; }
	void refreshParameters() { updateParams(); }

	// Complete position/yaw acceptance without changing the generated mission item.
	// Timed and unlimited holds still use MissionBlock's normal completion checks.
	void markPositionReached()
	{
		_waypoint_position_reached = true;
		_waypoint_yaw_reached = true;
		_time_wp_reached = hrt_absolute_time();
	}

	void ageWaypointReached(float elapsed_seconds)
	{
		_time_wp_reached -= static_cast<hrt_abstime>(elapsed_seconds * 1_s);
	}
};

class RtlDirectTest : public NavigatorDatamanTestBase
{
protected:
	static constexpr double kLatitude = 47.397742;
	static constexpr double kLongitude = 8.545594;
	static constexpr float kDestinationAltitude = 450.f;
	static constexpr float kReturnAltitude = 550.f;
	static constexpr float kDescentHeight = 30.f;
	static constexpr float kLoiterRadius = 60.f;
	static constexpr float kDestinationYaw = 0.4f;

	// navigator_mission_item.sequence_current is part of existing direct RTL logging.
	static constexpr uint16_t kLogClimb = 0;
	static constexpr uint16_t kLogMoveToApproach = 2;
	static constexpr uint16_t kLogDescend = 3;
	static constexpr uint16_t kLogHold = 4;
	static constexpr uint16_t kLogMoveToLand = 5;
	static constexpr uint16_t kLogTransition = 6;
	static constexpr uint16_t kLogHoverToLand = 7;
	static constexpr uint16_t kLogLand = 8;

	void SetUp() override
	{
		setParameter("RTL_LAND_DELAY", 0.f);
		setParameter("RTL_DESCEND_ALT", kDescentHeight);
		setParameter("RTL_LOITER_RAD", kLoiterRadius);
		setParameter("RTL_MIN_DIST", 1000.f);
		setParameter("RTL_PLD_MD", int32_t{0});
		setParameter("WV_EN", int32_t{0});
		_navigator = std::make_unique<Navigator>();
		_rtl = std::make_unique<RtlDirectTestPeer>(_navigator.get());
		_rtl->refreshParameters();
		_rtl->initialize();

		setVehicle(false, false);
		setPosition(kLatitude, kLongitude, kReturnAltitude);
		vehicle_land_detected_s landed{};
		landed.timestamp = hrt_absolute_time();
		landed.landed = false;
		*_navigator->get_land_detected() = landed;
		ASSERT_TRUE(_landed_pub.publish(landed));
		_navigator->get_local_position()->heading = kDestinationYaw;
		_navigator->get_local_position()->heading_good_for_control = true;

		_destination = {kLatitude + 0.002, kLongitude, kDestinationAltitude, kDestinationYaw};
		navigator_mission_item_s logged_item{};

		while (_mission_item_sub.update(&logged_item)) {}
	}

	void TearDown() override
	{
		if (_rtl && _rtl->isActive()) {
			_rtl->run(false);
		}

		_rtl.reset();
		_navigator.reset();

		for (const ParameterBackup &backup : _parameter_backups) {
			EXPECT_EQ(param_set_no_notification(backup.handle, &backup.value), PX4_OK);
		}
	}

	template<typename T>
	void setParameter(const char *name, T value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		bool backed_up = false;

		for (const ParameterBackup &backup : _parameter_backups) {
			backed_up |= backup.handle == handle;
		}

		if (!backed_up) {
			ParameterBackup backup{handle, {}};

			if (param_type(handle) == PARAM_TYPE_FLOAT) {
				ASSERT_EQ(param_get(handle, &backup.value.f), PX4_OK);

			} else {
				ASSERT_EQ(param_get(handle, &backup.value.i), PX4_OK);
			}

			_parameter_backups.push_back(backup);
		}

		ASSERT_EQ(param_set_no_notification(handle, &value), PX4_OK);

		if (_rtl) {
			_rtl->refreshParameters();
		}
	}

	void setVehicle(bool fixed_wing, bool vtol)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.is_vtol = vtol;
		*_navigator->get_vstatus() = status;
		ASSERT_TRUE(_status_pub.publish(status));
	}

	void setPosition(double latitude, double longitude, float altitude)
	{
		vehicle_global_position_s position{};
		position.timestamp = hrt_absolute_time();
		position.lat = latitude;
		position.lon = longitude;
		position.alt = altitude;
		*_navigator->get_global_position() = position;
		ASSERT_TRUE(_position_pub.publish(position));
	}

	void activate(const loiter_point_s &approach = {})
	{
		_rtl->setRtlAlt(kReturnAltitude);
		_rtl->setRtlPosition(_destination, approach);
		_rtl->run(true);
	}

	void completePositionItem()
	{
		const mission_item_s item = _rtl->missionItem();
		setPosition(item.lat, item.lon, item.altitude);
		_rtl->markPositionReached();
		_rtl->run(true);
	}

	void expectLoggedItem(uint16_t sequence)
	{
		navigator_mission_item_s logged_item{};
		ASSERT_TRUE(_mission_item_sub.update(&logged_item));
		EXPECT_EQ(logged_item.sequence_current, sequence);
		EXPECT_EQ(logged_item.nav_cmd, _rtl->missionItem().nav_cmd);
		EXPECT_FLOAT_EQ(logged_item.altitude, _rtl->missionItem().altitude);
	}

	const position_setpoint_triplet_s &triplet() const { return *_navigator->get_position_setpoint_triplet(); }

	struct ParameterBackup {
		param_t handle;
		param_value_u value;
	};

	std::vector<ParameterBackup> _parameter_backups;
	std::unique_ptr<Navigator> _navigator;
	std::unique_ptr<RtlDirectTestPeer> _rtl;
	PositionYawSetpoint _destination{};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_global_position_s> _position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s> _landed_pub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _mission_item_sub{ORB_ID(navigator_mission_item)};
};

TEST_F(RtlDirectTest, ClimbsBeforeMovingToTheDestination)
{
	setPosition(kLatitude, kLongitude, kReturnAltitude - 40.f);
	activate();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_DOUBLE_EQ(_rtl->missionItem().lat, kLatitude);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kReturnAltitude);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogClimb);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(triplet().current.lat, _destination.lat);
	EXPECT_FLOAT_EQ(triplet().current.alt, kReturnAltitude);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogMoveToApproach);
}

TEST_F(RtlDirectTest, EnforcedReturnAltitudeKeepsAltitudeStageWhenAlreadyAboveIt)
{
	setPosition(kLatitude, kLongitude, kReturnAltitude + 40.f);
	_rtl->setReturnAltMin(true);
	activate();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_DOUBLE_EQ(_rtl->missionItem().lat, kLatitude);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kReturnAltitude);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogClimb);
}

TEST_F(RtlDirectTest, MulticopterZeroDelayStartsLandingAtArrival)
{
	activate();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().yaw, kDestinationYaw);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogMoveToApproach);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LAND);
	EXPECT_EQ(triplet().current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kDestinationAltitude);
	EXPECT_FLOAT_EQ(_rtl->missionItem().yaw, kDestinationYaw);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogLand);
}

TEST_F(RtlDirectTest, PositiveDelayDescendsBeforeStartingATimedHold)
{
	static constexpr float kLandDelay = 5.f;
	setParameter("RTL_LAND_DELAY", kLandDelay);
	activate();
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kDestinationAltitude + kDescentHeight);
	EXPECT_FLOAT_EQ(_rtl->missionItem().yaw, kDestinationYaw);
	EXPECT_FALSE(triplet().previous.valid);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogDescend);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().time_inside, kLandDelay);
	EXPECT_TRUE(_rtl->missionItem().autocontinue);
	EXPECT_FLOAT_EQ(triplet().current.alt_acceptance_radius, FLT_MAX);
	expectLoggedItem(kLogHold);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FALSE(_mission_item_sub.updated());

	_rtl->ageWaypointReached(kLandDelay + 1.f);
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FALSE(triplet().previous.valid);
	expectLoggedItem(kLogHoverToLand);
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LAND);
	expectLoggedItem(kLogLand);
}

TEST_F(RtlDirectTest, NegativeDelayHoldsIndefinitelyAfterDescending)
{
	setParameter("RTL_LAND_DELAY", -1.f);
	activate();
	completePositionItem();
	expectLoggedItem(kLogDescend);
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_UNLIMITED);
	EXPECT_FALSE(_rtl->missionItem().autocontinue);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kDestinationAltitude + kDescentHeight);
	EXPECT_NE(triplet().current.alt_acceptance_radius, FLT_MAX);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogHold);

	completePositionItem();
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_UNLIMITED);
	EXPECT_FALSE(_mission_item_sub.updated());
}

TEST_F(RtlDirectTest, FixedWingArrivalAcceptsTheOrbitWithoutReachingItsCentre)
{
	setVehicle(true, false);
	loiter_point_s approach{};
	approach.lat = _destination.lat;
	approach.lon = _destination.lon;
	approach.height_m = kDestinationAltitude + kDescentHeight;
	approach.loiter_radius_m = -kLoiterRadius;
	activate(approach);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_EQ(triplet().current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_TRUE(triplet().current.loiter_direction_counter_clockwise);
	EXPECT_FLOAT_EQ(triplet().current.loiter_radius, kLoiterRadius);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogMoveToApproach);

	double orbit_latitude;
	double orbit_longitude;
	waypoint_from_heading_and_distance(approach.lat, approach.lon, 0.f, kLoiterRadius,
					   &orbit_latitude, &orbit_longitude);
	ASSERT_GT(get_distance_to_next_waypoint(orbit_latitude, orbit_longitude, approach.lat, approach.lon),
		  _navigator->get_acceptance_radius());
	setPosition(orbit_latitude, orbit_longitude, kReturnAltitude);
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, approach.height_m);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogDescend);
}

TEST_F(RtlDirectTest, VtolKeepsLandingAndTransitionSequenceNumbers)
{
	setVehicle(true, true);
	activate();
	expectLoggedItem(kLogMoveToApproach);
	completePositionItem();
	expectLoggedItem(kLogDescend);
	completePositionItem();
	expectLoggedItem(kLogHold);
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(_rtl->missionItem().vtol_back_transition);
	EXPECT_TRUE(triplet().previous.valid);
	expectLoggedItem(kLogMoveToLand);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	expectLoggedItem(kLogTransition);
	setVehicle(false, true);
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FALSE(triplet().previous.valid);
	expectLoggedItem(kLogHoverToLand);
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LAND);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogLand);
}

TEST_F(RtlDirectTest, WeatherVaneLeavesArrivalAndLandingYawUnspecified)
{
	setParameter("WV_EN", int32_t{1});
	activate();
	EXPECT_FALSE(PX4_ISFINITE(_rtl->missionItem().yaw));
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LAND);
	EXPECT_FALSE(PX4_ISFINITE(_rtl->missionItem().yaw));
}

TEST_F(RtlDirectTest, ReactivationStartsArrivalAgainAfterAnIndefiniteHold)
{
	setParameter("RTL_LAND_DELAY", -1.f);
	activate();
	EXPECT_FALSE(_rtl->isLanding());
	completePositionItem();
	EXPECT_TRUE(_rtl->isLanding());
	completePositionItem();
	ASSERT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_UNLIMITED);
	expectLoggedItem(kLogHold);

	_rtl->run(false);
	EXPECT_FALSE(_rtl->isLanding());
	setPosition(kLatitude, kLongitude, kReturnAltitude);
	activate();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, kReturnAltitude);
	EXPECT_FLOAT_EQ(_rtl->missionItem().time_inside, 0.f);
	EXPECT_TRUE(_rtl->missionItem().autocontinue);
	EXPECT_FALSE(_rtl->isLanding());
	expectLoggedItem(kLogMoveToApproach);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogDescend);
}

TEST_F(RtlDirectTest, FixedWingEstimatePreservesSignedLoiterRadius)
{
	constexpr float cruise_speed = 20.f;
	setParameter("FW_AIRSPD_TRIM", cruise_speed);
	setParameter("FW_GND_SPD_MIN", 5.f);
	setVehicle(true, false);
	wind_s wind{};
	wind.timestamp = hrt_absolute_time();
	uORB::Publication<wind_s> wind_pub{ORB_ID(wind)};
	ASSERT_TRUE(wind_pub.publish(wind));
	loiter_point_s approach{};
	approach.lat = _destination.lat;
	approach.lon = _destination.lon;
	approach.height_m = kDestinationAltitude + kDescentHeight;
	approach.loiter_radius_m = kLoiterRadius;
	activate(approach);
	const auto clockwise = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(clockwise.valid);

	_rtl->run(false);
	approach.loiter_radius_m = -kLoiterRadius;
	activate(approach);
	const auto counterclockwise = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(counterclockwise.valid);

	// Existing direct estimates differ by one orbit diameter when the radius sign changes.
	EXPECT_NEAR(counterclockwise.time_estimate - clockwise.time_estimate,
		    2.f * kLoiterRadius / cruise_speed, 0.01f);
}

TEST_F(RtlDirectTest, ActiveHoldEstimateKeepsFullConfiguredDelay)
{
	constexpr float land_delay = 8.f;
	constexpr float descent_speed = 2.f;
	setParameter("RTL_LAND_DELAY", land_delay);
	setParameter("MPC_Z_V_AUTO_DN", descent_speed);
	activate();
	completePositionItem();
	completePositionItem();
	ASSERT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	expectLoggedItem(kLogHold);
	completePositionItem();
	const auto initial = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(initial.valid);
	EXPECT_NEAR(initial.time_estimate, land_delay + kDescentHeight / descent_speed, 0.01f);

	_rtl->ageWaypointReached(3.f);
	_rtl->run(true);
	const auto after_wait = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(after_wait.valid);
	EXPECT_FLOAT_EQ(after_wait.time_estimate, initial.time_estimate);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FALSE(_mission_item_sub.updated());
}

TEST_F(RtlDirectTest, RepeatedEstimatesLeaveArrivalExecutionAndLoggingUnchanged)
{
	setParameter("RTL_LAND_DELAY", 5.f);
	activate();
	expectLoggedItem(kLogMoveToApproach);
	const auto estimate_without_execution = [&]() {
		const mission_item_s item = _rtl->missionItem();
		const position_setpoint_triplet_s setpoints = triplet();
		const bool landing = _rtl->isLanding();
		const auto initial = _rtl->calc_rtl_time_estimate();
		ASSERT_TRUE(initial.valid);

		for (unsigned estimate = 0; estimate < 3u; ++estimate) {
			const auto repeated = _rtl->calc_rtl_time_estimate();
			ASSERT_TRUE(repeated.valid);
			EXPECT_FLOAT_EQ(repeated.time_estimate, initial.time_estimate);
			EXPECT_FLOAT_EQ(repeated.safe_time_estimate, initial.safe_time_estimate);
		}

		EXPECT_EQ(_rtl->missionItem().nav_cmd, item.nav_cmd);
		EXPECT_DOUBLE_EQ(_rtl->missionItem().lat, item.lat);
		EXPECT_DOUBLE_EQ(_rtl->missionItem().lon, item.lon);
		EXPECT_FLOAT_EQ(_rtl->missionItem().altitude, item.altitude);
		EXPECT_FLOAT_EQ(_rtl->missionItem().yaw, item.yaw);
		EXPECT_FLOAT_EQ(_rtl->missionItem().time_inside, item.time_inside);
		EXPECT_EQ(_rtl->isLanding(), landing);
		EXPECT_EQ(triplet().current.type, setpoints.current.type);
		EXPECT_DOUBLE_EQ(triplet().current.lat, setpoints.current.lat);
		EXPECT_DOUBLE_EQ(triplet().current.lon, setpoints.current.lon);
		EXPECT_FLOAT_EQ(triplet().current.alt, setpoints.current.alt);
		EXPECT_EQ(triplet().previous.valid, setpoints.previous.valid);
		EXPECT_EQ(triplet().next.valid, setpoints.next.valid);
		EXPECT_FALSE(_mission_item_sub.updated());
	};
	estimate_without_execution();
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	expectLoggedItem(kLogDescend);
	estimate_without_execution();
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	expectLoggedItem(kLogHold);
}
