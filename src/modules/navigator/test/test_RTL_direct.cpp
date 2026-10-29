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

#include <cfloat>
#include <memory>

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
	static constexpr float kApproachOffset = 300.f;
	static constexpr float kFixedWingSpeed = 15.f;

	// Direct RTL states as logged in navigator_mission_item.sequence_current.
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
		param_reset_all();
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
		param_reset_all();
	}

	template<typename T>
	void setParameter(const char *name, T value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
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

	// A VTOL landing approach east of the destination, farther than the acceptance radius.
	loiter_point_s offsetApproach() const
	{
		loiter_point_s approach{};
		waypoint_from_heading_and_distance(_destination.lat, _destination.lon, M_PI_2_F, kApproachOffset,
						   &approach.lat, &approach.lon);
		approach.height_m = kDestinationAltitude + kDescentHeight;
		approach.loiter_radius_m = kLoiterRadius;
		return approach;
	}

	// Seen from the offset approach, the destination lies due west.
	void flyTowardsDestination() { _navigator->get_local_position()->vy = -kFixedWingSpeed; }
	void flyAwayFromDestination() { _navigator->get_local_position()->vy = kFixedWingSpeed; }

	void expectLoggedItem(uint16_t sequence)
	{
		navigator_mission_item_s logged_item{};
		ASSERT_TRUE(_mission_item_sub.update(&logged_item));
		EXPECT_EQ(logged_item.sequence_current, sequence);
		EXPECT_EQ(logged_item.nav_cmd, _rtl->missionItem().nav_cmd);
		EXPECT_FLOAT_EQ(logged_item.altitude, _rtl->missionItem().altitude);
		EXPECT_EQ(logged_item.force_heading, static_cast<bool>(_rtl->missionItem().force_heading));
	}

	const position_setpoint_triplet_s &triplet() const { return *_navigator->get_position_setpoint_triplet(); }

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
	// The move follows the line from the climb position.
	EXPECT_TRUE(triplet().previous.valid);
	EXPECT_DOUBLE_EQ(triplet().previous.lat, kLatitude);
	EXPECT_DOUBLE_EQ(triplet().previous.lon, kLongitude);
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

TEST_F(RtlDirectTest, MulticopterMoveBeyondMinimumDistanceLeavesYawUnspecified)
{
	setParameter("RTL_MIN_DIST", 100.f);
	activate();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FALSE(PX4_ISFINITE(_rtl->missionItem().yaw));
	expectLoggedItem(kLogMoveToApproach);
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
	EXPECT_FALSE(_rtl->missionItem().force_heading); // the approach is centred on the destination
	EXPECT_FALSE(triplet().previous.valid);
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogDescend);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FLOAT_EQ(_rtl->missionItem().time_inside, kLandDelay);
	EXPECT_TRUE(_rtl->missionItem().autocontinue);
	EXPECT_FALSE(_rtl->missionItem().force_heading);
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
	EXPECT_TRUE(_rtl->isLanding());
	expectLoggedItem(kLogMoveToLand);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	EXPECT_TRUE(_rtl->isLanding());
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

TEST_F(RtlDirectTest, OffsetApproachLeavesItsLoitersOnlyTowardsTheLandingPoint)
{
	setVehicle(true, true);
	const loiter_point_s approach = offsetApproach();
	activate(approach);
	EXPECT_FALSE(_rtl->missionItem().force_heading);
	expectLoggedItem(kLogMoveToApproach);

	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_TRUE(_rtl->missionItem().force_heading);
	EXPECT_EQ(triplet().next.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_DOUBLE_EQ(triplet().next.lat, _destination.lat);
	EXPECT_DOUBLE_EQ(triplet().next.lon, _destination.lon);
	expectLoggedItem(kLogDescend);

	// The descent ends only on the course to the landing point.
	flyAwayFromDestination();
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_FALSE(_mission_item_sub.updated());

	flyTowardsDestination();
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_TRUE(_rtl->missionItem().force_heading);
	expectLoggedItem(kLogHold);

	// So does the hold.
	flyAwayFromDestination();
	completePositionItem();
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FALSE(_mission_item_sub.updated());

	flyTowardsDestination();
	_rtl->run(true);
	EXPECT_EQ(_rtl->missionItem().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(_rtl->missionItem().vtol_back_transition);
	EXPECT_FALSE(_rtl->missionItem().force_heading);
	EXPECT_TRUE(triplet().previous.valid);
	EXPECT_DOUBLE_EQ(triplet().previous.lat, approach.lat);
	EXPECT_DOUBLE_EQ(triplet().previous.lon, approach.lon);
	EXPECT_DOUBLE_EQ(triplet().current.lat, _destination.lat);
	expectLoggedItem(kLogMoveToLand);
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

TEST_F(RtlDirectTest, InactiveEstimateAddsClimbMoveDescentDelayAndLanding)
{
	static constexpr float kCruiseSpeed = 10.f;
	static constexpr float kClimbSpeed = 4.f;
	static constexpr float kDescentSpeed = 2.f;
	static constexpr float kLandDelay = 4.f;
	setParameter("MPC_XY_CRUISE", kCruiseSpeed);
	setParameter("MPC_Z_V_AUTO_UP", kClimbSpeed);
	setParameter("MPC_Z_V_AUTO_DN", kDescentSpeed);
	setParameter("RTL_LAND_DELAY", kLandDelay);
	setPosition(kLatitude, kLongitude, kReturnAltitude - 20.f);
	_rtl->setRtlAlt(kReturnAltitude);
	_rtl->setRtlPosition(_destination, {});
	_rtl->run(false);

	const float distance = get_distance_to_next_waypoint(kLatitude, kLongitude, _destination.lat, _destination.lon);
	const float approach_altitude = kDestinationAltitude + kDescentHeight;
	const rtl_time_estimate_s estimate = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, 20.f / kClimbSpeed + distance / kCruiseSpeed
		    + (kReturnAltitude - approach_altitude) / kDescentSpeed + kLandDelay + kDescentHeight / kDescentSpeed, 0.01f);
}

TEST_F(RtlDirectTest, InactiveEstimateAfterAnEndedReturnStartsWithTheMove)
{
	static constexpr float kCruiseSpeed = 10.f;
	static constexpr float kDescentSpeed = 2.f;
	static constexpr float kLandDelay = 4.f;
	setParameter("MPC_XY_CRUISE", kCruiseSpeed);
	setParameter("MPC_Z_V_AUTO_DN", kDescentSpeed);
	setParameter("RTL_LAND_DELAY", kLandDelay);

	// A first Return that reaches the hold and ends there.
	activate();
	completePositionItem();
	expectLoggedItem(kLogDescend);
	completePositionItem();
	expectLoggedItem(kLogHold);
	_rtl->run(false);
	ASSERT_FALSE(_rtl->isActive());

	// Back at the return altitude, away from the destination: activation would start with the move.
	setPosition(kLatitude, kLongitude, kReturnAltitude);
	_rtl->setRtlAlt(kReturnAltitude);
	_rtl->setRtlPosition(_destination, {});
	_rtl->run(false);
	const float distance = get_distance_to_next_waypoint(kLatitude, kLongitude, _destination.lat, _destination.lon);
	const float approach_altitude = kDestinationAltitude + kDescentHeight;
	const rtl_time_estimate_s estimate = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, distance / kCruiseSpeed
		    + (kReturnAltitude - approach_altitude) / kDescentSpeed + kLandDelay + kDescentHeight / kDescentSpeed, 0.01f);
}

TEST_F(RtlDirectTest, DescentEstimateStartsAtTheCurrentAltitude)
{
	static constexpr float kCruiseSpeed = 10.f;
	static constexpr float kDescentSpeed = 2.f;
	static constexpr float kLandDelay = 4.f;
	setParameter("MPC_XY_CRUISE", kCruiseSpeed);
	setParameter("MPC_Z_V_AUTO_DN", kDescentSpeed);
	setParameter("RTL_LAND_DELAY", kLandDelay);
	activate();
	completePositionItem();
	expectLoggedItem(kLogDescend);

	// Partway down, a few metres off the centre: no horizontal leg, only the remaining descent.
	double latitude;
	double longitude;
	waypoint_from_heading_and_distance(_destination.lat, _destination.lon, 0.f, 5.f, &latitude, &longitude);
	setPosition(latitude, longitude, kDestinationAltitude + kDescentHeight + 40.f);
	const rtl_time_estimate_s estimate = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, 40.f / kDescentSpeed + kLandDelay + kDescentHeight / kDescentSpeed, 0.01f);
}

TEST_F(RtlDirectTest, HoldEstimateAddsTheDelayAndTheLandingOnly)
{
	static constexpr float kDescentSpeed = 1.5f; // differs from the fixed-wing sink rate
	setParameter("FW_AIRSPD_TRIM", kFixedWingSpeed);
	setParameter("MPC_Z_V_AUTO_DN", kDescentSpeed);
	setVehicle(true, true);
	const loiter_point_s approach = offsetApproach();
	const float approach_distance = get_distance_to_next_waypoint(approach.lat, approach.lon,
					_destination.lat, _destination.lon);
	flyTowardsDestination();

	for (const float land_delay : {5.f, -1.f}) {
		SCOPED_TRACE(land_delay);
		setParameter("RTL_LAND_DELAY", land_delay);
		setPosition(kLatitude, kLongitude, kReturnAltitude);
		activate(approach);
		completePositionItem();
		completePositionItem();
		expectLoggedItem(kLogHold);

		// Outside the orbit and above the hold altitude: neither adds time.
		double latitude;
		double longitude;
		waypoint_from_heading_and_distance(approach.lat, approach.lon, 0.f, 2.f * kLoiterRadius,
						   &latitude, &longitude);
		setPosition(latitude, longitude, approach.height_m + 3.f);
		const rtl_time_estimate_s estimate = _rtl->calc_rtl_time_estimate();
		ASSERT_TRUE(estimate.valid);

		if (land_delay > 0.f) {
			// The delay, the leg to the landing point and the hover descent.
			EXPECT_NEAR(estimate.time_estimate, land_delay + approach_distance / kFixedWingSpeed
				    + kDescentHeight / kDescentSpeed, 0.01f);

		} else {
			// An indefinite hold ends the estimate.
			EXPECT_FLOAT_EQ(estimate.time_estimate, 0.f);
		}

		_rtl->run(false);
	}
}

TEST_F(RtlDirectTest, FixedWingMoveEstimateEndsTheLegAtTheOrbit)
{
	static constexpr float kSinkRate = 3.f;
	setParameter("FW_AIRSPD_TRIM", kFixedWingSpeed);
	setParameter("FW_T_SINK_R_SP", kSinkRate);
	setVehicle(true, false);
	activate();
	expectLoggedItem(kLogMoveToApproach);

	// Partway to the destination: the leg ends at the orbit, then both descents use the sink rate.
	// The descent is costed from the return altitude, so the 20 m above it are ignored.
	double latitude;
	double longitude;
	waypoint_from_heading_and_distance(_destination.lat, _destination.lon, M_PI_F, 150.f, &latitude, &longitude);
	setPosition(latitude, longitude, kReturnAltitude + 20.f);
	const float distance = get_distance_to_next_waypoint(latitude, longitude, _destination.lat, _destination.lon);
	const rtl_time_estimate_s estimate = _rtl->calc_rtl_time_estimate();
	ASSERT_TRUE(estimate.valid);
	EXPECT_NEAR(estimate.time_estimate, (distance - kLoiterRadius) / kFixedWingSpeed
		    + (kReturnAltitude - kDestinationAltitude) / kSinkRate, 0.01f);
}
