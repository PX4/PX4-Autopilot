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
 * @file test_mission_block_yaw.cpp
 *
 * Heading acceptance of a rotary-wing waypoint: awaited for at most MIS_YAW_TMT counted from when the
 * heading got commanded so a vehicle that cannot turn (e.g. lost yaw authority) does not get stuck.
 */

#include <gtest/gtest.h>

#include "mission_block.h"
#include "navigator.h"
#include "vtol_takeoff.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>

namespace
{

constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr float kHeading = 0.1f; // rad, vehicle heading
constexpr float kItemYaw = kHeading + math::radians(90.f); // well outside MIS_YAW_ERR
constexpr float kTimeout = 0.2f; // s, short MIS_YAW_TMT to keep the tests fast

float paramFloat(const char *name)
{
	float value = NAN;
	param_get(param_find(name), &value);
	return value;
}

} // namespace

class MissionBlockYawTestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

class MissionBlockYawTestPeer : public MissionBlock
{
public:
	explicit MissionBlockYawTestPeer(Navigator *navigator)
		: MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {}

	mission_item_s &item() { return _mission_item; }
	bool reached() { return is_mission_item_reached_or_completed(); }
	bool yawReached() const { return _waypoint_yaw_reached; }
	void reset() { reset_mission_item_reached(); }
};

class MissionBlockYawTest : public NavigatorDatamanTestBase
{
protected:
	MissionBlockYawTestNavigator _navigator{};
	MissionBlockYawTestPeer _block{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		setYawTimeout(kTimeout);

		vehicle_status_s *status = _navigator.get_vstatus();
		status->timestamp = hrt_absolute_time();
		status->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status->is_vtol = false;
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		vehicle_local_position_s *local = _navigator.get_local_position();
		local->timestamp = hrt_absolute_time();
		local->heading = kHeading;
		local->heading_good_for_control = true;

		_navigator.get_land_detected()->landed = false;
		_navigator.get_mission_result()->failure = false;

		// a waypoint whose heading the vehicle has not reached
		mission_item_s &item = _block.item();
		item = {};
		item.nav_cmd = NAV_CMD_WAYPOINT;
		item.lat = kLat;
		item.lon = kLon;
		item.altitude = kAlt;
		item.altitude_is_relative = false;
		item.yaw = kItemYaw;
		item.acceptance_radius = NAN;
		item.time_inside = 0.f;
		_block.reset();

		setVehicleOnWaypoint(true);
	}

	void setVehicleOnWaypoint(bool on_waypoint)
	{
		vehicle_global_position_s *global = _navigator.get_global_position();
		global->timestamp = hrt_absolute_time();
		global->lat = kLat;
		global->lon = kLon;
		global->alt = on_waypoint ? kAlt : kAlt + 50.f; // e.g. still descending towards it
	}

	void setYawTimeout(float seconds)
	{
		ASSERT_EQ(param_set(param_find("MIS_YAW_TMT"), &seconds), PX4_OK);
		_navigator.updateParams();
	}

	void sleepPastTimeout() { px4_usleep((useconds_t)(kTimeout * 1.25e6f)); }

	// VTOL Takeoff hovering at the transition altitude, aligning to a direction 90 deg off its heading
	void startAligningVtolTakeoff(VtolTakeoff &takeoff)
	{
		vehicle_status_s *status = _navigator.get_vstatus();
		status->is_vtol = true;
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF;

		transitionCommanded(); // drop commands of earlier tests
		takeoff.setTransitionAltitudeAbsolute(kAlt);
		takeoff.setTransitionDirection(math::degrees(kItemYaw));
		takeoff.on_activation();
		takeoff.on_active();
		ASSERT_NEAR(_navigator.get_position_setpoint_triplet()->current.yaw, kItemYaw, 1e-4f);
		takeoff.on_active();
	}

	bool transitionCommanded()
	{
		bool commanded = false;
		vehicle_command_s vehicle_command{};

		while (_vehicle_command_sub.update(&vehicle_command)) {
			commanded |= vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		}

		return commanded;
	}

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
};

TEST_F(MissionBlockYawTest, HeadingIsAcceptedWhenReached)
{
	EXPECT_FALSE(_block.reached()); // on the waypoint, heading not reached

	_navigator.get_local_position()->heading = kItemYaw;
	EXPECT_TRUE(_block.reached());
	EXPECT_FALSE(_navigator.get_mission_result()->failure);
}

TEST_F(MissionBlockYawTest, ContinuesWithoutHeadingAfterTimeout)
{
	EXPECT_FALSE(_block.reached());
	px4_usleep((useconds_t)(kTimeout * 0.5e6f));
	EXPECT_FALSE(_block.reached()); // still inside the timeout
	px4_usleep((useconds_t)(kTimeout * 0.75e6f));
	EXPECT_TRUE(_block.reached()); // e.g. lost yaw authority: continue without the heading
	EXPECT_TRUE(_block.yawReached());
	EXPECT_FALSE(_navigator.get_mission_result()->failure);
}

TEST_F(MissionBlockYawTest, TimeoutIsCountedFromWhenHeadingIsCommanded)
{
	// the turn happens on the way, e.g. during the descent, which uses up the timeout
	setVehicleOnWaypoint(false);
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	EXPECT_FALSE(_block.reached()); // position not reached yet

	setVehicleOnWaypoint(true);
	EXPECT_TRUE(_block.reached()); // no extra wait once there
}

TEST_F(MissionBlockYawTest, TimeoutAppliesInMissionToo)
{
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	EXPECT_TRUE(_block.reached());
	EXPECT_FALSE(_navigator.get_mission_result()->failure);
}

TEST_F(MissionBlockYawTest, ForcedHeadingTimeoutFailsMission)
{
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	_block.item().force_heading = true;
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	EXPECT_FALSE(_block.reached()); // the mission relies on the heading, the failsafe takes over
	EXPECT_TRUE(_navigator.get_mission_result()->failure);
}

TEST_F(MissionBlockYawTest, ForcedHeadingTimeoutKeepsWaitingOutsideMission)
{
	_block.item().force_heading = true;
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	EXPECT_FALSE(_block.reached()); // what follows relies on the heading in every mode
	EXPECT_FALSE(_navigator.get_mission_result()->failure); // only Mission has a failsafe for it
}

TEST_F(MissionBlockYawTest, VtolIgnoresHeadingThatIsNotForced)
{
	_navigator.get_vstatus()->is_vtol = true;
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockYawTest, VtolAwaitsForcedHeading)
{
	_navigator.get_vstatus()->is_vtol = true;
	_block.item().force_heading = true;
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockYawTest, VtolTakeoffTransitionsOnceAligned)
{
	VtolTakeoff takeoff{&_navigator};
	ASSERT_NO_FATAL_FAILURE(startAligningVtolTakeoff(takeoff));
	EXPECT_FALSE(transitionCommanded());

	_navigator.get_local_position()->heading = kItemYaw;
	takeoff.on_active();
	EXPECT_TRUE(transitionCommanded());
}

TEST_F(MissionBlockYawTest, VtolTakeoffDoesNotTransitionMisaligned)
{
	VtolTakeoff takeoff{&_navigator};
	ASSERT_NO_FATAL_FAILURE(startAligningVtolTakeoff(takeoff));
	sleepPastTimeout();
	takeoff.on_active();
	EXPECT_FALSE(transitionCommanded()); // it would start off the requested direction
	EXPECT_FALSE(_navigator.get_mission_result()->failure); // no stale failure for a later mission

	_navigator.get_local_position()->heading = kItemYaw;
	takeoff.on_active();
	EXPECT_TRUE(transitionCommanded()); // in the requested direction once aligned
}

TEST_F(MissionBlockYawTest, DisabledTimeoutWaitsIndefinitely)
{
	setYawTimeout(-1.f);
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockYawTest, ZeroTimeoutDoesNotWait)
{
	setYawTimeout(0.f);
	_block.reached(); // first check is when the heading got commanded
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockYawTest, ResetRestartsTimeout)
{
	EXPECT_FALSE(_block.reached());
	sleepPastTimeout();
	_block.reset(); // e.g. heading alignment added to an already reached item
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockYawTest, DefaultTimeoutCoversHalfTurnAtDefaultYawLimits)
{
	param_reset_all();
	const float rate = paramFloat("MPC_YAWRAUTO_MAX"); // deg/s
	const float acc = paramFloat("MPC_YAWRAUTO_ACC"); // deg/s^2
	ASSERT_TRUE(PX4_ISFINITE(rate) && PX4_ISFINITE(acc));

	// trapezoidal profile: accelerate to the rate, cruise, decelerate
	const float half_turn_time = 180.f / rate + rate / acc;
	EXPECT_GT(paramFloat("MIS_YAW_TMT"), half_turn_time);
}
