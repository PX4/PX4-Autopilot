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
 * Heading acceptance of a rotary-wing waypoint: skipped when the control allocator reports yaw as not
 * controllable and in Return given up after RTL_YAW_TMT so a vehicle that cannot turn still lands.
 */

#include <gtest/gtest.h>

#include "mission_block.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace
{

constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr float kHeading = 0.1f; // rad, vehicle heading
constexpr float kItemYaw = kHeading + math::radians(90.f); // well outside MIS_YAW_ERR

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
		: MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {}

	mission_item_s &item() { return _mission_item; }
	bool reached() { return is_mission_item_reached_or_completed(); }
	bool yawReached() const { return _waypoint_yaw_reached; }
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
		_navigator.updateParams();

		vehicle_status_s *status = _navigator.get_vstatus();
		status->timestamp = hrt_absolute_time();
		status->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		vehicle_global_position_s *global = _navigator.get_global_position();
		global->timestamp = hrt_absolute_time();
		global->lat = kLat;
		global->lon = kLon;
		global->alt = kAlt;

		vehicle_local_position_s *local = _navigator.get_local_position();
		local->timestamp = hrt_absolute_time();
		local->heading = kHeading;
		local->heading_good_for_control = true;

		home_position_s *home = _navigator.get_home_position();
		home->timestamp = hrt_absolute_time();
		home->lat = kLat;
		home->lon = kLon;
		home->alt = kAlt;
		home->valid_hpos = true;
		home->valid_alt = true;

		_navigator.get_land_detected()->landed = false;

		// the vehicle sits exactly on a waypoint whose heading it has not reached
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

		setControllableAxes(vehicle_control_mode_s::CONTROLLABLE_AXIS_ALL);
	}

	void setControllableAxes(uint8_t axes)
	{
		vehicle_control_mode_s *control_mode = _navigator.get_control_mode();
		control_mode->timestamp = hrt_absolute_time();
		control_mode->controllable_axes = axes;
	}

	void setRtlYawTimeout(float seconds)
	{
		ASSERT_EQ(param_set(param_find("RTL_YAW_TMT"), &seconds), PX4_OK);
		_navigator.updateParams();
	}
};

TEST_F(MissionBlockYawTest, HeadingIsAwaitedWhenYawIsControllable)
{
	setRtlYawTimeout(-1.f);

	EXPECT_FALSE(_block.reached());
	EXPECT_FALSE(_block.yawReached());

	// turning to the commanded heading completes the item
	_navigator.get_local_position()->heading = kItemYaw;
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockYawTest, HeadingIsSkippedWhenYawIsNotControllable)
{
	setRtlYawTimeout(-1.f);
	setControllableAxes(vehicle_control_mode_s::CONTROLLABLE_AXIS_ALL & ~vehicle_control_mode_s::CONTROLLABLE_AXIS_YAW);

	EXPECT_TRUE(_block.reached());
	EXPECT_TRUE(_block.yawReached());
}

TEST_F(MissionBlockYawTest, HeadingIsAssumedControllableBeforeFirstReport)
{
	setRtlYawTimeout(-1.f);
	vehicle_control_mode_s *control_mode = _navigator.get_control_mode();
	control_mode->timestamp = 0;
	control_mode->controllable_axes = 0;

	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockYawTest, ReturnGivesUpHeadingAfterTimeout)
{
	setRtlYawTimeout(0.2f);

	EXPECT_FALSE(_block.reached()); // position reached now, heading not
	px4_usleep(100'000);
	EXPECT_FALSE(_block.reached()); // still inside the window
	px4_usleep(150'000);
	EXPECT_TRUE(_block.reached()); // window expired: continue without the heading
}

TEST_F(MissionBlockYawTest, TimeoutDoesNotApplyOutsideReturn)
{
	setRtlYawTimeout(0.2f);
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	EXPECT_FALSE(_block.reached());
	px4_usleep(300'000);
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockYawTest, TimeoutDisabledKeepsWaiting)
{
	setRtlYawTimeout(-1.f);

	EXPECT_FALSE(_block.reached());
	px4_usleep(300'000);
	EXPECT_FALSE(_block.reached());
}
