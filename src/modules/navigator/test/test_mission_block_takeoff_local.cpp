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
 * @file test_mission_block_takeoff_local.cpp
 *
 * NAV_CMD_TAKEOFF_LOCAL position-reached check and the post-takeoff loiter handoff, for a
 * multicopter taking off in the local frame (e.g. without a global position, as with optical
 * flow only).
 */

#include <gtest/gtest.h>

#include "mission_block.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>

namespace
{

// NED, so climbing above the takeoff position is negative. Deliberately smaller in magnitude
// than kAltAcceptanceRadius, so halving it (as the reached-check does) exercises the sign bug.
constexpr float kTakeoffHeight = -0.5f;
constexpr float kAltAcceptanceRadius = 1.f;

} // namespace

class MissionBlockTakeoffLocalTestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

class MissionBlockTakeoffLocalTestPeer : public MissionBlock
{
public:
	explicit MissionBlockTakeoffLocalTestPeer(Navigator *navigator)
		: MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF) {}

	mission_item_s &item() { return _mission_item; }
	bool reached() { return is_mission_item_reached_or_completed(); }
	void reset() { reset_mission_item_reached(); }

	void setLoiterFromCurrentSetpoint(mission_item_s &loiter_item, const position_setpoint_s &reference_setpoint)
	{
		setLoiterItemFromCurrentPositionSetpoint(loiter_item, reference_setpoint);
	}
};

class MissionBlockTakeoffLocalTest : public NavigatorDatamanTestBase
{
protected:
	MissionBlockTakeoffLocalTestNavigator _navigator{};
	MissionBlockTakeoffLocalTestPeer _block{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		setAltAcceptanceRadius(kAltAcceptanceRadius);

		vehicle_status_s *status = _navigator.get_vstatus();
		status->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status->is_vtol = false;

		_navigator.get_land_detected()->landed = false;

		mission_item_s &item = _block.item();
		item = {};
		item.nav_cmd = NAV_CMD_TAKEOFF_LOCAL;
		item.altitude = kTakeoffHeight;
		item.yaw = NAN; // set_local_takeoff_item() leaves the heading to the flight task
		_block.reset();

		setLocalZ(0.f); // still on the ground
	}

	void setLocalZ(float z)
	{
		vehicle_local_position_s *local = _navigator.get_local_position();
		local->timestamp = hrt_absolute_time();
		local->z = z;
	}

	void setAltAcceptanceRadius(float radius_m)
	{
		ASSERT_EQ(param_set(param_find("NAV_MC_ALT_RAD"), &radius_m), PX4_OK);
		_navigator.updateParams();
	}
};

TEST_F(MissionBlockTakeoffLocalTest, NotReachedWhileBelowTarget)
{
	setLocalZ(0.f); // on the ground, well below the target height
	EXPECT_FALSE(_block.reached());
}

TEST_F(MissionBlockTakeoffLocalTest, ReachedAtTargetHeightDespiteSmallAcceptanceRadius)
{
	// Regression for ddc148fa97: halving a negative (NED, upward) takeoff height left a negative
	// acceptance radius, so fabsf(z - takeoff_height) < negative_radius could never become true,
	// however close the vehicle actually got.
	setLocalZ(kTakeoffHeight); // exactly at the commanded height
	EXPECT_TRUE(_block.reached());
}

TEST_F(MissionBlockTakeoffLocalTest, LoiterHandoffKeepsLocalXY)
{
	position_setpoint_s &current = _navigator.get_position_setpoint_triplet()->current;
	current = {};
	current.valid = true;
	current.frame = position_setpoint_s::FRAME_TYPE_LOCAL;
	current.x = 12.5f;
	current.y = -3.25f;
	current.alt = kTakeoffHeight;

	// Regression for ddc148fa97: this used to always copy lat/lon into the loiter item, which
	// for a local-frame setpoint are stale/zero, instead of the real local x/y.
	mission_item_s loiter_item{};
	_block.setLoiterFromCurrentSetpoint(loiter_item, current);

	EXPECT_DOUBLE_EQ(loiter_item.x, 12.5);
	EXPECT_DOUBLE_EQ(loiter_item.y, -3.25);
}
