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

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>

#include "navigator.h"
#include "loiter.h"
#include "support/navigator_dataman_test.h"

namespace
{
constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kAlt = 500.f;
}

class LoiterTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};
	Loiter _loiter{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		_navigator.get_vstatus()->arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_navigator.get_land_detected()->landed = false;

		vehicle_global_position_s &global_pos = *_navigator.get_global_position();
		global_pos.lat = kLat;
		global_pos.lon = kLon;
		global_pos.alt = kAlt;

		vehicle_local_position_s &local_pos = *_navigator.get_local_position();
		local_pos.ref_timestamp = 1;
		local_pos.ref_alt = 400.f;
		local_pos.xy_reset_counter = 3;
		local_pos.z_reset_counter = 3;
	}

	void TearDown() override
	{
		param_control_autosave(true);
	}

	// The estimate resets by delta_north/delta_east/delta_down [m]
	void resetEstimate(float delta_north, float delta_east, float delta_down)
	{
		vehicle_local_position_s &local_pos = *_navigator.get_local_position();
		local_pos.delta_xy[0] = delta_north;
		local_pos.delta_xy[1] = delta_east;
		local_pos.xy_reset_counter++;
		local_pos.delta_z = delta_down;
		local_pos.z_reset_counter++;
	}

	const position_setpoint_s &currentSetpoint() { return _navigator.get_position_setpoint_triplet()->current; }
};

TEST_F(LoiterTest, holdFollowsEstimateReset)
{
	// GIVEN: a vehicle holding where it is
	_loiter.on_activation();
	_loiter.on_active();

	const position_setpoint_s hold = currentSetpoint();
	ASSERT_TRUE(hold.valid);

	// WHEN: the position estimate resets 1.5 m north and 0.8 m up, as on a switch of the GNSS receiver
	resetEstimate(1.5f, 0.f, -0.8f);
	_loiter.on_active();

	// THEN: the hold position moves with it, so that the vehicle stays where it is
	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(hold.lat, hold.lon, currentSetpoint().lat, currentSetpoint().lon, &north, &east);
	EXPECT_NEAR(north, 1.5f, 0.01f);
	EXPECT_NEAR(east, 0.f, 0.01f);
	EXPECT_NEAR(currentSetpoint().alt, hold.alt + 0.8f, 0.001f);
}

TEST_F(LoiterTest, holdIgnoresMovedOrigin)
{
	// GIVEN: a vehicle holding where it is
	_loiter.on_activation();
	_loiter.on_active();

	const position_setpoint_s hold = currentSetpoint();

	// WHEN: the origin of the local position moves, which resets the local but not the global position
	resetEstimate(20.f, 10.f, 5.f);
	_navigator.get_local_position()->ref_timestamp = 2;
	_navigator.get_local_position()->ref_alt = 405.f;
	_loiter.on_active();

	// THEN: the hold position stays
	EXPECT_DOUBLE_EQ(currentSetpoint().lat, hold.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, hold.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, hold.alt);
}

TEST_F(LoiterTest, repositionTargetIgnoresEstimateReset)
{
	// GIVEN: a vehicle holding, then sent to a reposition target
	_loiter.on_activation();
	_loiter.on_active();

	position_setpoint_triplet_s *reposition = _navigator.get_reposition_triplet();
	reposition->current.valid = true;
	reposition->current.timestamp = hrt_absolute_time();
	reposition->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	reposition->current.lat = kLat + 0.001;
	reposition->current.lon = kLon;
	reposition->current.alt = kAlt + 10.f;
	_loiter.on_active();

	const position_setpoint_s target = currentSetpoint();
	ASSERT_DOUBLE_EQ(target.lat, kLat + 0.001);

	// WHEN: the position estimate resets
	resetEstimate(1.5f, 0.f, -0.8f);
	_loiter.on_active();

	// THEN: the target stays where it was commanded
	EXPECT_DOUBLE_EQ(currentSetpoint().lat, target.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, target.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, target.alt);
}
