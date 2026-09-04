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
 * @file test_prec_takeoff.cpp
 * Tests for the precision takeoff helper.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "prec_takeoff.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/prec_takeoff_status.h>
#include <uORB/uORBManager.hpp>

using namespace time_literals;

class PrecTakeoffTestPeer : public PrecTakeoff
{
public:
	PrecTakeoffTestPeer() : PrecTakeoff(nullptr) {}
	using PrecTakeoff::updateParams;
};

class PrecTakeoffTest : public ::testing::Test
{
protected:
	static constexpr double kRefLat = 47.397742;
	static constexpr double kRefLon = 8.545594;
	static constexpr hrt_abstime kNow = 100_s;

	static void SetUpTestSuite() { uORB::Manager::initialize(); }

	void SetUp() override
	{
		param_control_autosave(false);
		_param = param_find("MIS_TKO_PREC");
		ASSERT_NE(_param, PARAM_INVALID);
		ASSERT_EQ(param_get(_param, &_param_backup), 0);
		setEnabled(true);

		_local_pos = {};
		_local_pos.xy_global = true;
		_local_pos.ref_lat = kRefLat;
		_local_pos.ref_lon = kRefLon;
		_local_pos.ref_timestamp = 1;

		_sp = {};
		_sp.valid = true;
		_sp.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		_sp.lat = kRefLat;
		_sp.lon = kRefLon;
		_sp.alt = 500.f;

		// Drop any status left over from a previous test
		prec_takeoff_status_s status;

		while (_status_sub.update(&status)) {}
	}

	void TearDown() override
	{
		param_set_no_notification(_param, &_param_backup);
		param_control_autosave(true);
	}

	void setEnabled(bool enabled)
	{
		int32_t value = enabled ? 1 : 0;
		ASSERT_EQ(param_set_no_notification(_param, &value), 0);
		_prec_takeoff.updateParams();
	}

	void publishTarget(float x_abs, float y_abs, bool abs_pos_valid, hrt_abstime timestamp)
	{
		landing_target_pose_s msg{};
		msg.timestamp = timestamp;
		msg.abs_pos_valid = abs_pos_valid;
		msg.x_abs = x_abs;
		msg.y_abs = y_abs;
		ASSERT_TRUE(_target_pub.publish(msg));
	}

	bool readStatus(uint8_t &state)
	{
		prec_takeoff_status_s status;

		if (_status_sub.update(&status)) {
			state = status.state;
			return true;
		}

		return false;
	}

	matrix::Vector2f setpointLocal() const
	{
		MapProjection ref{kRefLat, kRefLon};
		return ref.project(_sp.lat, _sp.lon);
	}

	PrecTakeoffTestPeer _prec_takeoff{};
	uORB::Publication<landing_target_pose_s> _target_pub{ORB_ID(landing_target_pose)};
	uORB::Subscription _status_sub{ORB_ID(prec_takeoff_status)};
	vehicle_local_position_s _local_pos{};
	position_setpoint_s _sp{};
	param_t _param{PARAM_INVALID};
	int32_t _param_backup{0};
};

TEST_F(PrecTakeoffTest, FreshTargetMovesSetpointOntoTarget)
{
	// GIVEN: A fresh target 3 m north and 4 m east of the local origin
	publishTarget(3.f, 4.f, true, kNow - 100_ms);

	// WHEN: The helper runs during the climb
	EXPECT_TRUE(_prec_takeoff.run(_local_pos, _sp, false, kNow));

	// THEN: The setpoint sits on the target, type and altitude are untouched
	const matrix::Vector2f sp_local = setpointLocal();
	EXPECT_NEAR(sp_local(0), 3.f, 0.01f);
	EXPECT_NEAR(sp_local(1), 4.f, 0.01f);
	EXPECT_EQ(_sp.type, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	EXPECT_FLOAT_EQ(_sp.alt, 500.f);
}

TEST_F(PrecTakeoffTest, StaleOrInvalidTargetKeepsSetpoint)
{
	// GIVEN: A stale target
	publishTarget(3.f, 4.f, true, kNow - 2_s);
	EXPECT_FALSE(_prec_takeoff.run(_local_pos, _sp, false, kNow));

	// GIVEN: A fresh target without absolute position
	publishTarget(3.f, 4.f, false, kNow - 100_ms);
	EXPECT_FALSE(_prec_takeoff.run(_local_pos, _sp, false, kNow));

	// GIVEN: A fresh target with a NAN position
	publishTarget(NAN, 4.f, true, kNow - 100_ms);
	EXPECT_FALSE(_prec_takeoff.run(_local_pos, _sp, false, kNow));

	// GIVEN: No global reference
	_local_pos.xy_global = false;
	publishTarget(3.f, 4.f, true, kNow - 100_ms);
	EXPECT_FALSE(_prec_takeoff.run(_local_pos, _sp, false, kNow));

	// THEN: The setpoint never moved
	EXPECT_DOUBLE_EQ(_sp.lat, kRefLat);
	EXPECT_DOUBLE_EQ(_sp.lon, kRefLon);
}

TEST_F(PrecTakeoffTest, ReachedTakeoffKeepsSetpoint)
{
	// GIVEN: A fresh target while the takeoff altitude is already reached
	publishTarget(3.f, 4.f, true, kNow - 100_ms);

	// THEN: The setpoint is left alone
	EXPECT_FALSE(_prec_takeoff.run(_local_pos, _sp, true, kNow));
	EXPECT_DOUBLE_EQ(_sp.lat, kRefLat);
	EXPECT_DOUBLE_EQ(_sp.lon, kRefLon);
}

TEST_F(PrecTakeoffTest, StatusFollowsTakeoffLifecycle)
{
	uint8_t state{};

	// GIVEN: No takeoff active, nothing to report
	_prec_takeoff.publish_status();
	EXPECT_FALSE(readStatus(state));

	// WHEN: A takeoff is active
	_prec_takeoff.run(_local_pos, _sp, false, kNow);
	_prec_takeoff.publish_status();

	// THEN: ONGOING is published once
	ASSERT_TRUE(readStatus(state));
	EXPECT_EQ(state, prec_takeoff_status_s::PREC_TAKEOFF_STATE_ONGOING);
	_prec_takeoff.run(_local_pos, _sp, false, kNow);
	_prec_takeoff.publish_status();
	EXPECT_FALSE(readStatus(state));

	// WHEN: The takeoff altitude is reached
	_prec_takeoff.run(_local_pos, _sp, true, kNow);
	_prec_takeoff.publish_status();

	// THEN: DONE is published
	ASSERT_TRUE(readStatus(state));
	EXPECT_EQ(state, prec_takeoff_status_s::PREC_TAKEOFF_STATE_DONE);

	// WHEN: The takeoff item is no longer active
	_prec_takeoff.publish_status();

	// THEN: STOPPED is published
	ASSERT_TRUE(readStatus(state));
	EXPECT_EQ(state, prec_takeoff_status_s::PREC_TAKEOFF_STATE_STOPPED);
}

TEST_F(PrecTakeoffTest, ParamEnablesHelper)
{
	EXPECT_TRUE(_prec_takeoff.enabled());
	setEnabled(false);
	EXPECT_FALSE(_prec_takeoff.enabled());
}
