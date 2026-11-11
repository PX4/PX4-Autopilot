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

#include "support/geofence_test_helpers.h"

#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>

// The geofence margin (GF_MARGIN) in the breach check that Navigator runs in flight. Home is at the origin.
class GeofenceMarginTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kMargin = 100.f;
	static constexpr float kAltitude = 450.f; // 50 m above Home
	static constexpr useconds_t kCheckInterval = 210000; // the breach check runs at 5 Hz

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence());

		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		*_navigator.get_vstatus() = status;

		// A check without any limit clears what an earlier test left in Navigator.
		checkAt(0.f, 0.f);
		_navigator.get_reposition_triplet()->current = position_setpoint_s{};
	}

	void TearDown() override { param_reset_all(); }

	void setParam(const char *name, float value)
	{
		ASSERT_EQ(param_set(param_find(name), &value), PX4_OK);
		_navigator.updateParams();
	}

	void setParam(const char *name, int32_t value)
	{
		ASSERT_EQ(param_set(param_find(name), &value), PX4_OK);
		_navigator.updateParams();
	}

	// A 1000 m square inclusion fence centred on Home.
	FencePoints inclusionSquare() const
	{
		return polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}});
	}

	// Place the vehicle and run one breach check.
	geofence_result_s checkAt(float north, float east, float altitude = kAltitude)
	{
		px4_usleep(kCheckInterval);
		const matrix::Vector2d vehicle = position(north, east);
		vehicle_global_position_s &global_position = *_navigator.get_global_position();
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = vehicle(0);
		global_position.lon = vehicle(1);
		global_position.alt = altitude;
		_navigator.geofence_breach_check();

		geofence_result_s result{};
		EXPECT_TRUE(_result_sub.update(&result));
		return result;
	}

	static bool breached(const geofence_result_s &result)
	{
		return result.geofence_max_dist_triggered || result.geofence_max_alt_triggered
		       || result.geofence_custom_fence_triggered;
	}

	const position_setpoint_s &reposition() { return _navigator.get_reposition_triplet()->current; }

	uORB::Subscription _result_sub{ORB_ID(geofence_result)};
};

TEST_F(GeofenceMarginTest, DisabledMarginNeverTriggers)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_FALSE(checkAt(0.f, 490.f).geofence_margin_triggered);
	// Outside the fence only the breach is reported.
	const geofence_result_s outside = checkAt(0.f, 510.f);
	EXPECT_TRUE(outside.geofence_custom_fence_triggered);
	EXPECT_FALSE(outside.geofence_margin_triggered);
}

// Without a margin the Hold after a breach works as before: at the breach position, latched while holding,
// and again on the next breach.
TEST_F(GeofenceMarginTest, BreachHoldWithoutAMarginIsUnchanged)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));

	EXPECT_FALSE(breached(checkAt(0.f, 490.f)));
	EXPECT_FALSE(reposition().valid);

	const geofence_result_s outside = checkAt(0.f, 510.f);
	ASSERT_TRUE(breached(outside));
	EXPECT_FALSE(outside.geofence_margin_triggered);
	ASSERT_TRUE(reposition().valid);
	const hrt_abstime first_hold = reposition().timestamp;
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lat, breach(0));
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));

	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	EXPECT_TRUE(breached(checkAt(0.f, 490.f)));
	EXPECT_EQ(reposition().timestamp, first_hold);

	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FALSE(breached(checkAt(0.f, 490.f)));
	ASSERT_TRUE(breached(checkAt(0.f, 520.f)));
	EXPECT_GT(reposition().timestamp, first_hold);
}

TEST_F(GeofenceMarginTest, TriggersCloserThanTheMarginToAFence)
{
	FencePoints points = inclusionSquare();
	const FencePoints exclusion = exclusionSquare(); // spans 250 m to 350 m east
	points.insert(points.end(), exclusion.begin(), exclusion.end());
	ASSERT_TRUE(loadFence(points));
	setParam("GF_MARGIN", kMargin);

	EXPECT_FALSE(checkAt(0.f, 0.f).geofence_margin_triggered);

	// 80 m from the northern edge of the inclusion.
	const geofence_result_s near_inclusion = checkAt(420.f, 0.f);
	EXPECT_TRUE(near_inclusion.geofence_margin_triggered);
	EXPECT_FALSE(breached(near_inclusion));

	// 60 m west of the exclusion.
	const geofence_result_s near_exclusion = checkAt(0.f, 190.f);
	EXPECT_TRUE(near_exclusion.geofence_margin_triggered);
	EXPECT_FALSE(breached(near_exclusion));

	// Outside the fence is inside its margin too.
	const geofence_result_s outside = checkAt(510.f, 0.f);
	EXPECT_TRUE(outside.geofence_margin_triggered);
	EXPECT_TRUE(outside.geofence_custom_fence_triggered);
}

TEST_F(GeofenceMarginTest, AppliesToTheMaximumDistanceFromHome)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	setParam("GF_MARGIN", kMargin);
	ASSERT_TRUE(loadFence({}));

	EXPECT_FALSE(checkAt(380.f, 0.f).geofence_margin_triggered);
	const geofence_result_s near_limit = checkAt(420.f, 0.f);
	EXPECT_TRUE(near_limit.geofence_margin_triggered);
	EXPECT_FALSE(breached(near_limit));
}

TEST_F(GeofenceMarginTest, AltitudeLimitsHaveNoMargin)
{
	setParam("GF_MAX_VER_DIST", 100.f);
	setParam("GF_MARGIN", kMargin);
	ASSERT_TRUE(loadFence(inclusionSquare()));

	// 5 m below the maximum altitude, then above it.
	EXPECT_FALSE(checkAt(0.f, 0.f, 495.f).geofence_margin_triggered);
	const geofence_result_s above = checkAt(0.f, 0.f, 510.f);
	EXPECT_TRUE(above.geofence_max_alt_triggered);
	EXPECT_FALSE(above.geofence_margin_triggered);
}

TEST_F(GeofenceMarginTest, HoldLoitersAtTheTriggerPosition)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);

	checkAt(0.f, 0.f);
	EXPECT_FALSE(reposition().valid);

	ASSERT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	ASSERT_TRUE(reposition().valid);
	EXPECT_EQ(reposition().type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	const matrix::Vector2d trigger = position(0.f, 420.f);
	EXPECT_DOUBLE_EQ(reposition().lat, trigger(0));
	EXPECT_DOUBLE_EQ(reposition().lon, trigger(1));
	EXPECT_FLOAT_EQ(reposition().alt, kAltitude);
}

TEST_F(GeofenceMarginTest, HoldNeedsAnArmedVehicle)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);
	_navigator.get_vstatus()->arming_state = vehicle_status_s::ARMING_STATE_DISARMED;

	EXPECT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	EXPECT_FALSE(reposition().valid);
}

TEST_F(GeofenceMarginTest, OtherActionsDoNotReposition)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);

	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_RTL, geofence_result_s::GF_ACTION_LAND}) {
		SCOPED_TRACE(action);
		setParam("GF_MARGIN_ACT", action);
		checkAt(0.f, 0.f);
		EXPECT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
		EXPECT_FALSE(reposition().valid);
	}
}

// With GF_ACTION Hold, the fence itself still gets its Hold when the margin only warns.
TEST_F(GeofenceMarginTest, BreachHoldStillRepositionsAfterAMarginWarning)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);
	setParam("GF_MARGIN_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));

	ASSERT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	ASSERT_FALSE(reposition().valid);

	ASSERT_TRUE(breached(checkAt(0.f, 510.f)));
	ASSERT_TRUE(reposition().valid);
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));
}

// A pilot who took over inside the margin and flies on across the fence gets the usual breach Hold.
TEST_F(GeofenceMarginTest, BreachHoldRepositionsAfterATakeoverInsideTheMargin)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);

	ASSERT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	const hrt_abstime margin_hold = reposition().timestamp;
	ASSERT_NE(margin_hold, hrt_abstime{0});

	// Taking over leaves Hold; staying inside the margin starts no new Hold.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	ASSERT_TRUE(checkAt(0.f, 440.f).geofence_margin_triggered);
	ASSERT_EQ(reposition().timestamp, margin_hold);

	ASSERT_TRUE(breached(checkAt(0.f, 510.f)));
	EXPECT_GT(reposition().timestamp, margin_hold);
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));

	// The breach Hold is latched like without a margin.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	EXPECT_TRUE(breached(checkAt(0.f, 490.f)));
}

// The margin triggers first. Breaching the fence while holding must not move the loiter out to the breach position.
TEST_F(GeofenceMarginTest, MarginHoldIsNotMovedByALaterBreach)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);

	ASSERT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	ASSERT_TRUE(reposition().valid);
	const position_setpoint_s hold = reposition();
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	ASSERT_TRUE(breached(checkAt(0.f, 510.f)));
	EXPECT_EQ(reposition().timestamp, hold.timestamp);
	EXPECT_DOUBLE_EQ(reposition().lon, hold.lon);
}

// Like a breach, the margin stays triggered while the vehicle holds, so that an orbit across the edge of the
// margin does not restart the Hold on every turn.
TEST_F(GeofenceMarginTest, StaysTriggeredWhileHolding)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	setParam("GF_MARGIN", kMargin);

	ASSERT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	const hrt_abstime first_hold = reposition().timestamp;
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	EXPECT_TRUE(checkAt(0.f, 300.f).geofence_margin_triggered);
	EXPECT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	EXPECT_EQ(reposition().timestamp, first_hold);

	// Leaving Hold releases it, and the next entry holds again.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FALSE(checkAt(0.f, 300.f).geofence_margin_triggered);
	EXPECT_TRUE(checkAt(0.f, 420.f).geofence_margin_triggered);
	EXPECT_GT(reposition().timestamp, first_hold);
}

TEST_F(GeofenceMarginTest, DisabledMarginIsClearEverywhere)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	const matrix::Vector2d outside = position(0.f, 600.f);
	EXPECT_TRUE(_fence.isClearOfMargin(outside(0), outside(1)));
	setParam("GF_MARGIN", kMargin);
	EXPECT_FALSE(_fence.isClearOfMargin(outside(0), outside(1)));
	// A negative value disables it as well.
	setParam("GF_MARGIN", -1.f);
	EXPECT_FLOAT_EQ(_fence.getMargin(), 0.f);
	EXPECT_TRUE(_fence.isClearOfMargin(outside(0), outside(1)));
}

TEST_F(GeofenceMarginTest, PublishesTheMarginAction)
{
	// Disabled: no action, whatever GF_MARGIN_ACT is.
	EXPECT_EQ(checkAt(0.f, 0.f).geofence_margin_action, geofence_result_s::GF_ACTION_NONE);

	setParam("GF_MARGIN", kMargin);
	EXPECT_EQ(checkAt(0.f, 0.f).geofence_margin_action, geofence_result_s::GF_ACTION_LOITER);
	setParam("GF_MARGIN_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	EXPECT_EQ(checkAt(0.f, 0.f).geofence_margin_action, geofence_result_s::GF_ACTION_LAND);

	// Values the margin does not offer only warn.
	for (const int32_t action : {geofence_result_s::GF_ACTION_NONE, geofence_result_s::GF_ACTION_TERMINATE}) {
		setParam("GF_MARGIN_ACT", action);
		EXPECT_EQ(checkAt(0.f, 0.f).geofence_margin_action, geofence_result_s::GF_ACTION_WARN);
	}
}

// Like GF_ACTION, a margin action Return needs Home.
TEST_F(GeofenceMarginTest, ReturnActionRequiresHome)
{
	setParam("GF_MARGIN_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_RTL));
	EXPECT_FALSE(_fence.isHomeRequired());
	setParam("GF_MARGIN", kMargin);
	EXPECT_TRUE(_fence.isHomeRequired());
	setParam("GF_MARGIN_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	EXPECT_FALSE(_fence.isHomeRequired());
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
// Return routes keep the margin clear, plus the tracking allowance they had before.
TEST_F(GeofenceMarginTest, ReturnAvoidanceKeepsTheMarginClear)
{
	setParam("NAV_ACC_RAD", 10.f);
	setParam("NAV_LOITER_RAD", 80.f);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), 20.f);
	setParam("GF_MARGIN", kMargin);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), 20.f + kMargin);
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), 80.f + kMargin);
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
