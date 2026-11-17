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

// Navigator's existing test friend, used here to reach the private target check.
class MissionBaseTestNavigator
{
public:
	static bool targetAllowed(Navigator &navigator, const vehicle_global_position_s &target)
	{
		return navigator.pre_geofence_allows_target(target);
	}
};

// The pre-geofence margin and its action in Navigator. Home is at the origin.
class PreGeofenceBreachTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kAltitude = 450.f; // 50 m above Home
	static constexpr float kMargin = 100.f;
	static constexpr useconds_t kCheckInterval = 210000; // the breach check runs at 5 Hz

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence());
		setVehicle(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

		// A check without any limit clears what an earlier test left in Navigator.
		checkAt(0.f, 0.f);
		_navigator.get_reposition_triplet()->current = position_setpoint_s{};
	}

	void TearDown() override { param_reset_all(); }

	void setVehicle(uint8_t vehicle_type, uint8_t arming_state = vehicle_status_s::ARMING_STATE_ARMED)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_type;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		status.arming_state = arming_state;
		*_navigator.get_vstatus() = status;
	}

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

	// A 1000 m square inclusion fence centred on Home, with the pre-geofence margin enabled.
	void loadSquareWithMargin()
	{
		setParam("GF_PRE_MARGIN", kMargin);
		ASSERT_TRUE(loadFence(polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}})));
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

	static bool geofenceBreached(const geofence_result_s &result)
	{
		return result.geofence_max_dist_triggered || result.geofence_max_alt_triggered
		       || result.geofence_custom_fence_triggered;
	}

	::testing::AssertionResult repositionAt(float north, float east)
	{
		const position_setpoint_s &setpoint = _navigator.get_reposition_triplet()->current;

		if (!setpoint.valid) {
			return ::testing::AssertionFailure() << "no reposition";
		}

		const matrix::Vector2d expected = position(north, east);
		const float error = get_distance_to_next_waypoint(setpoint.lat, setpoint.lon, expected(0), expected(1));

		if (error > 1.f) {
			return ::testing::AssertionFailure() << "reposition is " << error << " m from the expected point";
		}

		return ::testing::AssertionSuccess();
	}

	bool repositioned() { return _navigator.get_reposition_triplet()->current.valid; }

	vehicle_global_position_s target(float north, float east) const
	{
		const matrix::Vector2d at = position(north, east);
		vehicle_global_position_s target{};
		target.lat = at(0);
		target.lon = at(1);
		target.alt = kAltitude;
		return target;
	}

	uORB::Subscription _result_sub{ORB_ID(geofence_result)};
};

TEST_F(PreGeofenceBreachTest, DisabledByDefault)
{
	ASSERT_TRUE(loadFence(polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}})));
	const geofence_result_s result = checkAt(0.f, 490.f);
	EXPECT_FALSE(result.pre_geofence_triggered);
	EXPECT_EQ(result.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(repositioned());
}

TEST_F(PreGeofenceBreachTest, TriggersWithinTheMargin)
{
	loadSquareWithMargin();

	const geofence_result_s clear = checkAt(0.f, 390.f);
	EXPECT_FALSE(clear.pre_geofence_triggered);
	EXPECT_EQ(clear.pre_geofence_action, geofence_result_s::GF_ACTION_LOITER);

	const geofence_result_s within = checkAt(0.f, 410.f);
	EXPECT_TRUE(within.pre_geofence_triggered);
	EXPECT_FALSE(geofenceBreached(within));

	// Outside the fence, both are breached.
	const geofence_result_s outside = checkAt(0.f, 510.f);
	EXPECT_TRUE(outside.pre_geofence_triggered);
	EXPECT_TRUE(outside.geofence_custom_fence_triggered);
}

TEST_F(PreGeofenceBreachTest, MarginAppliesToTheHomeDistance)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	setParam("GF_PRE_MARGIN", kMargin);
	ASSERT_TRUE(loadFence({}));

	const geofence_result_s within = checkAt(420.f, 0.f);
	EXPECT_TRUE(within.pre_geofence_triggered);
	EXPECT_FALSE(within.geofence_max_dist_triggered);
}

TEST_F(PreGeofenceBreachTest, AltitudeLimitsHaveNoMargin)
{
	setParam("GF_MAX_VER_DIST", 100.f);
	loadSquareWithMargin();

	// 5 m below the maximum altitude, then above it.
	EXPECT_FALSE(checkAt(0.f, 0.f, 495.f).pre_geofence_triggered);
	const geofence_result_s above = checkAt(0.f, 0.f, 510.f);
	EXPECT_TRUE(above.geofence_max_alt_triggered);
	EXPECT_FALSE(above.pre_geofence_triggered);
}

TEST_F(PreGeofenceBreachTest, NoneActionDisablesTheMargin)
{
	setParam("GF_PRE_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_NONE));
	loadSquareWithMargin();
	const geofence_result_s result = checkAt(0.f, 410.f);
	EXPECT_FALSE(result.pre_geofence_triggered);
	EXPECT_EQ(result.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
}

TEST_F(PreGeofenceBreachTest, MulticopterHoldsWhereTheMarginIsBreached)
{
	loadSquareWithMargin();
	checkAt(0.f, 390.f);
	EXPECT_FALSE(repositioned());

	ASSERT_TRUE(checkAt(0.f, 410.f).pre_geofence_triggered);
	EXPECT_TRUE(repositionAt(0.f, 410.f));
	EXPECT_EQ(_navigator.get_reposition_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LOITER);
}

TEST_F(PreGeofenceBreachTest, FixedWingLoitersWhereTheMarginIsBreached)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	loadSquareWithMargin();
	checkAt(0.f, 380.f);

	ASSERT_TRUE(checkAt(0.f, 410.f).pre_geofence_triggered);
	EXPECT_TRUE(repositionAt(0.f, 410.f));
	EXPECT_FLOAT_EQ(_navigator.get_reposition_triplet()->current.loiter_radius, _navigator.get_default_loiter_rad());
}

TEST_F(PreGeofenceBreachTest, HoldNeedsAnArmedVehicle)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::ARMING_STATE_DISARMED);
	loadSquareWithMargin();
	EXPECT_TRUE(checkAt(0.f, 410.f).pre_geofence_triggered);
	EXPECT_FALSE(repositioned());
}

TEST_F(PreGeofenceBreachTest, OtherActionsDoNotReposition)
{
	loadSquareWithMargin();

	for (int32_t action : {
		     geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_RTL,
		     geofence_result_s::GF_ACTION_TERMINATE, geofence_result_s::GF_ACTION_LAND
	     }) {
		SCOPED_TRACE(action);
		setParam("GF_PRE_ACT", action);
		checkAt(0.f, 0.f);
		const geofence_result_s result = checkAt(0.f, 410.f);
		EXPECT_TRUE(result.pre_geofence_triggered);
		EXPECT_EQ(result.pre_geofence_action, action);
		EXPECT_FALSE(repositioned());
	}
}

TEST_F(PreGeofenceBreachTest, GeofenceActionSuppressesThePreGeofenceHold)
{
	setParam("GF_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	loadSquareWithMargin();

	const geofence_result_s result = checkAt(0.f, 510.f);
	EXPECT_TRUE(geofenceBreached(result));
	EXPECT_TRUE(result.pre_geofence_triggered);
	EXPECT_FALSE(repositioned());
}

TEST_F(PreGeofenceBreachTest, GeofenceHoldAfterAPreGeofenceWarning)
{
	setParam("GF_PRE_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	loadSquareWithMargin();

	ASSERT_TRUE(checkAt(0.f, 410.f).pre_geofence_triggered);
	EXPECT_FALSE(repositioned());

	ASSERT_TRUE(geofenceBreached(checkAt(0.f, 510.f)));
	EXPECT_TRUE(repositionAt(0.f, 510.f));
}

TEST_F(PreGeofenceBreachTest, CommandedTargetsMustKeepTheMargin)
{
	loadSquareWithMargin();
	checkAt(0.f, 0.f);

	EXPECT_TRUE(MissionBaseTestNavigator::targetAllowed(_navigator, target(0.f, 390.f)));
	EXPECT_FALSE(MissionBaseTestNavigator::targetAllowed(_navigator, target(0.f, 410.f)));

	// Holding at the current position stays possible within the margin.
	checkAt(0.f, 410.f);
	EXPECT_TRUE(MissionBaseTestNavigator::targetAllowed(_navigator, target(0.f, 410.f)));

	// A warning does not restrict targets.
	setParam("GF_PRE_ACT", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	EXPECT_TRUE(MissionBaseTestNavigator::targetAllowed(_navigator, target(0.f, 450.f)));
}
