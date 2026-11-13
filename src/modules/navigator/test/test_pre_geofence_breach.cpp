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

// Navigator's existing test friend, used here to reach the private target checks.
class MissionBaseTestNavigator
{
public:
	static bool targetAllowed(Navigator &navigator, const vehicle_global_position_s &target)
	{
		return navigator.geofence_allows_position(target) && navigator.pre_geofence_allows_target(target);
	}
};

// Separate hard and pre-geofence breach actions in Navigator. Home is at the origin.
class PreGeofenceBreachTest : public navigator_test::GeofenceTestBase
{
protected:
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

	FencePoints bothFences() const
	{
		FencePoints points = inclusionSquare();
		const FencePoints pre = polygon(true, {{-400.f, -400.f}, {400.f, -400.f}, {400.f, 400.f}, {-400.f, 400.f}}, FenceLevel::Pre);
		points.insert(points.end(), pre.begin(), pre.end());
		return points;
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

	static bool hardFenceBreached(const geofence_result_s &result)
	{
		return result.geofence_max_dist_triggered || result.geofence_max_alt_triggered
		       || result.geofence_custom_fence_triggered;
	}

	const position_setpoint_s &reposition() { return _navigator.get_reposition_triplet()->current; }

	uORB::Subscription _result_sub{ORB_ID(geofence_result)};
};

// Without a pre-geofence the Hold after a hard breach works as before: at the breach position, latched while holding,
// and again on the next breach.
TEST_F(PreGeofenceBreachTest, HardFenceHoldWithoutPreGeofenceIsUnchanged)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));

	const geofence_result_s inside = checkAt(0.f, 490.f);
	EXPECT_FALSE(hardFenceBreached(inside));
	EXPECT_FALSE(inside.pre_geofence_breached);
	EXPECT_EQ(inside.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(reposition().valid);

	const geofence_result_s outside = checkAt(0.f, 510.f);
	ASSERT_TRUE(hardFenceBreached(outside));
	EXPECT_FALSE(outside.pre_geofence_breached);
	ASSERT_TRUE(reposition().valid);
	const hrt_abstime first_hold = reposition().timestamp;
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lat, breach(0));
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));

	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	EXPECT_TRUE(hardFenceBreached(checkAt(0.f, 490.f)));
	EXPECT_EQ(reposition().timestamp, first_hold);

	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FALSE(hardFenceBreached(checkAt(0.f, 490.f)));
	ASSERT_TRUE(hardFenceBreached(checkAt(0.f, 520.f)));
	EXPECT_GT(reposition().timestamp, first_hold);
}

TEST_F(PreGeofenceBreachTest, TriggersAtEachDrawnPreGeofenceArea)
{
	FencePoints points = bothFences();
	const FencePoints exclusion = exclusionSquare();
	const FencePoints pre_exclusion = circle(false, {0.f, 300.f}, 150.f, FenceLevel::Pre);
	points.insert(points.end(), exclusion.begin(), exclusion.end());
	points.insert(points.end(), pre_exclusion.begin(), pre_exclusion.end());
	ASSERT_TRUE(loadFence(points));

	EXPECT_FALSE(checkAt(0.f, 0.f).pre_geofence_breached);

	// Outside the drawn pre-geofence inclusion, but inside the hard inclusion.
	const geofence_result_s outside_pre_inclusion = checkAt(420.f, 0.f);
	EXPECT_TRUE(outside_pre_inclusion.pre_geofence_breached);
	EXPECT_FALSE(hardFenceBreached(outside_pre_inclusion));

	// Inside the pre-geofence exclusion, but outside the hard exclusion.
	const geofence_result_s inside_pre_exclusion = checkAt(0.f, 190.f);
	EXPECT_TRUE(inside_pre_exclusion.pre_geofence_breached);
	EXPECT_FALSE(hardFenceBreached(inside_pre_exclusion));

	// This position breaches both drawn inclusion boundaries.
	const geofence_result_s outside = checkAt(510.f, 0.f);
	EXPECT_TRUE(outside.pre_geofence_breached);
	EXPECT_TRUE(outside.geofence_custom_fence_triggered);
}

TEST_F(PreGeofenceBreachTest, HomeDistanceRemainsAHardLimit)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	ASSERT_TRUE(loadFence({}));
	EXPECT_FALSE(checkAt(420.f, 0.f).pre_geofence_breached);
	const geofence_result_s outside = checkAt(510.f, 0.f);
	EXPECT_TRUE(outside.geofence_max_dist_triggered);
	EXPECT_FALSE(outside.pre_geofence_breached);
}

TEST_F(PreGeofenceBreachTest, AltitudeLimitsRemainHardLimits)
{
	setParam("GF_MAX_VER_DIST", 100.f);
	ASSERT_TRUE(loadFence(bothFences()));

	// 5 m below the maximum altitude, then above it.
	EXPECT_FALSE(checkAt(0.f, 0.f, 495.f).pre_geofence_breached);
	const geofence_result_s above = checkAt(0.f, 0.f, 510.f);
	EXPECT_TRUE(above.geofence_max_alt_triggered);
	EXPECT_FALSE(above.pre_geofence_breached);
}

TEST_F(PreGeofenceBreachTest, HoldLoitersAtTheTriggerPosition)
{
	ASSERT_TRUE(loadFence(bothFences()));

	checkAt(0.f, 0.f);
	EXPECT_FALSE(reposition().valid);

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	ASSERT_TRUE(reposition().valid);
	EXPECT_EQ(reposition().type, position_setpoint_s::SETPOINT_TYPE_LOITER);
	const matrix::Vector2d trigger = position(0.f, 420.f);
	EXPECT_DOUBLE_EQ(reposition().lat, trigger(0));
	EXPECT_DOUBLE_EQ(reposition().lon, trigger(1));
	EXPECT_FLOAT_EQ(reposition().alt, kAltitude);
}

TEST_F(PreGeofenceBreachTest, HoldNeedsAnArmedVehicle)
{
	ASSERT_TRUE(loadFence(bothFences()));
	_navigator.get_vstatus()->arming_state = vehicle_status_s::ARMING_STATE_DISARMED;

	EXPECT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	EXPECT_FALSE(reposition().valid);
}

TEST_F(PreGeofenceBreachTest, OtherActionsDoNotReposition)
{
	ASSERT_TRUE(loadFence(bothFences()));

	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_RTL, geofence_result_s::GF_ACTION_LAND}) {
		SCOPED_TRACE(action);
		setParam("GF_PRE_ACTION", action);
		checkAt(0.f, 0.f);
		EXPECT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
		EXPECT_FALSE(reposition().valid);
	}
}

// The hard fence still gets its Hold when the pre-geofence only warns.
TEST_F(PreGeofenceBreachTest, HardFenceHoldStillRepositionsAfterPreGeofenceWarning)
{
	ASSERT_TRUE(loadFence(bothFences()));
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	ASSERT_FALSE(reposition().valid);

	ASSERT_TRUE(hardFenceBreached(checkAt(0.f, 510.f)));
	ASSERT_TRUE(reposition().valid);
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));
}

// A pilot who took over after the pre-geofence breach still gets the hard-fence Hold.
TEST_F(PreGeofenceBreachTest, HardFenceHoldRepositionsAfterTakeoverOutsidePreGeofence)
{
	ASSERT_TRUE(loadFence(bothFences()));

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	const hrt_abstime pre_geofence_hold_time = reposition().timestamp;
	ASSERT_NE(pre_geofence_hold_time, hrt_abstime{0});

	// Taking over leaves Hold; remaining outside the pre-geofence starts no new Hold.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	ASSERT_TRUE(checkAt(0.f, 440.f).pre_geofence_breached);
	ASSERT_EQ(reposition().timestamp, pre_geofence_hold_time);

	ASSERT_TRUE(hardFenceBreached(checkAt(0.f, 510.f)));
	EXPECT_GT(reposition().timestamp, pre_geofence_hold_time);
	const matrix::Vector2d breach = position(0.f, 510.f);
	EXPECT_DOUBLE_EQ(reposition().lon, breach(1));

	// The hard-fence Hold remains latched while the vehicle is holding.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	EXPECT_TRUE(hardFenceBreached(checkAt(0.f, 490.f)));
}

// Breaching the hard fence while holding must not move the loiter out to the breach position.
TEST_F(PreGeofenceBreachTest, PreGeofenceHoldIsNotMovedByHardBreach)
{
	ASSERT_TRUE(loadFence(bothFences()));

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	ASSERT_TRUE(reposition().valid);
	const position_setpoint_s hold = reposition();
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	ASSERT_TRUE(hardFenceBreached(checkAt(0.f, 510.f)));
	EXPECT_EQ(reposition().timestamp, hold.timestamp);
	EXPECT_DOUBLE_EQ(reposition().lon, hold.lon);
}

// The pre-geofence stays triggered while holding, so an orbit across its boundary does not restart Hold.
TEST_F(PreGeofenceBreachTest, StaysTriggeredWhileHolding)
{
	ASSERT_TRUE(loadFence(bothFences()));

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	const hrt_abstime first_hold = reposition().timestamp;
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	EXPECT_TRUE(checkAt(0.f, 300.f).pre_geofence_breached);
	EXPECT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	EXPECT_EQ(reposition().timestamp, first_hold);

	// Leaving Hold releases it, and the next entry holds again.
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FALSE(checkAt(0.f, 300.f).pre_geofence_breached);
	EXPECT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	EXPECT_GT(reposition().timestamp, first_hold);
}

// Removing the pre-geofence releases its trigger without changing the Hold or the hard-fence trigger.
TEST_F(PreGeofenceBreachTest, RemovingPreGeofenceWhileHoldingReleasesItsFlag)
{
	ASSERT_TRUE(loadFence(bothFences()));

	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const geofence_result_s before_removal = checkAt(0.f, 510.f);
	ASSERT_TRUE(before_removal.pre_geofence_breached);
	ASSERT_TRUE(hardFenceBreached(before_removal));
	const position_setpoint_s hold = reposition();

	ASSERT_TRUE(loadFence(inclusionSquare()));
	const geofence_result_s after_removal = checkAt(0.f, 420.f);
	EXPECT_EQ(after_removal.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(after_removal.pre_geofence_breached);
	EXPECT_TRUE(after_removal.geofence_custom_fence_triggered);
	EXPECT_EQ(after_removal.geofence_action, before_removal.geofence_action);
	EXPECT_EQ(reposition().timestamp, hold.timestamp);
	EXPECT_DOUBLE_EQ(reposition().lat, hold.lat);
	EXPECT_DOUBLE_EQ(reposition().lon, hold.lon);
	EXPECT_FLOAT_EQ(reposition().alt, hold.alt);
}

TEST_F(PreGeofenceBreachTest, RemovalClearsPreGeofenceWithoutAValidPosition)
{
	ASSERT_TRUE(loadFence(bothFences()));
	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const position_setpoint_s hold = reposition();
	ASSERT_TRUE(loadFence(inclusionSquare()));
	_navigator.get_global_position()->timestamp = 0;
	px4_usleep(kCheckInterval);
	_navigator.geofence_breach_check();
	geofence_result_s result{};
	ASSERT_TRUE(_result_sub.update(&result));
	EXPECT_FALSE(result.pre_geofence_breached);
	EXPECT_EQ(result.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_EQ(reposition().timestamp, hold.timestamp);
	EXPECT_DOUBLE_EQ(reposition().lat, hold.lat);
	EXPECT_DOUBLE_EQ(reposition().lon, hold.lon);
}

TEST_F(PreGeofenceBreachTest, ReaddingPreGeofenceDoesNotRestoreRemovedBreach)
{
	ASSERT_TRUE(loadFence(bothFences()));
	ASSERT_TRUE(checkAt(0.f, 420.f).pre_geofence_breached);
	_navigator.get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const position_setpoint_s hold = reposition();
	ASSERT_TRUE(loadFence(inclusionSquare()));
	ASSERT_FALSE(checkAt(0.f, 420.f).pre_geofence_breached);

	// The new area contains the vehicle. Its Land action must not inherit the removed area's breach.
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	FencePoints replacement = inclusionSquare();
	const FencePoints pre = polygon(true, {{-450.f, -450.f}, {450.f, -450.f}, {450.f, 450.f}, {-450.f, 450.f}}, FenceLevel::Pre);
	replacement.insert(replacement.end(), pre.begin(), pre.end());
	ASSERT_TRUE(loadFence(replacement));
	const geofence_result_s after_readding = checkAt(0.f, 420.f);
	EXPECT_EQ(after_readding.pre_geofence_action, geofence_result_s::GF_ACTION_LAND);
	EXPECT_FALSE(after_readding.pre_geofence_breached);
	EXPECT_EQ(reposition().timestamp, hold.timestamp);
}

TEST_F(PreGeofenceBreachTest, PublishesThePreGeofenceAction)
{
	// Disabled: no action, whatever GF_PRE_ACTION is.
	EXPECT_EQ(checkAt(0.f, 0.f).pre_geofence_action, geofence_result_s::GF_ACTION_NONE);

	ASSERT_TRUE(loadFence(bothFences()));
	EXPECT_EQ(checkAt(0.f, 0.f).pre_geofence_action, geofence_result_s::GF_ACTION_LOITER);
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	EXPECT_EQ(checkAt(0.f, 0.f).pre_geofence_action, geofence_result_s::GF_ACTION_LAND);
}

TEST_F(PreGeofenceBreachTest, ResolvedTargetsRespectEachActionIndependently)
{
	ASSERT_TRUE(loadFence(bothFences()));
	checkAt(0.f, 0.f);
	vehicle_global_position_s target = *_navigator.get_global_position();
	const auto outside_pre = position(420.f, 0.f);
	target.lat = outside_pre(0); // Latitude-only orbit resolves longitude from current position.
	EXPECT_FALSE(MissionBaseTestNavigator::targetAllowed(_navigator, target));
	target = *_navigator.get_global_position();
	target.lon = position(0.f, 420.f)(1);
	EXPECT_FALSE(MissionBaseTestNavigator::targetAllowed(_navigator, target));
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	EXPECT_TRUE(MissionBaseTestNavigator::targetAllowed(_navigator, target));
	target.lon = position(0.f, 510.f)(1);
	EXPECT_FALSE(MissionBaseTestNavigator::targetAllowed(_navigator, target));
	setParam("GF_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_NONE));
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	target.lon = position(0.f, 420.f)(1);
	EXPECT_FALSE(MissionBaseTestNavigator::targetAllowed(_navigator, target));
}

TEST_F(PreGeofenceBreachTest, UnchangedPositionRemainsAvailableOutsidePreGeofence)
{
	ASSERT_TRUE(loadFence(bothFences()));
	checkAt(0.f, 420.f);
	EXPECT_TRUE(MissionBaseTestNavigator::targetAllowed(_navigator, *_navigator.get_global_position()));
}
