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

#include <ostream>

#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>

// Navigator's existing test friend, used here to reach its private geofence checks.
class MissionBaseTestNavigator
{
public:
	static bool targetAllowed(Navigator &navigator, const vehicle_global_position_s &target)
	{
		return navigator.geofence_emer_allows_position(target) && navigator.geofence_cont_allows_position(target);
	}

	// Run the breach check now instead of waiting for its 5 Hz interval.
	static void breachCheck(Navigator &navigator)
	{
		navigator._last_geofence_check = 0;
		navigator.geofence_breach_check();
	}
};

// The contingency geofence and its action in Navigator. Home is at the origin, 400 m AMSL.
class ContGeofenceBreachTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kHomeAltitude = 400.f;
	static constexpr float kAltitude = kHomeAltitude + 50.f;

	struct Place {
		float north;
		float east;
		float altitude;
	};

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence(kHomeAltitude));
		ASSERT_TRUE(loadFence({}));
		setVehicle(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

		// A check without any limit clears what an earlier test left in Navigator.
		setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		checkAt({0.f, 0.f, kAltitude});
		clearReposition();
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

	// The breach latch of a Hold is only dropped 100 ms after it was set.
	void setNavState(uint8_t nav_state)
	{
		_navigator.get_vstatus()->nav_state = nav_state;
		px4_usleep(110000);
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

	// Place the vehicle and run one breach check.
	geofence_result_s checkAt(const Place &place)
	{
		const matrix::Vector2d vehicle = position(place.north, place.east);
		vehicle_global_position_s &global_position = *_navigator.get_global_position();
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = vehicle(0);
		global_position.lon = vehicle(1);
		global_position.alt = place.altitude;
		MissionBaseTestNavigator::breachCheck(_navigator);

		geofence_result_s result{};
		EXPECT_TRUE(_result_sub.update(&result));
		return result;
	}

	static bool emergencyBreached(const geofence_result_s &result)
	{
		return result.geofence_emer_max_dist_triggered || result.geofence_emer_max_alt_triggered
		       || result.geofence_emer_custom_fence_triggered;
	}

	static bool contingencyBreached(const geofence_result_s &result)
	{
		return result.geofence_cont_max_dist_triggered || result.geofence_cont_custom_fence_triggered;
	}

	// A 1000 m square inclusion fence centred on Home.
	FencePoints inclusionSquare() const
	{
		return polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}});
	}

	const position_setpoint_s &reposition() { return _navigator.get_reposition_triplet()->current; }

	void clearReposition() { _navigator.get_reposition_triplet()->current = position_setpoint_s{}; }

	::testing::AssertionResult repositionedAt(const Place &place)
	{
		if (!reposition().valid) {
			return ::testing::AssertionFailure() << "no reposition";
		}

		const matrix::Vector2d expected = position(place.north, place.east);
		const float error = get_distance_to_next_waypoint(reposition().lat, reposition().lon, expected(0), expected(1));

		if (error > 0.1f || fabsf(reposition().alt - place.altitude) > 0.1f) {
			return ::testing::AssertionFailure() << "reposition is " << error << " m away, at " << reposition().alt << " m";
		}

		return ::testing::AssertionSuccess();
	}

	vehicle_global_position_s target(const Place &place) const
	{
		const matrix::Vector2d at = position(place.north, place.east);
		vehicle_global_position_s target{};
		target.lat = at(0);
		target.lon = at(1);
		target.alt = place.altitude;
		return target;
	}

	bool targetAllowed(const Place &place) { return MissionBaseTestNavigator::targetAllowed(_navigator, target(place)); }

	uORB::Subscription _result_sub{ORB_ID(geofence_result)};
};

TEST_F(ContGeofenceBreachTest, DisabledByDefault)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	setParam("GF_MAX_VER_DIST", 100.f);

	const geofence_result_s inside = checkAt({0.f, 490.f, kHomeAltitude + 90.f});
	EXPECT_FALSE(contingencyBreached(inside));
	EXPECT_EQ(inside.geofence_cont_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(emergencyBreached(inside));
	EXPECT_FALSE(reposition().valid);
	EXPECT_TRUE(targetAllowed({0.f, 490.f, kHomeAltitude + 90.f}));

	const geofence_result_s outside = checkAt({0.f, 510.f, kAltitude});
	EXPECT_TRUE(outside.geofence_emer_max_dist_triggered);
	EXPECT_FALSE(contingencyBreached(outside));
}

// Without a contingency geofence the Hold after an emergency geofence breach works as before: at the breach
// position, latched while holding, again on the next breach, and once per breach.
TEST_F(ContGeofenceBreachTest, GeofenceHoldWithoutContGeofenceIsUnchanged)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	const Place inside{0.f, 490.f, kAltitude};

	ASSERT_TRUE(emergencyBreached(checkAt({0.f, 510.f, kAltitude})));
	ASSERT_TRUE(repositionedAt({0.f, 510.f, kAltitude}));
	const hrt_abstime first_hold = reposition().timestamp;

	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	EXPECT_TRUE(emergencyBreached(checkAt(inside)));
	EXPECT_EQ(reposition().timestamp, first_hold);

	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_FALSE(emergencyBreached(checkAt(inside)));
	ASSERT_TRUE(emergencyBreached(checkAt({0.f, 520.f, kAltitude})));
	EXPECT_GT(reposition().timestamp, first_hold);

	// A pilot who takes over and stays outside starts no new Hold.
	const hrt_abstime second_hold = reposition().timestamp;
	setNavState(vehicle_status_s::NAVIGATION_STATE_POSCTL);
	EXPECT_TRUE(emergencyBreached(checkAt({0.f, 530.f, kAltitude})));
	EXPECT_EQ(reposition().timestamp, second_hold);
}

TEST_F(ContGeofenceBreachTest, ContGeofenceReturnNeedsHome)
{
	EXPECT_FALSE(_fence.isHomeRequired());

	// The action alone does nothing while the contingency geofence is disabled.
	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_RTL));
	EXPECT_FALSE(_fence.isHomeRequired());

	// The margin is relative to the emergency geofence, so only its Return action needs Home.
	setParam("GF_CONT_H_MARGIN", 100.f);
	EXPECT_TRUE(_fence.isHomeRequired());
	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	EXPECT_FALSE(_fence.isHomeRequired());
}

// A margin that leaves no room inside the Home distance limit is breached everywhere, not disabled.
TEST_F(ContGeofenceBreachTest, MarginWiderThanTheHomeDistanceLimit)
{
	setParam("GF_MAX_HOR_DIST", 100.f);
	setParam("GF_CONT_H_MARGIN", 100.f);

	const geofence_result_s at_home = checkAt({0.f, 0.f, kAltitude});
	EXPECT_TRUE(at_home.geofence_cont_max_dist_triggered);
	EXPECT_FALSE(emergencyBreached(at_home));
	EXPECT_FALSE(targetAllowed({0.f, 0.f, kAltitude}));
}

// The contingency geofence is horizontal only: the maximum altitude has no margin.
TEST_F(ContGeofenceBreachTest, MaximumAltitudeHasNoContGeofence)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	setParam("GF_MAX_VER_DIST", 100.f);
	setParam("GF_CONT_H_MARGIN", 50.f);

	const geofence_result_s below = checkAt({0.f, 0.f, kHomeAltitude + 80.f});
	EXPECT_FALSE(contingencyBreached(below));
	EXPECT_FALSE(emergencyBreached(below));

	const geofence_result_s above = checkAt({0.f, 0.f, kHomeAltitude + 110.f});
	EXPECT_FALSE(contingencyBreached(above));
	EXPECT_TRUE(above.geofence_emer_max_alt_triggered);
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
// Return routes around the fences with the margin added, so that it does not trigger GF_CONT_ACTION.
TEST_F(ContGeofenceBreachTest, ReturnKeepsTheMargin)
{
	const float without_margin = _navigator.geofence_avoidance_margin();
	setParam("GF_CONT_H_MARGIN", 100.f);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), without_margin + 100.f);

	setVehicle(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), _navigator.get_default_loiter_rad() + 100.f);
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

// The emergency geofence that GF_CONT_H_MARGIN, 100 m, is kept from.
enum class Source {
	HomeDistance, // GF_MAX_HOR_DIST 500 m
	Plan, // a 1000 m square plan fence
};

static std::ostream &operator<<(std::ostream &os, Source source)
{
	switch (source) {
	case Source::HomeDistance: return os << "HomeDistance";

	case Source::Plan: return os << "Plan";
	}

	return os;
}

// Behaviour that every contingency geofence source shares.
class ContGeofenceSourceTest : public ContGeofenceBreachTest, public ::testing::WithParamInterface<Source>
{
protected:
	void enableContGeofence()
	{
		if (GetParam() == Source::HomeDistance) {
			setParam("GF_MAX_HOR_DIST", 500.f);

		} else {
			ASSERT_TRUE(loadFence(inclusionSquare()));
		}

		setParam("GF_CONT_H_MARGIN", 100.f);
	}

	// Inside both, outside the contingency geofence only, and outside both.
	Place inside() const { return {0.f, 300.f, kAltitude}; }
	Place between() const { return {0.f, 420.f, kAltitude}; }
	Place outside() const { return {0.f, 510.f, kAltitude}; }

	// Only the source under test is reported.
	bool sourceTriggered(const geofence_result_s &result) const
	{
		return GetParam() == Source::HomeDistance
		       ? result.geofence_cont_max_dist_triggered && !result.geofence_cont_custom_fence_triggered
		       : result.geofence_cont_custom_fence_triggered && !result.geofence_cont_max_dist_triggered;
	}

	// Leave Hold and return inside, as if the pilot had taken over and flown back.
	void backInside()
	{
		setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		const geofence_result_s result = checkAt(inside());
		ASSERT_FALSE(contingencyBreached(result));
		ASSERT_FALSE(emergencyBreached(result));
		clearReposition();
	}
};

TEST_P(ContGeofenceSourceTest, TriggersBeforeTheGeofence)
{
	enableContGeofence();

	const geofence_result_s clear = checkAt(inside());
	EXPECT_FALSE(contingencyBreached(clear));
	EXPECT_FALSE(emergencyBreached(clear));
	EXPECT_EQ(clear.geofence_cont_action, geofence_result_s::GF_ACTION_LOITER);

	const geofence_result_s contingency = checkAt(between());
	EXPECT_TRUE(sourceTriggered(contingency));
	EXPECT_FALSE(emergencyBreached(contingency));

	const geofence_result_s both = checkAt(outside());
	EXPECT_TRUE(sourceTriggered(both));
	EXPECT_TRUE(emergencyBreached(both));
}

TEST_P(ContGeofenceSourceTest, PublishesTheContGeofenceAction)
{
	EXPECT_EQ(checkAt(inside()).geofence_cont_action, geofence_result_s::GF_ACTION_NONE);

	enableContGeofence();
	EXPECT_EQ(checkAt(inside()).geofence_cont_action, geofence_result_s::GF_ACTION_LOITER);
	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	EXPECT_EQ(checkAt(inside()).geofence_cont_action, geofence_result_s::GF_ACTION_LAND);

	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_NONE));
	EXPECT_EQ(checkAt(inside()).geofence_cont_action, geofence_result_s::GF_ACTION_NONE);
}

TEST_P(ContGeofenceSourceTest, HoldLoitersAtTheTriggerPosition)
{
	enableContGeofence();
	checkAt(inside());
	EXPECT_FALSE(reposition().valid);

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_TRUE(repositionedAt(between()));
	EXPECT_EQ(reposition().type, position_setpoint_s::SETPOINT_TYPE_LOITER);
}

TEST_P(ContGeofenceSourceTest, FixedWingLoitersAroundTheTriggerPosition)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	enableContGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_TRUE(repositionedAt(between()));
	EXPECT_FLOAT_EQ(reposition().loiter_radius, _navigator.get_default_loiter_rad());
}

TEST_P(ContGeofenceSourceTest, HoldNeedsAnArmedVehicle)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::ARMING_STATE_DISARMED);
	enableContGeofence();

	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_FALSE(reposition().valid);
}

TEST_P(ContGeofenceSourceTest, OtherActionsDoNotReposition)
{
	enableContGeofence();

	for (const int32_t action : {
		     geofence_result_s::GF_ACTION_NONE, geofence_result_s::GF_ACTION_WARN,
		     geofence_result_s::GF_ACTION_RTL, geofence_result_s::GF_ACTION_LAND
	     }) {
		SCOPED_TRACE(action);
		setParam("GF_CONT_ACTION", action);
		checkAt(inside());
		const geofence_result_s result = checkAt(between());
		EXPECT_TRUE(sourceTriggered(result));
		EXPECT_EQ(result.geofence_cont_action, action);
		EXPECT_FALSE(reposition().valid);
	}
}

// Commander ranks the two actions. An emergency geofence that only warns, or does nothing, leaves the
// contingency geofence Hold in place when both are breached at once.
TEST_P(ContGeofenceSourceTest, GeofenceWarningDoesNotReplaceTheHold)
{
	enableContGeofence();

	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_NONE}) {
		SCOPED_TRACE(action);
		setParam("GF_ACTION", action);
		ASSERT_TRUE(emergencyBreached(checkAt(outside())));
		EXPECT_TRUE(repositionedAt(outside()));
		backInside();
	}
}

// The emergency geofence still gets its Hold when the contingency geofence only warns.
TEST_P(ContGeofenceSourceTest, GeofenceHoldAfterAContGeofenceWarning)
{
	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	enableContGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	ASSERT_FALSE(reposition().valid);

	ASSERT_TRUE(emergencyBreached(checkAt(outside())));
	EXPECT_TRUE(repositionedAt(outside()));
}

// A pilot who took over after the contingency geofence breach is still held at the emergency geofence.
TEST_P(ContGeofenceSourceTest, GeofenceHoldAfterATakeover)
{
	enableContGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime hold_contingency = reposition().timestamp;
	ASSERT_NE(hold_contingency, hrt_abstime{0});

	// Taking over leaves Hold; staying outside the contingency geofence starts no new Hold.
	setNavState(vehicle_status_s::NAVIGATION_STATE_POSCTL);
	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	ASSERT_EQ(reposition().timestamp, hold_contingency);

	ASSERT_TRUE(emergencyBreached(checkAt(outside())));
	EXPECT_GT(reposition().timestamp, hold_contingency);
	EXPECT_TRUE(repositionedAt(outside()));

	// The emergency geofence Hold stays latched while the vehicle is holding.
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	EXPECT_TRUE(emergencyBreached(checkAt(between())));
}

// Breaching the emergency geofence while holding must not move the loiter out to the breach position.
TEST_P(ContGeofenceSourceTest, HoldIsNotMovedByAGeofenceBreach)
{
	enableContGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime hold = reposition().timestamp;
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	ASSERT_TRUE(emergencyBreached(checkAt(outside())));
	EXPECT_EQ(reposition().timestamp, hold);
	EXPECT_TRUE(repositionedAt(between()));
}

// The breach stays set while holding, so an orbit across the boundary does not restart the Hold.
TEST_P(ContGeofenceSourceTest, StaysTriggeredWhileHolding)
{
	enableContGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime first_hold = reposition().timestamp;
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	EXPECT_TRUE(sourceTriggered(checkAt(inside())));
	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_EQ(reposition().timestamp, first_hold);

	// Leaving Hold releases it, and the next breach holds again.
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_FALSE(contingencyBreached(checkAt(inside())));
	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_GT(reposition().timestamp, first_hold);
}

TEST_P(ContGeofenceSourceTest, CommandedTargetsMustStayInside)
{
	enableContGeofence();
	checkAt(inside());

	EXPECT_TRUE(targetAllowed(inside()));
	EXPECT_FALSE(targetAllowed(between()));
	EXPECT_FALSE(targetAllowed(outside()));

	// Warning and None do not restrict targets, the emergency geofence still does.
	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_NONE}) {
		setParam("GF_CONT_ACTION", action);
		EXPECT_TRUE(targetAllowed(between()));
		EXPECT_FALSE(targetAllowed(outside()));
	}

	// Each level follows its own action.
	setParam("GF_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_NONE));
	setParam("GF_CONT_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	EXPECT_FALSE(targetAllowed(between()));
}

INSTANTIATE_TEST_SUITE_P(Sources, ContGeofenceSourceTest,
			 ::testing::Values(Source::HomeDistance, Source::Plan),
			 ::testing::PrintToStringParamName());
