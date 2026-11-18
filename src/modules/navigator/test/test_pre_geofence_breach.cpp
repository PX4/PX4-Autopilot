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
		return navigator.geofence_allows_position(target) && navigator.pre_geofence_allows_target(target);
	}

	// Run the breach check now instead of waiting for its 5 Hz interval.
	static void breachCheck(Navigator &navigator)
	{
		navigator._last_geofence_check = 0;
		navigator.geofence_breach_check();
	}
};

// The pre-geofence and its action in Navigator. Home is at the origin, 400 m AMSL.
class PreGeofenceBreachTest : public navigator_test::GeofenceTestBase
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

	static bool geofenceBreached(const geofence_result_s &result)
	{
		return result.geofence_max_dist_triggered || result.geofence_max_alt_triggered
		       || result.geofence_custom_fence_triggered;
	}

	static bool preGeofenceBreached(const geofence_result_s &result)
	{
		return result.pre_geofence_max_dist_triggered || result.pre_geofence_max_alt_triggered
		       || result.pre_geofence_custom_fence_triggered;
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

TEST_F(PreGeofenceBreachTest, DisabledByDefault)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	setParam("GF_MAX_VER_DIST", 100.f);

	const geofence_result_s inside = checkAt({0.f, 490.f, kHomeAltitude + 90.f});
	EXPECT_FALSE(preGeofenceBreached(inside));
	EXPECT_EQ(inside.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(geofenceBreached(inside));
	EXPECT_FALSE(reposition().valid);
	EXPECT_TRUE(targetAllowed({0.f, 490.f, kHomeAltitude + 90.f}));

	const geofence_result_s outside = checkAt({0.f, 510.f, kAltitude});
	EXPECT_TRUE(outside.geofence_max_dist_triggered);
	EXPECT_FALSE(preGeofenceBreached(outside));
}

// Without a pre-geofence the Hold after a geofence breach works as before: at the breach position, latched
// while holding, and again on the next breach.
TEST_F(PreGeofenceBreachTest, GeofenceHoldWithoutPreGeofenceIsUnchanged)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	const Place inside{0.f, 490.f, kAltitude};

	ASSERT_TRUE(geofenceBreached(checkAt({0.f, 510.f, kAltitude})));
	ASSERT_TRUE(repositionedAt({0.f, 510.f, kAltitude}));
	const hrt_abstime first_hold = reposition().timestamp;

	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	EXPECT_TRUE(geofenceBreached(checkAt(inside)));
	EXPECT_EQ(reposition().timestamp, first_hold);

	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_FALSE(geofenceBreached(checkAt(inside)));
	ASSERT_TRUE(geofenceBreached(checkAt({0.f, 520.f, kAltitude})));
	EXPECT_GT(reposition().timestamp, first_hold);
}

TEST_F(PreGeofenceBreachTest, PreGeofenceLimitsNeedHome)
{
	EXPECT_FALSE(_fence.isHomeRequired());

	// The action alone does nothing while the pre-geofence is disabled.
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_RTL));
	EXPECT_FALSE(_fence.isHomeRequired());

	setParam("GF_PRE_MAX_VDIST", 80.f);
	EXPECT_TRUE(_fence.isHomeRequired());
	setParam("GF_PRE_MAX_VDIST", 0.f);
	setParam("GF_PRE_MAX_HDIST", 400.f);
	EXPECT_TRUE(_fence.isHomeRequired());

	// The margin is relative to the plan, so only its Return action needs Home.
	setParam("GF_PRE_MAX_HDIST", 0.f);
	setParam("GF_PRE_MARGIN", 100.f);
	EXPECT_TRUE(_fence.isHomeRequired());
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	EXPECT_FALSE(_fence.isHomeRequired());
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
// Return routes around the fences with the margin added, so that it does not trigger the pre-geofence action.
TEST_F(PreGeofenceBreachTest, ReturnKeepsTheMargin)
{
	const float without_margin = _navigator.geofence_avoidance_margin();
	setParam("GF_PRE_MARGIN", 100.f);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), without_margin + 100.f);

	setVehicle(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	EXPECT_FLOAT_EQ(_navigator.geofence_avoidance_margin(), _navigator.get_default_loiter_rad() + 100.f);
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

// A command without a position or an altitude keeps the vehicle's own, also outside the pre-geofence.
TEST_F(PreGeofenceBreachTest, TargetsKeepTheVehiclePositionAndAltitude)
{
	setParam("GF_PRE_MAX_HDIST", 400.f);
	setParam("GF_PRE_MAX_VDIST", 80.f);
	const Place vehicle{0.f, 420.f, kHomeAltitude + 90.f};
	ASSERT_TRUE(preGeofenceBreached(checkAt(vehicle)));

	EXPECT_TRUE(targetAllowed(vehicle));
	// Back inside horizontally, or down below the altitude limit.
	EXPECT_TRUE(targetAllowed({0.f, 300.f, vehicle.altitude}));
	EXPECT_TRUE(targetAllowed({vehicle.north, vehicle.east, kAltitude}));
	// Further out or up.
	EXPECT_FALSE(targetAllowed({0.f, 440.f, vehicle.altitude}));
	EXPECT_FALSE(targetAllowed({vehicle.north, vehicle.east, kHomeAltitude + 95.f}));
}

// What triggers the pre-geofence. Each source has its geofence twin further out.
enum class Source {
	HomeDistance, // GF_PRE_MAX_HDIST 400 m inside GF_MAX_HOR_DIST 500 m
	HomeAltitude, // GF_PRE_MAX_VDIST 80 m below GF_MAX_VER_DIST 100 m
	Margin, // GF_PRE_MARGIN 100 m inside a 1000 m square plan fence
};

static std::ostream &operator<<(std::ostream &os, Source source)
{
	switch (source) {
	case Source::HomeDistance: return os << "HomeDistance";

	case Source::HomeAltitude: return os << "HomeAltitude";

	case Source::Margin: return os << "Margin";
	}

	return os;
}

// Behaviour that every pre-geofence source shares.
class PreGeofenceSourceTest : public PreGeofenceBreachTest, public ::testing::WithParamInterface<Source>
{
protected:
	// extra moves the pre-geofence outwards without reaching the geofence.
	void enablePreGeofence(float extra = 0.f)
	{
		switch (GetParam()) {
		case Source::HomeDistance:
			setParam("GF_MAX_HOR_DIST", 500.f);
			setParam("GF_PRE_MAX_HDIST", 400.f + extra);
			break;

		case Source::HomeAltitude:
			setParam("GF_MAX_VER_DIST", 100.f);
			setParam("GF_PRE_MAX_VDIST", 80.f + extra);
			break;

		case Source::Margin:
			ASSERT_TRUE(loadFence(inclusionSquare()));
			setParam("GF_PRE_MARGIN", 100.f - extra);
			break;
		}
	}

	void disablePreGeofence()
	{
		setParam("GF_PRE_MAX_HDIST", 0.f);
		setParam("GF_PRE_MAX_VDIST", 0.f);
		setParam("GF_PRE_MARGIN", 0.f);
	}

	// Inside both, outside the pre-geofence only, and outside both.
	Place inside() const { return {0.f, 300.f, kAltitude}; }

	Place between() const
	{
		return GetParam() == Source::HomeAltitude ? Place{0.f, 300.f, kHomeAltitude + 90.f} : Place{0.f, 420.f, kAltitude};
	}

	Place outside() const
	{
		return GetParam() == Source::HomeAltitude ? Place{0.f, 300.f, kHomeAltitude + 110.f} : Place{0.f, 510.f, kAltitude};
	}

	// Only the source under test is reported.
	bool sourceTriggered(const geofence_result_s &result) const
	{
		switch (GetParam()) {
		case Source::HomeDistance:
			return result.pre_geofence_max_dist_triggered && !result.pre_geofence_max_alt_triggered
			       && !result.pre_geofence_custom_fence_triggered;

		case Source::HomeAltitude:
			return result.pre_geofence_max_alt_triggered && !result.pre_geofence_max_dist_triggered
			       && !result.pre_geofence_custom_fence_triggered;

		case Source::Margin:
			return result.pre_geofence_custom_fence_triggered && !result.pre_geofence_max_dist_triggered
			       && !result.pre_geofence_max_alt_triggered;
		}

		return false;
	}

	// Leave Hold and return inside, as if the pilot had taken over and flown back.
	void backInside()
	{
		setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		const geofence_result_s result = checkAt(inside());
		ASSERT_FALSE(preGeofenceBreached(result));
		ASSERT_FALSE(geofenceBreached(result));
		clearReposition();
	}
};

TEST_P(PreGeofenceSourceTest, TriggersBeforeTheGeofence)
{
	enablePreGeofence();

	const geofence_result_s clear = checkAt(inside());
	EXPECT_FALSE(preGeofenceBreached(clear));
	EXPECT_FALSE(geofenceBreached(clear));
	EXPECT_EQ(clear.pre_geofence_action, geofence_result_s::GF_ACTION_LOITER);

	const geofence_result_s pre = checkAt(between());
	EXPECT_TRUE(sourceTriggered(pre));
	EXPECT_FALSE(geofenceBreached(pre));

	const geofence_result_s both = checkAt(outside());
	EXPECT_TRUE(sourceTriggered(both));
	EXPECT_TRUE(geofenceBreached(both));
}

TEST_P(PreGeofenceSourceTest, PublishesThePreGeofenceAction)
{
	EXPECT_EQ(checkAt(inside()).pre_geofence_action, geofence_result_s::GF_ACTION_NONE);

	enablePreGeofence();
	EXPECT_EQ(checkAt(inside()).pre_geofence_action, geofence_result_s::GF_ACTION_LOITER);
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	EXPECT_EQ(checkAt(inside()).pre_geofence_action, geofence_result_s::GF_ACTION_LAND);

	// None and Terminate are not offered.
	for (const int32_t action : {geofence_result_s::GF_ACTION_NONE, geofence_result_s::GF_ACTION_TERMINATE}) {
		setParam("GF_PRE_ACTION", action);
		EXPECT_EQ(checkAt(inside()).pre_geofence_action, geofence_result_s::GF_ACTION_WARN);
	}
}

TEST_P(PreGeofenceSourceTest, HoldLoitersAtTheTriggerPosition)
{
	enablePreGeofence();
	checkAt(inside());
	EXPECT_FALSE(reposition().valid);

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_TRUE(repositionedAt(between()));
	EXPECT_EQ(reposition().type, position_setpoint_s::SETPOINT_TYPE_LOITER);
}

TEST_P(PreGeofenceSourceTest, FixedWingLoitersAroundTheTriggerPosition)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_TRUE(repositionedAt(between()));
	EXPECT_FLOAT_EQ(reposition().loiter_radius, _navigator.get_default_loiter_rad());
}

TEST_P(PreGeofenceSourceTest, HoldNeedsAnArmedVehicle)
{
	setVehicle(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::ARMING_STATE_DISARMED);
	enablePreGeofence();

	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_FALSE(reposition().valid);
}

TEST_P(PreGeofenceSourceTest, OtherActionsDoNotReposition)
{
	enablePreGeofence();

	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_RTL, geofence_result_s::GF_ACTION_LAND}) {
		SCOPED_TRACE(action);
		setParam("GF_PRE_ACTION", action);
		checkAt(inside());
		const geofence_result_s result = checkAt(between());
		EXPECT_TRUE(sourceTriggered(result));
		EXPECT_EQ(result.pre_geofence_action, action);
		EXPECT_FALSE(reposition().valid);
	}
}

// Commander takes the more severe action. Navigator must agree on whether the pre-geofence Hold is flown.
TEST_P(PreGeofenceSourceTest, OnlyAMoreSevereGeofenceActionReplacesTheHold)
{
	enablePreGeofence();

	for (const int32_t action : {geofence_result_s::GF_ACTION_RTL, geofence_result_s::GF_ACTION_TERMINATE, geofence_result_s::GF_ACTION_LAND}) {
		SCOPED_TRACE(action);
		setParam("GF_ACTION", action);
		const geofence_result_s result = checkAt(outside());
		EXPECT_TRUE(geofenceBreached(result));
		EXPECT_TRUE(sourceTriggered(result));
		EXPECT_FALSE(reposition().valid);
		backInside();
	}

	// A geofence that only warns, or does nothing, leaves the pre-geofence Hold in place.
	for (const int32_t action : {geofence_result_s::GF_ACTION_WARN, geofence_result_s::GF_ACTION_NONE}) {
		SCOPED_TRACE(action);
		setParam("GF_ACTION", action);
		ASSERT_TRUE(geofenceBreached(checkAt(outside())));
		EXPECT_TRUE(repositionedAt(outside()));
		backInside();
	}
}

// The geofence still gets its Hold when the pre-geofence only warns.
TEST_P(PreGeofenceSourceTest, GeofenceHoldAfterAPreGeofenceWarning)
{
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	ASSERT_FALSE(reposition().valid);

	ASSERT_TRUE(geofenceBreached(checkAt(outside())));
	EXPECT_TRUE(repositionedAt(outside()));
}

// A pilot who took over after the pre-geofence breach is still held at the geofence.
TEST_P(PreGeofenceSourceTest, GeofenceHoldAfterATakeover)
{
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime pre_geofence_hold = reposition().timestamp;
	ASSERT_NE(pre_geofence_hold, hrt_abstime{0});

	// Taking over leaves Hold; staying outside the pre-geofence starts no new Hold.
	setNavState(vehicle_status_s::NAVIGATION_STATE_POSCTL);
	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	ASSERT_EQ(reposition().timestamp, pre_geofence_hold);

	ASSERT_TRUE(geofenceBreached(checkAt(outside())));
	EXPECT_GT(reposition().timestamp, pre_geofence_hold);
	EXPECT_TRUE(repositionedAt(outside()));

	// The geofence Hold stays latched while the vehicle is holding.
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	EXPECT_TRUE(geofenceBreached(checkAt(between())));
}

// Breaching the geofence while holding must not move the loiter out to the breach position.
TEST_P(PreGeofenceSourceTest, HoldIsNotMovedByAGeofenceBreach)
{
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime hold = reposition().timestamp;
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	ASSERT_TRUE(geofenceBreached(checkAt(outside())));
	EXPECT_EQ(reposition().timestamp, hold);
	EXPECT_TRUE(repositionedAt(between()));
}

// The breach stays set while holding, so an orbit across the boundary does not restart the Hold.
TEST_P(PreGeofenceSourceTest, StaysTriggeredWhileHolding)
{
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	const hrt_abstime first_hold = reposition().timestamp;
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	EXPECT_TRUE(sourceTriggered(checkAt(inside())));
	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_EQ(reposition().timestamp, first_hold);

	// Leaving Hold releases it, and the next breach holds again.
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_FALSE(preGeofenceBreached(checkAt(inside())));
	EXPECT_TRUE(sourceTriggered(checkAt(between())));
	EXPECT_GT(reposition().timestamp, first_hold);
}

// Disabling the pre-geofence releases its breach without touching the Hold or the geofence breach.
TEST_P(PreGeofenceSourceTest, DisablingReleasesTheBreachWhileHolding)
{
	enablePreGeofence();

	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	const geofence_result_s before = checkAt(outside());
	ASSERT_TRUE(sourceTriggered(before));
	ASSERT_TRUE(geofenceBreached(before));
	const hrt_abstime hold = reposition().timestamp;

	disablePreGeofence();
	const geofence_result_s after = checkAt(between());
	EXPECT_EQ(after.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(preGeofenceBreached(after));
	EXPECT_TRUE(geofenceBreached(after));
	EXPECT_EQ(reposition().timestamp, hold);
	EXPECT_TRUE(repositionedAt(between()));
}

TEST_P(PreGeofenceSourceTest, DisablingReleasesTheBreachWithoutAPosition)
{
	enablePreGeofence();
	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

	disablePreGeofence();
	_navigator.get_global_position()->timestamp = 0;
	MissionBaseTestNavigator::breachCheck(_navigator);

	geofence_result_s result{};
	ASSERT_TRUE(_result_sub.update(&result));
	EXPECT_FALSE(preGeofenceBreached(result));
	EXPECT_EQ(result.pre_geofence_action, geofence_result_s::GF_ACTION_NONE);
	EXPECT_TRUE(repositionedAt(between()));
}

// A pre-geofence that is enabled again must not inherit the breach that was released, with its new action.
TEST_P(PreGeofenceSourceTest, EnablingAgainDoesNotRestoreAReleasedBreach)
{
	enablePreGeofence();
	ASSERT_TRUE(sourceTriggered(checkAt(between())));
	setNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	const hrt_abstime hold = reposition().timestamp;

	disablePreGeofence();
	ASSERT_FALSE(preGeofenceBreached(checkAt(between())));

	// The new limit contains the vehicle.
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LAND));
	enablePreGeofence(GetParam() == Source::HomeAltitude ? 15.f : 50.f);
	const geofence_result_s result = checkAt(between());
	EXPECT_EQ(result.pre_geofence_action, geofence_result_s::GF_ACTION_LAND);
	EXPECT_FALSE(preGeofenceBreached(result));
	EXPECT_EQ(reposition().timestamp, hold);
}

TEST_P(PreGeofenceSourceTest, CommandedTargetsMustStayInside)
{
	enablePreGeofence();
	checkAt(inside());

	EXPECT_TRUE(targetAllowed(inside()));
	EXPECT_FALSE(targetAllowed(between()));
	EXPECT_FALSE(targetAllowed(outside()));

	// A warning does not restrict targets, the geofence still does.
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_WARN));
	EXPECT_TRUE(targetAllowed(between()));
	EXPECT_FALSE(targetAllowed(outside()));

	// Each level follows its own action.
	setParam("GF_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_NONE));
	setParam("GF_PRE_ACTION", static_cast<int32_t>(geofence_result_s::GF_ACTION_LOITER));
	EXPECT_FALSE(targetAllowed(between()));
}

INSTANTIATE_TEST_SUITE_P(Sources, PreGeofenceSourceTest,
			 ::testing::Values(Source::HomeDistance, Source::HomeAltitude, Source::Margin),
			 ::testing::PrintToStringParamName());
