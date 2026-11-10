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
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include "navigator.h"
#include "loiter.h"
#include "support/navigator_dataman_test.h"

namespace
{
constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kAlt = 500.f;

// Runs exactly one real Navigator iteration per runOnce() without a worker thread, stopping at the triplet publication
class NavigatorIteration : public Navigator, public uORB::SubscriptionCallback
{
public:
	NavigatorIteration() : SubscriptionCallback(ORB_ID(position_setpoint_triplet)) {}

	void runOnce()
	{
		_task_should_exit.store(false);
		set_position_setpoint_triplet_updated();
		Navigator::run();
	}

	void call(unsigned) override { request_stop(); }
};
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

		// A Navigator created in a test still reads the last command an earlier test published
		vehicle_command_s no_op{};
		no_op.timestamp = hrt_absolute_time();
		no_op.command = vehicle_command_s::VEHICLE_CMD_CUSTOM_0;
		_command_pub.publish(no_op);
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

	// A reposition as navigator fills it from a command
	void repositionTo(double lat, double lon, float alt, Navigator::RepositionSource source_xy,
			  Navigator::RepositionSource source_z)
	{
		position_setpoint_triplet_s *reposition = _navigator.get_reposition_triplet();
		reposition->current.valid = true;
		reposition->current.timestamp = hrt_absolute_time();
		reposition->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		reposition->current.lat = lat;
		reposition->current.lon = lon;
		reposition->current.alt = alt;
		*_navigator.get_reposition_sources() = {source_xy, source_z};
	}

	// One navigator cycle in Hold, in the order Navigator runs it
	void runCycle()
	{
		_loiter.followPositionResets();
		_loiter.on_active();
	}

	// Publish Hold and the estimate as the test has set them, then run one Navigator iteration
	void runNavigator(NavigatorIteration &navigator)
	{
		vehicle_status_s status = *_navigator.get_vstatus();
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		status.timestamp = hrt_absolute_time();
		ASSERT_TRUE(_status_pub.publish(status));
		ASSERT_TRUE(_local_pos_pub.publish(*_navigator.get_local_position()));
		ASSERT_TRUE(_global_pos_pub.publish(*_navigator.get_global_position()));
		ASSERT_TRUE(_land_detected_pub.publish(*_navigator.get_land_detected()));
		navigator.runOnce();
	}

	// A reposition command as QGC sends it, NAN for an unset field
	static vehicle_command_s repositionCommand(double lat, double lon, float alt)
	{
		vehicle_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
		command.param1 = -1.f;
		command.param2 = 1.f;
		command.param4 = NAN;
		command.param5 = lat;
		command.param6 = lon;
		command.param7 = alt;
		return command;
	}

	const position_setpoint_s &currentSetpoint() { return _navigator.get_position_setpoint_triplet()->current; }

	static void expectShift(const position_setpoint_s &before, const position_setpoint_s &after, float delta_north,
				float delta_east, float delta_up)
	{
		float north = 0.f;
		float east = 0.f;
		get_vector_to_next_waypoint(before.lat, before.lon, after.lat, after.lon, &north, &east);
		EXPECT_NEAR(north, delta_north, 0.01f);
		EXPECT_NEAR(east, delta_east, 0.01f);
		EXPECT_NEAR(after.alt, before.alt + delta_up, 0.001f);
	}

	void expectSetpointShift(const position_setpoint_s &before, float delta_north, float delta_east, float delta_up)
	{
		expectShift(before, currentSetpoint(), delta_north, delta_east, delta_up);
	}

	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_local_position_s> _local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s> _global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
};

TEST_F(LoiterTest, holdFollowsEstimateReset)
{
	// GIVEN: a vehicle holding where it is
	_loiter.on_activation();
	runCycle();

	const position_setpoint_s hold = currentSetpoint();
	ASSERT_TRUE(hold.valid);

	// WHEN: the position estimate resets 1.5 m north and 0.8 m up, as on a switch of the GNSS receiver
	resetEstimate(1.5f, 0.f, -0.8f);
	runCycle();

	// THEN: the hold position moves with it, so that the vehicle stays where it is
	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(hold.lat, hold.lon, currentSetpoint().lat, currentSetpoint().lon, &north, &east);
	EXPECT_NEAR(north, 1.5f, 0.01f);
	EXPECT_NEAR(east, 0.f, 0.01f);
	EXPECT_NEAR(currentSetpoint().alt, hold.alt + 0.8f, 0.001f);
}

TEST_F(LoiterTest, fixedWingHoldFollowsOnlyAltitudeReset)
{
	// GIVEN: a fixed-wing holding where it is
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	_loiter.on_activation();
	runCycle();

	const position_setpoint_s hold = currentSetpoint();
	ASSERT_TRUE(hold.valid);

	// WHEN: the position estimate resets, as when GNSS returns after dead reckoning on wind
	resetEstimate(150.f, -80.f, -0.8f);
	runCycle();

	// THEN: the orbit stays where Hold was engaged, the altitude moves with the reset
	expectSetpointShift(hold, 0.f, 0.f, 0.8f);
}

TEST_F(LoiterTest, holdIgnoresMovedOrigin)
{
	// GIVEN: a vehicle holding where it is
	_loiter.on_activation();
	runCycle();

	const position_setpoint_s hold = currentSetpoint();

	// WHEN: the origin of the local position moves, which resets the local but not the global position
	resetEstimate(20.f, 10.f, 5.f);
	_navigator.get_local_position()->ref_timestamp = 2;
	_navigator.get_local_position()->ref_alt = 405.f;
	runCycle();

	// THEN: the hold position stays
	EXPECT_DOUBLE_EQ(currentSetpoint().lat, hold.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, hold.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, hold.alt);
}

TEST_F(LoiterTest, holdIgnoresMovedAltitudeOrigin)
{
	// GIVEN: a vehicle holding where it is
	_loiter.on_activation();
	runCycle();

	const position_setpoint_s hold = currentSetpoint();

	// WHEN: the estimate resets horizontally while only the altitude origin moves, which EKF2 does without a new
	// ref_timestamp when it sets a new origin altitude
	resetEstimate(1.5f, 0.5f, 5.f);
	_navigator.get_local_position()->ref_alt = 405.f;
	runCycle();

	// THEN: the hold position moves with the horizontal reset and keeps its altitude
	expectSetpointShift(hold, 1.5f, 0.5f, 0.f);
}

TEST_F(LoiterTest, repositionTargetIgnoresEstimateReset)
{
	// GIVEN: a vehicle holding, then sent to a reposition target
	_loiter.on_activation();
	runCycle();

	position_setpoint_triplet_s *reposition = _navigator.get_reposition_triplet();
	reposition->current.valid = true;
	reposition->current.timestamp = hrt_absolute_time();
	reposition->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	reposition->current.lat = kLat + 0.001;
	reposition->current.lon = kLon;
	reposition->current.alt = kAlt + 10.f;
	runCycle();

	const position_setpoint_s target = currentSetpoint();
	ASSERT_DOUBLE_EQ(target.lat, kLat + 0.001);

	// WHEN: the position estimate resets
	resetEstimate(1.5f, 0.f, -0.8f);
	runCycle();

	// THEN: the target stays where it was commanded
	EXPECT_DOUBLE_EQ(currentSetpoint().lat, target.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, target.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, target.alt);
}

TEST_F(LoiterTest, pauseFollowsEstimateReset)
{
	// GIVEN: a vehicle paused into Hold. Navigator turns QGC's Pause, which has no coordinates, into a reposition at
	// the vehicle's position.
	repositionTo(kLat, kLon, kAlt, Navigator::RepositionSource::Vehicle, Navigator::RepositionSource::Vehicle);
	_loiter.on_activation();
	runCycle();

	const position_setpoint_s hold = currentSetpoint();
	ASSERT_TRUE(hold.valid);

	// WHEN: the position estimate resets 1.5 m north and 0.8 m up
	resetEstimate(1.5f, 0.f, -0.8f);
	runCycle();

	// THEN: the hold position moves with it, so that the vehicle stays where it is
	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(hold.lat, hold.lon, currentSetpoint().lat, currentSetpoint().lon, &north, &east);
	EXPECT_NEAR(north, 1.5f, 0.01f);
	EXPECT_NEAR(east, 0.f, 0.01f);
	EXPECT_NEAR(currentSetpoint().alt, hold.alt + 0.8f, 0.001f);
}

TEST_F(LoiterTest, altitudeChangeKeepsFollowingHorizontally)
{
	// GIVEN: a vehicle holding where it is, then sent to a new altitude only
	_loiter.on_activation();
	runCycle();
	repositionTo(currentSetpoint().lat, currentSetpoint().lon, kAlt + 10.f, Navigator::RepositionSource::Setpoint,
		     Navigator::RepositionSource::Command);
	runCycle();

	const position_setpoint_s target = currentSetpoint();
	ASSERT_FLOAT_EQ(target.alt, kAlt + 10.f);

	// WHEN: the position estimate resets 1.5 m north and 0.8 m up
	resetEstimate(1.5f, 0.f, -0.8f);
	runCycle();

	// THEN: the hold position moves with it horizontally, the commanded altitude stays
	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(target.lat, target.lon, currentSetpoint().lat, currentSetpoint().lon, &north, &east);
	EXPECT_NEAR(north, 1.5f, 0.01f);
	EXPECT_NEAR(east, 0.f, 0.01f);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, target.alt);
}

TEST_F(LoiterTest, navigatorAppliesResetBeforeAltitudeCommand)
{
	NavigatorIteration navigator;
	ASSERT_TRUE(navigator.registerCallback());
	runNavigator(navigator);
	const position_setpoint_s hold = navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(hold.valid);

	// Queue both inputs before the next iteration, so command processing sees the new reset counter.
	resetEstimate(1.5f, 0.5f, -0.8f);
	ASSERT_TRUE(_command_pub.publish(repositionCommand(NAN, NAN, kAlt + 10.f)));
	runNavigator(navigator);

	const position_setpoint_s target = navigator.get_position_setpoint_triplet()->current;
	expectShift(hold, target, 1.5f, 0.5f, 10.f);

	// The next iteration must preserve the corrected target without applying the same reset twice.
	runNavigator(navigator);
	expectShift(target, navigator.get_position_setpoint_triplet()->current, 0.f, 0.f, 0.f);
}

TEST_F(LoiterTest, navigatorPauseFollowsEstimateReset)
{
	// GIVEN: a vehicle paused into Hold by a reposition without coordinates or altitude, as QGC sends it
	NavigatorIteration navigator;
	ASSERT_TRUE(navigator.registerCallback());
	runNavigator(navigator);
	ASSERT_TRUE(_command_pub.publish(repositionCommand(NAN, NAN, NAN)));
	runNavigator(navigator);
	ASSERT_FALSE(navigator.get_reposition_triplet()->current.valid);
	const position_setpoint_s hold = navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(hold.valid);

	// WHEN: the position estimate resets
	resetEstimate(1.5f, 0.5f, -0.8f);
	runNavigator(navigator);

	// THEN: the hold position moves with it on both axes, so that the vehicle stays where it is
	expectShift(hold, navigator.get_position_setpoint_triplet()->current, 1.5f, 0.5f, 0.8f);
}

TEST_F(LoiterTest, navigatorGoToKeepsCommandedTarget)
{
	// GIVEN: a vehicle in Hold sent to a location and altitude
	NavigatorIteration navigator;
	ASSERT_TRUE(navigator.registerCallback());
	runNavigator(navigator);
	ASSERT_TRUE(_command_pub.publish(repositionCommand(kLat + 0.001, kLon, kAlt + 10.f)));
	runNavigator(navigator);
	ASSERT_FALSE(navigator.get_reposition_triplet()->current.valid);
	const position_setpoint_s target = navigator.get_position_setpoint_triplet()->current;
	ASSERT_DOUBLE_EQ(target.lat, kLat + 0.001);

	// WHEN: the position estimate resets
	resetEstimate(1.5f, 0.5f, -0.8f);
	runNavigator(navigator);

	// THEN: the target stays where it was commanded
	expectShift(target, navigator.get_position_setpoint_triplet()->current, 0.f, 0.f, 0.f);
}

TEST_F(LoiterTest, navigatorChangeAltitudeFollowsOnlyHorizontalReset)
{
	// GIVEN: a vehicle holding where it is, then sent to a new altitude
	NavigatorIteration navigator;
	ASSERT_TRUE(navigator.registerCallback());
	runNavigator(navigator);
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE;
	command.param1 = kAlt + 10.f;
	ASSERT_TRUE(_command_pub.publish(command));
	runNavigator(navigator);
	ASSERT_FALSE(navigator.get_reposition_triplet()->current.valid);
	const position_setpoint_s target = navigator.get_position_setpoint_triplet()->current;
	ASSERT_FLOAT_EQ(target.alt, kAlt + 10.f);

	// WHEN: the position estimate resets
	resetEstimate(1.5f, 0.5f, -0.8f);
	runNavigator(navigator);

	// THEN: the hold position moves with the horizontal reset, the commanded altitude stays
	expectShift(target, navigator.get_position_setpoint_triplet()->current, 1.5f, 0.5f, 0.f);
}

TEST_F(LoiterTest, horizontalCommandFollowsOnlyAltitudeReset)
{
	// A horizontal reposition without an altitude keeps the vehicle's estimated altitude.
	repositionTo(kLat + 0.001, kLon, kAlt, Navigator::RepositionSource::Command,
		     Navigator::RepositionSource::Vehicle);
	_loiter.on_activation();
	const position_setpoint_s target = currentSetpoint();

	resetEstimate(1.5f, 0.5f, -0.8f);
	runCycle();

	EXPECT_DOUBLE_EQ(currentSetpoint().lat, target.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, target.lon);
	EXPECT_NEAR(currentSetpoint().alt, target.alt + 0.8f, 0.001f);
}

TEST_F(LoiterTest, altitudeChangeKeepsCommandedHorizontalTarget)
{
	repositionTo(kLat + 0.001, kLon, kAlt, Navigator::RepositionSource::Command,
		     Navigator::RepositionSource::Command);
	_loiter.on_activation();

	// Updating altitude must preserve the reset behavior of the commanded horizontal target.
	repositionTo(currentSetpoint().lat, currentSetpoint().lon, kAlt + 10.f, Navigator::RepositionSource::Setpoint,
		     Navigator::RepositionSource::Command);
	runCycle();
	const position_setpoint_s target = currentSetpoint();

	resetEstimate(1.5f, 0.5f, -0.8f);
	runCycle();

	EXPECT_DOUBLE_EQ(currentSetpoint().lat, target.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, target.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, target.alt);
}

TEST_F(LoiterTest, freshVehicleTargetDoesNotApplyResetTwice)
{
	_loiter.on_activation();
	runCycle();
	const position_setpoint_s hold = currentSetpoint();

	// A pause built from the new estimate already includes this cycle's reset.
	resetEstimate(1.5f, 0.5f, -0.8f);
	vehicle_global_position_s &global_pos = *_navigator.get_global_position();
	add_vector_to_global_position(global_pos.lat, global_pos.lon, 1.5f, 0.5f, &global_pos.lat, &global_pos.lon);
	global_pos.alt += 0.8f;
	repositionTo(global_pos.lat, global_pos.lon, global_pos.alt, Navigator::RepositionSource::Vehicle,
		     Navigator::RepositionSource::Vehicle);
	runCycle();

	expectSetpointShift(hold, 1.5f, 0.5f, 0.8f);

	// Subsequent resets still move the hold target.
	resetEstimate(1.5f, 0.5f, -0.8f);
	runCycle();
	expectSetpointShift(hold, 3.f, 1.f, 1.6f);
}

TEST_F(LoiterTest, modeReentryDoesNotReusePreviousHoldResetPolicy)
{
	_loiter.on_activation();
	runCycle();
	_loiter.on_inactive();

	// Another mode leaves a commanded target, then an altitude-only reposition enters Hold again.
	position_setpoint_s &previous = _navigator.get_position_setpoint_triplet()->current;
	previous.lat = kLat + 0.001;
	repositionTo(previous.lat, previous.lon, kAlt + 10.f, Navigator::RepositionSource::Setpoint,
		     Navigator::RepositionSource::Command);
	_loiter.on_activation();
	const position_setpoint_s target = currentSetpoint();

	resetEstimate(1.5f, 0.5f, -0.8f);
	runCycle();

	EXPECT_DOUBLE_EQ(currentSetpoint().lat, target.lat);
	EXPECT_DOUBLE_EQ(currentSetpoint().lon, target.lon);
	EXPECT_FLOAT_EQ(currentSetpoint().alt, target.alt);
}

TEST_F(LoiterTest, geofenceHoldFollowsEstimateReset)
{
	// Configure a 100 m fence before constructing the Navigator that reads these parameters.
	const int32_t action = geofence_result_s::GF_ACTION_LOITER;
	const float maximum_distance = 100.f;
	ASSERT_EQ(param_set_no_notification(param_find("GF_ACTION"), &action), 0);
	ASSERT_EQ(param_set_no_notification(param_find("GF_MAX_HOR_DIST"), &maximum_distance), 0);
	Navigator navigator{};
	Loiter loiter{&navigator};
	*navigator.get_vstatus() = *_navigator.get_vstatus();
	*navigator.get_land_detected() = *_navigator.get_land_detected();
	*navigator.get_global_position() = *_navigator.get_global_position();
	*navigator.get_local_position() = *_navigator.get_local_position();

	home_position_s &home = *navigator.get_home_position();
	home.valid_hpos = true;
	home.valid_alt = true;
	home.alt = kAlt;
	add_vector_to_global_position(kLat, kLon, 200.f, 0.f, &home.lat, &home.lon);
	navigator.get_global_position()->timestamp = hrt_absolute_time();
	navigator.geofence_breach_check();

	// Use the producer's target without supplying any reset-source metadata in the test.
	ASSERT_TRUE(navigator.get_reposition_triplet()->current.valid);
	loiter.on_activation();
	const position_setpoint_s hold = navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(hold.valid);
	EXPECT_DOUBLE_EQ(hold.lat, kLat);
	EXPECT_DOUBLE_EQ(hold.lon, kLon);
	EXPECT_FLOAT_EQ(hold.alt, kAlt);

	vehicle_local_position_s &local_pos = *navigator.get_local_position();
	local_pos.delta_xy[0] = 1.5f;
	local_pos.delta_xy[1] = 0.5f;
	local_pos.xy_reset_counter++;
	local_pos.delta_z = -0.8f;
	local_pos.z_reset_counter++;
	loiter.followPositionResets();
	loiter.on_active();

	const position_setpoint_s &current = navigator.get_position_setpoint_triplet()->current;
	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(hold.lat, hold.lon, current.lat, current.lon, &north, &east);
	EXPECT_NEAR(north, 1.5f, 0.01f);
	EXPECT_NEAR(east, 0.5f, 0.01f);
	EXPECT_NEAR(current.alt, hold.alt + 0.8f, 0.001f);
}
