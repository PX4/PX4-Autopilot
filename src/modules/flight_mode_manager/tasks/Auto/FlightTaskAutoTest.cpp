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
 * @file FlightTaskAutoTest.cpp
 *
 * Tests for how FlightTaskAuto takes the position setpoint triplet over from the navigator, in particular the
 * velocity constraint on the next setpoint, which the navigator may republish on its own without touching the
 * waypoints.
 */

#include <gtest/gtest.h>
#include "FlightTaskAuto.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>

using matrix::Vector3f;

class FlightTaskAutoTestPeer : public FlightTaskAuto
{
public:
	Vector3f nextVelocityConstraint() const { return _next_velocity_constraint; }
	float nextAcceptanceRadius() const { return _next_acceptance_radius; }
};

class FlightTaskAutoTest : public ::testing::Test
{
protected:
	static constexpr double kRefLat{47.0};
	static constexpr double kRefLon{8.0};

	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		publishLocalPosition();
		_triplet = makeTriplet();
	}

	/* A valid local position with a global reference, so that the task accepts the triplet */
	void publishLocalPosition()
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = local_position.z_valid = true;
		local_position.v_xy_valid = local_position.v_z_valid = true;
		local_position.xy_global = local_position.z_global = true;
		local_position.ref_lat = kRefLat;
		local_position.ref_lon = kRefLon;
		local_position.ref_alt = 0.f;
		local_position.ref_timestamp = 1;
		local_position.heading_good_for_control = true;
		_local_position_pub.publish(local_position);
	}

	static position_setpoint_s makePositionSetpoint(double lat, double lon)
	{
		position_setpoint_s setpoint{};
		setpoint.timestamp = hrt_absolute_time();
		setpoint.valid = true;
		setpoint.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		setpoint.lat = lat;
		setpoint.lon = lon;
		setpoint.alt = 100.f;
		setpoint.yaw = NAN;
		setpoint.cruising_speed = NAN;
		setpoint.acceptance_radius = 5.f;
		Vector3f(NAN, NAN, NAN).copyTo(setpoint.velocity_constraint);
		return setpoint;
	}

	/* Current at the reference, next a bit north of it, no constraint known yet */
	static position_setpoint_triplet_s makeTriplet()
	{
		position_setpoint_triplet_s triplet{};
		triplet.timestamp = hrt_absolute_time();
		triplet.current = makePositionSetpoint(kRefLat, kRefLon);
		triplet.next = makePositionSetpoint(kRefLat + 0.001, kRefLon);
		triplet.previous.valid = false;
		return triplet;
	}

	void publishTripletAndUpdate()
	{
		_triplet.timestamp = hrt_absolute_time();
		_triplet_pub.publish(_triplet);
		EXPECT_TRUE(_task.updateInitialize());
	}

	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<position_setpoint_triplet_s> _triplet_pub{ORB_ID(position_setpoint_triplet)};
	position_setpoint_triplet_s _triplet{};
	FlightTaskAutoTestPeer _task{};
};

TEST_F(FlightTaskAutoTest, TakesTheNextVelocityConstraintFromTheTriplet)
{
	// GIVEN: a triplet whose next setpoint carries a velocity constraint
	const Vector3f constraint{3.f, 0.f, 0.f};
	constraint.copyTo(_triplet.next.velocity_constraint);
	_triplet.next.acceptance_radius = 7.f;

	// WHEN: the task evaluates it
	publishTripletAndUpdate();

	// THEN: the constraint and the acceptance radius of next are the ones the planner gets
	EXPECT_EQ(_task.nextVelocityConstraint(), constraint);
	EXPECT_FLOAT_EQ(_task.nextAcceptanceRadius(), 7.f);
}

TEST_F(FlightTaskAutoTest, ConstraintOnlyRepublishIsTakenOver)
{
	// GIVEN: the triplet was evaluated while the navigator did not know the constraint yet
	publishTripletAndUpdate();
	ASSERT_FALSE(_task.nextVelocityConstraint().isAllFinite());

	// WHEN: the navigator republishes the same waypoints with only the constraint of next filled in, as it does
	// once its dataman cache has been loaded after activation
	const Vector3f constraint{0.f, 4.f, 0.f};
	constraint.copyTo(_triplet.next.velocity_constraint);
	publishTripletAndUpdate();

	// THEN: the planner gets the constraint although no waypoint changed
	EXPECT_EQ(_task.nextVelocityConstraint(), constraint);
}

TEST_F(FlightTaskAutoTest, ConstraintIsDroppedWhenNextBecomesInvalid)
{
	// GIVEN: a constraint was taken over
	Vector3f(3.f, 0.f, 0.f).copyTo(_triplet.next.velocity_constraint);
	publishTripletAndUpdate();
	ASSERT_TRUE(_task.nextVelocityConstraint().isAllFinite());

	// WHEN: the navigator invalidates next, e.g. to make the vehicle stop at current
	_triplet.next.valid = false;
	publishTripletAndUpdate();

	// THEN: the constraint is unknown again, the planner assumes a stop at current
	EXPECT_FALSE(_task.nextVelocityConstraint().isAllFinite());
	EXPECT_FLOAT_EQ(_task.nextAcceptanceRadius(), _triplet.current.acceptance_radius);
}

TEST_F(FlightTaskAutoTest, ConstraintIsIgnoredWhileLoitering)
{
	// GIVEN: the current setpoint is a loiter, next has a constraint anyway
	_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	Vector3f(3.f, 0.f, 0.f).copyTo(_triplet.next.velocity_constraint);

	// WHEN: the task evaluates it
	publishTripletAndUpdate();

	// THEN: a loiter is not left along the mission, no constraint
	EXPECT_FALSE(_task.nextVelocityConstraint().isAllFinite());
}
