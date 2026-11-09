/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
#include "StickAccelerationXY.hpp"

#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/takeoff_status.h>

using namespace matrix;

static constexpr float kDt = 0.01f;

class StickAccelerationXYTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		takeoff_status_s takeoff_status{};
		takeoff_status.tilt_limit = math::radians(35.f);
		_takeoff_status_pub.publish(takeoff_status);
	}

	// One loop at the given forward stick, with the speed limit set as a flight task does
	void step(float stick, float velocity_limit)
	{
		_stick_acceleration_xy.setVelocityLimit(velocity_limit);
		_stick_acceleration_xy.generateSetpoints(Vector2f(stick, 0.f), 0.f, 0.f, Vector3f(), Vector2f(NAN, NAN), kDt);
		Vector3f position;
		_stick_acceleration_xy.getSetpoints(position, _velocity, _acceleration);
	}

	uORB::Publication<takeoff_status_s> _takeoff_status_pub{ORB_ID(takeoff_status)};
	StickAccelerationXY _stick_acceleration_xy{nullptr};
	Vector3f _velocity;
	Vector3f _acceleration;
};

// Seconds of full forward stick until the speed reaches the target, and the highest speed on the way. The speed is capped
// by a limit, or by a constraint that also scales the stick acceleration down.
static void fullStick(float speed, bool as_constraint, float target, float &time_to_target, float &max_speed)
{
	StickAccelerationXY stick_acceleration_xy{nullptr};
	Vector3f position, velocity, acceleration;
	time_to_target = NAN;
	max_speed = 0.f;

	// hover first, so a constraint has slewed down to its value as it would have in flight
	for (int i = -500; i < 1000; i++) {
		const float stick = (i < 0) ? 0.f : 1.f;

		if (as_constraint) {
			stick_acceleration_xy.setVelocityConstraint(speed);

		} else {
			stick_acceleration_xy.setVelocityLimit(speed);
		}

		stick_acceleration_xy.generateSetpoints(Vector2f(stick, 0.f), 0.f, 0.f, Vector3f(), Vector2f(NAN, NAN), kDt);
		stick_acceleration_xy.getSetpoints(position, velocity, acceleration);
		max_speed = fmaxf(max_speed, velocity.xy().norm());

		if ((i >= 0) && !PX4_ISFINITE(time_to_target) && (velocity(0) >= target)) {
			time_to_target = i * kDt;
		}
	}
}

TEST_F(StickAccelerationXYTest, LimitKeepsStickAcceleration)
{
	// GIVEN: the default 3 m/s^2, 8 m/s^3 and 10 m/s, and limits such as optical flow sets from 0.08 to 1.4 m
	for (const float limit : {0.3f, 1.f, 2.f, 5.f}) {
		// WHEN: the stick is held fully forward, under the limit and under a constraint of the same speed
		float time_limited, max_limited, time_constrained, max_constrained;
		fullStick(limit, false, 0.95f * limit, time_limited, max_limited);
		fullStick(limit, true, 0.95f * limit, time_constrained, max_constrained);

		// THEN: the speed never exceeds the limit
		EXPECT_LE(max_limited, limit + 1e-4f) << "limit " << limit;
		// THEN: it gets there sooner than under the constraint's scaled-down acceleration, by far near the ground
		EXPECT_LT(time_limited, 0.67f * time_constrained) << "limit " << limit;
	}
}

TEST_F(StickAccelerationXYTest, LimitScalesStickSpeed)
{
	// GIVEN: a 1 m/s limit

	// WHEN: the stick is held halfway forward
	for (int i = 0; i < 300; i++) {
		step(.5f, 1.f);
	}

	// THEN: it settles at half the limit, so the whole stick travel maps onto the available speed
	EXPECT_NEAR(_velocity(0), .5f, .02f);
}

TEST_F(StickAccelerationXYTest, BrakesFromTheLimit)
{
	// GIVEN: flying at a 1 m/s limit with the stick fully forward
	for (int i = 0; i < 300; i++) {
		step(1.f, 1.f);
	}

	// WHEN: the stick is released
	float time_to_stop = NAN;

	for (int i = 0; i < 300; i++) {
		step(0.f, 1.f);

		// THEN: it brakes right away, without first unwinding acceleration that was cut at the limit
		if (i == 1) {
			EXPECT_LT(_acceleration(0), 0.f);
		}

		if (!PX4_ISFINITE(time_to_stop) && (_velocity(0) < 0.05f)) {
			time_to_stop = i * kDt;
		}
	}

	// THEN: it stops within a second
	EXPECT_LT(time_to_stop, 1.f);
}

TEST_F(StickAccelerationXYTest, LimitBelowCurrentSpeedComesDownGradually)
{
	// GIVEN: flying fast with the stick fully forward and no limit
	for (int i = 0; i < 1000; i++) {
		step(1.f, INFINITY);
	}

	ASSERT_GT(_velocity(0), 5.f);

	// WHEN: a 2 m/s limit appears, as when optical flow becomes the only horizontal aiding
	float max_deceleration = 0.f;
	float previous_speed = _velocity(0);

	for (int i = 0; i < 500; i++) {
		step(1.f, 2.f);
		max_deceleration = fmaxf(max_deceleration, (previous_speed - _velocity(0)) / kDt);
		previous_speed = _velocity(0);
	}

	// THEN: the setpoint comes down at the default 3 m/s^2 instead of in one step, and settles at the limit
	EXPECT_LT(max_deceleration, 1.05f * 3.f);
	EXPECT_NEAR(_velocity(0), 2.f, .01f);
}
