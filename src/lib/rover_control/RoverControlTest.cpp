/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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

/******************************************************************
 * Test code for the Pure Pursuit algorithm
 * Run this test only using "make tests TESTFILTER=RoverControl"
******************************************************************/

#include <gtest/gtest.h>
#include "RoverControl.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

TEST(calcWaypointTransitionAngle, invalidInputs)
{
	Vector2f prev_wp_ned(NAN, NAN);
	Vector2f curr_wp_ned(10.f, 10.f);
	Vector2f next_wp_ned(10.f, 10.f);
	float prevInvalid = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	prev_wp_ned = Vector2f(10.f, 10.f);
	curr_wp_ned = Vector2f(NAN, NAN);
	float currInvalid = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	curr_wp_ned = Vector2f(10.f, 10.f);
	next_wp_ned = Vector2f(NAN, NAN);
	float nextInvalid = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FALSE(PX4_ISFINITE(prevInvalid));
	EXPECT_FALSE(PX4_ISFINITE(currInvalid));
	EXPECT_FALSE(PX4_ISFINITE(nextInvalid));

}

TEST(calcWaypointTransitionAngle, validInputs)
{
	// P -- C -- N
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(20.f, 0.f);
	const float angle1 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FLOAT_EQ(angle1, M_PI_F);

	/**
	 *	    N
	 *	  /
	 * P -- C
	 */
	prev_wp_ned = Vector2f(0.f, 0.f);
	curr_wp_ned = Vector2f(10.f, 0.f);
	next_wp_ned = Vector2f(20.f, 10.f);
	const float angle2 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FLOAT_EQ(angle2, M_PI_F - M_PI_4_F);

	/**
	 *	N
	 *	|
	 * P -- C
	 */
	prev_wp_ned = Vector2f(0.f, 0.f);
	curr_wp_ned = Vector2f(10.f, 0.f);
	next_wp_ned = Vector2f(10.f, 10.f);
	const float angle3 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FLOAT_EQ(angle3, M_PI_2_F);

	/**
	 * N
	 *    \
	 * P -- C
	 */
	prev_wp_ned = Vector2f(0.f, 0.f);
	curr_wp_ned = Vector2f(10.f, 0.f);
	next_wp_ned = Vector2f(0.f, 10.f);
	const float angle4 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FLOAT_EQ(angle4, M_PI_4_F);

	// P/C -- N
	prev_wp_ned = Vector2f(0.f, 0.f);
	curr_wp_ned = Vector2f(0.f, 0.f);
	next_wp_ned = Vector2f(10.f, 0.f);
	const float angle5 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FALSE(PX4_ISFINITE(angle5));

	// P -- C/N
	prev_wp_ned = Vector2f(0.f, 0.f);
	curr_wp_ned = Vector2f(10.f, 0.f);
	next_wp_ned = Vector2f(10.f, 0.f);
	const float angle6 = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);
	EXPECT_FALSE(PX4_ISFINITE(angle6));
}

static vehicle_local_position_s localPosition(const float vx, const float vy, const float heading,
		const hrt_abstime timestamp)
{
	vehicle_local_position_s vehicle_local_position{};
	vehicle_local_position.timestamp = timestamp;
	vehicle_local_position.v_xy_valid = true;
	vehicle_local_position.vx = vx;
	vehicle_local_position.vy = vy;
	vehicle_local_position.heading = heading;
	vehicle_local_position.evh = 0.1f;
	return vehicle_local_position;
}

TEST(DriftAngleEstimator, zeroUntilEstimated)
{
	RoverControl::DriftAngleEstimator drift_angle_estimator;
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);

	// Too slow, backwards, without velocity or without heading the velocity direction says nothing about the drift
	drift_angle_estimator.update(localPosition(0.3f, 0.1f, 0.f, 1_s));
	drift_angle_estimator.update(localPosition(-2.f, 0.5f, 0.f, 2_s));
	vehicle_local_position_s no_velocity = localPosition(2.f, 0.5f, 0.f, 3_s);
	no_velocity.v_xy_valid = false;
	drift_angle_estimator.update(no_velocity);
	drift_angle_estimator.update(localPosition(2.f, 0.5f, NAN, 4_s));
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);

	// Forward speed below the velocity uncertainty
	vehicle_local_position_s uncertain = localPosition(2.f, 0.5f, 0.f, 5_s);
	uncertain.evh = 1.f;
	drift_angle_estimator.update(uncertain);
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, angleFromHeadingToCourse)
{
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// Heading north, moving north with a drift to the east: the course is to the right of the heading
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);

	// The same drift relative to a heading east
	RoverControl::DriftAngleEstimator rotated;
	rotated.update(localPosition(-0.2f, 2.f, M_PI_2_F, 1_s));
	EXPECT_NEAR(rotated.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);

	// Limited
	RoverControl::DriftAngleEstimator limited;
	limited.update(localPosition(1.f, 5.f, 0.f, 1_s));
	EXPECT_FLOAT_EQ(limited.getDriftAngle(), RoverControl::DriftAngleEstimator::MAX_DRIFT_ANGLE);
}

TEST(DriftAngleEstimator, filteredAndReset)
{
	RoverControl::DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s));
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);

	// A new drift is approached, not jumped to
	hrt_abstime timestamp = 1_s;

	for (int i = 0; i < 10; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	const float drift_angle = atan2f(0.4f, 2.f);
	EXPECT_GT(drift_angle_estimator.getDriftAngle(), 0.f);
	EXPECT_LT(drift_angle_estimator.getDriftAngle(), drift_angle);

	for (int i = 0; i < 1000; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), drift_angle, 1e-3f);

	drift_angle_estimator.reset();
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}
