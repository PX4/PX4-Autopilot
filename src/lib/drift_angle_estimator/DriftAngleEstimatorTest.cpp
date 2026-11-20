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
 * Test code for the DriftAngleEstimator
 * Run this test only using "make tests TESTFILTER=DriftAngleEstimator"
 */

#include <gtest/gtest.h>
#include "DriftAngleEstimator.hpp"

using namespace time_literals;

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

TEST(DriftAngleEstimator, zeroBeforeFirstUpdate)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: no update has happened yet
	const float drift_angle = drift_angle_estimator.getDriftAngle();

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle, 0.f);
}

TEST(DriftAngleEstimator, ignoresLowSpeed)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves too slowly for the velocity direction to say anything about the drift
	drift_angle_estimator.update(localPosition(0.3f, 0.1f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresReverse)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover drives backwards
	drift_angle_estimator.update(localPosition(-2.f, 0.5f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresInvalidVelocity)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the horizontal velocity is flagged invalid
	vehicle_local_position_s no_velocity = localPosition(2.f, 0.5f, 0.f, 1_s);
	no_velocity.v_xy_valid = false;
	drift_angle_estimator.update(no_velocity);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresNanHeading)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the heading is NaN
	drift_angle_estimator.update(localPosition(2.f, 0.5f, NAN, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresUncertainVelocity)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the speed is below the velocity uncertainty
	vehicle_local_position_s uncertain = localPosition(2.f, 0.5f, 0.f, 1_s);
	uncertain.evh = 1.f;
	drift_angle_estimator.update(uncertain);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingNorth)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads north and moves north with a drift to the east
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));

	// THEN: the drift angle is the angle from the heading to the course, positive since the course is to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingEast)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads east with the same drift relative to its heading
	drift_angle_estimator.update(localPosition(-0.2f, 2.f, M_PI_2_F, 1_s));

	// THEN: the drift angle is the same as when heading north
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, limited)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves mostly sideways
	drift_angle_estimator.update(localPosition(1.f, 5.f, 0.f, 1_s));

	// THEN: the drift angle is limited
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), DriftAngleEstimator::MAX_DRIFT_ANGLE);
}

TEST(DriftAngleEstimator, ignoresNanExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the expected direction of travel is NaN
	drift_angle_estimator.update(localPosition(2.f, 0.5f, 0.f, 1_s), NAN);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresMovementAcrossExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle is expected to move to its right but moves forwards
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s), M_PI_2_F);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, driftToTheRightExpectedRight)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads north, is expected to move to its right (east) and moves east with a drift to the south
	drift_angle_estimator.update(localPosition(-0.2f, 2.f, 0.f, 1_s), M_PI_2_F);

	// THEN: the drift angle is the angle from the expected direction to the course, positive since the course is to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, driftToTheRightExpectedBackwards)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads north, is expected to move backwards (south) and moves south with a drift to the west
	drift_angle_estimator.update(localPosition(-2.f, -0.2f, 0.f, 1_s), M_PI_F);

	// THEN: the drift angle is positive since the course is to the right of the expected direction
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, noDriftAlongDiagonalExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads east, is expected to move forwards left (north east) and moves exactly north east
	drift_angle_estimator.update(localPosition(2.f, 2.f, M_PI_2_F, 1_s), -M_PI_4_F);

	// THEN: there is no drift
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), 0.f, 1e-5f);
}

TEST(DriftAngleEstimator, approachesNewDrift)
{
	// GIVEN: a drift angle estimator that has seen the rover drive straight
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s));
	ASSERT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
	hrt_abstime timestamp = 1_s;

	// WHEN: a new drift appears for 0.1 s
	for (int i = 0; i < 10; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	// THEN: the drift angle moves towards the new drift without jumping to it
	EXPECT_GT(drift_angle_estimator.getDriftAngle(), 0.f);
	EXPECT_LT(drift_angle_estimator.getDriftAngle(), atan2f(0.4f, 2.f));
}

TEST(DriftAngleEstimator, convergesToNewDrift)
{
	// GIVEN: a drift angle estimator that has seen the rover drive straight
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s));
	ASSERT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
	hrt_abstime timestamp = 1_s;

	// WHEN: a new drift persists for 10 s
	for (int i = 0; i < 1000; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	// THEN: the drift angle has converged to the new drift
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.4f, 2.f), 1e-3f);
}

TEST(DriftAngleEstimator, reset)
{
	// GIVEN: a drift angle estimator with a non-zero drift angle
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));
	ASSERT_GT(drift_angle_estimator.getDriftAngle(), 0.f);

	// WHEN: it is reset
	drift_angle_estimator.reset();

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}
