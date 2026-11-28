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

// Speed setpoint passed to getDriftAngle() in the tests that expect no estimate, so a zero drift angle shows that
// nothing was estimated rather than that no speed was known
static constexpr float SPEED_SETPOINT{2.f};

TEST(DriftAngleEstimator, zeroBeforeFirstUpdate)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: no update has happened yet
	const float drift_angle = drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT);

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle, 0.f);
}

TEST(DriftAngleEstimator, ignoresLowSpeed)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves too slowly for the velocity to say anything about the current
	drift_angle_estimator.update(localPosition(0.3f, 0.1f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, ignoresLowSpeedSetpoint)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves fast but only a low speed is commanded
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s), 0.f, 0.3f);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, ignoresReverse)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover drives backwards
	drift_angle_estimator.update(localPosition(-2.f, 0.5f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
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
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, ignoresNanHeading)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the heading is NaN
	drift_angle_estimator.update(localPosition(2.f, 0.5f, NAN, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
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
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, ignoresNanExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the expected direction of travel is NaN
	drift_angle_estimator.update(localPosition(2.f, 0.5f, 0.f, 1_s), NAN);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, ignoresMovementAcrossExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle is expected to move to its right but moves forwards
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s), M_PI_2_F);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(DriftAngleEstimator, zeroWithoutCourseSetpoint)
{
	// GIVEN: a drift angle estimator that has seen a current to the east
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s), 0.f, 2.f);

	// WHEN: the drift angle is requested without a course setpoint
	const float drift_angle = drift_angle_estimator.getDriftAngle(NAN, 2.f);

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle, 0.f);
}

TEST(DriftAngleEstimator, zeroAtStandstill)
{
	// GIVEN: a drift angle estimator that has seen a current to the east
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s), 0.f, 2.f);

	// WHEN: the drift angle is requested for a zero speed setpoint
	const float drift_angle = drift_angle_estimator.getDriftAngle(0.f, 0.f);

	// THEN: the drift angle is zero rather than limited
	EXPECT_FLOAT_EQ(drift_angle, 0.f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingNorth)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads north at 2 m/s and moves north with a current of 0.2 m/s to the east
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));

	// THEN: holding a course to the north needs a drift angle whose velocity across the course cancels the current,
	// positive since the current sets the rover to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(0.f), asinf(0.2f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingEast)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads east with the same current relative to its heading
	drift_angle_estimator.update(localPosition(-0.2f, 2.f, M_PI_2_F, 1_s));

	// THEN: holding a course to the east needs the same drift angle as holding north when heading north
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(M_PI_2_F), asinf(0.2f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, limited)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves mostly sideways
	drift_angle_estimator.update(localPosition(1.f, 5.f, 0.f, 1_s));

	// THEN: the drift angle is limited
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(0.f), DriftAngleEstimator::MAX_DRIFT_ANGLE, 1e-6f);
}

TEST(DriftAngleEstimator, driftToTheRightExpectedRight)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads north, is expected to move to its right (east) and moves east with a current to the south
	drift_angle_estimator.update(localPosition(-0.2f, 2.f, 0.f, 1_s), M_PI_2_F);

	// THEN: holding a course to the east needs a positive drift angle since the current sets it to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(M_PI_2_F), asinf(0.2f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, driftToTheRightExpectedBackwards)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads north, is expected to move backwards (south) and moves south with a current to the west
	drift_angle_estimator.update(localPosition(-2.f, -0.2f, 0.f, 1_s), M_PI_F);

	// THEN: holding a course to the south needs a positive drift angle since the current sets it to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(M_PI_F), asinf(0.2f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, noDriftAlongDiagonalExpectedDirection)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads east, is expected to move forwards left (north east) and moves exactly north east
	drift_angle_estimator.update(localPosition(2.f, 2.f, M_PI_2_F, 1_s), -M_PI_4_F);

	// THEN: there is no drift angle for a course to the north east
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(M_PI_4_F), 0.f, 1e-5f);
}

TEST(DriftAngleEstimator, currentStaysFixedWhenTheCourseChanges)
{
	// GIVEN: a drift angle estimator that has seen a current of 0.4 m/s to the east while heading north at 2 m/s
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, 1_s), 0.f, 2.f);

	// WHEN: the course setpoint turns to the east, along the current
	const float drift_angle = drift_angle_estimator.getDriftAngle(M_PI_2_F, 2.f);

	// THEN: no drift angle is needed, as the current no longer sets the vehicle off the course
	EXPECT_NEAR(drift_angle, 0.f, 1e-5f);
}

TEST(DriftAngleEstimator, currentAlongTheDirectionOfTravel)
{
	// GIVEN: a drift angle estimator that has seen the rover move north at 2.5 m/s while 2 m/s were commanded,
	// so a current of 0.5 m/s to the north
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.5f, 0.f, 0.f, 1_s), 0.f, 2.f);

	// WHEN: the course setpoint turns to the east
	const float drift_angle = drift_angle_estimator.getDriftAngle(M_PI_2_F, 2.f);

	// THEN: the drift angle cancels the current, which now sets the vehicle to the left of the course
	EXPECT_NEAR(drift_angle, asinf(-0.5f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, fasterSpeedSetpointNeedsSmallerDriftAngle)
{
	// GIVEN: a drift angle estimator that has seen a current of 0.4 m/s to the east at 2 m/s
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, 1_s), 0.f, 2.f);

	// WHEN: a speed of 4 m/s is commanded along a course to the north
	const float drift_angle = drift_angle_estimator.getDriftAngle(0.f, 4.f);

	// THEN: the faster vehicle needs a smaller drift angle to cancel the same current
	EXPECT_NEAR(drift_angle, asinf(0.4f / 4.f), 1e-5f);
}

TEST(DriftAngleEstimator, reverseSpeedSetpointUsesItsMagnitude)
{
	// GIVEN: a new drift angle estimator
	DriftAngleEstimator drift_angle_estimator;

	// WHEN: the vehicle heads north and reverses south at a commanded 2 m/s with a current of 0.2 m/s to the west
	drift_angle_estimator.update(localPosition(-2.f, -0.2f, 0.f, 1_s), M_PI_F, -2.f);

	// THEN: holding a course to the south needs a positive drift angle since the current sets it to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(M_PI_F, -2.f), asinf(0.2f / 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, approachesNewDrift)
{
	// GIVEN: a drift angle estimator that has seen the rover drive straight
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s));
	ASSERT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f), 0.f);
	hrt_abstime timestamp = 1_s;

	// WHEN: a new current appears for 0.1 s
	for (int i = 0; i < 10; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	// THEN: the drift angle moves towards the new current without jumping to it
	EXPECT_GT(drift_angle_estimator.getDriftAngle(0.f), 0.f);
	EXPECT_LT(drift_angle_estimator.getDriftAngle(0.f), asinf(0.4f / 2.f));
}

TEST(DriftAngleEstimator, convergesToNewDrift)
{
	// GIVEN: a drift angle estimator that has seen the rover drive straight
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.f, 0.f, 1_s));
	ASSERT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f), 0.f);
	hrt_abstime timestamp = 1_s;

	// WHEN: a new current persists for 10 s
	for (int i = 0; i < 1000; i++) {
		timestamp += 10_ms;
		drift_angle_estimator.update(localPosition(2.f, 0.4f, 0.f, timestamp));
	}

	// THEN: the drift angle has converged to the one that cancels the new current
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(0.f), asinf(0.4f / 2.f), 1e-3f);
}

TEST(DriftAngleEstimator, reset)
{
	// GIVEN: a drift angle estimator with a non-zero drift angle
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));
	ASSERT_GT(drift_angle_estimator.getDriftAngle(0.f), 0.f);

	// WHEN: it is reset
	drift_angle_estimator.reset();

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(0.f, SPEED_SETPOINT), 0.f);
}

TEST(calcHeadingSetpoint, forwardsSubtractsDriftAngle)
{
	// GIVEN: a course setpoint to the east and a drift angle of 0.1 rad, the course to the right of the heading
	// WHEN: driving forwards
	const float heading_setpoint = calcHeadingSetpoint(M_PI_2_F, 0.1f);

	// THEN: the heading setpoint points left of the course setpoint by the drift angle
	EXPECT_NEAR(heading_setpoint, M_PI_2_F - 0.1f, 1e-6f);
}

TEST(calcHeadingSetpoint, reversePointsAwayAndSubtractsDriftAngle)
{
	// GIVEN: a course setpoint to the north east and a drift angle of 0.1 rad
	// WHEN: driving in reverse
	const float heading_setpoint = calcHeadingSetpoint(M_PI_4_F, 0.1f, M_PI_F);

	// THEN: the heading setpoint points to the south west, corrected by the drift angle and wrapped into [-pi, pi]
	EXPECT_NEAR(heading_setpoint, M_PI_4_F - M_PI_F - 0.1f, 1e-6f);
}

TEST(calcHeadingSetpoint, wrapsAcrossPi)
{
	// GIVEN: a course setpoint to the south and a drift angle to the left
	// WHEN: driving forwards
	const float heading_setpoint = calcHeadingSetpoint(M_PI_F, -0.1f);

	// THEN: the heading setpoint is wrapped into [-pi, pi]
	EXPECT_NEAR(heading_setpoint, -M_PI_F + 0.1f, 1e-6f);
}

TEST(calcHeadingSetpoint, sidewaysDirectionOfTravel)
{
	// GIVEN: a course setpoint to the north east and no drift
	// WHEN: the vehicle is expected to move to its right
	const float heading_setpoint = calcHeadingSetpoint(M_PI_4_F, 0.f, M_PI_2_F);

	// THEN: the vehicle heads north west, so its right points along the course setpoint
	EXPECT_NEAR(heading_setpoint, -M_PI_4_F, 1e-6f);
}

TEST(calcHeadingSetpoint, invertsDriftAngleEstimate)
{
	// GIVEN: a drift angle estimated driving in reverse, heading north and moving south with a drift to the west
	DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(-2.f, -0.2f, 0.f, 1_s), M_PI_F);
	const float course = atan2f(-0.2f, -2.f);

	// WHEN: the heading setpoint is calculated to hold that course in reverse
	const float heading_setpoint = calcHeadingSetpoint(course, drift_angle_estimator.getDriftAngle(course), M_PI_F);

	// THEN: it is the heading the drift angle was estimated at
	EXPECT_NEAR(heading_setpoint, 0.f, 1e-5f);
}
