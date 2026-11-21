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
 * Test code for the RoverControl library
 * Run this test only using "make tests TESTFILTER=RoverControl"
******************************************************************/

#include <gtest/gtest.h>
#include "RoverControl.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

TEST(calcWaypointTransitionAngle, nanPreviousWaypoint)
{
	// GIVEN: a NaN previous waypoint
	Vector2f prev_wp_ned(NAN, NAN);
	Vector2f curr_wp_ned(10.f, 10.f);
	Vector2f next_wp_ned(10.f, 10.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is NaN
	EXPECT_FALSE(PX4_ISFINITE(angle));
}

TEST(calcWaypointTransitionAngle, nanCurrentWaypoint)
{
	// GIVEN: a NaN current waypoint
	Vector2f prev_wp_ned(10.f, 10.f);
	Vector2f curr_wp_ned(NAN, NAN);
	Vector2f next_wp_ned(10.f, 10.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is NaN
	EXPECT_FALSE(PX4_ISFINITE(angle));
}

TEST(calcWaypointTransitionAngle, nanNextWaypoint)
{
	// GIVEN: a NaN next waypoint
	Vector2f prev_wp_ned(10.f, 10.f);
	Vector2f curr_wp_ned(10.f, 10.f);
	Vector2f next_wp_ned(NAN, NAN);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is NaN
	EXPECT_FALSE(PX4_ISFINITE(angle));
}

TEST(calcWaypointTransitionAngle, straightLine)
{
	// GIVEN: three waypoints on a straight line
	// P -- C -- N
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(20.f, 0.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is pi
	EXPECT_FLOAT_EQ(angle, M_PI_F);
}

TEST(calcWaypointTransitionAngle, shallowTurn)
{
	// GIVEN: a 45 degree turn at the current waypoint
	/**
	 *	    N
	 *	  /
	 * P -- C
	 */
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(20.f, 10.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is 3/4 pi
	EXPECT_FLOAT_EQ(angle, M_PI_F - M_PI_4_F);
}

TEST(calcWaypointTransitionAngle, rightAngleTurn)
{
	// GIVEN: a 90 degree turn at the current waypoint
	/**
	 *	N
	 *	|
	 * P -- C
	 */
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(10.f, 10.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is pi/2
	EXPECT_FLOAT_EQ(angle, M_PI_2_F);
}

TEST(calcWaypointTransitionAngle, sharpTurn)
{
	// GIVEN: a 135 degree turn at the current waypoint
	/**
	 * N
	 *    \
	 * P -- C
	 */
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(0.f, 10.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is pi/4
	EXPECT_FLOAT_EQ(angle, M_PI_4_F);
}

TEST(calcWaypointTransitionAngle, previousEqualsCurrent)
{
	// GIVEN: a previous waypoint on top of the current one
	// P/C -- N
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(0.f, 0.f);
	Vector2f next_wp_ned(10.f, 0.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is undefined (NaN)
	EXPECT_FALSE(PX4_ISFINITE(angle));
}

TEST(calcWaypointTransitionAngle, nextEqualsCurrent)
{
	// GIVEN: a next waypoint on top of the current one
	// P -- C/N
	Vector2f prev_wp_ned(0.f, 0.f);
	Vector2f curr_wp_ned(10.f, 0.f);
	Vector2f next_wp_ned(10.f, 0.f);

	// WHEN: the transition angle is calculated
	const float angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

	// THEN: the angle is undefined (NaN)
	EXPECT_FALSE(PX4_ISFINITE(angle));
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

TEST(DriftAngleEstimator, zeroBeforeFirstUpdate)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: no update has happened yet
	const float drift_angle = drift_angle_estimator.getDriftAngle();

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle, 0.f);
}

TEST(DriftAngleEstimator, ignoresLowSpeed)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves too slowly for the velocity direction to say anything about the drift
	drift_angle_estimator.update(localPosition(0.3f, 0.1f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresReverse)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover drives backwards
	drift_angle_estimator.update(localPosition(-2.f, 0.5f, 0.f, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresInvalidVelocity)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

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
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the heading is NaN
	drift_angle_estimator.update(localPosition(2.f, 0.5f, NAN, 1_s));

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, ignoresUncertainVelocity)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the forward speed is below the velocity uncertainty
	vehicle_local_position_s uncertain = localPosition(2.f, 0.5f, 0.f, 1_s);
	uncertain.evh = 1.f;
	drift_angle_estimator.update(uncertain);

	// THEN: the drift angle stays zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingNorth)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads north and moves north with a drift to the east
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));

	// THEN: the drift angle is the angle from the heading to the course, positive since the course is to the right
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, driftToTheRightHeadingEast)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover heads east with the same drift relative to its heading
	drift_angle_estimator.update(localPosition(-0.2f, 2.f, M_PI_2_F, 1_s));

	// THEN: the drift angle is the same as when heading north
	EXPECT_NEAR(drift_angle_estimator.getDriftAngle(), atan2f(0.2f, 2.f), 1e-5f);
}

TEST(DriftAngleEstimator, limited)
{
	// GIVEN: a new drift angle estimator
	RoverControl::DriftAngleEstimator drift_angle_estimator;

	// WHEN: the rover moves mostly sideways
	drift_angle_estimator.update(localPosition(1.f, 5.f, 0.f, 1_s));

	// THEN: the drift angle is limited
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), RoverControl::DriftAngleEstimator::MAX_DRIFT_ANGLE);
}

TEST(DriftAngleEstimator, approachesNewDrift)
{
	// GIVEN: a drift angle estimator that has seen the rover drive straight
	RoverControl::DriftAngleEstimator drift_angle_estimator;
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
	RoverControl::DriftAngleEstimator drift_angle_estimator;
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
	RoverControl::DriftAngleEstimator drift_angle_estimator;
	drift_angle_estimator.update(localPosition(2.f, 0.2f, 0.f, 1_s));
	ASSERT_GT(drift_angle_estimator.getDriftAngle(), 0.f);

	// WHEN: it is reset
	drift_angle_estimator.reset();

	// THEN: the drift angle is zero
	EXPECT_FLOAT_EQ(drift_angle_estimator.getDriftAngle(), 0.f);
}

// Sticks of a mode 2 transmitter, each at a different value so a swapped axis is caught:
// throttle = left vertical, yaw = left horizontal, pitch = right vertical, roll = right horizontal
static manual_control_setpoint_s distinctSticks()
{
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 0.1f;
	manual_control_setpoint.yaw = 0.2f;
	manual_control_setpoint.pitch = 0.3f;
	manual_control_setpoint.roll = 0.4f;
	return manual_control_setpoint;
}

TEST(mapSticks, mode1)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with mode 1
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 1);

	// THEN: left: forward & yaw, right: lateral
	EXPECT_FLOAT_EQ(sticks.forward, 0.1f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.4f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.2f);
}

TEST(mapSticks, mode2)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with mode 2
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 2);

	// THEN: left: yaw, right: forward & lateral
	EXPECT_FLOAT_EQ(sticks.forward, 0.3f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.4f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.2f);
}

TEST(mapSticks, mode3)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with mode 3
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 3);

	// THEN: left: forward & lateral, right: yaw
	EXPECT_FLOAT_EQ(sticks.forward, 0.1f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.2f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.4f);
}

TEST(mapSticks, mode4)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with mode 4
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 4);

	// THEN: left: lateral, right: forward & yaw
	EXPECT_FLOAT_EQ(sticks.forward, 0.3f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.2f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.4f);
}

TEST(mapSticks, negativeFallsBackToMode3)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with an out of range negative value
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, -1);

	// THEN: the default mapping (mode 3) is used
	EXPECT_FLOAT_EQ(sticks.forward, 0.1f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.2f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.4f);
}

TEST(mapSticks, zeroFallsBackToMode3)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with 0, which is below the first mode
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 0);

	// THEN: the default mapping (mode 3) is used
	EXPECT_FLOAT_EQ(sticks.forward, 0.1f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.2f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.4f);
}

TEST(mapSticks, aboveRangeFallsBackToMode3)
{
	// GIVEN: each stick at a different value
	const manual_control_setpoint_s manual_control_setpoint = distinctSticks();

	// WHEN: mapped with 5, which is above the last mode
	const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, 5);

	// THEN: the default mapping (mode 3) is used
	EXPECT_FLOAT_EQ(sticks.forward, 0.1f);
	EXPECT_FLOAT_EQ(sticks.lateral, 0.2f);
	EXPECT_FLOAT_EQ(sticks.yaw, 0.4f);
}

static constexpr float kDt = 0.1f; // Controller update interval used by all tests below [s]

TEST(throttleControl, nanThrottleSetpoint)
{
	// GIVEN: a throttle controller with acceleration and deceleration limits and a motor setpoint of 0.5
	SlewRate<float> motor_setpoint(0.5f);

	// WHEN: the throttle setpoint is NaN
	const float throttle = RoverControl::throttleControl(motor_setpoint, NAN, 0.f, 1.f, 1.f, 2.f, kDt);

	// THEN: the output is NaN and the motor setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(motor_setpoint.getState(), 0.5f);
}

TEST(throttleControl, nanCurrentMotorSetpoint)
{
	// GIVEN: a throttle controller with acceleration and deceleration limits and a motor setpoint of 0.5
	SlewRate<float> motor_setpoint(0.5f);

	// WHEN: the current motor setpoint is NaN
	const float throttle = RoverControl::throttleControl(motor_setpoint, 1.f, NAN, 1.f, 1.f, 2.f, kDt);

	// THEN: the output is NaN and the motor setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(motor_setpoint.getState(), 0.5f);
}

TEST(throttleControl, nanDt)
{
	// GIVEN: a throttle controller with acceleration and deceleration limits and a motor setpoint of 0.5
	SlewRate<float> motor_setpoint(0.5f);

	// WHEN: the update interval is NaN
	const float throttle = RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 1.f, 1.f, 2.f, NAN);

	// THEN: the output is NaN and the motor setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(motor_setpoint.getState(), 0.5f);
}

TEST(throttleControl, accelerationLimit)
{
	// GIVEN: a throttle controller at standstill with a deceleration limit different from the acceleration limit,
	// so the test notices if the wrong one is used
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 2.f; // [m/s^2]
	const float max_thr_spd = 2.f; // [m/s]
	const float step = max_accel / max_thr_spd * kDt; // Largest throttle change per update
	SlewRate<float> motor_setpoint(0.f);
	float throttle = 0.f;

	// WHEN: full throttle is requested for five updates
	// THEN: the throttle rises by one acceleration step per update
	for (int i = 1; i <= 5; i++) {
		throttle = RoverControl::throttleControl(motor_setpoint, 1.f, throttle, max_accel, max_decel, max_thr_spd, kDt);
		EXPECT_NEAR(throttle, step * i, 1e-5f);
	}
}

TEST(throttleControl, decelerationLimit)
{
	// GIVEN: a throttle controller at full throttle (sign changes are covered by decelerationLimitOnSignChange)
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 2.f; // [m/s^2]
	const float max_thr_spd = 2.f; // [m/s]
	const float step = max_decel / max_thr_spd * kDt; // Largest throttle change per update
	SlewRate<float> motor_setpoint(1.f);
	float throttle = 1.f;

	// WHEN: half throttle is requested for three updates
	// THEN: the throttle drops by one deceleration step per update
	for (int i = 1; i <= 3; i++) {
		throttle = RoverControl::throttleControl(motor_setpoint, 0.5f, throttle, max_accel, max_decel, max_thr_spd, kDt);
		EXPECT_NEAR(throttle, 1.f - step * i, 1e-5f);
	}
}

TEST(throttleControl, noAccelerationLimit)
{
	// GIVEN: a throttle controller at standstill without an acceleration limit
	SlewRate<float> motor_setpoint(0.f);

	// WHEN: full throttle is requested
	const float throttle = RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 0.f, 1.f, 2.f, kDt);

	// THEN: the output jumps straight to the setpoint
	EXPECT_FLOAT_EQ(throttle, 1.f);
}

TEST(throttleControl, noSpeedAtFullThrottle)
{
	// GIVEN: a throttle controller at standstill without a speed at full throttle, so the limits can't be converted
	SlewRate<float> motor_setpoint(0.f);

	// WHEN: full throttle is requested
	const float throttle = RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 1.f, 1.f, 0.f, kDt);

	// THEN: the output jumps straight to the setpoint
	EXPECT_FLOAT_EQ(throttle, 1.f);
}

TEST(throttleControl, reinitializeWhenCurrentIsCloser)
{
	// GIVEN: a slew rate state (0) that lags far behind the applied throttle (0.9)
	SlewRate<float> motor_setpoint(0.f);

	// WHEN: full throttle is requested, and one limited step would move further from the applied throttle
	// than the setpoint is
	const float throttle = RoverControl::throttleControl(motor_setpoint, 1.f, 0.9f, 1.f, 1.f, 2.f, kDt);

	// THEN: the output and the slew rate state snap to the setpoint
	EXPECT_FLOAT_EQ(throttle, 1.f);
	EXPECT_FLOAT_EQ(motor_setpoint.getState(), 1.f);
}

TEST(attitudeControl, nanVehicleYaw)
{
	// GIVEN: a yaw controller without a slew rate and an adjusted yaw setpoint of 0.5 rad
	SlewRateYaw<float> adjusted_yaw_setpoint;
	adjusted_yaw_setpoint.setForcedValue(0.5f);
	PID pid_yaw;

	// WHEN: the vehicle yaw is NaN
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, NAN, 0.f, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(yaw_rate));
	EXPECT_FLOAT_EQ(adjusted_yaw_setpoint.getState(), 0.5f);
}

TEST(attitudeControl, nanYawSetpoint)
{
	// GIVEN: a yaw controller without a slew rate and an adjusted yaw setpoint of 0.5 rad
	SlewRateYaw<float> adjusted_yaw_setpoint;
	adjusted_yaw_setpoint.setForcedValue(0.5f);
	PID pid_yaw;

	// WHEN: the yaw setpoint is NaN
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, NAN, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(yaw_rate));
	EXPECT_FLOAT_EQ(adjusted_yaw_setpoint.getState(), 0.5f);
}

TEST(attitudeControl, nanDt)
{
	// GIVEN: a yaw controller without a slew rate and an adjusted yaw setpoint of 0.5 rad
	SlewRateYaw<float> adjusted_yaw_setpoint;
	adjusted_yaw_setpoint.setForcedValue(0.5f);
	PID pid_yaw;

	// WHEN: the update interval is NaN
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, 0.f, NAN);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(yaw_rate));
	EXPECT_FLOAT_EQ(adjusted_yaw_setpoint.getState(), 0.5f);
}

TEST(attitudeControl, wrapAround)
{
	// GIVEN: a P-only yaw controller with the vehicle at a yaw of +3.1 rad
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);

	// WHEN: the yaw setpoint is -3.1 rad, 0.083 rad away across +-pi and 6.2 rad the long way round
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 3.1f, -3.1f, kDt);

	// THEN: the vehicle turns the short way
	EXPECT_NEAR(yaw_rate, 2.f * M_PI_F - 6.2f, 1e-4f);
	EXPECT_GT(yaw_rate, 0.f);
}

TEST(attitudeControl, slewRate)
{
	// GIVEN: a P-only yaw controller with a yaw slew rate and the vehicle at a yaw of 0
	const float yaw_slew_rate = 0.5f; // [rad/s]
	const float step = yaw_slew_rate * kDt; // Largest change of the adjusted setpoint per update
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);

	// WHEN: a yaw setpoint of 1 rad is requested for three updates
	// THEN: the adjusted setpoint and the yaw rate rise by one slew step per update
	for (int i = 1; i <= 3; i++) {
		const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, yaw_slew_rate, 0.f, 1.f, kDt);
		EXPECT_NEAR(adjusted_yaw_setpoint.getState(), step * i, 1e-5f);
		EXPECT_NEAR(yaw_rate, step * i, 1e-5f); // P gain 1 at yaw 0
	}
}

TEST(attitudeControl, noSlewRate)
{
	// GIVEN: a P-only yaw controller without a slew rate and the vehicle at a yaw of 0
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);

	// WHEN: a yaw setpoint of 1 rad is requested
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, 1.f, kDt);

	// THEN: the adjusted setpoint jumps straight to the setpoint
	EXPECT_FLOAT_EQ(yaw_rate, 1.f);
	EXPECT_FLOAT_EQ(adjusted_yaw_setpoint.getState(), 1.f);
}

TEST(speedControl, nanSpeedSetpoint)
{
	// GIVEN: a speed controller with an adjusted speed setpoint of 0.5 m/s
	SlewRate<float> speed_with_rate_limit(0.5f);
	PID pid_speed;

	// WHEN: the speed setpoint is NaN
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, NAN, 0.f, 0.f, 0.f, 2.f, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(speed_with_rate_limit.getState(), 0.5f);
}

TEST(speedControl, nanVehicleSpeed)
{
	// GIVEN: a speed controller with an adjusted speed setpoint of 0.5 m/s
	SlewRate<float> speed_with_rate_limit(0.5f);
	PID pid_speed;

	// WHEN: the vehicle speed is NaN
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, NAN, 0.f, 0.f, 2.f, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(speed_with_rate_limit.getState(), 0.5f);
}

TEST(speedControl, nanDt)
{
	// GIVEN: a speed controller with an adjusted speed setpoint of 0.5 m/s
	SlewRate<float> speed_with_rate_limit(0.5f);
	PID pid_speed;

	// WHEN: the update interval is NaN
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 2.f, NAN);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(throttle));
	EXPECT_FLOAT_EQ(speed_with_rate_limit.getState(), 0.5f);
}

TEST(speedControl, feedForwardOnly)
{
	// GIVEN: a speed controller with all PID gains at 0, driving at the setpoint
	const float max_thr_speed = 2.f; // [m/s]
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setOutputLimit(1.f);

	// WHEN: the speed setpoint is half the speed at full throttle
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, max_thr_speed / 2.f,
			       max_thr_speed / 2.f, 0.f, 0.f, max_thr_speed, kDt);

	// THEN: the output is half throttle
	EXPECT_NEAR(throttle, 0.5f, 1e-5f);
}

TEST(speedControl, feedbackOnly)
{
	// GIVEN: a P-only speed controller at standstill without a speed at full throttle, so there is no feed-forward
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(0.5f, 0.f, 0.f);
	pid_speed.setOutputLimit(1.f);

	// WHEN: a speed of 1 m/s is requested
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 0.f, kDt);

	// THEN: only the P term acts on the 1 m/s speed error
	EXPECT_NEAR(throttle, 0.5f, 1e-5f);
}

TEST(speedControl, zeroSetpointResetsIntegral)
{
	// GIVEN: an I-only speed controller that has integrated a 1 m/s speed error for one update
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(0.f, 1.f, 0.f);
	pid_speed.setIntegralLimit(1.f);
	pid_speed.setOutputLimit(1.f);
	RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 0.f, kDt);
	ASSERT_NEAR(pid_speed.getIntegral(), 1.f * kDt, 1e-5f); // I gain 1, error 1 m/s for one update

	// WHEN: the speed setpoint drops to zero
	RoverControl::speedControl(speed_with_rate_limit, pid_speed, 0.f, 0.f, 0.f, 0.f, 0.f, kDt);

	// THEN: the integral is cleared
	EXPECT_FLOAT_EQ(pid_speed.getIntegral(), 0.f);
}

TEST(speedControl, outputClampedForwards)
{
	// GIVEN: a P-only speed controller at standstill with a speed at full throttle of 1 m/s
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(1.f, 0.f, 0.f);
	pid_speed.setOutputLimit(10.f);

	// WHEN: a speed of 1 m/s is requested, so full feed-forward plus the P term exceed full throttle
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 1.f, kDt);

	// THEN: the output is clamped to full throttle
	EXPECT_FLOAT_EQ(throttle, 1.f);
}

TEST(speedControl, outputClampedBackwards)
{
	// GIVEN: a P-only speed controller at standstill with a speed at full throttle of 1 m/s
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(1.f, 0.f, 0.f);
	pid_speed.setOutputLimit(10.f);

	// WHEN: a speed of -1 m/s is requested, so full feed-forward plus the P term exceed full reverse throttle
	const float throttle = RoverControl::speedControl(speed_with_rate_limit, pid_speed, -1.f, 0.f, 0.f, 0.f, 1.f, kDt);

	// THEN: the output is clamped to full reverse throttle
	EXPECT_FLOAT_EQ(throttle, -1.f);
}

TEST(rateControl, nanYawRateSetpoint)
{
	// GIVEN: a yaw rate controller with an adjusted yaw rate setpoint of 0.5 rad/s
	SlewRate<float> adjusted_yaw_rate_setpoint(0.5f);
	PID pid_yaw_rate;

	// WHEN: the yaw rate setpoint is NaN
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, NAN, 0.f, 1.f, 1.f,
					    0.f, 0.f, 0.5f, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(speed_diff_normalized));
	EXPECT_FLOAT_EQ(adjusted_yaw_rate_setpoint.getState(), 0.5f);
}

TEST(rateControl, nanVehicleYawRate)
{
	// GIVEN: a yaw rate controller with an adjusted yaw rate setpoint of 0.5 rad/s
	SlewRate<float> adjusted_yaw_rate_setpoint(0.5f);
	PID pid_yaw_rate;

	// WHEN: the vehicle yaw rate is NaN
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, NAN, 1.f, 1.f,
					    0.f, 0.f, 0.5f, kDt);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(speed_diff_normalized));
	EXPECT_FLOAT_EQ(adjusted_yaw_rate_setpoint.getState(), 0.5f);
}

TEST(rateControl, nanDt)
{
	// GIVEN: a yaw rate controller with an adjusted yaw rate setpoint of 0.5 rad/s
	SlewRate<float> adjusted_yaw_rate_setpoint(0.5f);
	PID pid_yaw_rate;

	// WHEN: the update interval is NaN
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f,
					    0.f, 0.f, 0.5f, NAN);

	// THEN: the output is NaN and the adjusted setpoint is left unchanged
	EXPECT_FALSE(PX4_ISFINITE(speed_diff_normalized));
	EXPECT_FLOAT_EQ(adjusted_yaw_rate_setpoint.getState(), 0.5f);
}

TEST(rateControl, feedForward)
{
	// GIVEN: a yaw rate controller with all PID gains at 0, turning at the setpoint
	const float yaw_rate_setpoint = 2.f; // [rad/s]
	const float wheel_track = 0.5f; // [m]
	const float yaw_rate_corr = 1.5f;
	const float max_thr_speed = 1.f; // [m/s]
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setOutputLimit(1.f);

	// WHEN: a yaw rate of 2 rad/s is requested
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, yaw_rate_setpoint,
					    yaw_rate_setpoint, max_thr_speed, yaw_rate_corr, 0.f, 0.f, wheel_track, kDt);

	// THEN: the speed difference is yaw rate * wheel_track / 2 * yaw_rate_corr = 2 * 0.5 / 2 * 1.5 = 0.75 m/s,
	// which is 0.75 of the speed at full throttle (1 m/s)
	EXPECT_NEAR(speed_diff_normalized, 0.75f, 1e-5f);
}

TEST(rateControl, feedbackOnly)
{
	// GIVEN: a P-only yaw rate controller, not turning, without a wheel track, so there is no feed-forward
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(0.5f, 0.f, 0.f);
	pid_yaw_rate.setOutputLimit(1.f);

	// WHEN: a yaw rate of 1 rad/s is requested
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f,
					    0.f, 0.f, 0.f, kDt);

	// THEN: only the P term acts on the 1 rad/s yaw rate error
	EXPECT_NEAR(speed_diff_normalized, 0.5f, 1e-5f);
}

TEST(rateControl, zeroSetpointResetsIntegral)
{
	// GIVEN: an I-only yaw rate controller that has integrated a 1 rad/s yaw rate error for one update
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(0.f, 1.f, 0.f);
	pid_yaw_rate.setIntegralLimit(1.f);
	pid_yaw_rate.setOutputLimit(1.f);
	RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, kDt);
	ASSERT_NEAR(pid_yaw_rate.getIntegral(), 1.f * kDt, 1e-5f); // I gain 1, error 1 rad/s for one update

	// WHEN: the yaw rate setpoint drops to zero
	RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, kDt);

	// THEN: the integral is cleared
	EXPECT_FLOAT_EQ(pid_yaw_rate.getIntegral(), 0.f);
}

TEST(rateControl, outputClampedPositive)
{
	// GIVEN: a P-only yaw rate controller, not turning, with a wheel track of 2 m, so a yaw rate of 1 rad/s (clockwise)
	// asks for the full 1 m/s speed difference
	const float wheel_track = 2.f; // [m]
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(1.f, 0.f, 0.f);
	pid_yaw_rate.setOutputLimit(10.f);

	// WHEN: a yaw rate of 1 rad/s is requested, so full feed-forward plus the P term exceed the normalized range
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f,
					    0.f, 0.f, wheel_track, kDt);

	// THEN: the output is clamped to 1
	EXPECT_FLOAT_EQ(speed_diff_normalized, 1.f);
}

TEST(rateControl, outputClampedNegative)
{
	// GIVEN: a P-only yaw rate controller, not turning, with a wheel track of 2 m, so a yaw rate of -1 rad/s (counterclockwise)
	// asks for the full -1 m/s speed difference
	const float wheel_track = 2.f; // [m]
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(1.f, 0.f, 0.f);
	pid_yaw_rate.setOutputLimit(10.f);

	// WHEN: a yaw rate of -1 rad/s is requested, so full feed-forward plus the P term exceed the normalized range
	const float speed_diff_normalized = RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, -1.f, 0.f, 1.f, 1.f,
					    0.f, 0.f, wheel_track, kDt);

	// THEN: the output is clamped to -1
	EXPECT_FLOAT_EQ(speed_diff_normalized, -1.f);
}

static position_setpoint_s waypoint(const bool valid, const double lat, const double lon)
{
	position_setpoint_s position_setpoint{};
	position_setpoint.valid = valid;
	position_setpoint.lat = lat;
	position_setpoint.lon = lon;
	return position_setpoint;
}

TEST(globalToLocalSetpointTriplet, validTriplet)
{
	// GIVEN: a reference point and three valid nearby waypoints, distinct so a swapped output is caught
	MapProjection global_ned_proj_ref;
	global_ned_proj_ref.initReference(47.397742, 8.545594);
	position_setpoint_triplet_s position_setpoint_triplet{};
	position_setpoint_triplet.previous = waypoint(true, 47.397742, 8.545594);
	position_setpoint_triplet.current = waypoint(true, 47.398742, 8.545594);
	position_setpoint_triplet.next = waypoint(true, 47.398742, 8.546594);
	Vector2f curr_pos_ned(1.f, 2.f);
	Vector2f curr_wp_ned, prev_wp_ned, next_wp_ned;

	// WHEN: the triplet is converted to local coordinates
	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);

	// THEN: each waypoint is projected around the reference
	EXPECT_EQ(prev_wp_ned, global_ned_proj_ref.project(47.397742, 8.545594));
	EXPECT_EQ(curr_wp_ned, global_ned_proj_ref.project(47.398742, 8.545594));
	EXPECT_EQ(next_wp_ned, global_ned_proj_ref.project(47.398742, 8.546594));
	EXPECT_GT(curr_wp_ned(0), 100.f); // About 111 m north of the reference
}

TEST(globalToLocalSetpointTriplet, invalidWaypointsFallBackToCurrentPosition)
{
	// GIVEN: a NaN previous waypoint, invalid current and next waypoints and a valid current position
	MapProjection global_ned_proj_ref;
	global_ned_proj_ref.initReference(47.397742, 8.545594);
	position_setpoint_triplet_s position_setpoint_triplet{};
	position_setpoint_triplet.previous = waypoint(true, static_cast<double>(NAN), 8.545594);
	position_setpoint_triplet.current = waypoint(false, 47.398742, 8.545594);
	position_setpoint_triplet.next = waypoint(false, 47.398742, 8.546594);
	Vector2f curr_pos_ned(1.f, 2.f);
	Vector2f curr_wp_ned, prev_wp_ned, next_wp_ned;

	// WHEN: the triplet is converted to local coordinates
	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);

	// THEN: the current and previous waypoints fall back to the current position, and the next one is NaN
	EXPECT_EQ(curr_wp_ned, curr_pos_ned);
	EXPECT_EQ(prev_wp_ned, curr_pos_ned);
	EXPECT_FALSE(next_wp_ned.isAllFinite());
}

TEST(globalToLocalSetpointTriplet, invalidWaypointsWithoutCurrentPosition)
{
	// GIVEN: a NaN previous waypoint, invalid current and next waypoints and a NaN current position
	MapProjection global_ned_proj_ref;
	global_ned_proj_ref.initReference(47.397742, 8.545594);
	position_setpoint_triplet_s position_setpoint_triplet{};
	position_setpoint_triplet.previous = waypoint(true, static_cast<double>(NAN), 8.545594);
	position_setpoint_triplet.current = waypoint(false, 47.398742, 8.545594);
	position_setpoint_triplet.next = waypoint(false, 47.398742, 8.546594);
	Vector2f curr_pos_ned(NAN, NAN);
	Vector2f curr_wp_ned, prev_wp_ned, next_wp_ned;

	// WHEN: the triplet is converted to local coordinates
	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);

	// THEN: there is nothing to fall back to, so the current and previous waypoints are NaN
	EXPECT_FALSE(curr_wp_ned.isAllFinite());
	EXPECT_FALSE(prev_wp_ned.isAllFinite());
}

// Distinct acceleration and deceleration limits so the applied one can be read from a single step
static constexpr float kSignChangeMaxAccel = 1.f; // [m/s^2]
static constexpr float kSignChangeMaxDecel = 4.f; // [m/s^2]
static constexpr float kNoLimit = -1.f;
static constexpr float kSignChangeMaxThrSpd = 2.f; // [m/s]
static constexpr float kThrottleAccelStep = kSignChangeMaxAccel / kSignChangeMaxThrSpd * kDt;
static constexpr float kThrottleDecelStep = kSignChangeMaxDecel / kSignChangeMaxThrSpd * kDt;

// One throttleControl update starting from current_motor_setpoint
static float throttleControlStep(const float throttle_setpoint, const float current_motor_setpoint, const float max_accel,
				 const float max_decel)
{
	SlewRate<float> motor_setpoint(current_motor_setpoint);
	return RoverControl::throttleControl(motor_setpoint, throttle_setpoint, current_motor_setpoint, max_accel, max_decel,
					     kSignChangeMaxThrSpd, kDt);
}

// One speedControl update starting from the rate limited setpoint adjusted_speed, without feed-forward,
// returning the new rate limited speed setpoint
static float speedControlStep(const float speed_setpoint, const float vehicle_speed, const float adjusted_speed,
			      const float max_accel, const float max_decel)
{
	SlewRate<float> speed_with_rate_limit(adjusted_speed);
	PID pid_speed;
	RoverControl::speedControl(speed_with_rate_limit, pid_speed, speed_setpoint, vehicle_speed, max_accel, max_decel, 0.f,
				   kDt);
	return speed_with_rate_limit.getState();
}

TEST(throttleControl, signChangeReverseWhileDrivingForward)
{
	// GIVEN: a throttle controller with the motor setpoint at 0.5 (driving forwards)
	// WHEN: full reverse is requested
	const float throttle = throttleControlStep(-1.f, 0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle brakes towards zero first with the deceleration limit
	EXPECT_NEAR(throttle, 0.5f - kThrottleDecelStep, 1e-6f);
}

TEST(throttleControl, signChangeForwardWhileReversing)
{
	// GIVEN: a throttle controller with the motor setpoint at -0.5 (driving backwards)
	// WHEN: full forward is requested
	const float throttle = throttleControlStep(1.f, -0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle brakes towards zero first with the deceleration limit
	EXPECT_NEAR(throttle, -0.5f + kThrottleDecelStep, 1e-6f);
}

TEST(throttleControl, signChangeSmallReverseWhileDrivingForward)
{
	// GIVEN: a throttle controller with the motor setpoint at 0.5 (driving forwards)
	// WHEN: a reverse throttle smaller in magnitude (-0.2) is requested
	const float throttle = throttleControlStep(-0.2f, 0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle brakes with the deceleration limit
	EXPECT_NEAR(throttle, 0.5f - kThrottleDecelStep, 1e-6f);
}

TEST(throttleControl, signChangeReverseWithoutAccelerationLimit)
{
	// GIVEN: a throttle controller with the motor setpoint at 0.5 (driving forwards) and no acceleration limit
	// WHEN: full reverse is requested
	const float throttle = throttleControlStep(-1.f, 0.5f, kNoLimit, kSignChangeMaxDecel);

	// THEN: the throttle still brakes with the deceleration limit
	EXPECT_NEAR(throttle, 0.5f - kThrottleDecelStep, 1e-6f);
}

TEST(throttleControl, signChangeReverseAfterSignChange)
{
	// GIVEN: a throttle controller with the motor setpoint at -0.2 (already driving backwards)
	// WHEN: full reverse is requested
	const float throttle = throttleControlStep(-1.f, -0.2f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle accelerates with the acceleration limit
	EXPECT_NEAR(throttle, -0.2f - kThrottleAccelStep, 1e-6f);
}

TEST(throttleControl, signChangeForwardSameSign)
{
	// GIVEN: a throttle controller with the motor setpoint at 0.2 (driving forwards)
	// WHEN: full forward is requested
	const float throttle = throttleControlStep(1.f, 0.2f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle accelerates with the acceleration limit
	EXPECT_NEAR(throttle, 0.2f + kThrottleAccelStep, 1e-6f);
}

TEST(throttleControl, signChangeReverseFromStandstill)
{
	// GIVEN: a throttle controller with the motor setpoint at 0 (standing still)
	// WHEN: full reverse is requested
	const float throttle = throttleControlStep(-1.f, 0.f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the throttle accelerates with the acceleration limit
	EXPECT_NEAR(throttle, -kThrottleAccelStep, 1e-6f);
}

TEST(throttleControl, signChangeReverseWithoutDecelerationLimit)
{
	// GIVEN: a throttle controller with the motor setpoint at 0.5 (driving forwards) and no deceleration limit
	// WHEN: full reverse is requested
	const float throttle = throttleControlStep(-1.f, 0.5f, kSignChangeMaxAccel, kNoLimit);

	// THEN: the acceleration limit applies as before
	EXPECT_NEAR(throttle, 0.5f - kThrottleAccelStep, 1e-6f);
}

TEST(speedControl, signChangeReverseWhileDrivingForward)
{
	// GIVEN: a speed controller with the rover driving forwards at 1 m/s
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, 1.f, 1.f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint brakes towards zero first with the deceleration limit
	EXPECT_NEAR(adjusted_speed, 1.f - kSignChangeMaxDecel * kDt, 1e-6f);
}

TEST(speedControl, signChangeForwardWhileReversing)
{
	// GIVEN: a speed controller with the rover driving backwards at 1 m/s
	// WHEN: a forward speed of 1 m/s is requested
	const float adjusted_speed = speedControlStep(1.f, -1.f, -1.f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint brakes towards zero first with the deceleration limit
	EXPECT_NEAR(adjusted_speed, -1.f + kSignChangeMaxDecel * kDt, 1e-6f);
}

TEST(speedControl, signChangeLargerReverseWhileDrivingForward)
{
	// GIVEN: a speed controller with the rover driving forwards at 1 m/s
	// WHEN: a larger reverse speed of -2 m/s is requested
	const float adjusted_speed = speedControlStep(-2.f, 1.f, 1.f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint brakes with the deceleration limit
	EXPECT_NEAR(adjusted_speed, 1.f - kSignChangeMaxDecel * kDt, 1e-6f);
}

TEST(speedControl, signChangeReverseWithoutAccelerationLimit)
{
	// GIVEN: a speed controller with the rover driving forwards at 1 m/s and no acceleration limit
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, 1.f, 1.f, kNoLimit, kSignChangeMaxDecel);

	// THEN: the speed setpoint still brakes with the deceleration limit
	EXPECT_NEAR(adjusted_speed, 1.f - kSignChangeMaxDecel * kDt, 1e-6f);
}

TEST(speedControl, signChangeSmallerSameSignSetpoint)
{
	// GIVEN: a speed controller with the rover driving forwards at 0.5 m/s
	// WHEN: a smaller forward speed of 0.1 m/s is requested
	const float adjusted_speed = speedControlStep(0.1f, 0.5f, 0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint drops with the deceleration limit
	EXPECT_NEAR(adjusted_speed, 0.5f - kSignChangeMaxDecel * kDt, 1e-6f);
}

TEST(speedControl, signChangeReverseAfterSignChange)
{
	// GIVEN: a speed controller with the rover already driving backwards at 0.5 m/s
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, -0.5f, -0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint accelerates with the acceleration limit
	EXPECT_NEAR(adjusted_speed, -0.5f - kSignChangeMaxAccel * kDt, 1e-6f);
}

TEST(speedControl, signChangeForwardSameSign)
{
	// GIVEN: a speed controller with the rover driving forwards at 0.5 m/s
	// WHEN: a forward speed of 1 m/s is requested
	const float adjusted_speed = speedControlStep(1.f, 0.5f, 0.5f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint accelerates with the acceleration limit
	EXPECT_NEAR(adjusted_speed, 0.5f + kSignChangeMaxAccel * kDt, 1e-6f);
}

TEST(speedControl, signChangeReverseFromStandstill)
{
	// GIVEN: a speed controller with the rover standing still
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, 0.f, 0.f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint accelerates with the acceleration limit
	EXPECT_NEAR(adjusted_speed, -kSignChangeMaxAccel * kDt, 1e-6f);
}

TEST(speedControl, signChangeAdjustedSetpointAlreadyReversed)
{
	// GIVEN: a speed controller with the rover still moving forwards at 0.5 m/s while the adjusted speed setpoint has already reached -0.1 m/s
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, 0.5f, -0.1f, kSignChangeMaxAccel, kSignChangeMaxDecel);

	// THEN: the speed setpoint accelerates with the acceleration limit
	EXPECT_NEAR(adjusted_speed, -0.1f - kSignChangeMaxAccel * kDt, 1e-6f);
}

TEST(speedControl, signChangeReverseWithoutDecelerationLimit)
{
	// GIVEN: a speed controller with the rover driving forwards at 1 m/s and no deceleration limit
	// WHEN: a reverse speed of -1 m/s is requested
	const float adjusted_speed = speedControlStep(-1.f, 1.f, 1.f, kSignChangeMaxAccel, kNoLimit);

	// THEN: the acceleration limit applies as before
	EXPECT_NEAR(adjusted_speed, 1.f - kSignChangeMaxAccel * kDt, 1e-6f);
}

static constexpr int32_t kStickMapMode2 = static_cast<int32_t>(RoverControl::StickMapping::Mode2);
static constexpr bool kCanDriveSideways = true; // Mecanum, so the lateral stick is checked too

TEST(sticksCommandMotion, centeredSticks)
{
	// GIVEN: all sticks centered
	manual_control_setpoint_s manual_control_setpoint{};

	// WHEN: checked with stick mapping mode 2 (left: yaw, right: forward & lateral)
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: no motion is commanded
	EXPECT_FALSE(commands_motion);
}

TEST(sticksCommandMotion, forwardOnRightStick)
{
	// GIVEN: the right stick pushed fully forward, with the left vertical stick centered
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = 1.f;

	// WHEN: checked with stick mapping mode 2, where the right vertical stick drives forward
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: motion is commanded
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, backwardOnRightStick)
{
	// GIVEN: the right stick pulled fully back
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = -1.f;

	// WHEN: checked with stick mapping mode 2
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: motion is commanded
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, lateralOnRightStick)
{
	// GIVEN: the right stick pushed fully to the side
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.roll = 1.f;

	// WHEN: checked with stick mapping mode 2, where the right horizontal stick drives sideways
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: motion is commanded
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, lateralOnRoverThatCannotDriveSideways)
{
	// GIVEN: the right stick pushed fully to the side
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.roll = 1.f;

	// WHEN: checked for a rover that doesn't use the lateral stick (differential, Ackermann)
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, false, false);

	// THEN: no motion is commanded
	EXPECT_FALSE(commands_motion);
}

TEST(sticksCommandMotion, unusedLeftVerticalStick)
{
	// GIVEN: the left vertical stick pushed fully forward, which is the stick commander checked before
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 1.f;

	// WHEN: checked with stick mapping mode 2, where the left vertical stick is not used
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: no motion is commanded
	EXPECT_FALSE(commands_motion);
}

TEST(sticksCommandMotion, yawOnly)
{
	// GIVEN: the left stick pushed fully to the side
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.yaw = 1.f;

	// WHEN: checked with stick mapping mode 2, where the left horizontal stick turns the rover
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: turning alone does not count as motion
	EXPECT_FALSE(commands_motion);
}

TEST(sticksCommandMotion, belowThreshold)
{
	// GIVEN: the forward stick deflected just below the 0.2 threshold
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = 0.19f;

	// WHEN: checked with stick mapping mode 2
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: no motion is commanded
	EXPECT_FALSE(commands_motion);
}

TEST(sticksCommandMotion, atThreshold)
{
	// GIVEN: the forward stick deflected exactly to the 0.2 threshold
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = 0.2f;

	// WHEN: checked with stick mapping mode 2
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, kCanDriveSideways, false);

	// THEN: motion is commanded, matching the previous throttle check that only allowed arming below 0.2
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, parkingLeftStickForward)
{
	// GIVEN: the left stick pushed fully forward
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 1.f;

	// WHEN: checked in manual parking, where the left vertical stick drives the left wheels whatever the stick mapping
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, false, true);

	// THEN: motion is commanded
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, parkingRightStickForward)
{
	// GIVEN: the right stick pushed fully forward
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = 1.f;

	// WHEN: checked in manual parking, where the right vertical stick drives the right wheels
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, false, true);

	// THEN: motion is commanded
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, parkingTurnOnTheSpot)
{
	// GIVEN: the left stick pushed fully forward and the right stick pulled fully back
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 1.f;
	manual_control_setpoint.pitch = -1.f;

	// WHEN: checked in manual parking, where this turns the rover on the spot
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, false, true);

	// THEN: motion is commanded, since both sides of wheels are driven
	EXPECT_TRUE(commands_motion);
}

TEST(sticksCommandMotion, parkingOneStickPartly)
{
	// GIVEN: the left stick pushed to 0.3 and the right stick centered
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 0.3f;

	// WHEN: checked in manual parking
	const bool commands_motion = RoverControl::sticksCommandMotion(manual_control_setpoint, kStickMapMode2, false, true);

	// THEN: motion is commanded, since the left wheels are driven
	EXPECT_TRUE(commands_motion);
}
