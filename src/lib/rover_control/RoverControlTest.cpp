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

TEST(mapSticks, mappings)
{
	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.throttle = 0.1f;
	manual_control_setpoint.yaw = 0.2f;
	manual_control_setpoint.pitch = 0.3f;
	manual_control_setpoint.roll = 0.4f;

	const auto expectSticks = [&](int32_t stick_map, float forward, float lateral, float yaw) {
		const RoverControl::ManualSticks sticks = RoverControl::mapSticks(manual_control_setpoint, stick_map);
		EXPECT_FLOAT_EQ(sticks.forward, forward);
		EXPECT_FLOAT_EQ(sticks.lateral, lateral);
		EXPECT_FLOAT_EQ(sticks.yaw, yaw);
	};

	// Sticks of a mode 2 transmitter: throttle = left vertical, yaw = left horizontal,
	// pitch = right vertical, roll = right horizontal
	expectSticks(1, 0.1f, 0.4f, 0.2f); // Left: forward & yaw, right: lateral
	expectSticks(2, 0.3f, 0.4f, 0.2f); // Left: yaw, right: forward & lateral
	expectSticks(3, 0.1f, 0.2f, 0.4f); // Left: forward & lateral, right: yaw
	expectSticks(4, 0.3f, 0.2f, 0.4f); // Left: lateral, right: forward & yaw

	// Out of range values fall back to the default mapping (mode 3)
	expectSticks(-1, 0.1f, 0.2f, 0.4f);
	expectSticks(0, 0.1f, 0.2f, 0.4f);
	expectSticks(5, 0.1f, 0.2f, 0.4f);
}

static constexpr float kDt = 0.1f; // Controller update interval used by all tests below [s]

TEST(throttleControl, invalidInputs)
{
	SlewRate<float> motor_setpoint;
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::throttleControl(motor_setpoint, NAN, 0.f, 1.f, 1.f, 2.f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::throttleControl(motor_setpoint, 1.f, NAN, 1.f, 1.f, 2.f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 1.f, 1.f, 2.f, NAN)));
}

TEST(throttleControl, accelerationLimit)
{
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 2.f; // [m/s^2], different from max_accel so the test notices if the wrong one is used
	const float max_thr_spd = 2.f; // [m/s]
	const float step = max_accel / max_thr_spd * kDt; // Largest throttle change per update
	SlewRate<float> motor_setpoint(0.f);
	float throttle = 0.f;

	for (int i = 1; i <= 5; i++) {
		throttle = RoverControl::throttleControl(motor_setpoint, 1.f, throttle, max_accel, max_decel, max_thr_spd, kDt);
		EXPECT_NEAR(throttle, step * i, 1e-5f);
	}
}

TEST(throttleControl, decelerationLimit)
{
	// Same sign only: a setpoint of opposite sign is covered by #29727
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 2.f; // [m/s^2]
	const float max_thr_spd = 2.f; // [m/s]
	const float step = max_decel / max_thr_spd * kDt; // Largest throttle change per update
	SlewRate<float> motor_setpoint(1.f);
	float throttle = 1.f;

	for (int i = 1; i <= 3; i++) {
		throttle = RoverControl::throttleControl(motor_setpoint, 0.5f, throttle, max_accel, max_decel, max_thr_spd, kDt);
		EXPECT_NEAR(throttle, 1.f - step * i, 1e-5f);
	}
}

TEST(throttleControl, noLimitConfigured)
{
	SlewRate<float> motor_setpoint(0.f);
	// No acceleration limit: the output jumps straight to the setpoint
	EXPECT_FLOAT_EQ(RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 0.f, 1.f, 2.f, kDt), 1.f);

	// No speed at full throttle: the limits can't be converted, so the output also jumps
	motor_setpoint.setForcedValue(0.f);
	EXPECT_FLOAT_EQ(RoverControl::throttleControl(motor_setpoint, 1.f, 0.f, 1.f, 1.f, 0.f, kDt), 1.f);
}

TEST(throttleControl, reinitializeWhenCurrentIsCloser)
{
	// The slew rate state (0) lags far behind the applied throttle (0.9). One limited step would move it
	// further from the applied throttle than the setpoint is, so it snaps to the setpoint instead.
	SlewRate<float> motor_setpoint(0.f);
	EXPECT_FLOAT_EQ(RoverControl::throttleControl(motor_setpoint, 1.f, 0.9f, 1.f, 1.f, 2.f, kDt), 1.f);
	EXPECT_FLOAT_EQ(motor_setpoint.getState(), 1.f);
}

TEST(attitudeControl, invalidInputs)
{
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, NAN, 0.f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, NAN, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, 0.f, NAN)));
}

TEST(attitudeControl, wrapAround)
{
	// Yaw +3.1 rad and setpoint -3.1 rad are 0.083 rad apart across +-pi, not 6.2 rad the long way round
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);
	const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 3.1f, -3.1f, kDt);
	EXPECT_NEAR(yaw_rate, 2.f * M_PI_F - 6.2f, 1e-4f);
	EXPECT_GT(yaw_rate, 0.f);
}

TEST(attitudeControl, slewRate)
{
	const float yaw_slew_rate = 0.5f; // [rad/s]
	const float step = yaw_slew_rate * kDt; // Largest change of the adjusted setpoint per update
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);

	for (int i = 1; i <= 3; i++) {
		const float yaw_rate = RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, yaw_slew_rate, 0.f, 1.f, kDt);
		EXPECT_NEAR(adjusted_yaw_setpoint.getState(), step * i, 1e-5f);
		EXPECT_NEAR(yaw_rate, step * i, 1e-5f); // P gain 1 at yaw 0
	}
}

TEST(attitudeControl, noSlewRate)
{
	SlewRateYaw<float> adjusted_yaw_setpoint;
	PID pid_yaw;
	pid_yaw.setGains(1.f, 0.f, 0.f);
	pid_yaw.setOutputLimit(10.f);
	EXPECT_FLOAT_EQ(RoverControl::attitudeControl(adjusted_yaw_setpoint, pid_yaw, 0.f, 0.f, 1.f, kDt), 1.f);
	EXPECT_FLOAT_EQ(adjusted_yaw_setpoint.getState(), 1.f);
}

TEST(speedControl, invalidInputs)
{
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::speedControl(speed_with_rate_limit, pid_speed, NAN, 0.f, 0.f, 0.f, 2.f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, NAN, 0.f, 0.f, 2.f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 2.f, NAN)));
}

TEST(speedControl, feedForwardOnly)
{
	// With all PID gains at 0, half the speed at full throttle maps to half throttle
	const float max_thr_speed = 2.f; // [m/s]
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setOutputLimit(1.f);
	EXPECT_NEAR(RoverControl::speedControl(speed_with_rate_limit, pid_speed, max_thr_speed / 2.f, max_thr_speed / 2.f,
					       0.f, 0.f, max_thr_speed, kDt), 0.5f, 1e-5f);
}

TEST(speedControl, feedbackOnly)
{
	// Without a speed at full throttle there is no feed-forward, so only the P term acts on the speed error
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(0.5f, 0.f, 0.f);
	pid_speed.setOutputLimit(1.f);
	EXPECT_NEAR(RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 0.f, kDt), 0.5f,
		    1e-5f);
}

TEST(speedControl, zeroSetpointResetsIntegral)
{
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(0.f, 1.f, 0.f);
	pid_speed.setIntegralLimit(1.f);
	pid_speed.setOutputLimit(1.f);

	RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 0.f, kDt);
	EXPECT_NEAR(pid_speed.getIntegral(), 1.f * kDt, 1e-5f); // I gain 1, error 1 m/s for one update

	RoverControl::speedControl(speed_with_rate_limit, pid_speed, 0.f, 0.f, 0.f, 0.f, 0.f, kDt);
	EXPECT_FLOAT_EQ(pid_speed.getIntegral(), 0.f);
}

TEST(speedControl, outputClamped)
{
	// Full feed-forward plus a positive P term would exceed full throttle
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;
	pid_speed.setGains(1.f, 0.f, 0.f);
	pid_speed.setOutputLimit(10.f);
	EXPECT_FLOAT_EQ(RoverControl::speedControl(speed_with_rate_limit, pid_speed, 1.f, 0.f, 0.f, 0.f, 1.f, kDt), 1.f);
	EXPECT_FLOAT_EQ(RoverControl::speedControl(speed_with_rate_limit, pid_speed, -1.f, 0.f, 0.f, 0.f, 1.f, kDt), -1.f);
}

TEST(rateControl, invalidInputs)
{
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, NAN, 0.f, 1.f, 1.f,
				  0.f, 0.f, 0.5f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, NAN, 1.f, 1.f,
				  0.f, 0.f, 0.5f, kDt)));
	EXPECT_FALSE(PX4_ISFINITE(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f,
				  0.f, 0.f, 0.5f, NAN)));
}

TEST(rateControl, feedForward)
{
	// speed difference = yaw rate * wheel_track / 2 * yaw_rate_corr = 2 * 0.5 / 2 * 1.5 = 0.75 m/s,
	// which is 0.75 of the speed at full throttle (1 m/s)
	const float yaw_rate_setpoint = 2.f; // [rad/s]
	const float wheel_track = 0.5f; // [m]
	const float yaw_rate_corr = 1.5f;
	const float max_thr_speed = 1.f; // [m/s]
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setOutputLimit(1.f);
	EXPECT_NEAR(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, yaw_rate_setpoint,
					      yaw_rate_setpoint, max_thr_speed, yaw_rate_corr, 0.f, 0.f, wheel_track, kDt), 0.75f, 1e-5f);
}

TEST(rateControl, feedbackOnly)
{
	// Without a wheel track there is no feed-forward, so only the P term acts on the yaw rate error
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(0.5f, 0.f, 0.f);
	pid_yaw_rate.setOutputLimit(1.f);
	EXPECT_NEAR(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f,
					      kDt), 0.5f, 1e-5f);
}

TEST(rateControl, zeroSetpointResetsIntegral)
{
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(0.f, 1.f, 0.f);
	pid_yaw_rate.setIntegralLimit(1.f);
	pid_yaw_rate.setOutputLimit(1.f);

	RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, kDt);
	EXPECT_NEAR(pid_yaw_rate.getIntegral(), 1.f * kDt, 1e-5f); // I gain 1, error 1 rad/s for one update

	RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, kDt);
	EXPECT_FLOAT_EQ(pid_yaw_rate.getIntegral(), 0.f);
}

TEST(rateControl, outputClamped)
{
	// Feed-forward of a full speed difference plus a positive P term would exceed the normalized range
	const float wheel_track = 2.f; // [m], so a yaw rate of 1 rad/s asks for the full 1 m/s speed difference
	SlewRate<float> adjusted_yaw_rate_setpoint;
	PID pid_yaw_rate;
	pid_yaw_rate.setGains(1.f, 0.f, 0.f);
	pid_yaw_rate.setOutputLimit(10.f);
	EXPECT_FLOAT_EQ(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f,
			wheel_track, kDt), 1.f);
	EXPECT_FLOAT_EQ(RoverControl::rateControl(adjusted_yaw_rate_setpoint, pid_yaw_rate, -1.f, 0.f, 1.f, 1.f, 0.f, 0.f,
			wheel_track, kDt), -1.f);
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
	// Reference point and three nearby waypoints, distinct so a swapped output is caught
	MapProjection global_ned_proj_ref;
	global_ned_proj_ref.initReference(47.397742, 8.545594);
	position_setpoint_triplet_s position_setpoint_triplet{};
	position_setpoint_triplet.previous = waypoint(true, 47.397742, 8.545594);
	position_setpoint_triplet.current = waypoint(true, 47.398742, 8.545594);
	position_setpoint_triplet.next = waypoint(true, 47.398742, 8.546594);
	Vector2f curr_pos_ned(1.f, 2.f);
	Vector2f curr_wp_ned, prev_wp_ned, next_wp_ned;

	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);

	EXPECT_EQ(prev_wp_ned, global_ned_proj_ref.project(47.397742, 8.545594));
	EXPECT_EQ(curr_wp_ned, global_ned_proj_ref.project(47.398742, 8.545594));
	EXPECT_EQ(next_wp_ned, global_ned_proj_ref.project(47.398742, 8.546594));
	EXPECT_GT(curr_wp_ned(0), 100.f); // About 111 m north of the reference
}

TEST(globalToLocalSetpointTriplet, invalidWaypoints)
{
	MapProjection global_ned_proj_ref;
	global_ned_proj_ref.initReference(47.397742, 8.545594);
	position_setpoint_triplet_s position_setpoint_triplet{};
	position_setpoint_triplet.previous = waypoint(true, static_cast<double>(NAN), 8.545594);
	position_setpoint_triplet.current = waypoint(false, 47.398742, 8.545594);
	position_setpoint_triplet.next = waypoint(false, 47.398742, 8.546594);
	Vector2f curr_pos_ned(1.f, 2.f);
	Vector2f curr_wp_ned, prev_wp_ned, next_wp_ned;

	// Invalid or NaN current and previous waypoints fall back to the current position, an invalid next one is NaN
	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);
	EXPECT_EQ(curr_wp_ned, curr_pos_ned);
	EXPECT_EQ(prev_wp_ned, curr_pos_ned);
	EXPECT_FALSE(next_wp_ned.isAllFinite());

	// Without a valid current position there is nothing to fall back to
	curr_pos_ned = Vector2f(NAN, NAN);
	RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
			curr_pos_ned, global_ned_proj_ref);
	EXPECT_FALSE(curr_wp_ned.isAllFinite());
	EXPECT_FALSE(prev_wp_ned.isAllFinite());
}

TEST(throttleControl, decelerationLimitOnSignChange)
{
	// Distinct limits so the applied one can be read from a single step
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 4.f; // [m/s^2]
	const float no_limit = -1.f;
	const float max_thr_spd = 2.f; // [m/s]
	const float dt = 0.1f; // [s]
	const float accel_step = max_accel / max_thr_spd * dt;
	const float decel_step = max_decel / max_thr_spd * dt;
	SlewRate<float> motor_setpoint;

	const auto step = [&](float throttle_setpoint, float current_motor_setpoint, float accel, float decel) {
		motor_setpoint.setForcedValue(current_motor_setpoint);
		return RoverControl::throttleControl(motor_setpoint, throttle_setpoint, current_motor_setpoint, accel, decel,
						     max_thr_spd, dt);
	};

	// Full reverse while driving forwards (and vice versa) brakes towards zero first: deceleration limit
	EXPECT_NEAR(step(-1.f, 0.5f, max_accel, max_decel), 0.5f - decel_step, 1e-6f);
	EXPECT_NEAR(step(1.f, -0.5f, max_accel, max_decel), -0.5f + decel_step, 1e-6f);
	EXPECT_NEAR(step(-0.2f, 0.5f, max_accel, max_decel), 0.5f - decel_step, 1e-6f);
	EXPECT_NEAR(step(-1.f, 0.5f, no_limit, max_decel), 0.5f - decel_step, 1e-6f);

	// Once the sign has changed or from standstill: acceleration limit
	EXPECT_NEAR(step(-1.f, -0.2f, max_accel, max_decel), -0.2f - accel_step, 1e-6f);
	EXPECT_NEAR(step(1.f, 0.2f, max_accel, max_decel), 0.2f + accel_step, 1e-6f);
	EXPECT_NEAR(step(-1.f, 0.f, max_accel, max_decel), -accel_step, 1e-6f);

	// Without a deceleration limit the acceleration limit applies as before
	EXPECT_NEAR(step(-1.f, 0.5f, max_accel, no_limit), 0.5f - accel_step, 1e-6f);
}

TEST(speedControl, decelerationLimitOnSignChange)
{
	// Distinct limits so the applied one can be read from a single step
	const float max_accel = 1.f; // [m/s^2]
	const float max_decel = 4.f; // [m/s^2]
	const float no_limit = -1.f;
	const float max_thr_speed = 0.f; // [m/s] No feed forward, only the rate limited speed setpoint is checked
	const float dt = 0.1f; // [s]
	SlewRate<float> speed_with_rate_limit;
	PID pid_speed;

	// One update starting from the rate limited setpoint adjusted_speed
	const auto step = [&](float speed_setpoint, float vehicle_speed, float adjusted_speed, float accel, float decel) {
		speed_with_rate_limit.setForcedValue(adjusted_speed);
		RoverControl::speedControl(speed_with_rate_limit, pid_speed, speed_setpoint, vehicle_speed, accel, decel,
					   max_thr_speed, dt);
		return speed_with_rate_limit.getState();
	};

	// Reverse setpoint while driving forwards (and vice versa) brakes towards zero first: deceleration limit
	EXPECT_NEAR(step(-1.f, 1.f, 1.f, max_accel, max_decel), 1.f - max_decel * dt, 1e-6f);
	EXPECT_NEAR(step(1.f, -1.f, -1.f, max_accel, max_decel), -1.f + max_decel * dt, 1e-6f);
	EXPECT_NEAR(step(-2.f, 1.f, 1.f, max_accel, max_decel), 1.f - max_decel * dt, 1e-6f);
	EXPECT_NEAR(step(-1.f, 1.f, 1.f, no_limit, max_decel), 1.f - max_decel * dt, 1e-6f);

	// Smaller setpoint of the same sign: deceleration limit
	EXPECT_NEAR(step(0.1f, 0.5f, 0.5f, max_accel, max_decel), 0.5f - max_decel * dt, 1e-6f);

	// Once the sign has changed or from standstill: acceleration limit
	EXPECT_NEAR(step(-1.f, -0.5f, -0.5f, max_accel, max_decel), -0.5f - max_accel * dt, 1e-6f);
	EXPECT_NEAR(step(1.f, 0.5f, 0.5f, max_accel, max_decel), 0.5f + max_accel * dt, 1e-6f);
	EXPECT_NEAR(step(-1.f, 0.f, 0.f, max_accel, max_decel), -max_accel * dt, 1e-6f);

	// The rate limited setpoint has changed sign while the vehicle speed lags behind: acceleration limit
	EXPECT_NEAR(step(-1.f, 0.5f, -0.1f, max_accel, max_decel), -0.1f - max_accel * dt, 1e-6f);

	// Without a deceleration limit the acceleration limit applies as before
	EXPECT_NEAR(step(-1.f, 1.f, 1.f, max_accel, no_limit), 1.f - max_accel * dt, 1e-6f);
}
