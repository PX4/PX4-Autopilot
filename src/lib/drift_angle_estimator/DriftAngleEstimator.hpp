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
 * @file DriftAngleEstimator.hpp
 *
 * Estimates the drift angle of a surface vehicle from its ground velocity.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/math.hpp>
#include <uORB/topics/vehicle_local_position.h>

/**
 * Estimates the drift angle, the angle from the expected direction of travel to the course over ground.
 * The expected direction of travel is given in body frame, forwards by default. An omnidirectional vehicle passes
 * the direction it pushes in, e.g. its thrust direction.
 * Course control subtracts the drift angle from the course to get the heading setpoint, like the fixed wing corrects
 * its course setpoint for the wind, so the vehicle moves over ground along the course despite wheel slip or current.
 *
 * Internally the current of the wind and current triangle is estimated, the part of the ground velocity that the
 * commanded speed along the expected direction does not explain (current, wind or wheel slip). It is fixed in NED
 * while the vehicle turns, unlike the drift angle, so the drift angle is solved from it for each course setpoint.
 */
class DriftAngleEstimator
{
public:
	/**
	 * Update the current estimate from the ground velocity minus the commanded velocity.
	 * Only updated while moving along the expected direction fast enough for the velocity to be meaningful,
	 * kept otherwise.
	 * @param vehicle_local_position Local position with the velocity and heading estimate
	 * @param expected_direction [rad] [@frame Body] Expected direction of travel, 0 forwards, pi/2 right, pi backwards
	 * @param speed_setpoint [m/s] Commanded speed, only its magnitude is used as the speed along the expected direction.
	 * NaN if the speed is not controlled: the measured speed along the expected direction is used instead, so only
	 * the current across the expected direction is estimated.
	 */
	void update(const vehicle_local_position_s &vehicle_local_position, float expected_direction = 0.f,
		    float speed_setpoint = NAN);

	/**
	 * Solve the wind and current triangle: the offset of the direction of travel whose velocity across the course
	 * setpoint cancels the current across it.
	 * @param course_setpoint [rad] [@frame NED] Intended direction of motion over ground
	 * @param speed_setpoint [m/s] Commanded speed, only its magnitude is used. NaN if the speed is not controlled:
	 * the speed measured at the last update is used instead.
	 * @return Drift angle [rad], 0 until estimated since the last reset, without a course setpoint or at standstill
	 */
	float getDriftAngle(float course_setpoint, float speed_setpoint = NAN) const;

	void reset() { _valid = false; }

	static constexpr float MIN_SPEED{.5f}; ///< [m/s] Speed along the expected direction, measured and commanded, below which nothing is estimated
	static constexpr float EVH_FACTOR{3.f}; ///< Factor of the velocity standard deviation the speed has to exceed
	static constexpr float MAX_DRIFT_ANGLE{M_PI_F / 4.f}; ///< [rad] Limits the heading offset from the course

private:
	static constexpr uint64_t TIME_CONSTANT_US{1000000}; ///< Filters the yaw oscillations of the vehicle
	static constexpr uint64_t MAX_DT_US{100000}; ///< Sample interval limit, as the estimate pauses at low speed

	// Filtered as a vector in NED: filtering its direction and speed separately wraps at +-pi and biases the speed
	AlphaFilter<matrix::Vector2f> _current_velocity_filter{TIME_CONSTANT_US};
	float _measured_speed{0.f}; ///< [m/s] Speed along the expected direction at the last update, for an uncontrolled speed
	bool _valid{false};
	hrt_abstime _timestamp{0};
};

/**
 * Heading setpoint that moves the vehicle over ground along the course setpoint, the inverse of the drift angle estimate:
 * course = heading + expected direction of travel + drift angle.
 * @param course_setpoint [rad] [@frame NED] Intended direction of motion over ground
 * @param drift_angle [rad] Drift angle relative to the expected direction of travel, see DriftAngleEstimator
 * @param expected_direction [rad] [@frame Body] Expected direction of travel, 0 forwards, pi/2 right, pi backwards
 * @return [rad] [@range -pi, pi] Heading setpoint
 */
float calcHeadingSetpoint(float course_setpoint, float drift_angle, float expected_direction = 0.f);
