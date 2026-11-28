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

#include "DriftAngleEstimator.hpp"

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

void DriftAngleEstimator::update(const vehicle_local_position_s &vehicle_local_position, const float expected_direction,
				 const float speed_setpoint)
{
	if (!vehicle_local_position.v_xy_valid || !PX4_ISFINITE(vehicle_local_position.heading)
	    || !PX4_ISFINITE(expected_direction)) {
		return;
	}

	const float direction = vehicle_local_position.heading + expected_direction;
	const matrix::Vector2f unit_direction{cosf(direction), sinf(direction)};
	const matrix::Vector2f ground_velocity{vehicle_local_position.vx, vehicle_local_position.vy};
	const float measured_speed = ground_velocity.dot(unit_direction);

	// The velocity of a slow vehicle, or of one moving against the expected direction, is dominated by noise and
	// maneuvering, not by the current
	if (measured_speed < math::max(MIN_SPEED, EVH_FACTOR * vehicle_local_position.evh)) {
		return;
	}

	// Without a speed setpoint all of the speed along the expected direction is attributed to the propulsion
	const float commanded_speed = PX4_ISFINITE(speed_setpoint) ? fabsf(speed_setpoint) : measured_speed;

	if (commanded_speed < MIN_SPEED) {
		return;
	}

	const matrix::Vector2f current_velocity = ground_velocity - unit_direction * commanded_speed;

	if (!_valid) {
		_current_velocity_filter.reset(current_velocity);
		_valid = true;

	} else {
		// After a pause in the estimation the stale estimate is filtered towards the new one rather than replaced
		_current_velocity_filter.update(current_velocity,
						math::min(vehicle_local_position.timestamp - _timestamp, MAX_DT_US));
	}

	_measured_speed = measured_speed;
	_timestamp = vehicle_local_position.timestamp;
}

float DriftAngleEstimator::getDriftAngle(const float course_setpoint, const float speed_setpoint) const
{
	const float speed = PX4_ISFINITE(speed_setpoint) ? fabsf(speed_setpoint) : _measured_speed;

	if (!_valid || !PX4_ISFINITE(course_setpoint) || speed < FLT_EPSILON) {
		return 0.f;
	}

	const matrix::Vector2f current_velocity = _current_velocity_filter.getState();
	// Positive when the current sets the vehicle to the right of the course setpoint
	const float current_across = -current_velocity(0) * sinf(course_setpoint) + current_velocity(1) * cosf(
					     course_setpoint);
	// speed * sin(drift_angle) has to cancel the current across the course. Limiting the ratio limits the angle and
	// keeps asinf in its domain when the current is faster than the vehicle.
	const float max_ratio = sinf(MAX_DRIFT_ANGLE);
	return asinf(math::constrain(current_across / speed, -max_ratio, max_ratio));
}

float calcHeadingSetpoint(const float course_setpoint, const float drift_angle, const float expected_direction)
{
	return matrix::wrap_pi(course_setpoint - expected_direction - drift_angle);
}
