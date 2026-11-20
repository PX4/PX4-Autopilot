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

void DriftAngleEstimator::update(const vehicle_local_position_s &vehicle_local_position, const float expected_direction)
{
	if (!vehicle_local_position.v_xy_valid || !PX4_ISFINITE(vehicle_local_position.heading)
	    || !PX4_ISFINITE(expected_direction)) {
		return;
	}

	// Ground velocity along and across the expected direction of travel
	const float direction = vehicle_local_position.heading + expected_direction;
	const float speed_along = vehicle_local_position.vx * cosf(direction) + vehicle_local_position.vy * sinf(direction);
	const float speed_across = -vehicle_local_position.vx * sinf(direction) + vehicle_local_position.vy * cosf(direction);

	// The direction of a slow velocity, or one against the expected direction, says nothing about the drift angle
	if (speed_along < math::max(MIN_SPEED, EVH_FACTOR * vehicle_local_position.evh)) {
		return;
	}

	const float drift_angle = math::constrain(atan2f(speed_across, speed_along), -MAX_DRIFT_ANGLE, MAX_DRIFT_ANGLE);

	if (!_valid) {
		_filter.reset(drift_angle);
		_valid = true;

	} else {
		// After a pause in the estimation the stale estimate is filtered towards the new one rather than replaced
		_filter.update(drift_angle, math::min(vehicle_local_position.timestamp - _timestamp, MAX_DT_US));
	}

	_timestamp = vehicle_local_position.timestamp;
}
