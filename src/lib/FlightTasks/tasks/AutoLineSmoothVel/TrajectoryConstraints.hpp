/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_defines.h>

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace math
{
namespace trajectory
{
using matrix::Vector3f;
using matrix::Vector2f;

struct vehicle_dynamic_limits {
	float z_accept_rad;
	float xy_accept_rad;

	float max_acc_xy;
	float max_jerk;

	float max_speed_xy;

	// TODO: remove this
	float max_acc_xy_radius_scale;
};

/*
 * Compute the maximum allowed speed at the waypoint assuming that we want to
 * connect the two lines (current-target and target-next)
 * with a tangent circle with constant speed and desired centripetal acceleration: a_centripetal = speed^2 / radius
 * The circle should in theory start and end at the intersection of the lines and the waypoint's acceptance radius.
 * This is not exactly true in reality since Navigator switches the waypoint so we have to take in account that
 * the real acceptance radius is smaller.
 *
 */
inline float computeStartXYSpeedFromWaypoints(const Vector3f &start_position, const Vector3f &target,
		const Vector3f &next_target, float final_speed, const vehicle_dynamic_limits &config)
{
	float speed_at_target = 0.0f;
	const float distance_target_next = (target - next_target).xy().norm();

	const bool target_next_different = distance_target_next  > 0.001f;
	const bool waypoint_overlap = (target - next_target).xy().norm() < config.xy_accept_rad;
	const bool has_reached_altitude = fabsf(target(2) - start_position(2)) < config.z_accept_rad;
	const bool altitude_stays_same = fabsf(next_target(2) - target(2)) < config.z_accept_rad;

	if (target_next_different &&
	    !waypoint_overlap &&
	    has_reached_altitude &&
	    altitude_stays_same
	   ) {
		const float max_speed_current_next = math::trajectory::computeMaxSpeedFromDistance(config.max_jerk,
						     config.max_acc_xy,
						     distance_target_next,
						     final_speed);
		const float alpha = acosf(Vector2f((target - start_position).xy()).unit_or_zero().dot(
						  Vector2f((target - next_target).xy()).unit_or_zero()));
		float accel_tmp = config.max_acc_xy_radius_scale * config.max_acc_xy;
		float max_speed_in_turn = math::trajectory::computeMaxSpeedInWaypoint(alpha,
					  accel_tmp,
					  config.xy_accept_rad);
		speed_at_target = math::min(math::min(max_speed_in_turn,
						      max_speed_current_next),
					    config.max_speed_xy);
	}

	return math::min(math::trajectory::computeMaxSpeedFromDistance(config.max_jerk,
			 config.max_acc_xy,
			 (start_position - target).xy().norm(),
			 speed_at_target), config.max_speed_xy);
}

inline void clampToXYNorm(Vector3f &target, float maxXYNorm)
{
	const float xynorm = target.xy().norm();
	const float scale_factor = maxXYNorm / xynorm;

	if (scale_factor < 1 && PX4_ISFINITE(scale_factor) && xynorm > FLT_EPSILON) {
		target *= scale_factor;
	}
}

inline void clampToZNorm(Vector3f &target, float maxZNorm)
{
	float znorm = fabs(target(2));
	const float scale_factor = maxZNorm / znorm;

	if (scale_factor < 1 && PX4_ISFINITE(scale_factor) && znorm > FLT_EPSILON) {
		target *= scale_factor;
	}
}

}
}
