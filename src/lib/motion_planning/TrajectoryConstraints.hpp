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

struct VehicleDynamicLimits {
	float accept_rad;

	float max_acc;
	float max_jerk;

	float max_speed;

	// TODO: remove this
	float max_acc_radius_scale;
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
		const Vector3f &next_target, float exit_speed, const VehicleDynamicLimits &config_xy)
{
	const float distance_target_next = (target - next_target).xy().norm();

	const bool target_next_different = distance_target_next  > 0.001f;
	const bool waypoint_overlap = distance_target_next < config_xy.accept_rad;

	float speed_at_target = 0.0f;

	if (target_next_different &&
	    !waypoint_overlap
	   ) {
		const float alpha = acosf(Vector2f((target - start_position).xy()).unit_or_zero().dot(
						  Vector2f((target - next_target).xy()).unit_or_zero()));
		const float safe_alpha = constrain(alpha, 0.f, M_PI_F - FLT_EPSILON);
		float accel_tmp = config_xy.max_acc_radius_scale * config_xy.max_acc;
		float max_speed_in_turn = computeMaxSpeedInWaypoint(safe_alpha, accel_tmp, config_xy.accept_rad);
		speed_at_target = min(max_speed_in_turn, exit_speed, config_xy.max_speed);
	}

	float start_to_target = (start_position - target).xy().norm();
	float max_speed = computeMaxSpeedFromDistance(config_xy.max_jerk, config_xy.max_acc, start_to_target, speed_at_target);

	return min(config_xy.max_speed, max_speed);
}

/*
 * This function computes the maximum speed XY that can be travelled, given a set of waypoints and vehicle dynamics
 *
 * The first waypoint should be the starting location, and the later waypoints the desired points to be followed.
 *
 * @param waypoints the list of waypoints to be followed, the first of which should be the starting location
 * @param config the vehicle dynamic limits
 *
 * @return the maximum speed at waypoint[0] which allows it to follow the trajectory while respecting the dynamic limits
 */
template <int N>
float computeXYSpeedFromWaypoints(const Vector3f waypoints[N], const VehicleDynamicLimits &config_xy)
{
	static_assert(N >= 2, "Need at least 2 points to compute speed");

	float max_speed = 0.f;

	// go backwards through the waypoints
	for (int i = (N - 2); i >= 0; i--) {
		max_speed = computeStartXYSpeedFromWaypoints(waypoints[i],
				waypoints[i + 1],
				waypoints[min(i + 2, N - 1)],
				max_speed, config_xy);
	}

	return max_speed;
}

inline float computeStartZSpeedFromWaypoints(const Vector3f &start_position, const Vector3f &target,
		float exit_speed, const VehicleDynamicLimits &config_z)
{
	const float start_to_target = fabsf(target(2) - start_position(2));
	const float accel_tmp = config_z.max_acc_radius_scale * config_z.max_acc;
	const float max_speed = computeMaxSpeedFromDistance(config_z.max_jerk, accel_tmp, start_to_target, exit_speed);

	return min(config_z.max_speed, max_speed);
}

template <int N>
float computeZSpeedFromWaypoints(const Vector3f waypoints[N], const VehicleDynamicLimits &config_z)
{
	static_assert(N >= 2, "Need at least 2 points to compute speed");

	float max_speed = 0.f;

	// go backwards through the waypoints
	for (int i = (N - 2); i >= 0; i--) {
		max_speed = computeStartZSpeedFromWaypoints(waypoints[i],
				waypoints[i + 1],
				max_speed, config_z);
	}

	return max_speed;
}

/*
 * Constrain the 3D vector given a maximum XY norm
 * If the XY norm of the 3D vector is larger than the maximum norm, the whole vector
 * is scaled down to respect the constraint.
 * If the maximum norm is small (defined by the "accuracy" parameter),
 * only the XY components are scaled down to avoid affecting
 * Z in case of numerical issues
 */
inline void clampToXYNorm(Vector3f &target, float max_xy_norm, float accuracy = FLT_EPSILON)
{
	const float xynorm = target.xy().norm();
	const float scale_factor = (xynorm > FLT_EPSILON)
				   ? max_xy_norm / xynorm
				   : 1.f;

	if (scale_factor < 1.f) {
		if (max_xy_norm < accuracy && xynorm < accuracy) {
			target.xy() = Vector2f(target) * scale_factor;

		} else {
			target *= scale_factor;
		}
	}
}

/*
 * Constrain the 3D vector given a maximum Z norm
 * If the Z component of the 3D vector is larger than the maximum norm, the whole vector
 * is scaled down to respect the constraint.
 * If the maximum norm is small (defined by the "accuracy parameter),
 * only the Z component is scaled down to avoid affecting
 * XY in case of numerical issues
 */
inline void clampToZNorm(Vector3f &target, float max_z_norm, float accuracy = FLT_EPSILON)
{
	const float znorm = fabs(target(2));
	const float scale_factor = (znorm > FLT_EPSILON)
				   ? max_z_norm / znorm
				   : 1.f;

	if (scale_factor < 1.f) {
		if (max_z_norm < accuracy && znorm < accuracy) {
			target(2) *= scale_factor;

		} else {
			target *= scale_factor;
		}
	}
}

}
}
