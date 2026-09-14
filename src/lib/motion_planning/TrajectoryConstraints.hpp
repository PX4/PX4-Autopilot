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
 * If the next waypoint is closer to the target than the acceptance radius, the tangent circle cannot be
 * anchored at the acceptance radius. Instead of forcing a full stop at the target in that case, the circle is
 * shrunk to the length of the short segment: the speed limit is then still derived from the turn angle, so
 * a waypoint that is passed (nearly) in a straight line costs no speed, while a sharp turn onto a short
 * segment is limited to the (small) speed that turn actually allows.
 *
 * @param exit_speed the speed the vehicle may have when leaving the target towards next_target
 * @return the maximum speed at the target, zero if no continuation past the target is known
 */
inline float computeXYSpeedAtWaypoint(const Vector3f &start_position, const Vector3f &target,
				      const Vector3f &next_target, float exit_speed, const VehicleDynamicLimits &config)
{
	const float distance_target_next = (target - next_target).xy().norm();

	const bool target_next_different = distance_target_next  > 0.001f;

	if (!target_next_different) {
		return 0.f;
	}

	const float alpha = acosf(Vector2f((target - start_position).xy()).unit_or_zero().dot(
					  Vector2f((target - next_target).xy()).unit_or_zero()));
	const float safe_alpha = constrain(alpha, 0.f, M_PI_F - FLT_EPSILON);
	const float accel_tmp = config.max_acc_xy_radius_scale * config.max_acc_xy;
	// the turn circle can only be as large as the shorter of the acceptance radius and the next segment
	const float turn_anchor_distance = min(config.xy_accept_rad, distance_target_next);
	const float max_speed_in_turn = computeMaxSpeedInWaypoint(safe_alpha, accel_tmp, turn_anchor_distance);

	return min(max_speed_in_turn, exit_speed, config.max_speed_xy);
}

/*
 * Compute the maximum speed at the start position such that the vehicle can still slow down to the speed
 * the turn at the target allows, see computeXYSpeedAtWaypoint().
 */
inline float computeStartXYSpeedFromWaypoints(const Vector3f &start_position, const Vector3f &target,
		const Vector3f &next_target, float exit_speed, const VehicleDynamicLimits &config)
{
	const float speed_at_target = computeXYSpeedAtWaypoint(start_position, target, next_target, exit_speed, config);

	float start_to_target = (start_position - target).xy().norm();
	float max_speed = computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, start_to_target, speed_at_target);

	return min(config.max_speed_xy, max_speed);
}

/*
 * This function computes the maximum speed XY that can be travelled, given a set of waypoints and vehicle dynamics
 *
 * The first waypoint should be the starting location, and the later waypoints the desired points to be followed.
 *
 * @param waypoints the list of waypoints to be followed, the first of which should be the starting location
 * @param num_waypoints number of entries in waypoints (and acceptance_radii if given), at least 2
 * @param final_velocity velocity the vehicle may have when leaving the last waypoint: the norm is the speed, the
 *        direction the one of the path continuing after it. A zero or non-finite vector means the vehicle stops there.
 * @param config the vehicle dynamic limits, config.xy_accept_rad is used for every waypoint without its own radius
 * @param acceptance_radii optional acceptance radius of each waypoint, used for the turn at that waypoint
 *
 * @return the maximum speed at waypoint[0] which allows it to follow the trajectory while respecting the dynamic limits
 */
inline float computeXYSpeedFromWaypoints(const Vector3f waypoints[], int num_waypoints, const Vector3f &final_velocity,
		VehicleDynamicLimits config, const float acceptance_radii[] = nullptr)
{
	if (num_waypoints < 2) {
		return 0.f;
	}

	const float default_acceptance_radius = config.xy_accept_rad;
	const Vector3f &last = waypoints[num_waypoints - 1];

	// The turn at the last waypoint is evaluated against a point placed along the continuing path at the distance
	// the turn circle would be anchored at anyway, so the final speed is only limited by the turn angle.
	const bool continues_past_last = final_velocity.isAllFinite() && final_velocity.xy().longerThan(FLT_EPSILON);
	const float last_acceptance_radius = acceptance_radii ? acceptance_radii[num_waypoints - 1] : default_acceptance_radius;
	Vector3f after_last = last;
	float max_speed = 0.f;

	if (continues_past_last) {
		const Vector2f direction_after_last = Vector2f(final_velocity.xy()).unit_or_zero();
		after_last.xy() += direction_after_last * last_acceptance_radius;
		max_speed = final_velocity.xy().norm();
	}

	// go backwards through the waypoints
	for (int i = (num_waypoints - 2); i >= 0; i--) {
		const Vector3f &next = (i + 2 < num_waypoints) ? waypoints[i + 2] : after_last;
		config.xy_accept_rad = acceptance_radii ? acceptance_radii[i + 1] : default_acceptance_radius;
		max_speed = computeStartXYSpeedFromWaypoints(waypoints[i], waypoints[i + 1], next, max_speed, config);
	}

	return max_speed;
}

/*
 * Same as above for a fixed number of waypoints, assuming a full stop at the last one.
 */
template <int N>
float computeXYSpeedFromWaypoints(const Vector3f waypoints[N], const VehicleDynamicLimits &config)
{
	static_assert(N >= 2, "Need at least 2 points to compute speed");

	return computeXYSpeedFromWaypoints(waypoints, N, Vector3f{}, config);
}

}
}
