/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "PurePursuit.hpp"
using namespace matrix;
namespace PurePursuit
{
float calcTargetBearing(pure_pursuit_status_s &pure_pursuit_status, const float lookahead_gain,
			const float lookahead_max, const float lookahead_min, const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
			const Vector2f &curr_pos_ned, const float vehicle_speed)
{
	// Check input validity
	if (!curr_wp_ned.isAllFinite() || !curr_pos_ned.isAllFinite() || !PX4_ISFINITE(vehicle_speed)
	    || !prev_wp_ned.isAllFinite()) {
		return NAN;
	}

	const float lookahead_distance = math::constrain(lookahead_gain * fabsf(vehicle_speed), lookahead_min, lookahead_max);
	const Vector2f curr_pos_to_curr_wp = curr_wp_ned - curr_pos_ned;
	const Vector2f prev_wp_to_curr_wp = curr_wp_ned - prev_wp_ned;
	const Vector2f prev_wp_to_curr_pos = curr_pos_ned - prev_wp_ned;
	const Vector2f prev_wp_to_curr_wp_u = prev_wp_to_curr_wp.unit_or_zero();
	const Vector2f position_along_path = (prev_wp_to_curr_pos * prev_wp_to_curr_wp_u) *
					     prev_wp_to_curr_wp_u; // Projection of prev_wp_to_curr_pos onto prev_wp_to_curr_wp
	const Vector2f curr_pos_to_path = position_along_path -
					  prev_wp_to_curr_pos; // Shortest vector from the current position to the path
	const float crosstrack_error = sign(prev_wp_to_curr_wp(1) * curr_pos_to_path(
			0) - prev_wp_to_curr_wp(0) * curr_pos_to_path(1)) * curr_pos_to_path.norm();
	const float bearing_to_curr_waypoint = matrix::wrap_pi(atan2f(curr_pos_to_curr_wp(1), curr_pos_to_curr_wp(0)));
	float target_bearing{NAN};

	if (curr_pos_to_curr_wp.norm() < lookahead_distance
	    || prev_wp_to_curr_wp.norm() <
	    FLT_EPSILON) { // Target current waypoint if closer to it than lookahead or waypoints overlap
		target_bearing = bearing_to_curr_waypoint;

	} else if (fabsf(crosstrack_error) >
		   lookahead_distance) { // Path segment is outside of lookahead (No intersection point)

		const Vector2f prev_wp_to_closest_point_on_path = curr_pos_to_path + prev_wp_to_curr_pos;
		const Vector2f curr_wp_to_closest_point_on_path = curr_pos_to_path - curr_pos_to_curr_wp;

		if (prev_wp_to_closest_point_on_path * prev_wp_to_curr_wp <
		    FLT_EPSILON) { // Target previous waypoint if closest point is on the the extended path segment "behind" previous waypoint
			target_bearing = matrix::wrap_pi(atan2f(-prev_wp_to_curr_pos(1), -prev_wp_to_curr_pos(0)));

		} else if (curr_wp_to_closest_point_on_path * prev_wp_to_curr_wp >
			   FLT_EPSILON) { // Target current waypoint if closest point is on the extended path segment "ahead" of current waypoint
			target_bearing = bearing_to_curr_waypoint;

		} else { // Target closest point on path
			target_bearing = matrix::wrap_pi(atan2f(curr_pos_to_path(1), curr_pos_to_path(0)));
		}


	} else { // Regular pure pursuit
		const float line_extension = sqrt(powf(lookahead_distance, 2.f) - powf(curr_pos_to_path.norm(),
						  2.f)); // Length of the vector from the endpoint of distance_on_line_segment to the intersection point
		const Vector2f prev_wp_to_intersection_point = position_along_path + line_extension *
				prev_wp_to_curr_wp_u;
		const Vector2f curr_pos_to_intersection_point = prev_wp_to_intersection_point - prev_wp_to_curr_pos;
		target_bearing = matrix::wrap_pi(atan2f(curr_pos_to_intersection_point(1), curr_pos_to_intersection_point(0)));
	}


	pure_pursuit_status.lookahead_distance = lookahead_distance;
	pure_pursuit_status.target_bearing = target_bearing;
	pure_pursuit_status.crosstrack_error = crosstrack_error;
	pure_pursuit_status.distance_to_waypoint = curr_pos_to_curr_wp.norm();
	pure_pursuit_status.bearing_to_waypoint = bearing_to_curr_waypoint;
	return target_bearing;

}
} // Pure Pursuit
