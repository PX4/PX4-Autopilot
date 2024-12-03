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
#include <mathlib/mathlib.h>


PurePursuit::PurePursuit(ModuleParams *parent) : ModuleParams(parent)
{
	_param_handles.lookahead_gain = param_find("PP_LOOKAHD_GAIN");
	_param_handles.lookahead_max = param_find("PP_LOOKAHD_MAX");
	_param_handles.lookahead_min = param_find("PP_LOOKAHD_MIN");
	_pure_pursuit_pub.advertise();
	updateParams();
}

void PurePursuit::updateParams()
{
	param_get(_param_handles.lookahead_gain, &_params.lookahead_gain);
	param_get(_param_handles.lookahead_max, &_params.lookahead_max);
	param_get(_param_handles.lookahead_min, &_params.lookahead_min);

	ModuleParams::updateParams();

}

float PurePursuit::updatePurePursuit(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
				     const Vector2f &curr_pos_ned, const float vehicle_speed)
{
	_target_bearing = calcTargetBearing(curr_wp_ned, prev_wp_ned, curr_pos_ned, vehicle_speed);

	if (PX4_ISFINITE(_target_bearing)) {
		publishPurePursuit();
	}

	return _target_bearing;
}

float PurePursuit::calcTargetBearing(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
				     const Vector2f &curr_pos_ned, const float vehicle_speed)
{
	// Check input validity
	if (!curr_wp_ned.isAllFinite() || !curr_pos_ned.isAllFinite() || !PX4_ISFINITE(vehicle_speed)
	    || !prev_wp_ned.isAllFinite()) {
		return NAN;
	}

	// Pure pursuit
	_lookahead_distance = math::constrain(_params.lookahead_gain * fabsf(vehicle_speed),
					      _params.lookahead_min, _params.lookahead_max);
	_curr_pos_to_curr_wp = curr_wp_ned - curr_pos_ned;
	const Vector2f prev_wp_to_curr_wp = curr_wp_ned - prev_wp_ned;
	const Vector2f prev_wp_to_curr_pos = curr_pos_ned - prev_wp_ned;
	const Vector2f prev_wp_to_curr_wp_u = prev_wp_to_curr_wp.unit_or_zero();
	_distance_along_path = (prev_wp_to_curr_pos * prev_wp_to_curr_wp_u) *
			       prev_wp_to_curr_wp_u; // Projection of prev_wp_to_curr_pos onto prev_wp_to_curr_wp
	const Vector2f curr_pos_to_path = _distance_along_path -
					  prev_wp_to_curr_pos; // Shortest vector from the current position to the path
	_crosstrack_error = sign(prev_wp_to_curr_wp(1) * curr_pos_to_path(
					 0) - prev_wp_to_curr_wp(0) * curr_pos_to_path(1)) * curr_pos_to_path.norm();

	if (_curr_pos_to_curr_wp.norm() < _lookahead_distance
	    || prev_wp_to_curr_wp.norm() <
	    FLT_EPSILON) { // Target current waypoint if closer to it than lookahead or waypoints overlap
		return matrix::wrap_pi(atan2f(_curr_pos_to_curr_wp(1), _curr_pos_to_curr_wp(0)));

	} else {

		if (fabsf(_crosstrack_error) > _lookahead_distance) { // Path segment is outside of lookahead (No intersection point)
			const Vector2f prev_wp_to_closest_point_on_path = curr_pos_to_path + prev_wp_to_curr_pos;
			const Vector2f curr_wp_to_closest_point_on_path = curr_pos_to_path - _curr_pos_to_curr_wp;

			if (prev_wp_to_closest_point_on_path * prev_wp_to_curr_wp <
			    FLT_EPSILON) { // Target previous waypoint if closest point is on the the extended path segment "behind" previous waypoint
				return matrix::wrap_pi(atan2f(-prev_wp_to_curr_pos(1), -prev_wp_to_curr_pos(0)));

			} else if (curr_wp_to_closest_point_on_path * prev_wp_to_curr_wp >
				   FLT_EPSILON) { // Target current waypoint if closest point is on the extended path segment "ahead" of current waypoint
				return matrix::wrap_pi(atan2f(_curr_pos_to_curr_wp(1), _curr_pos_to_curr_wp(0)));

			} else { // Target closest point on path
				return matrix::wrap_pi(atan2f(curr_pos_to_path(1), curr_pos_to_path(0)));
			}


		} else {
			const float line_extension = sqrt(powf(_lookahead_distance, 2.f) - powf(curr_pos_to_path.norm(),
							  2.f)); // Length of the vector from the endpoint of distance_on_line_segment to the intersection point
			const Vector2f prev_wp_to_intersection_point = _distance_along_path + line_extension *
					prev_wp_to_curr_wp_u;
			const Vector2f curr_pos_to_intersection_point = prev_wp_to_intersection_point - prev_wp_to_curr_pos;
			return matrix::wrap_pi(atan2f(curr_pos_to_intersection_point(1), curr_pos_to_intersection_point(0)));
		}

	}

}

void PurePursuit::publishPurePursuit()
{
	pure_pursuit_s pure_pursuit{};
	pure_pursuit.timestamp = hrt_absolute_time();
	pure_pursuit.target_bearing = _target_bearing;
	pure_pursuit.lookahead_distance = _lookahead_distance;
	pure_pursuit.crosstrack_error = _crosstrack_error;
	pure_pursuit.distance_to_waypoint = _curr_pos_to_curr_wp.norm();
	_pure_pursuit_pub.publish(pure_pursuit);
}
