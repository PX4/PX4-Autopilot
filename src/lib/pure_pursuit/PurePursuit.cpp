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
	updateParams();
}

void PurePursuit::updateParams()
{
	param_get(_param_handles.lookahead_gain, &_params.lookahead_gain);
	param_get(_param_handles.lookahead_max, &_params.lookahead_max);
	param_get(_param_handles.lookahead_min, &_params.lookahead_min);

	ModuleParams::updateParams();

}

float PurePursuit::calcDesiredHeading(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
				      const Vector2f &curr_pos_ned,
				      const float vehicle_speed)
{
	// Check input validity
	if (!curr_wp_ned.isAllFinite() || !curr_pos_ned.isAllFinite() || vehicle_speed < -FLT_EPSILON
	    || !PX4_ISFINITE(vehicle_speed) || !prev_wp_ned.isAllFinite()) {
		return NAN;
	}

	_lookahead_distance = math::constrain(_params.lookahead_gain * vehicle_speed,
					      _params.lookahead_min, _params.lookahead_max);

	// Pure pursuit
	const Vector2f curr_pos_to_curr_wp = curr_wp_ned - curr_pos_ned;
	const Vector2f prev_wp_to_curr_wp = curr_wp_ned - prev_wp_ned;

	if (curr_pos_to_curr_wp.norm() < _lookahead_distance
	    || prev_wp_to_curr_wp.norm() <
	    FLT_EPSILON) { // Target current waypoint if closer to it than lookahead or waypoints overlap
		return atan2f(curr_pos_to_curr_wp(1), curr_pos_to_curr_wp(0));
	}

	const Vector2f prev_wp_to_curr_pos = curr_pos_ned - prev_wp_ned;
	const Vector2f prev_wp_to_curr_wp_u = prev_wp_to_curr_wp.unit_or_zero();
	_distance_on_line_segment = (prev_wp_to_curr_pos * prev_wp_to_curr_wp_u) * prev_wp_to_curr_wp_u;
	_curr_pos_to_path = _distance_on_line_segment - prev_wp_to_curr_pos;

	if (_curr_pos_to_path.norm() > _lookahead_distance) { // Target closest point on path if there is no intersection point
		return atan2f(_curr_pos_to_path(1), _curr_pos_to_path(0));
	}

	const float line_extension = sqrt(powf(_lookahead_distance, 2.f) - powf(_curr_pos_to_path.norm(),
					  2.f)); // Length of the vector from the endpoint of _distance_on_line_segment to the intersection point
	const Vector2f prev_wp_to_intersection_point = _distance_on_line_segment + line_extension *
			prev_wp_to_curr_wp_u;
	const Vector2f curr_pos_to_intersection_point = prev_wp_to_intersection_point - prev_wp_to_curr_pos;
	return atan2f(curr_pos_to_intersection_point(1), curr_pos_to_intersection_point(0));

}
