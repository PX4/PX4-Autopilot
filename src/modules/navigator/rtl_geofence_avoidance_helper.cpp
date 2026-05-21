/***************************************************************************
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

#include "rtl_geofence_avoidance_helper.h"

#include "navigator.h"

#include <lib/geo/geo.h>
#include <lib/rtl/rtl_time_estimator.h>

matrix::Vector2d add_geofence_avoidance_path_distance(
	RtlTimeEstimator &estimator,
	Navigator &navigator,
	const matrix::Vector2d &current_position,
	int num_waypoints,
	int current_index)
{
	matrix::Vector2d end_position = current_position;

	if (num_waypoints <= 0) {
		return end_position;
	}

	GeofenceAvoidancePlanner &planner = navigator.get_geofence_avoidance_planner();

	if (!planner.start_is_current_position() && current_index <= 1) {
		// If the path was planned from a stored anchor (vehicle was outside the fence at plan time),
		// the leg from the current position back to point 0 is unaccounted for in the path itself.
		const matrix::Vector2d first_waypoint = planner.get_point_at_index(0);

		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(current_position(0), current_position(1),
					    first_waypoint(0), first_waypoint(1),
					    &direction(0), &direction(1));
		const float dist = get_distance_to_next_waypoint(current_position(0), current_position(1),
				   first_waypoint(0), first_waypoint(1));

		estimator.addDistance(dist, direction, 0.f);
		end_position = first_waypoint;

	} else if (current_index < num_waypoints) {
		// Vehicle is mid-leg between geofence waypoints; include the partial leg from the
		// current position to the next geofence waypoint that the for-loop below picks up from.
		const matrix::Vector2d next_waypoint = planner.get_point_at_index(current_index);

		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(current_position(0), current_position(1),
					    next_waypoint(0), next_waypoint(1),
					    &direction(0), &direction(1));
		const float dist = get_distance_to_next_waypoint(current_position(0), current_position(1),
				   next_waypoint(0), next_waypoint(1));

		estimator.addDistance(dist, direction, 0.f);
		end_position = next_waypoint;
	}

	for (int i = current_index; i < num_waypoints - 1; ++i) {
		const matrix::Vector2d start = planner.get_point_at_index(i);
		const matrix::Vector2d end = planner.get_point_at_index(i + 1);

		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(start(0), start(1), end(0), end(1), &direction(0), &direction(1));
		const float dist = get_distance_to_next_waypoint(start(0), start(1), end(0), end(1));

		estimator.addDistance(dist, direction, 0.f);
		end_position = end;
	}

	return end_position;
}
