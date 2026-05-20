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
/**
 * @file rtl_geofence_avoidance_helper.h
 *
 * Shared helper for accumulating the remaining horizontal legs of a
 * geofence-avoidance path into an RtlTimeEstimator.
 */

#pragma once

#include <matrix/math.hpp>

class Navigator;
class RtlTimeEstimator;

/**
 * @brief Accumulate the remaining horizontal legs of the geofence-avoidance path into the estimator.
 *
 * Adds, in order:
 *  - The leg from current_position back to point 0, when the path was planned from a stored anchor
 *    (i.e. !navigator.geofencePlannerStartIsCurrentPosition()) and the vehicle has not yet reached
 *    point 0 (current_index <= 1).
 *  - Otherwise, the partial leg from current_position to the next geofence waypoint
 *    (the one at current_index), so the for-loop's first leg connects to current_position.
 *  - The remaining inter-waypoint legs from current_index up to num_waypoints - 1.
 *
 * @param estimator         Time estimator to accumulate distances into.
 * @param navigator         Navigator providing geofence path points.
 * @param current_position  Vehicle's current global position (lat, lon).
 * @param num_waypoints     Number of points in the geofence-avoidance path.
 * @param current_index     Index of the next geofence path point to consume.
 * @return Horizontal position at the end of the accumulated path, or current_position if no path is active.
 */
matrix::Vector2d add_geofence_avoidance_path_distance(
	RtlTimeEstimator &estimator,
	const Navigator &navigator,
	const matrix::Vector2d &current_position,
	int num_waypoints,
	int current_index);
