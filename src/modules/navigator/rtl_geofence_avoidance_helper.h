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
#include "RTLPlanner/geofence_avoidance_planner.h"

class RtlTimeEstimator;

/**
 * @brief Accumulate the remaining geofence-avoidance legs into the time estimator.
 *
 * Iterates from the planner's current cursor position to the end of the materialized path,
 * adding each leg from current_position (or the previous waypoint) to the next waypoint.
 *
 * @param estimator        Time estimator to accumulate distances into.
 * @param planner          The geofence avoidance planner (read-only).
 * @param current_position Vehicle's current global position (lat, lon).
 * @return Horizontal position at the end of the path, or current_position if the path is empty.
 */
matrix::Vector2d add_geofence_avoidance_path_distance(
	RtlTimeEstimator &estimator,
	const GeofenceAvoidancePlanner &planner,
	const matrix::Vector2d &current_position);
