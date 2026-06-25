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

#include "FirstOrderHoldAltitude.hpp"

#include <math.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

float calculateFirstOrderHoldAltitude(const position_setpoint_s &pos_sp_curr, const double current_lat,
				      const double current_lon, const float current_altitude, const float acc_rad,
				      FirstOrderHoldAltitudeState &state)
{
	const float target_altitude = pos_sp_curr.alt;

	// The current altitude is considered reached within the acceptance radius (or loiter circle) of the target.
	const float completion_radius = math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius));

	const float d_curr = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, current_lat, current_lon);

	// Start a new ramp whenever the target altitude changes (a genuinely new altitude setpoint). Ramp updates
	// that keep the same target altitude leave the ongoing ramp untouched so it keeps progressing smoothly.
	const bool new_target = !PX4_ISFINITE(state.target_altitude)
				|| fabsf(target_altitude - state.target_altitude) > FLT_EPSILON;

	if (new_target) {
		state.target_altitude = target_altitude;
		// Ramp from the last commanded altitude setpoint, or the current altitude if nothing was commanded yet.
		state.ramp_start_altitude = PX4_ISFINITE(state.last_altitude_setpoint) ? state.last_altitude_setpoint :
					    current_altitude;
		state.ramp_start_distance = d_curr;
		state.min_distance = d_curr;
	}

	// Track the closest horizontal approach so the ramp only ever progresses toward the target.
	state.min_distance = math::min(state.min_distance, d_curr);

	float position_sp_alt = target_altitude;

	// Only ramp if the target was still outside the completion radius when the ramp started, otherwise there is
	// no meaningful distance to interpolate over and we command the target altitude directly.
	if (state.ramp_start_distance > completion_radius) {
		// The setpoint is interpolated linearly from the ramp start altitude (at the ramp start distance) to the
		// target altitude (reached at the completion radius around the target).
		const float grad = (target_altitude - state.ramp_start_altitude) / (completion_radius - state.ramp_start_distance);
		const float progress_distance = math::constrain(state.min_distance, completion_radius, state.ramp_start_distance);
		position_sp_alt = target_altitude + grad * (progress_distance - completion_radius);
	}

	state.last_altitude_setpoint = position_sp_alt;

	return position_sp_alt;
}
