/***************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file safe_point_land.hpp
 * This file defines helper structs that are used to define land approaches which consists of a land location and a number of
 * loiter circles. Each loiter circle defines a possible approach for landing at the land location.
 *
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>

struct loiter_point_s {
	loiter_point_s() { reset(); }
	double lat;
	double lon;
	float height_m;
	float loiter_radius_m;

	void reset()
	{
		lat = lon = static_cast<double>(NAN);
		height_m = NAN;
		loiter_radius_m = NAN;
	}

	bool isValid() const { return PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(height_m); }
};

// defines one land location and a maximum of num_approaches_max loiter points
struct land_approaches_s {

	static constexpr uint8_t num_approaches_max = 8;
	loiter_point_s approaches[num_approaches_max];
	matrix::Vector2d land_location_lat_lon;

	land_approaches_s()
	{
		resetAllApproaches();
	}

	void resetAllApproaches()
	{
		for (uint8_t i = 0; i < num_approaches_max; i++) {
			approaches[i].reset();
		}
	}

	bool isAnyApproachValid() const
	{
		for (uint8_t i = 0; i < num_approaches_max; i++) {
			if (approaches[i].isValid()) {
				return true;
			}
		}

		return false;
	}

	float getMaxDistLandToLoiterCircle() const
	{
		// returns negative infinity if there is no valid approach
		float dist_max = -INFINITY;

		for (uint8_t i = 0; i < num_approaches_max; i++) {
			if (approaches[i].isValid()) {
				float dist = get_distance_to_next_waypoint(land_location_lat_lon(0), land_location_lat_lon(1), approaches[i].lat,
						approaches[i].lon) + approaches[i].loiter_radius_m;

				if (dist > dist_max) {
					dist_max = dist;
				}
			}
		}

		return dist_max;
	}
};
