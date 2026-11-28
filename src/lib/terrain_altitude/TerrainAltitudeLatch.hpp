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


/**
 * @file TerrainAltitudeLatch.hpp
 *
 * Terrain altitude below the vehicle, kept at its last valid value when the estimate is lost,
 * and the AMSL altitude of terrain-relative position setpoints (agl_alt) it gives.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>

class TerrainAltitudeLatch
{
public:
	/**
	 * Update with the latest local position, latch the terrain altitude if its estimate is valid
	 */
	void update(const vehicle_local_position_s &local_pos)
	{
		_valid = local_pos.z_global && local_pos.dist_bottom_valid && PX4_ISFINITE(local_pos.ref_alt)
			 && PX4_ISFINITE(local_pos.z) && PX4_ISFINITE(local_pos.dist_bottom);

		if (_valid) {
			_terrain_alt = local_pos.ref_alt - local_pos.z - local_pos.dist_bottom;
		}
	}

	/**
	 * @return [m AMSL] Terrain altitude below the vehicle, the last valid one if currently invalid, NAN if never valid
	 */
	float get() const { return _terrain_alt; }

	/**
	 * @return true if the last update had a valid terrain estimate
	 */
	bool isValid() const { return _valid; }

	/**
	 * @return true if the terrain altitude is a latched one: there was a valid estimate, but not anymore
	 */
	bool isLatched() const { return !_valid && PX4_ISFINITE(_terrain_alt); }

	/**
	 * @return [m AMSL] Altitude setpoint: the terrain altitude plus agl_alt for a valid terrain-relative setpoint,
	 *         else (or without any terrain estimate yet) alt
	 */
	float setpointAltitude(const position_setpoint_s &setpoint) const
	{
		if (setpoint.valid && PX4_ISFINITE(setpoint.agl_alt) && PX4_ISFINITE(_terrain_alt)) {
			return _terrain_alt + setpoint.agl_alt;
		}

		return setpoint.alt;
	}

	/**
	 * Replace alt of the terrain-relative setpoints of a triplet with their AMSL altitude
	 */
	void convertTriplet(position_setpoint_triplet_s &triplet) const
	{
		triplet.previous.alt = setpointAltitude(triplet.previous);
		triplet.current.alt = setpointAltitude(triplet.current);
		triplet.next.alt = setpointAltitude(triplet.next);
	}

private:
	float _terrain_alt{NAN};
	bool _valid{false};
};
