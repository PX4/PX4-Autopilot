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
 * @file TerrainFlat.hpp
 * Flat terrain at the default altitude, the base of every SIH terrain model.
 */

#pragma once

#include <float.h>
#include <math.h>

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>

class TerrainFlat
{
public:
	virtual ~TerrainFlat() = default;

	/** load terrain data from a file, flat terrain has none, @return false (the reason is printed) */
	bool load(const char *path)
	{
		PX4_ERR("terrain: cannot load %s, PX4 was built without terrain DEM support", path);
		return false;
	}

	/** flat ground altitude [m AMSL], also used where the terrain model has no data */
	void setDefaultAltitude(float alt) { _default_alt = alt; }
	float defaultAltitude() const { return _default_alt; }

	/**
	 * Terrain altitude [m AMSL] at a WGS84 position.
	 * @return false outside the terrain data, alt is then the default altitude
	 */
	virtual bool altitude(double lat_deg, double lon_deg, float &alt) const
	{
		alt = _default_alt;
		return true;
	}

	/**
	 * Terrain altitude [m AMSL] below the vehicle at a WGS84 position, unlike altitude() it
	 * keeps track of the vehicle leaving and reentering the terrain data.
	 */
	virtual float vehicleGroundAltitude(double lat_deg, double lon_deg)
	{
		return _default_alt;
	}

	/**
	 * Distance [m] from a WGS84 position along a direction to the terrain.
	 * @param alt altitude of the start point [m AMSL]
	 * @param dir_ned unit vector in the local NED frame
	 * @param max_distance search range [m], a result above it may be returned
	 * @return NAN if the ray does not hit the terrain within max_distance
	 */
	virtual float rayDistance(double lat_deg, double lon_deg, float alt, const matrix::Vector3f &dir_ned,
				  float max_distance) const
	{
		const float cos_tilt = dir_ned(2);

		if (cos_tilt < FLT_EPSILON) {
			// pointing away from the ground
			return NAN;
		}

		// the fake ground lets the vehicle settle marginally below the ground altitude, clamp it
		// so that a landed vehicle reads zero instead of dropping out of range
		const float height = alt - _default_alt;
		return (height > 0.f ? height : 0.f) / cos_tilt;
	}

	virtual void printStatus() const
	{
		PX4_INFO("terrain: flat at %.2f m", (double)_default_alt);
	}

protected:
	float _default_alt{0.f};
};
