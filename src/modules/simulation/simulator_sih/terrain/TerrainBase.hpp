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
 * @file TerrainBase.hpp
 * Terrain model interface for SIH: ground altitude and ray casting against the terrain.
 */

#pragma once

#include <matrix/matrix/math.hpp>

class TerrainBase
{
public:
	virtual ~TerrainBase() = default;

	/** flat ground altitude [m AMSL], also used where the terrain model has no data */
	void setDefaultAltitude(float alt) { _default_alt = alt; }

	/**
	 * Terrain altitude [m AMSL] at a WGS84 position.
	 * @return false outside the terrain data, alt is then the default altitude
	 */
	virtual bool altitude(double lat_deg, double lon_deg, float &alt) const = 0;

	/**
	 * Distance [m] from a WGS84 position along a direction to the terrain.
	 * @param alt altitude of the start point [m AMSL]
	 * @param dir_ned unit vector in the local NED frame
	 * @param max_distance search range [m], a result above it may be returned
	 * @return NAN if the ray does not hit the terrain within max_distance
	 */
	virtual float rayDistance(double lat_deg, double lon_deg, float alt, const matrix::Vector3f &dir_ned,
				  float max_distance) const = 0;

	virtual void printStatus() const = 0;

protected:
	float _default_alt{0.f};
};
