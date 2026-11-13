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
 * @file TerrainSynthetic.hpp
 * Synthetic terrain from a seed behind the SIH terrain interface. The terrain is
 * anchored at the projection reference, SIH's home, and terrain_sim thinks in
 * meters north and east of it, so each call projects the WGS84 position once.
 */

#pragma once

#include "TerrainFlat.hpp"

#include <lib/geo/geo.h>
#include <lib/terrain_sim/terrain_sim.h>

class TerrainSynthetic : public TerrainFlat
{
public:
	/** the terrain is zero at the reference, so the vehicle starts on the ground */
	void setReference(double lat_deg, double lon_deg) { _ref.initReference(lat_deg, lon_deg); }

	bool altitude(double lat_deg, double lon_deg, float &alt) const override
	{
		float north, east;
		_ref.project(lat_deg, lon_deg, north, east);
		alt = _default_alt + terrain_sim::height(north, east);
		return true;
	}

	float vehicleGroundAltitude(double lat_deg, double lon_deg) override
	{
		float alt;
		altitude(lat_deg, lon_deg, alt);
		return alt;
	}

	float rayDistance(double lat_deg, double lon_deg, float alt, const matrix::Vector3f &dir_ned,
			  float max_distance) const override
	{
		if (dir_ned(2) <= 0.f) {
			return NAN;
		}

		float north, east;
		_ref.project(lat_deg, lon_deg, north, east);
		// terrain_sim measures altitude up from the reference and returns max_t on a miss
		const float distance = terrain_sim::raycast(north, east, alt - _default_alt, dir_ned(0), dir_ned(1), -dir_ned(2),
				       max_distance + 1.f);
		return distance > max_distance ? NAN : distance;
	}

	void printStatus() const override
	{
		PX4_INFO("terrain: synthetic, formula %u", terrain_sim::formula());
	}

private:
	MapProjection _ref{};
};
