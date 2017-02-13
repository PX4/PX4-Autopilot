/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once
#include <lib/geo/geo.h>
#include "constants.hpp"

class Origin
{
public:
	Origin() :
		_map_ref(),
		_alt(0),
		_altInitialized(false),
		_altTimestamp(0)
	{
		_map_ref.init_done = false;
		_map_ref.lat_rad = 0;
		_map_ref.lon_rad = 0;
		_map_ref.timestamp = 0;
	}
	inline bool altInitialized() const
	{
		return _altInitialized;
	}

	inline bool xyInitialized() const
	{
		return _map_ref.init_done;
	}

	inline uint64_t getXYTimestamp() const
	{
		return _map_ref.timestamp;
	}

	inline bool getAltTimestamp() const
	{
		return _altTimestamp;
	}

	void globalToLocalRelAlt(const double &lat_deg, const double &lon_deg, const float &asl, float &pos_n, float &pos_e,
				 float &rel_alt)
	{
		map_projection_project(&_map_ref, lat_deg, lon_deg, &pos_n, &pos_e);
		rel_alt = asl - _alt;
	}

	void globalToLocalXY(const double &lat_deg, const double &lon_deg, float &pos_n, float &pos_e)
	{
		map_projection_project(&_map_ref, lat_deg, lon_deg, &pos_n, &pos_e);
	}

	void northEastToLatLon(const float &pos_n, const float &pos_e, double &lat_deg, double &lon_deg)
	{
		map_projection_reproject(&_map_ref, pos_n, pos_e, &lat_deg, &lon_deg);
	}

	void xyInitialize(double lat_deg, double lon_deg, uint64_t timestamp)
	{
		map_projection_init_timestamped(&_map_ref,
						lat_deg, lon_deg, timestamp);
	}
	inline float getAlt() const
	{
		return _alt;
	}
	void altInitialize(float alt, uint64_t timestamp)
	{
		_alt = alt;
		_altInitialized = true;
		_altTimestamp = timestamp;
	}
	inline double getLatRad() const
	{
		return _map_ref.lat_rad;
	}
	inline double getLonRad() const
	{
		return _map_ref.lon_rad;
	}
	inline double getLatDeg() const
	{
		return rad2deg * getLatRad();
	}
	inline double getLonDeg() const
	{
		return rad2deg * getLonRad();
	}
private:
	struct map_projection_reference_s _map_ref;
	float _alt;
	bool _altInitialized;
	uint64_t _altTimestamp;
};
