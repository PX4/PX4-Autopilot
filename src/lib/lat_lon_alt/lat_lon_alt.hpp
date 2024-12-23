/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "mathlib/math/Limits.hpp"
#include <matrix/math.hpp>

class LatLonAlt
{
public:
	LatLonAlt() = default;
	LatLonAlt(const LatLonAlt &lla)
	{
		_latitude_rad = lla.latitude_rad();
		_longitude_rad = lla.longitude_rad();
		_altitude = lla.altitude();
	}

	LatLonAlt(const double latitude_deg, const double longitude_deg, const float altitude_m)
	{
		_latitude_rad = math::radians(latitude_deg);
		_longitude_rad = math::radians(longitude_deg);
		_altitude = altitude_m;
	}


	static LatLonAlt fromEcef(const matrix::Vector3d &p_ecef);
	matrix::Vector3d toEcef() const;

	void setZero() { _latitude_rad = 0.0; _longitude_rad = 0.0; _altitude = 0.f; }

	double latitude_deg() const { return math::degrees(latitude_rad()); }
	double longitude_deg() const { return math::degrees(longitude_rad()); }

	const double &latitude_rad() const { return _latitude_rad; }
	const double &longitude_rad() const { return _longitude_rad; }
	float altitude() const { return _altitude; }

	void setLatitudeDeg(const double &latitude_deg) { _latitude_rad = math::radians(latitude_deg); }
	void setLongitudeDeg(const double &longitude_deg) { _longitude_rad = math::radians(longitude_deg); }
	void setAltitude(const float altitude) { _altitude = altitude; }

	void setLatLon(const LatLonAlt &lla) { _latitude_rad = lla.latitude_rad(); _longitude_rad = lla.longitude_rad(); }
	void setLatLonDeg(const double latitude, const double longitude) { _latitude_rad = math::radians(latitude); _longitude_rad = math::radians(longitude); }
	void setLatLonRad(const double latitude, const double longitude) { _latitude_rad = latitude; _longitude_rad = longitude; }

	void print() const { printf("latitude = %f (deg), longitude = %f (deg), altitude = %f (m)\n", _latitude_rad, _longitude_rad, (double)_altitude); }

	/*
	 * The plus and minus operators below use approximations and should only be used when the Cartesian component is small
	 */
	LatLonAlt operator+(const matrix::Vector3f &delta_pos) const;
	void operator+=(const matrix::Vector3f &delta_pos);
	void operator+=(const matrix::Vector2f &delta_pos);
	matrix::Vector3f operator-(const LatLonAlt &lla) const;

	void operator=(const LatLonAlt &lla)
	{
		_latitude_rad = lla.latitude_rad();
		_longitude_rad = lla.longitude_rad();
		_altitude = lla.altitude();
	}

	/*
	 * Compute the angular rate of the local navigation frame at the current latitude and height
	 * with respect to an inertial frame and resolved in the navigation frame
	 */
	matrix::Vector3f computeAngularRateNavFrame(const matrix::Vector3f &v_ned) const;

	struct Wgs84 {
		static constexpr double equatorial_radius = 6378137.0;
		static constexpr double eccentricity = 0.0818191908425;
		static constexpr double eccentricity2 = eccentricity * eccentricity;
		static constexpr double meridian_radius_of_curvature_numerator = equatorial_radius * (1.0 - eccentricity2);
		static constexpr double gravity_equator = 9.7803253359;
	};

private:
	// Convert between curvilinear and cartesian errors
	static matrix::Vector2d deltaLatLonToDeltaXY(const double latitude, const float altitude);

	static void computeRadiiOfCurvature(const double latitude, double &meridian_radius_of_curvature,
					    double &transverse_radius_of_curvature);

	double _latitude_rad{0.0};
	double _longitude_rad{0.0};
	float _altitude{0.0};
};
