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

#include "lat_lon_alt.hpp"

using matrix::Vector3f;
using matrix::Vector3d;
using matrix::Vector2d;

LatLonAlt LatLonAlt::fromEcef(const Vector3d &p_ecef)
{
	// Convert position using Borkowski closed-form exact solution
	// P. D. Groves, "Principles of GNSS, inertial, and multisensor integrated navigation systems, 2nd edition (appendix C)
	const double k1 = sqrt(1 - Wgs84::eccentricity2) * std::abs(p_ecef(2));
	const double k2 = Wgs84::eccentricity2 * Wgs84::equatorial_radius;
	const double beta = sqrt(p_ecef(0) * p_ecef(0) + p_ecef(1) * p_ecef(1));
	const double E = (k1 - k2) / beta;
	const double F = (k1 + k2) / beta;

	const double P = 4.0 / 3.0 * (E * F + 1);
	const double Q = 2 * (E * E - F * F);
	const double D = P * P * P + Q * Q;
	const double V = pow(sqrt(D) - Q, 1.0 / 3.0) - pow(sqrt(D) + Q, 1.0 / 3.0);
	const double G = 0.5 * (sqrt(E * E + V) + E);
	const double T = sqrt(G * G + (F - V * G) / (2 * G - E)) - G;

	const double lon = atan2(p_ecef(1), p_ecef(0));
	const double lat = matrix::sign(p_ecef(2)) * atan((1.0 - T * T) / (2.0 * T * sqrt(1.0 - Wgs84::eccentricity2)));
	const double alt = (beta - Wgs84::equatorial_radius * T) * cos(lat) +
			   (p_ecef(2) - matrix::sign(p_ecef(2)) * Wgs84::equatorial_radius * sqrt(1.0 - Wgs84::eccentricity2)) * sin(lat);

	LatLonAlt lla;
	lla.setLatLonRad(lat, lon);
	lla.setAltitude(static_cast<float>(alt));
	return lla;
}

Vector3d LatLonAlt::toEcef() const
{
	const double cos_lat = cos(_latitude_rad);
	const double sin_lat = sin(_latitude_rad);
	const double cos_lon = cos(_longitude_rad);
	const double sin_lon = sin(_longitude_rad);

	const double r_e = Wgs84::equatorial_radius / sqrt(1.0 - std::pow(Wgs84::eccentricity * sin_lat, 2.0));
	const double r_total = r_e + static_cast<double>(_altitude);

	return Vector3d(r_total * cos_lat * cos_lon,
			r_total * cos_lat * sin_lon,
			((1.0 - Wgs84::eccentricity2) * r_total) * sin_lat);
}

Vector3f LatLonAlt::computeAngularRateNavFrame(const Vector3f &v_ned) const
{
	double r_n;
	double r_e;
	computeRadiiOfCurvature(_latitude_rad, r_n, r_e);
	return Vector3f(
		       v_ned(1) / (static_cast<float>(r_e) + _altitude),
		       -v_ned(0) / (static_cast<float>(r_n) + _altitude),
		       -v_ned(1) * tanf(_latitude_rad) / (static_cast<float>(r_e) + _altitude));
}

Vector2d LatLonAlt::deltaLatLonToDeltaXY(const double latitude, const float altitude)
{
	double r_n;
	double r_e;
	computeRadiiOfCurvature(latitude, r_n, r_e);
	const double dn_dlat = r_n + static_cast<double>(altitude);
	const double de_dlon = (r_e + static_cast<double>(altitude)) * cos(latitude);

	return Vector2d(dn_dlat, de_dlon);
}

void LatLonAlt::computeRadiiOfCurvature(const double latitude, double &meridian_radius_of_curvature,
					double &transverse_radius_of_curvature)
{
	const double tmp = 1.0 - pow(Wgs84::eccentricity * sin(latitude), 2);
	const double sqrt_tmp = std::sqrt(tmp);
	meridian_radius_of_curvature = Wgs84::meridian_radius_of_curvature_numerator / (tmp * tmp * sqrt_tmp);
	transverse_radius_of_curvature = Wgs84::equatorial_radius / sqrt_tmp;
}

LatLonAlt LatLonAlt::operator+(const matrix::Vector3f &delta_pos) const
{
	const matrix::Vector2d d_lat_lon_to_d_xy = deltaLatLonToDeltaXY(latitude_rad(), altitude());
	const double latitude_rad = matrix::wrap_pi(_latitude_rad + static_cast<double>(delta_pos(0)) / d_lat_lon_to_d_xy(0));
	const double longitude_rad = matrix::wrap_pi(_longitude_rad + static_cast<double>(delta_pos(1)) / d_lat_lon_to_d_xy(1));
	const float altitude = _altitude - delta_pos(2);

	LatLonAlt lla_new;
	lla_new.setLatLonRad(latitude_rad, longitude_rad);
	lla_new.setAltitude(altitude);
	return lla_new;
}

void LatLonAlt::operator+=(const matrix::Vector3f &delta_pos)
{
	matrix::Vector2d d_lat_lon_to_d_xy = deltaLatLonToDeltaXY(_latitude_rad, _altitude);
	_latitude_rad = matrix::wrap_pi(_latitude_rad + static_cast<double>(delta_pos(0)) / d_lat_lon_to_d_xy(0));
	_longitude_rad = matrix::wrap_pi(_longitude_rad + static_cast<double>(delta_pos(1)) / d_lat_lon_to_d_xy(1));
	_altitude -= delta_pos(2);
}

void LatLonAlt::operator+=(const matrix::Vector2f &delta_pos)
{
	matrix::Vector2d d_lat_lon_to_d_xy = deltaLatLonToDeltaXY(_latitude_rad, _altitude);
	_latitude_rad = matrix::wrap_pi(_latitude_rad + static_cast<double>(delta_pos(0)) / d_lat_lon_to_d_xy(0));
	_longitude_rad = matrix::wrap_pi(_longitude_rad + static_cast<double>(delta_pos(1)) / d_lat_lon_to_d_xy(1));
}

matrix::Vector3f LatLonAlt::operator-(const LatLonAlt &lla) const
{
	const double delta_lat = matrix::wrap_pi(_latitude_rad - lla.latitude_rad());
	const double delta_lon = matrix::wrap_pi(_longitude_rad - lla.longitude_rad());
	const float delta_alt = _altitude - lla.altitude();

	const matrix::Vector2d d_lat_lon_to_d_xy = deltaLatLonToDeltaXY(_latitude_rad, _altitude);
	return matrix::Vector3f(static_cast<float>(delta_lat * d_lat_lon_to_d_xy(0)),
				static_cast<float>(delta_lon * d_lat_lon_to_d_xy(1)),
				-delta_alt);
}
