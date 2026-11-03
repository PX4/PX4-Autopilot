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

#pragma once

#include <lib/lat_lon_alt/lat_lon_alt.hpp>
#include <matrix/math.hpp>

/**
 * Closed-form navigation truth over the rotating WGS-84 ellipsoid together with the
 * strapdown gyro and accelerometer increments that produce it.
 *
 * An exact strapdown algorithm using normal gravity along the geodetic down reproduces
 * the truth from these increments to numerical precision, so any residual error belongs
 * to the algorithm under test.
 *
 * The inertial frame is the ECEF frame frozen at t = 0.
 *
 * Reference: P. G. Savage, Strapdown Analytics, 2nd ed., Sect. 11.2.4 (Gen Nav simulator)
 */
class GenNav
{
public:
	/**
	 * Savage's aircraft profile scaled to a drone. The vehicle is at rest until t_start_s,
	 * accelerates along a quarter cosine to the cruise velocity and holds it. The altitude
	 * follows a half cosine up to the climb height, then oscillates above it.
	 */
	struct Profile {
		double latitude_deg{47.3566094};
		double longitude_deg{8.5190237};
		float altitude_m{422.f};
		float t_start_s{0.f};
		float acceleration_time_s{10.f};
		float cruise_vel_north{15.f}; // m/s
		float cruise_vel_east{-10.f}; // m/s
		float climb_height_m{50.f};
		float altitude_oscillation_m{5.f};
		float altitude_oscillation_period_s{20.f};
	};

	GenNav(const Profile &profile, const matrix::Eulerf &initial_attitude);

	// Body angular rate relative to the local NED frame, held until changed (rad/s)
	void setBodyRate(const matrix::Vector3f &rate) { _body_rate = matrix::Vector3d(rate); }

	// Advances the truth to t and returns the increments over the step, in body axes (rad, m/s)
	void step(double t, matrix::Vector3f &delta_angle, matrix::Vector3f &delta_velocity);

	double time() const { return _time; }
	matrix::Quatf attitude() const { return matrix::Quatf(matrix::Dcmf(dcmBodyToNed())); }
	const matrix::Quatd &attitudeInertial() const { return _q_inertial; }

	LatLonAlt position(double t) const;
	matrix::Vector3d velocityNed(double t) const;
	matrix::Vector3d inertialVelocity(double t) const;

	// Mass attraction in the inertial frame, consistent with normal gravity on the rotating Earth
	matrix::Vector3d gravitation(double t) const;

	static matrix::Dcmd rotEcefToInertial(double t);

	// Mean of the rotation matrix over a step of constant rate: D = ∫₀¹ exp(s·[φ×]) ds
	static matrix::SquareMatrix<double, 3> meanRotation(const matrix::Vector3d &rotation_vector);

private:
	struct Geodetic {
		double latitude;
		double longitude;
		double altitude;
		double latitude_rate;
		double longitude_rate;
		double altitude_rate;
	};

	Geodetic geodetic(double t) const;
	matrix::Dcmd dcmBodyToNed() const;

	const Profile _profile;

	double _horizontal_omega{};
	double _latitude_amplitude{};
	double _longitude_amplitude{};
	double _climb_omega{};
	double _oscillation_omega{};

	double _time{0.0};
	matrix::Quatd _q_inertial{};
	matrix::Vector3d _body_rate{};
	matrix::Vector3d _inertial_velocity{};
	matrix::Vector3d _gravitation{};
};
