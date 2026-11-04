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


#include "gen_nav.hpp"

#include <lib/geo/geo.h>

using matrix::Dcmd;
using matrix::Quatd;
using matrix::Vector3d;
using matrix::Vector3f;

static constexpr double kEarthRate = static_cast<double>(CONSTANTS_EARTH_SPIN_RATE);

// Offset negative cosine for a quarter cycle, then the ramp matching its value and slope
static void quarterCosineThenRamp(double tau, double omega, double amplitude, double &value, double &rate)
{
	if (tau <= 0.0) {
		value = 0.0;
		rate = 0.0;

	} else if (omega * tau < M_PI_2) {
		value = amplitude * (1.0 - cos(omega * tau));
		rate = amplitude * omega * sin(omega * tau);

	} else {
		value = amplitude * (1.0 + omega * tau - M_PI_2);
		rate = amplitude * omega;
	}
}

GenNav::GenNav(const Profile &profile, const matrix::Eulerf &initial_attitude) :
	_profile(profile)
{
	// Savage's T0 / 4 quarter-cycle time, with the climb completing at the same time
	_horizontal_omega = M_PI_2 / static_cast<double>(profile.acceleration_time_s);
	_climb_omega = M_PI / static_cast<double>(profile.acceleration_time_s);
	_oscillation_omega = 2.0 * M_PI / static_cast<double>(profile.altitude_oscillation_period_s);

	const double latitude = math::radians(profile.latitude_deg);
	const double cruise_altitude = static_cast<double>(profile.altitude_m + profile.climb_height_m);
	double meridian_radius;
	double transverse_radius;
	LatLonAlt::computeRadiiOfCurvature(latitude, meridian_radius, transverse_radius);

	// The cruise velocity is only reached approximately; the truth is whatever the profile produces
	_latitude_amplitude = static_cast<double>(profile.cruise_vel_north)
			      / ((meridian_radius + cruise_altitude) * _horizontal_omega);
	_longitude_amplitude = static_cast<double>(profile.cruise_vel_east)
			       / ((transverse_radius + cruise_altitude) * cos(latitude) * _horizontal_omega);

	const LatLonAlt start = position(0.0);
	const Dcmd ned_to_ecef = start.computeRotEcefToNed<double>().transpose();
	_q_inertial = Quatd(rotEcefToInertial(0.0) * ned_to_ecef * Dcmd(matrix::Eulerd(initial_attitude)));
	_q_inertial.normalize();

	_inertial_velocity = inertialVelocity(0.0);
	_gravitation = gravitation(0.0);
}

GenNav::Geodetic GenNav::geodetic(double t) const
{
	const double tau = t - static_cast<double>(_profile.t_start_s);

	Geodetic g{};
	quarterCosineThenRamp(tau, _horizontal_omega, _latitude_amplitude, g.latitude, g.latitude_rate);
	quarterCosineThenRamp(tau, _horizontal_omega, _longitude_amplitude, g.longitude, g.longitude_rate);

	const double climb_amplitude = 0.5 * static_cast<double>(_profile.climb_height_m);
	const double oscillation_amplitude = 0.5 * static_cast<double>(_profile.altitude_oscillation_m);

	if (tau <= 0.0) {
		g.altitude = 0.0;
		g.altitude_rate = 0.0;

	} else if (_climb_omega * tau < M_PI) {
		g.altitude = climb_amplitude * (1.0 - cos(_climb_omega * tau));
		g.altitude_rate = climb_amplitude * _climb_omega * sin(_climb_omega * tau);

	} else {
		const double phase = _oscillation_omega * (tau - M_PI / _climb_omega);
		g.altitude = 2.0 * climb_amplitude + oscillation_amplitude * (1.0 - cos(phase));
		g.altitude_rate = oscillation_amplitude * _oscillation_omega * sin(phase);
	}

	g.latitude += math::radians(_profile.latitude_deg);
	g.longitude += math::radians(_profile.longitude_deg);
	g.altitude += static_cast<double>(_profile.altitude_m);

	return g;
}

// The profile integrates latitude straight through a pole. Past it, the same point lies on
// the opposite meridian, where north and east are reversed.
static bool isPastPole(double latitude)
{
	return fabs(latitude) > M_PI_2;
}

LatLonAlt GenNav::position(double t) const
{
	const Geodetic g = geodetic(t);
	LatLonAlt lla;

	if (isPastPole(g.latitude)) {
		lla.setLatLonRad(copysign(M_PI, g.latitude) - g.latitude, matrix::wrap_pi(g.longitude + M_PI));

	} else {
		lla.setLatLonRad(g.latitude, matrix::wrap_pi(g.longitude));
	}

	lla.setAltitude(static_cast<float>(g.altitude));
	return lla;
}

Vector3d GenNav::velocityNed(double t) const
{
	const Geodetic g = geodetic(t);
	double meridian_radius;
	double transverse_radius;
	LatLonAlt::computeRadiiOfCurvature(g.latitude, meridian_radius, transverse_radius);

	Vector3d velocity((meridian_radius + g.altitude) * g.latitude_rate,
			  (transverse_radius + g.altitude) * cos(g.latitude) * g.longitude_rate,
			  -g.altitude_rate);

	if (isPastPole(g.latitude)) {
		return Vector3d(-velocity(0), -velocity(1), velocity(2));
	}

	return velocity;
}

Vector3d GenNav::inertialVelocity(double t) const
{
	const LatLonAlt lla = position(t);
	const Vector3d velocity_ecef = lla.computeRotEcefToNed<double>().transpose() * velocityNed(t);
	const Vector3d earth_rate(0.0, 0.0, kEarthRate);

	return rotEcefToInertial(t) * (velocity_ecef + earth_rate.cross(lla.toEcef()));
}

Vector3d GenNav::gravitation(double t) const
{
	const LatLonAlt lla = position(t);
	const Vector3d gravity_ned(0.0, 0.0, static_cast<double>(LatLonAlt::Wgs84::gravity(lla.latitude_rad())));
	const Vector3d gravity_ecef = lla.computeRotEcefToNed<double>().transpose() * gravity_ned;
	const Vector3d earth_rate(0.0, 0.0, kEarthRate);

	// normal gravity already contains the centrifugal acceleration of the rotating Earth
	return rotEcefToInertial(t) * (gravity_ecef + earth_rate.cross(earth_rate.cross(lla.toEcef())));
}

Dcmd GenNav::rotEcefToInertial(double t)
{
	return Dcmd(matrix::Eulerd(0.0, 0.0, kEarthRate * t));
}

matrix::SquareMatrix<double, 3> GenNav::meanRotation(const Vector3d &rotation_vector)
{
	const double angle = rotation_vector.norm();
	const double angle2 = angle * angle;
	double coeff1;
	double coeff2;

	if (angle < 1e-2) {
		coeff1 = 0.5 - angle2 / 24.0 + angle2 * angle2 / 720.0;
		coeff2 = 1.0 / 6.0 - angle2 / 120.0 + angle2 * angle2 / 5040.0;

	} else {
		coeff1 = (1.0 - cos(angle)) / angle2;
		coeff2 = (1.0 - sin(angle) / angle) / angle2;
	}

	const matrix::SquareMatrix<double, 3> skew = rotation_vector.hat();
	return matrix::eye<double, 3>() + coeff1 * skew + coeff2 * skew * skew;
}

Dcmd GenNav::dcmBodyToNed() const
{
	const Dcmd ecef_to_ned = position(_time).computeRotEcefToNed<double>();
	return ecef_to_ned * rotEcefToInertial(_time).transpose() * Dcmd(_q_inertial);
}

void GenNav::step(double t, Vector3f &delta_angle, Vector3f &delta_velocity)
{
	const double dt = t - _time;

	if (dt <= 0.0) {
		delta_angle.zero();
		delta_velocity.zero();
		return;
	}

	// Constant inertial rate over the step. The truth attitude is propagated with this same
	// rotation vector, so sampling the nav-frame rate at mid-step only shapes the profile.
	const double t_mid = _time + 0.5 * dt;
	const LatLonAlt position_mid = position(t_mid);
	const Vector3d velocity_mid = velocityNed(t_mid);
	const Vector3d earth_rate_ned(kEarthRate * cos(position_mid.latitude_rad()), 0.0,
				      -kEarthRate * sin(position_mid.latitude_rad()));
	const Vector3d transport_rate_ned(position_mid.computeAngularRateNavFrame(Vector3f(velocity_mid)));
	const Vector3d rotation_vector = (_body_rate + dcmBodyToNed().transpose() * (earth_rate_ned + transport_rate_ned)) * dt;

	const Vector3d inertial_velocity = inertialVelocity(t);
	const Vector3d gravitation_next = gravitation(t);
	const Vector3d delta_velocity_inertial = inertial_velocity - _inertial_velocity
			- 0.5 * (_gravitation + gravitation_next) * dt;

	// Savage (7.2.2.2.1-5): the specific force integrated in the body frame frozen at the
	// start of the step equals D times the accelerometer increment for constant rate and force
	const Vector3d delta_velocity_body = meanRotation(rotation_vector).I() * (Dcmd(_q_inertial).transpose() * delta_velocity_inertial);

	_q_inertial = _q_inertial * Quatd(matrix::AxisAngled(rotation_vector));
	_q_inertial.normalize();
	_inertial_velocity = inertial_velocity;
	_gravitation = gravitation_next;
	_time = t;

	delta_angle = Vector3f(rotation_vector);
	delta_velocity = Vector3f(delta_velocity_body);
}
