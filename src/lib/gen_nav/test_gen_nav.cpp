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


#include <gtest/gtest.h>
#include <lib/geo/geo.h>

#include "gen_nav.hpp"

using matrix::Dcmd;
using matrix::Eulerf;
using matrix::Quatd;
using matrix::Vector3d;
using matrix::Vector3f;

static constexpr double kDt = 0.005;

static double rotationAngle(const Quatd &q_a, const Quatd &q_b)
{
	return matrix::AxisAngled(q_a.inversed() * q_b).angle();
}

// Body rate script exercising roll, pitch and yaw during the cruise phase
static Vector3f maneuverRate(double t)
{
	if (t > 20.0 && t <= 25.0) { return Vector3f(0.f, 0.f, 0.3f); }

	if (t > 30.0 && t <= 31.0) { return Vector3f(0.6f, 0.f, 0.f); }

	if (t > 31.0 && t <= 32.0) { return Vector3f(-0.6f, 0.f, 0.f); }

	if (t > 40.0 && t <= 41.0) { return Vector3f(0.f, 0.4f, -0.2f); }

	if (t > 41.0 && t <= 42.0) { return Vector3f(0.f, -0.4f, 0.2f); }

	return Vector3f();
}

TEST(GenNavTest, atRestMeasuresEarthRateAndGravity)
{
	GenNav::Profile profile{};
	profile.t_start_s = 1000.f;
	const Eulerf initial_attitude(0.1f, -0.2f, 1.f);
	GenNav gen_nav(profile, initial_attitude);

	const LatLonAlt position = gen_nav.position(0.0);
	const double earth_rate = static_cast<double>(CONSTANTS_EARTH_SPIN_RATE);
	const Vector3d earth_rate_ned(earth_rate * cos(position.latitude_rad()), 0.0, -earth_rate * sin(position.latitude_rad()));
	const Vector3d gravity_ned(0.0, 0.0, static_cast<double>(LatLonAlt::Wgs84::gravity(position.latitude_rad())));
	const Dcmd ned_to_body = Dcmd(matrix::Eulerd(initial_attitude)).transpose();

	for (int i = 1; i <= 2000; i++) {
		Vector3f delta_angle;
		Vector3f delta_velocity;
		gen_nav.step(i * kDt, delta_angle, delta_velocity);

		EXPECT_LT(Vector3d(Vector3d(delta_angle) / kDt - ned_to_body * earth_rate_ned).norm(), 1e-10);
		EXPECT_LT(Vector3d(Vector3d(delta_velocity) / kDt + ned_to_body * gravity_ned).norm(), 1e-5);
	}

	EXPECT_LT(gen_nav.velocityNed(gen_nav.time()).norm(), 1e-12);
	EXPECT_LT(rotationAngle(Quatd(gen_nav.attitude()), Quatd(matrix::Eulerd(initial_attitude))), 1e-6);
}

static void expectVelocityIsPositionDerivative(const GenNav &gen_nav, double duration)
{
	// float altitude in LatLonAlt limits the finite difference resolution
	const double h = 0.05;

	for (double t = 0.0; t < duration; t += 0.7) {
		const Vector3d ecef_rate = Vector3d(gen_nav.position(t + h).toEcef() - gen_nav.position(t - h).toEcef()) / (2.0 * h);
		const Vector3d velocity_ecef = gen_nav.position(t).computeRotEcefToNed<double>().transpose() * gen_nav.velocityNed(t);
		EXPECT_LT(Vector3d(ecef_rate - velocity_ecef).norm(), 2e-3) << "t = " << t;
	}
}

TEST(GenNavTest, velocityIsPositionDerivative)
{
	GenNav::Profile profile{};
	profile.t_start_s = 2.f;
	GenNav gen_nav(profile, Eulerf());

	expectVelocityIsPositionDerivative(gen_nav, 60.0);

	const Vector3d cruise_velocity = gen_nav.velocityNed(40.0);
	EXPECT_NEAR(cruise_velocity(0), static_cast<double>(profile.cruise_vel_north), 0.01);
	EXPECT_NEAR(cruise_velocity(1), static_cast<double>(profile.cruise_vel_east), 0.01);
	EXPECT_NEAR(gen_nav.position(static_cast<double>(profile.t_start_s + profile.acceleration_time_s)).altitude(),
		    profile.altitude_m + profile.climb_height_m, 1e-3f);
}

TEST(GenNavTest, crossesNorthPole)
{
	GenNav::Profile profile{};
	profile.latitude_deg = 89.997;
	profile.t_start_s = 2.f;
	profile.cruise_vel_east = 0.f;
	GenNav gen_nav(profile, Eulerf());

	expectVelocityIsPositionDerivative(gen_nav, 60.0);

	const LatLonAlt past_pole = gen_nav.position(60.0);
	EXPECT_LT(past_pole.latitude_deg(), 90.0);
	EXPECT_NEAR(fabs(past_pole.longitude_deg() - profile.longitude_deg), 180.0, 1e-9);
	EXPECT_LT(gen_nav.velocityNed(60.0)(0), 0.0);
}

// An exact strapdown algorithm in the inertial frame, with the matching gravity model,
// must reproduce the truth from the increments
TEST(GenNavTest, exactStrapdownReproducesTruth)
{
	GenNav::Profile profile{};
	profile.t_start_s = 5.f;
	GenNav gen_nav(profile, Eulerf(0.05f, 0.1f, 2.f));

	Quatd attitude = gen_nav.attitudeInertial();
	Vector3d velocity = gen_nav.inertialVelocity(0.0);
	Vector3d position = gen_nav.position(0.0).toEcef();
	Vector3d gravitation = gen_nav.gravitation(0.0);

	double max_attitude_error = 0.0;
	double max_velocity_error = 0.0;
	double max_position_error = 0.0;

	for (int i = 1; i <= 12000; i++) {
		const double t = i * kDt;
		gen_nav.setBodyRate(maneuverRate(t));

		Vector3f delta_angle_f;
		Vector3f delta_velocity_f;
		gen_nav.step(t, delta_angle_f, delta_velocity_f);
		const Vector3d delta_angle(delta_angle_f);
		const Vector3d delta_velocity(delta_velocity_f);

		const double angle = delta_angle.norm();
		const matrix::SquareMatrix<double, 3> skew = delta_angle.hat();
		const matrix::SquareMatrix<double, 3> D = matrix::eye<double, 3>()
				+ (angle > 0.0 ? (1.0 - cos(angle)) / (angle * angle) : 0.5) * skew
				+ (angle > 0.0 ? (1.0 - sin(angle) / angle) / (angle * angle) : 1.0 / 6.0) * skew * skew;

		const Vector3d gravitation_next = gen_nav.gravitation(t);
		const Vector3d velocity_next = velocity + Dcmd(attitude) * (D * delta_velocity)
					       + 0.5 * (gravitation + gravitation_next) * kDt;
		position += 0.5 * (velocity + velocity_next) * kDt;
		velocity = velocity_next;
		gravitation = gravitation_next;
		attitude = attitude * Quatd(matrix::AxisAngled(delta_angle));
		attitude.normalize();

		const Vector3d position_truth = GenNav::rotEcefToInertial(t) * gen_nav.position(t).toEcef();
		max_attitude_error = math::max(max_attitude_error, rotationAngle(attitude, gen_nav.attitudeInertial()));
		max_velocity_error = math::max(max_velocity_error, Vector3d(velocity - gen_nav.inertialVelocity(t)).norm());
		max_position_error = math::max(max_position_error, Vector3d(position - position_truth).norm());
	}

	printf("max errors: attitude %.3g rad, velocity %.3g m/s, position %.3g m\n",
	       max_attitude_error, max_velocity_error, max_position_error);

	EXPECT_LT(max_attitude_error, 1e-6);
	EXPECT_LT(max_velocity_error, 1e-4);
	EXPECT_LT(max_position_error, 1e-2);
}
