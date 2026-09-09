/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <cfloat>
#include <cmath>
#include <gtest/gtest.h>

#include "EKF/ekf.h"

#include "../EKF/python/ekf_derivation/generated/predict_vel_pos_closed_form.h"
#include "../EKF/python/ekf_derivation/generated/trig_series.h"

using namespace matrix;

namespace
{

// The relative accuracy trig_series_derivation.py bounds the single precision coefficients to
constexpr double kMaxRelativeError = 16.0 * static_cast<double>(FLT_EPSILON);

constexpr double kGravity = 9.80665;

// Double precision references for the three coefficients. The closed forms cancel in double
// precision too, just several decades lower in theta, so the same switch is needed; with these many
// series terms and this threshold both branches are accurate to better than 1e-13 relative.
constexpr double kReferenceThetaSwitch = 0.5;

double referenceSeries(const double theta_sq, const unsigned factorial_offset)
{
	double sum = 0.0;
	double power = 1.0;
	double factorial = 1.0;

	for (unsigned i = 1; i <= factorial_offset; i++) {
		factorial *= i;
	}

	for (unsigned n = 0; n < 8; n++) {
		sum += (n % 2 == 0 ? 1.0 : -1.0) * power / factorial;
		power *= theta_sq;
		factorial *= (2 * n + factorial_offset + 1) * (2 * n + factorial_offset + 2);
	}

	return sum;
}

double referenceC1(const double theta_sq)
{
	if (theta_sq < kReferenceThetaSwitch * kReferenceThetaSwitch) {
		return referenceSeries(theta_sq, 2);
	}

	return (1.0 - std::cos(std::sqrt(theta_sq))) / theta_sq;
}

double referenceC2(const double theta_sq)
{
	if (theta_sq < kReferenceThetaSwitch * kReferenceThetaSwitch) {
		return referenceSeries(theta_sq, 3);
	}

	const double theta = std::sqrt(theta_sq);
	return (theta - std::sin(theta)) / (theta_sq * theta);
}

double referenceC3(const double theta_sq)
{
	if (theta_sq < kReferenceThetaSwitch * kReferenceThetaSwitch) {
		return referenceSeries(theta_sq, 4);
	}

	return (0.5 * theta_sq + std::cos(std::sqrt(theta_sq)) - 1.0) / (theta_sq * theta_sq);
}

// Exact solution of the strapdown initial value problem for a body rate and a body specific force
// that are both constant over the interval, evaluated in double precision
void referencePropagation(const Quatf &quat, const Vector3f &vel, const Vector3f &d_vel, const Vector3f &d_ang,
			  const float dt, Vector3<double> &vel_new, Vector3<double> &delta_pos)
{
	const Quaternion<double> q(static_cast<double>(quat(0)), static_cast<double>(quat(1)),
				   static_cast<double>(quat(2)), static_cast<double>(quat(3)));
	const Dcm<double> r_to_earth(q);

	Vector3<double> omega;
	Vector3<double> accel;
	Vector3<double> vel_0;

	for (int i = 0; i < 3; i++) {
		omega(i) = static_cast<double>(d_ang(i));
		accel(i) = static_cast<double>(d_vel(i)) / static_cast<double>(dt);
		vel_0(i) = static_cast<double>(vel(i));
	}

	const double theta_sq = omega.norm_squared();
	const SquareMatrix<double, 3> theta_hat = omega.hat();
	const SquareMatrix<double, 3> theta_hat_sq = theta_hat * theta_hat;
	const SquareMatrix<double, 3> identity = eye<double, 3>();

	// integral over the interval of exp(theta_hat s), and of (dt - s) exp(theta_hat s), normalised by dt and dt^2
	const SquareMatrix<double, 3> j1 = identity + theta_hat * referenceC1(theta_sq) + theta_hat_sq * referenceC2(
			theta_sq);
	const SquareMatrix<double, 3> j2 = identity * 0.5 + theta_hat * referenceC2(theta_sq) + theta_hat_sq * referenceC3(
			theta_sq);

	const Vector3<double> gravity(0.0, 0.0, kGravity);
	const double dt_d = static_cast<double>(dt);

	vel_new = vel_0 + r_to_earth * (j1 * accel) * dt_d + gravity * dt_d;
	delta_pos = vel_0 * dt_d + r_to_earth * (j2 * accel) * (dt_d * dt_d) + gravity * (0.5 * dt_d * dt_d);
}

void predict(const Quatf &quat, const Vector3f &vel, const Vector3f &d_vel, const Vector3f &d_ang, const float dt,
	     Vector3f &vel_new, Vector3f &delta_pos)
{
	StateSample state{};
	state.quat_nominal = quat;
	state.vel = vel;

	const float theta_sq = d_ang.norm_squared();

	sym::PredictVelPosClosedForm(state.vector(), d_vel, d_ang, dt, static_cast<float>(kGravity),
				     math::trig_series::c1(theta_sq),
				     math::trig_series::c2(theta_sq),
				     math::trig_series::c3(theta_sq),
				     &vel_new, &delta_pos);
}

// Trapezoidal integration, the scheme the closed form replaces
void predictTrapezoidal(const Quatf &quat, const Vector3f &vel, const Vector3f &d_vel, const Vector3f &d_ang,
			const float dt, Vector3f &vel_new, Vector3f &delta_pos)
{
	const Dcmf r_to_earth(Quatf(quat * Quatf(AxisAnglef(d_ang))).normalized());
	vel_new = vel + r_to_earth * d_vel;
	vel_new(2) += static_cast<float>(kGravity) * dt;
	delta_pos = (vel + vel_new) * dt * 0.5f;
}

double maxRelativeError(const Vector3f &value, const Vector3<double> &reference)
{
	double worst = 0.0;

	for (int i = 0; i < 3; i++) {
		if (std::fabs(reference(i)) > 1e-9) {
			worst = fmax(worst, std::fabs(static_cast<double>(value(i)) - reference(i)) / std::fabs(reference(i)));
		}
	}

	return worst;
}

double relativeError(const float value, const double reference)
{
	return std::fabs(static_cast<double>(value) / reference - 1.0);
}

} // namespace

TEST(TrigSeries, limitsAtZero)
{
	EXPECT_FLOAT_EQ(math::trig_series::c1(0.f), 0.5f);
	EXPECT_FLOAT_EQ(math::trig_series::c2(0.f), 1.f / 6.f);
	EXPECT_FLOAT_EQ(math::trig_series::c3(0.f), 1.f / 24.f);
}

TEST(TrigSeries, accuracyOverFullRange)
{
	// GIVEN: rotation angles from far below what an IMU sample can resolve up to a half turn.
	// Over the band a 250 Hz IMU produces, theta in [1e-5, 1e-2], the closed forms lose all
	// significance to cancellation: c1 and c2 underflow to zero below theta = 1e-4, and c3 does so
	// over the whole band.
	for (int i = -700; i <= 50; i++) {
		const float theta = powf(10.f, i / 100.f);

		if (theta > M_PI_F) {
			break;
		}

		const float theta_sq = theta * theta;
		const double theta_sq_d = static_cast<double>(theta) * static_cast<double>(theta);

		// THEN: every coefficient stays within the bound the generator guarantees
		EXPECT_LT(relativeError(math::trig_series::c1(theta_sq), referenceC1(theta_sq_d)), kMaxRelativeError)
				<< "c1 at theta = " << theta;
		EXPECT_LT(relativeError(math::trig_series::c2(theta_sq), referenceC2(theta_sq_d)), kMaxRelativeError)
				<< "c2 at theta = " << theta;
		EXPECT_LT(relativeError(math::trig_series::c3(theta_sq), referenceC3(theta_sq_d)), kMaxRelativeError)
				<< "c3 at theta = " << theta;
	}
}

TEST(TrigSeries, matchesSeriesDefinition)
{
	// GIVEN: an angle small enough that the leading term dominates.
	// The statement of Theorem 1 of arXiv:2310.04886 prints c1 and c3 without the n = 0 term of
	// the series its own proof derives them from; these limits pin down which form is used.
	const float theta_sq = 1e-8f;

	// THEN: each coefficient tends to the n = 0 term of sum (-1)^n theta^2n / (2n+k)!
	EXPECT_NEAR(math::trig_series::c1(theta_sq), 1.f / 2.f, 1e-6f);
	EXPECT_NEAR(math::trig_series::c2(theta_sq), 1.f / 6.f, 1e-6f);
	EXPECT_NEAR(math::trig_series::c3(theta_sq), 1.f / 24.f, 1e-6f);
}

TEST(PredictVelPosClosedForm, matchesExactSolution)
{
	const float dt = 0.004f;
	const Quatf quat(Eulerf(radians(20.f), radians(-35.f), radians(110.f)));
	const Vector3f vel(5.f, -2.f, 1.f);
	const Vector3f accel(0.5f, -0.3f, -9.7f);

	// GIVEN: body rates spanning a still vehicle up to well past any gyro full scale range
	const float rates[] = {0.f, 1e-3f, 1e-2f, 0.1f, 1.f, 5.f, 35.f, 125.f, 300.f};

	for (const float rate : rates) {
		const Vector3f d_ang = Vector3f(1.f, 0.5f, -0.3f).unit() * rate * dt;

		Vector3f vel_new;
		Vector3f delta_pos;
		predict(quat, vel, accel * dt, d_ang, dt, vel_new, delta_pos);

		Vector3<double> vel_new_ref;
		Vector3<double> delta_pos_ref;
		referencePropagation(quat, vel, accel * dt, d_ang, dt, vel_new_ref, delta_pos_ref);

		// THEN: the propagation is exact to single precision, at every rate
		EXPECT_LT(maxRelativeError(vel_new, vel_new_ref), 1e-6) << "velocity at " << rate << " rad/s";
		EXPECT_LT(maxRelativeError(delta_pos, delta_pos_ref), 1e-6) << "position at " << rate << " rad/s";
	}
}

TEST(PredictVelPosClosedForm, finiteAtAndNearZeroRotation)
{
	// GIVEN: rotation angles down to and including exactly zero. Every coefficient is a 0/0 limit
	// there, so evaluating the closed forms directly returns NaN and poisons the whole filter state.
	const float dt = 0.004f;
	const Quatf quat(Eulerf(radians(5.f), radians(-10.f), radians(80.f)));
	const Vector3f vel(4.f, 1.f, -0.5f);
	const Vector3f accel(0.3f, 0.2f, -9.8f);

	for (const float rate : {0.f, FLT_MIN, 1e-9f, 1e-6f, 1e-3f}) {
		const Vector3f d_ang = Vector3f(1.f, 0.5f, -0.3f).unit() * rate * dt;

		Vector3f vel_new;
		Vector3f delta_pos;
		predict(quat, vel, accel * dt, d_ang, dt, vel_new, delta_pos);

		Vector3<double> vel_new_ref;
		Vector3<double> delta_pos_ref;
		referencePropagation(quat, vel, accel * dt, d_ang, dt, vel_new_ref, delta_pos_ref);

		// THEN: the propagation stays finite and correct
		for (int i = 0; i < 3; i++) {
			ASSERT_TRUE(std::isfinite(vel_new(i))) << "velocity at " << rate << " rad/s";
			ASSERT_TRUE(std::isfinite(delta_pos(i))) << "position at " << rate << " rad/s";
		}

		EXPECT_LT(maxRelativeError(vel_new, vel_new_ref), 1e-6) << "velocity at " << rate << " rad/s";
		EXPECT_LT(maxRelativeError(delta_pos, delta_pos_ref), 1e-6) << "position at " << rate << " rad/s";
	}
}

TEST(PredictVelPosClosedForm, reducesToConstantAccelerationWhenNotRotating)
{
	const float dt = 0.01f;
	const Quatf quat(Eulerf(0.f, 0.f, radians(45.f)));
	const Vector3f vel(3.f, 1.f, -0.5f);
	const Vector3f accel(1.f, 2.f, -3.f);

	Vector3f vel_new;
	Vector3f delta_pos;
	predict(quat, vel, accel * dt, Vector3f(), dt, vel_new, delta_pos);

	// THEN: with no rotation the result is the textbook constant acceleration solution
	const Vector3f accel_earth = Dcmf(quat) * accel + Vector3f(0.f, 0.f, static_cast<float>(kGravity));
	const Vector3f vel_expected = vel + accel_earth * dt;
	const Vector3f delta_pos_expected = vel * dt + accel_earth * (0.5f * dt * dt);

	EXPECT_TRUE(isEqual(vel_new, vel_expected, 1e-5f));
	EXPECT_TRUE(isEqual(delta_pos, delta_pos_expected, 1e-6f));
}

TEST(PredictVelPosClosedForm, moreAccurateThanTrapezoidalIntegration)
{
	// GIVEN: a rate high enough that the attitude changes appreciably within one IMU sample
	const float dt = 0.004f;
	const Quatf quat(Eulerf(radians(10.f), radians(5.f), radians(-70.f)));
	const Vector3f vel(12.f, -4.f, 0.5f);
	const Vector3f accel(2.f, -1.f, -9.5f);
	const Vector3f d_ang = Vector3f(0.2f, -0.9f, 0.4f).unit() * 6.f * dt;

	Vector3f vel_closed_form;
	Vector3f delta_pos_closed_form;
	predict(quat, vel, accel * dt, d_ang, dt, vel_closed_form, delta_pos_closed_form);

	Vector3f vel_trapezoidal;
	Vector3f delta_pos_trapezoidal;
	predictTrapezoidal(quat, vel, accel * dt, d_ang, dt, vel_trapezoidal, delta_pos_trapezoidal);

	Vector3<double> vel_ref;
	Vector3<double> delta_pos_ref;
	referencePropagation(quat, vel, accel * dt, d_ang, dt, vel_ref, delta_pos_ref);

	// THEN: the closed form is closer to the exact solution than the scheme it replaces
	EXPECT_LT(maxRelativeError(vel_closed_form, vel_ref), maxRelativeError(vel_trapezoidal, vel_ref));
	EXPECT_LT(maxRelativeError(delta_pos_closed_form, delta_pos_ref), maxRelativeError(delta_pos_trapezoidal,
			delta_pos_ref));
}

TEST(PredictVelPosClosedForm, oneStepMatchesManySubsteps)
{
	// GIVEN: a constant body rate and specific force integrated over one long interval
	const float dt = 0.1f;
	const int n_substeps = 100;
	const Quatf quat_0(Eulerf(radians(-15.f), radians(25.f), radians(200.f)));
	const Vector3f vel_0(8.f, 3.f, -1.f);
	const Vector3f accel(0.8f, 0.4f, -9.9f);
	const Vector3f gyro = Vector3f(0.3f, 0.7f, -0.2f).unit() * 3.f;

	Vector3f vel_one_step;
	Vector3f delta_pos_one_step;
	predict(quat_0, vel_0, accel * dt, gyro * dt, dt, vel_one_step, delta_pos_one_step);

	// WHEN: the same interval is integrated as many short closed-form steps
	const float dt_substep = dt / n_substeps;
	Quatf quat = quat_0;
	Vector3f vel = vel_0;
	Vector3f delta_pos_total;

	for (int i = 0; i < n_substeps; i++) {
		Vector3f delta_pos;
		predict(quat, vel, accel * dt_substep, gyro * dt_substep, dt_substep, vel, delta_pos);
		delta_pos_total += delta_pos;
		quat = Quatf(quat * Quatf(AxisAnglef(gyro * dt_substep))).normalized();
	}

	// THEN: both agree, so the single step carries no step size error
	EXPECT_TRUE(isEqual(vel_one_step, vel, 1e-4f));
	EXPECT_TRUE(isEqual(delta_pos_one_step, delta_pos_total, 1e-5f));
}
