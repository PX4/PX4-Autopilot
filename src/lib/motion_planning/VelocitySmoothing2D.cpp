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

#include "VelocitySmoothing2D.hpp"

#include <float.h>
#include <mathlib/mathlib.h>

using matrix::Vector2f;
using matrix::SquareMatrix;

VelocitySmoothing2D::VelocitySmoothing2D(const Vector2f &initial_accel, const Vector2f &initial_vel,
		const Vector2f &initial_pos)
{
	reset(initial_accel, initial_vel, initial_pos);
}

void VelocitySmoothing2D::reset(const Vector2f &accel, const Vector2f &vel, const Vector2f &pos)
{
	_state.j.setZero();
	_state.a = accel;
	_state.v = vel;
	_state.x = pos;

	_state_init = _state;
}

Vector2f VelocitySmoothing2D::saturateT1ForAccel(const Vector2f &a0, float j_max, const Vector2f &T1,
		float a_max, const Vector2f &v3) const
{
	/* Check maximum acceleration, saturate and recompute T1 if needed */
	Vector2f accel_T1 = a0 + j_max * T1;
	Vector2f T1_new = T1;

	if (accel_T1.longerThan(a_max)) {
		// On the acceleration limit the direction of the constant acceleration is not free anymore: it has to be
		// the one of the velocity change left after the two ramps, which itself depends on T1. Iterate the two.
		for (int i = 0; i < 10; i++) {
			T1_new = (a_max * accel_T1.unit_or_zero() - a0) / j_max;
			const Vector2f accel_T1_new = a_max * (v3 - 0.5f * a0 * T1_new.norm()).unit_or_zero();

			if (!(accel_T1_new - accel_T1).longerThan(1e-4f)) {
				break;
			}

			accel_T1 = accel_T1_new;
		}

		T1_new = (accel_T1 - a0) / j_max;
	}

	return T1_new;
}

Vector2f VelocitySmoothing2D::solveHoldAccel(const Vector2f &a0, const Vector2f &v3, float j_max, float a_max,
		float T123, bool minimize_total_time, Vector2f a1) const
{
	// Newton's method on F(a1) = delta_v(a1) - v3 = 0 with
	// delta_v = (a0 + a1) / 2 * T1 + a1 * T2 + a1 / 2 * T3,   T1 = |a1 - a0| / j,   T3 = |a1| / j
	// and T2 = 0 (minimum total time) or T2 = T123 - T1 - T3 (given total time).
	// The derivatives of the durations are dT1/da1 = (a1 - a0)^T / (j^2 T1) and dT3/da1 = a1^T / (j^2 T3).
	const float c = minimize_total_time ? 0.f : 1.f; // T2 depends on a1 only when the total time is given

	for (int i = 0; i < 10; i++) {
		const Vector2f delta_a = a1 - a0;
		const float T1 = delta_a.norm() / j_max;
		const float T3 = a1.norm() / j_max;
		const float T2 = minimize_total_time ? 0.f : T123 - T1 - T3;

		const Vector2f F = 0.5f * (a0 + a1) * T1 + a1 * T2 + 0.5f * a1 * T3 - v3;

		if (!F.longerThan(1e-4f)) {
			break;
		}

		SquareMatrix<float, 2> dF = (0.5f * T1 + T2 + 0.5f * T3) * matrix::eye<float, 2>();

		if (T1 > FLT_EPSILON) {
			dF += SquareMatrix<float, 2>((0.5f * (a0 + a1) - c * a1) * delta_a.transpose()) / (j_max * j_max * T1);
		}

		if (T3 > FLT_EPSILON) {
			dF += SquareMatrix<float, 2>((0.5f - c) * a1 * a1.transpose()) / (j_max * j_max * T3);
		}

		SquareMatrix<float, 2> dF_inv;

		if (!matrix::inv(dF, dF_inv)) {
			break;
		}

		a1 -= dF_inv * F;

		if (a1.longerThan(a_max)) {
			a1 = a_max * a1.unit_or_zero();
		}
	}

	return a1;
}

Vector2f VelocitySmoothing2D::computeT1(const Vector2f &a0, const Vector2f &v3, float j_max, float a_max) const
{
	// Start from the acceleration a pure ramp up / ramp down profile from zero would reach
	const Vector2f a1_init = v3.longerThan(FLT_EPSILON) ?
				 Vector2f(math::min(a_max, sqrtf(j_max * v3.norm())) * v3.unit_or_zero()) : Vector2f(-0.5f * a0);
	const Vector2f a1 = solveHoldAccel(a0, v3, j_max, a_max, 0.f, true, a1_init);
	Vector2f T1 = (a1 - a0) / j_max;

	T1 = saturateT1ForAccel(a0, j_max, T1, a_max, v3);

	return T1;
}

Vector2f VelocitySmoothing2D::computeT1(float T123, const Vector2f &a0, const Vector2f &v3, float j_max,
					float a_max) const
{
	// Start from the minimum-time solution computed in updateDurations()
	const Vector2f a1_init = a0 + j_max * _T1 * _direction;
	const Vector2f a1 = solveHoldAccel(a0, v3, j_max, a_max, T123, false, a1_init);

	return (a1 - a0) / j_max;
}

float VelocitySmoothing2D::computeT2(const Vector2f &T1, const Vector2f &T3, const Vector2f &a0, const Vector2f &v3,
				     float j_max) const
{
	float T2 = 0.f;

	const float T1_norm = T1.norm();
	const float T3_norm = T3.norm();
	const Vector2f den = a0 + j_max * T1;

	if (den.longerThan(FLT_EPSILON)) {
		const Vector2f num = -0.5f * T1 * T1_norm * j_max - T1 * T3_norm * j_max - a0 * T1_norm + 0.5f * T3 * T3_norm * j_max -
				     a0 * T3_norm + v3;
		T2 = (num * den) / den.norm_squared();
	}

	return math::max(T2, 0.f);
}

float VelocitySmoothing2D::computeT2(float T123, float T1, float T3) const
{
	float T2 = T123 - T1 - T3;
	return math::max(T2, 0.f);
}

Vector2f VelocitySmoothing2D::computeT3(const Vector2f &T1, const Vector2f &a0, float j_max) const
{
	return a0 / j_max + T1;
}

void VelocitySmoothing2D::setDurations(const Vector2f &T1, float T2, const Vector2f &T3)
{
	_T1 = T1.norm();
	_direction = T1.unit_or_zero();
	_T2 = T2;
	_T3 = T3.norm();
	_direction_T3 = -T3.unit_or_zero();
}

void VelocitySmoothing2D::updateDurations(const Vector2f &vel_setpoint)
{
	_vel_sp = vel_setpoint;

	if (_vel_sp.longerThan(_max_vel)) {
		_vel_sp = _max_vel * _vel_sp.unit_or_zero();
	}

	_local_time = 0.f;
	_state_init = _state;

	updateDurationsMinimizeTotalTime();
}

void VelocitySmoothing2D::updateDurationsMinimizeTotalTime()
{
	const Vector2f delta_v = _vel_sp - _state.v;

	if (_max_jerk > FLT_EPSILON) { // zero jerk would lead to division by zero
		// compute increasing acceleration time
		const Vector2f T1 = computeT1(_state.a, delta_v, _max_jerk, _max_accel);

		// compute decreasing acceleration time
		const Vector2f T3 = computeT3(T1, _state.a, _max_jerk);

		// compute constant acceleration time
		const float T2 = computeT2(T1, T3, _state.a, delta_v, _max_jerk);

		setDurations(T1, T2, T3);

	} else {
		_T1 = _T2 = _T3 = 0.f;
	}
}

void VelocitySmoothing2D::updateDurationsGivenTotalTime(float T123)
{
	const Vector2f delta_v = _vel_sp - _state.v;

	if (_max_jerk > FLT_EPSILON) { // zero jerk would lead to division by zero
		// compute increasing acceleration time
		const Vector2f T1 = computeT1(T123, _state.a, delta_v, _max_jerk, _max_accel);

		// compute decreasing acceleration time
		const Vector2f T3 = computeT3(T1, _state.a, _max_jerk);

		// compute constant acceleration time
		const float T2 = computeT2(T123, T1.norm(), T3.norm());

		setDurations(T1, T2, T3);

	} else {
		_T1 = _T2 = _T3 = 0.f;
	}
}

Trajectory2D VelocitySmoothing2D::evaluatePoly(float j, const Vector2f &a0, const Vector2f &v0, const Vector2f &x0,
		float t, const Vector2f &d) const
{
	Trajectory2D traj;
	Vector2f jt = d * j;
	float t2 = t * t;
	float t3 = t2 * t;

	traj.j = jt;
	traj.a = a0 + jt * t;
	traj.v = v0 + a0 * t + 0.5f * jt * t2;
	traj.x = x0 + v0 * t + 0.5f * a0 * t2 + 1.f / 6.f * jt * t3;

	return traj;
}

void VelocitySmoothing2D::updateTraj(float dt, float time_stretch)
{
	_local_time += dt * time_stretch;
	float t_remain = _local_time;

	float t1 = math::min(t_remain, _T1);
	_state = evaluatePoly(_max_jerk, _state_init.a, _state_init.v, _state_init.x, t1, _direction);
	t_remain -= t1;

	if (t_remain > 0.f) {
		float t2 = math::min(t_remain, _T2);
		_state = evaluatePoly(0.f, _state.a, _state.v, _state.x, t2, Vector2f{});
		t_remain -= t2;
	}

	if (t_remain > 0.f) {
		float t3 = math::min(t_remain, _T3);
		_state = evaluatePoly(_max_jerk, _state.a, _state.v, _state.x, t3, _direction_T3);
		t_remain -= t3;
	}

	if (t_remain > 0.f) {
		_state = evaluatePoly(0.f, Vector2f{}, _state.v, _state.x, t_remain, Vector2f{});
	}
}
