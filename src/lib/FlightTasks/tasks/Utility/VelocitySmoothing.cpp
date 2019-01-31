/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "VelocitySmoothing.hpp"

#include <cstdio>
#include <float.h>
#include <math.h>
#include <px4_defines.h>

#include <mathlib/mathlib.h>

VelocitySmoothing::VelocitySmoothing(float initial_accel, float initial_vel, float initial_pos)
{
	reset(initial_accel, initial_vel, initial_pos);
}

void VelocitySmoothing::reset(float accel, float vel, float pos)
{
	_jerk = 0.f;
	_accel = accel;
	_vel = vel;
	_pos = pos;
}

float VelocitySmoothing::saturateT1ForAccel(float accel_prev, float max_jerk, float T1)
{
	/* Check maximum acceleration, saturate and recompute T1 if needed */
	float accel_T1 = accel_prev + max_jerk * T1;
	float T1_new = T1;

	if (accel_T1 > _max_accel) {
		T1_new = (_max_accel - accel_prev) / max_jerk;

	} else if (accel_T1 < -_max_accel) {
		T1_new = (-_max_accel - accel_prev) / max_jerk;
	}

	return T1_new;
}

float VelocitySmoothing::computeT1(float accel_prev, float vel_prev, float vel_setpoint, float max_jerk)
{
	float b = 2.f * accel_prev / max_jerk;
	float c = vel_prev / max_jerk + accel_prev * accel_prev / (2.f * max_jerk * max_jerk) - vel_setpoint / max_jerk;
	float delta = b * b - 4.f * c;

	if (delta < 0.f) {
		// Solution is not real
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float T1_plus = (-b + sqrt_delta) * 0.5f;
	float T1_minus = (-b - sqrt_delta) * 0.5f;

	float T3_plus = accel_prev / max_jerk + T1_plus;
	float T3_minus = accel_prev / max_jerk + T1_minus;

	float T1 = 0.f;

	if (T1_plus >= 0.f && T3_plus >= 0.f) {
		T1 = T1_plus;

	} else if ( T1_minus >= 0.f && T3_minus >= 0.f) {
		T1 = T1_minus;
	}

	T1 = saturateT1ForAccel(accel_prev, max_jerk, T1);

	if (T1 < _dt) {
		T1 = 0.f;
	}

	return math::max(T1, 0.f);
}

float VelocitySmoothing::computeT1(float T123, float accel_prev, float vel_prev, float vel_setpoint, float max_jerk)
{
	float a = -max_jerk;
	float b = max_jerk * T123 - accel_prev;
	float delta = T123 * T123 * max_jerk * max_jerk + 2.f * T123 * accel_prev * max_jerk - accel_prev * accel_prev
		      + 4.f * max_jerk * (vel_prev - vel_setpoint);

	float sqrt_delta = sqrtf(delta);
	float denominator_inv = 1.f / (2.f * a);
	float T1_plus = math::max((-b + sqrt_delta) * denominator_inv, 0.f);
	float T1_minus = math::max((-b - sqrt_delta) * denominator_inv, 0.f);

	float T3_plus = computeT3(T1_plus, accel_prev, max_jerk);
	float T3_minus = computeT3(T1_minus, accel_prev, max_jerk);

	float T13_plus = T1_plus + T3_plus;
	float T13_minus = T1_minus + T3_minus;

	float T1 = 0.f;

	if (T13_plus > T123) {
		T1 = T1_minus;

	} else if (T13_minus > T123) {
		T1 = T1_plus;
	}

	T1 = saturateT1ForAccel(accel_prev, max_jerk, T1);

	if (T1 < _dt) {
		T1 = 0.f;
	}

	return T1;
}


float VelocitySmoothing::computeT2(float T1, float T3, float accel_prev, float vel_prev, float vel_setpoint,
				   float max_jerk)
{
	float f = accel_prev * T1 + max_jerk * T1 * T1 * 0.5f + vel_prev + accel_prev * T3 + max_jerk * T1 * T3
		  - max_jerk * T3 * T3 * 0.5f;
	float T2 = (vel_setpoint - f) / (accel_prev + max_jerk * T1);

	if (T2 < _dt) {
		T2 = 0.f;
	}

	return math::max(T2, 0.f);
}

float VelocitySmoothing::computeT2(float T123, float T1, float T3)
{
	float T2 = T123 - T1 - T3;
	return math::max(T2, 0.f);
}

float VelocitySmoothing::computeT3(float T1, float accel_prev, float max_jerk)
{
	float T3 = accel_prev / max_jerk + T1;

	if (T1 < FLT_EPSILON && T3 < _dt && T3 > 0.f) {
		T3 = _dt;
		_max_jerk_T1 = accel_prev / T3;
	}

	return math::max(T3, 0.f);
}

void VelocitySmoothing::integrateT(float dt, float jerk, float accel_prev, float vel_prev, float pos_prev,
				   float &accel_out, float &vel_out, float &pos_out)
{
	accel_out = jerk * dt + accel_prev;

	vel_out = dt * 0.5f * (accel_out + accel_prev) + vel_prev;

	pos_out = dt / 3.f * (vel_out + accel_prev * dt * 0.5f + 2.f * vel_prev) + _pos;
}

void VelocitySmoothing::updateDurations(float dt, float vel_setpoint)
{
	_vel_sp = math::constrain(vel_setpoint, -_max_vel, _max_vel);
	_dt = math::max(dt, FLT_EPSILON);
	updateDurations();
}

void VelocitySmoothing::updateDurations(float T123)
{
	float T1, T2, T3;

	/* Depending of the direction, start accelerating positively or negatively */
	_max_jerk_T1 = (_vel_sp - _vel > 0.f) ? _max_jerk : -_max_jerk;

	// compute increasing acceleration time
	if (PX4_ISFINITE(T123)) {
		T1 = computeT1(T123, _accel, _vel, _vel_sp, _max_jerk_T1);

	} else {
		T1 = computeT1(_accel, _vel, _vel_sp, _max_jerk_T1);
	}

	// compute decreasing acceleration time
	T3 = computeT3(T1, _accel, _max_jerk_T1);

	// compute constant acceleration time
	if (PX4_ISFINITE(T123)) {
		T2 = computeT2(T123, T1, T3);

	} else {
		T2 = computeT2(T1, T3, _accel, _vel, _vel_sp, _max_jerk_T1);
	}

	_T1 = T1;
	_T2 = T2;
	_T3 = T3;
}

void VelocitySmoothing::integrate(float &accel_setpoint_smooth, float &vel_setpoint_smooth,
				  float &pos_setpoint_smooth)
{
	integrate(_dt, 1.f, accel_setpoint_smooth, vel_setpoint_smooth, pos_setpoint_smooth);
}

void VelocitySmoothing::integrate(float dt, float integration_scale_factor, float &accel_setpoint_smooth,
				  float &vel_setpoint_smooth,
				  float &pos_setpoint_smooth)
{
	/* Apply correct jerk (min, max or zero) */
	if (_T1 > FLT_EPSILON) {
		_jerk = _max_jerk_T1;

		if (_T1 < dt && dt > _dt) {
			// _T1 was supposed to be _dt, however, now, dt is bigger than _dt. We have to reduce the jerk to avoid an acceleration overshoot.
			_jerk *= _dt / dt; // Keep the same area _dt * _jerk = dt * jerk_new
		}

	} else if (_T2 > FLT_EPSILON) {
		_jerk = 0.f;

	} else if (_T3 > FLT_EPSILON) {
		_jerk = -_max_jerk_T1;

		if (_T3 < dt && dt > _dt) {
			// Same as for _T1 < dt above
			_jerk *= _dt / dt;
		}

	} else {
		_jerk = 0.f;
	}

	/* Integrate the trajectory */
	float accel_new, vel_new, pos_new;
	integrateT(dt * integration_scale_factor, _jerk, _accel, _vel, _pos, accel_new, vel_new, pos_new);

	_accel = accel_new;
	_vel = vel_new;
	_pos = pos_new;

	/* set output variables */
	accel_setpoint_smooth = _accel;
	vel_setpoint_smooth = _vel;
	pos_setpoint_smooth = _pos;
}

void VelocitySmoothing::timeSynchronization(VelocitySmoothing *traj, int n_traj)
{
	float desired_time = 0.f;
	int longest_traj_index = 0;

	for (int i = 0; i < n_traj; i++) {
		const float T123 = traj[i].getTotalTime();

		if (T123 > desired_time) {
			desired_time = T123;
			longest_traj_index = i;
		}
	}

	if (desired_time > FLT_EPSILON) {
		for (int i = 0; i < n_traj; i++) {
			if (i != longest_traj_index) {
				traj[i].updateDurations(desired_time);
			}
		}
	}
}
