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

float VelocitySmoothing::computeT1(float accel_prev, float vel_prev, float vel_setpoint, float max_jerk)
{
	float b = 2.f * accel_prev / max_jerk;
	float c = vel_prev / max_jerk + accel_prev * accel_prev / (2.f * max_jerk * max_jerk) - vel_setpoint / max_jerk;
	float delta = b * b - 4.f * c;

	if (delta < 0.f) {
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float T1_plus = (-b + sqrt_delta) * 0.5f;
	float T1_minus = (-b - sqrt_delta) * 0.5f;

	float T1 = math::max(math::max(T1_plus, T1_minus), 0.f);

//	if (T1 < FLT_EPSILON) {
//		// debug
//		printf("No feasible solution found, set T1 = 0\n");
//		printf("T1_plus = %.3f T1_minus = %.3f\n", (double) T1_plus, (double) T1_minus);
//	}

	/* Check maximum acceleration, saturate and recompute T1 if needed */
	float a1 = accel_prev + max_jerk * T1;

	if (a1 > _max_accel) {
		T1 = (_max_accel - accel_prev) / max_jerk;

	} else if (a1 < -_max_accel) {
		T1 = (-_max_accel - accel_prev) / max_jerk;
	}

	return math::max(T1, 0.f);
}


float VelocitySmoothing::computeT2(float T1, float T3, float accel_prev, float vel_prev, float vel_setpoint,
				   float max_jerk)
{
	float f = accel_prev * T1 + max_jerk * T1 * T1 * 0.5f + vel_prev + accel_prev * T3 + max_jerk * T1 * T3
		  - max_jerk * T3 * T3 * 0.5f;
	float T2 = (vel_setpoint - f) / (accel_prev + max_jerk * T1);
	return math::max(T2, 0.f);
}

float VelocitySmoothing::computeT3(float T1, float accel_prev, float max_jerk)
{
	float T3 = accel_prev / max_jerk + T1;
	return math::max(T3, 0.f);
}

void VelocitySmoothing::integrateT(float jerk, float accel_prev, float vel_prev, float pos_prev, float dt,
				   float &accel_out, float &vel_out, float &pos_out)
{
	accel_out = jerk * dt + accel_prev;

	if (accel_out > _max_accel) {
		accel_out = _max_accel;

	} else if (accel_out < -_max_accel) {
		accel_out = -_max_accel;
	}

	vel_out = dt * 0.5f * (accel_out + accel_prev) + vel_prev;

	if (vel_out > _max_vel) {
		vel_out = _max_vel;

	} else if (vel_out < -_max_vel) {
		vel_out = -_max_vel;
	}

	pos_out = dt / 3.f * (vel_out + accel_prev * dt * 0.5f + 2.f * vel_prev) + _pos;
}

void VelocitySmoothing::update(float dt, float pos, float vel_setpoint, float &vel_setpoint_smooth,
			       float &pos_setpoint_smooth)
{
	/* Depending of the direction, start accelerating positively or negatively */
	const float max_jerk = (vel_setpoint - _vel > 0.f) ? _max_jerk : -_max_jerk;

	// compute increasing acceleration time
	float T1 = computeT1(_accel, _vel, vel_setpoint, max_jerk);

	/* Force T1/2/3 to zero if smaller than an epoch to avoid chattering */
	if (T1 < dt) {
		T1 = 0.f;
	}

	// compute decreasing acceleration time
	float T3 = computeT3(T1, _accel, max_jerk);

	if (T3 < dt) {
		T3 = 0.f;
	}

	// compute constant acceleration time
	float T2 = computeT2(T1, T3, _accel, _vel, vel_setpoint, max_jerk);

	if (T2 < dt) {
		T2 = 0.f;
	}

	/* Integrate the trajectory */
	float accel_new, vel_new, pos_new;
	integrateT(_jerk, _accel, _vel, _pos, dt, accel_new, vel_new, pos_new);

	/* Apply correct jerk (min, max or zero) */
	if (T1 > 0.f) {
		_jerk = max_jerk;

	} else if (T2 > 0.f) {
		_jerk = 0.f;

	} else if (T3 > 0.f) {
		_jerk = -max_jerk;

	} else {
		_jerk = 0.f;
	}

	_accel = accel_new;
	_vel = vel_new;

	/* Lock the position setpoint if the error is bigger than some value */
	float x_err = pos_new - pos;

	if (fabsf(x_err) <= max_pos_err) {
		_pos = pos_new;
	} // else: keep last position

	/* set output variables */
	vel_setpoint_smooth = _vel;
	pos_setpoint_smooth = _pos;
}


