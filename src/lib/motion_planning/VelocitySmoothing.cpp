/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using matrix::sign;

/**
 * Constructor for the VelocitySmoothing class. Initializes the state with the given initial acceleration, velocity, and position.
 * @param initial_accel Initial acceleration
 * @param initial_vel Initial velocity
 * @param initial_pos Initial position
 */
VelocitySmoothing::VelocitySmoothing(float initial_accel, float initial_vel, float initial_pos)
{
	reset(initial_accel, initial_vel, initial_pos);
}

/**
 * Reset the state of the VelocitySmoothing object with the given acceleration, velocity, and position.
 * @param accel Current acceleration
 * @param vel Current velocity
 * @param pos Current position
 *
 * Jerk is set to zero, and both _state and _state_init are updated to reflect the new state.
 */
void VelocitySmoothing::reset(float accel, float vel, float pos)
{
	_state.j = 0.f;
	_state.a = accel;
	_state.v = vel;
	_state.x = pos;

	_state_init = _state;
}

/**
 * Saturate T1 to ensure the acceleration at the end of T1 stays within [-a_max, a_max].
 * @param a0 Initial acceleration
 * @param j_max Signed jerk applied during T1 (direction * _max_jerk)
 * @param T1 Unconstrained time interval for increasing acceleration
 * @param a_max Maximum acceleration magnitude (_max_accel)
 * @return Saturated T1
 */
float VelocitySmoothing::saturateT1ForAccel(float a0, float j_max, float T1, float a_max) const
{
	/* Check maximum acceleration, saturate and recompute T1 if needed */
	// Acceleration at the end of T1 assuming constant jerk j_max
	float accel_T1 = a0 + j_max * T1;
	float T1_new = T1;

	// If the acceleration at the end of T1 exceeds [-a_max, a_max], clamp T1 by solving
	// a0 + j_max * T1_new = +/-a_max  =>  T1_new = (+/-a_max - a0) / j_max.
	if (accel_T1 > a_max) {
		T1_new = (a_max - a0) / j_max;

	} else if (accel_T1 < -a_max) {
		T1_new = (-a_max - a0) / j_max;
	}

	return T1_new;
}

/**
 * Compute increasing acceleration time (T1) for a minimum-time profile.
 * @param a0 Initial acceleration
 * @param v3 Desired velocity change (vel_sp - v0)
 * @param j_max Signed jerk applied during T1 (direction * _max_jerk)
 * @param a_max Maximum acceleration magnitude (_max_accel)
 * @return Computed T1
 *
 * First assumes a triangular acceleration profile (T2 = 0) with final acceleration a3 = 0:
 *   a1 = a0 + j_max * T1                  (peak acceleration at the end of T1)
 *   a3 = a1 - j_max * T3 = 0              => T3 = a0 / j_max + T1
 *
 * The total velocity change across T1 and T3 (with T2 = 0) is the area under the acceleration curve:
 *   v3 = dv_T1 + dv_T3 = (a0 * T1 + 0.5 * j_max * T1^2) + (a1 * T3 - 0.5 * j_max * T3^2)
 *
 * Substituting a1 and T3 yields a quadratic equation in T1:
 *   j_max * T1^2 + 2 * a0 * T1 + (0.5 * a0^2 / j_max - v3) = 0
 *   T1 = (-a0 +/- 0.5 * sqrt(delta)) / j_max,  where delta = 2 * a0^2 + 4 * j_max * v3
 *
 * If delta < 0, no real solution exists and T1 = 0 is returned.
 * Between the two roots (T1_plus, T1_minus), the physically valid root is the one that yields
 * non-negative durations (T1 >= 0 and T3 >= 0).
 * Finally, T1 is saturated via saturateT1ForAccel() if the peak acceleration exceeds a_max
 * (in which case computeT2() will later insert a constant-acceleration phase T2 > 0).
 */
float VelocitySmoothing::computeT1(float a0, float v3, float j_max, float a_max) const
{
	float delta = 2.f * a0 * a0 + 4.f * j_max * v3;

	if (delta < 0.f) {
		// Solution is not real
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float T1_plus = (-a0 + 0.5f * sqrt_delta) / j_max;
	float T1_minus = (-a0 - 0.5f * sqrt_delta) / j_max;

	float T3_plus = a0 / j_max + T1_plus;
	float T3_minus = a0 / j_max + T1_minus;

	float T1 = 0.f;

	if (T1_plus >= 0.f && T3_plus >= 0.f) {
		T1 = T1_plus;

	} else if (T1_minus >= 0.f && T3_minus >= 0.f) {
		T1 = T1_minus;
	}

	T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

	return math::max(T1, 0.f);
}

/**
 * Compute increasing acceleration time (T1) when the total trajectory time T123 is fixed.
 * @param T123 Desired total trajectory duration (T1 + T2 + T3)
 * @param a0 Initial acceleration
 * @param v3 Desired velocity change (vel_sp - v0)
 * @param j_max Signed jerk applied during T1 (direction * _max_jerk)
 * @param a_max Maximum acceleration magnitude (_max_accel)
 * @return Computed T1
 *
 * Given the three constraints:
 *   1. Final acceleration is zero: T3 = a0 / j_max + T1  (with peak acceleration a1 = a0 + j_max * T1)
 *   2. Fixed total duration:       T2 = T123 - T1 - T3 = T123 - 2 * T1 - a0 / j_max
 *   3. Total velocity change across T1, T2, and T3 equals v3:
 *      v3 = dv_T1 + dv_T2 + dv_T3
 *         = (a0 * T1 + 0.5 * j_max * T1^2) + (a1 * T2) + (a1 * T3 - 0.5 * j_max * T3^2)
 *
 * Substituting a1, T2, and T3 yields the quadratic equation a * T1^2 + b * T1 + c = 0:
 *   -j_max * T1^2 + (j_max * T123 - a0) * T1 + (a0 * T123 - 0.5 * a0^2 / j_max - v3) = 0
 * where:
 *   a = -j_max
 *   b = j_max * T123 - a0
 *   delta = b^2 - 4 * a * c = T123^2 * j_max^2 + 2 * T123 * a0 * j_max - a0^2 - 4 * j_max * v3
 *
 * If delta < 0, no real solution exists and T1 = 0 is returned.
 * The physically valid root is chosen such that T1 >= 0, T3 >= 0, and T1 + T3 <= T123 (meaning T2 >= 0).
 * Finally, T1 is saturated so that peak acceleration stays within [-a_max, a_max].
 */
float VelocitySmoothing::computeT1(float T123, float a0, float v3, float j_max, float a_max) const
{
	float a = -j_max;
	float b = j_max * T123 - a0;
	float delta = T123 * T123 * j_max * j_max + 2.f * T123 * a0 * j_max - a0 * a0 - 4.f * j_max * v3;

	if (delta < 0.f) {
		// Solution is not real
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float denominator_inv = 1.f / (2.f * a);
	float T1_plus = math::max((-b + sqrt_delta) * denominator_inv, 0.f);
	float T1_minus = math::max((-b - sqrt_delta) * denominator_inv, 0.f);

	float T3_plus = a0 / j_max + T1_plus;
	float T3_minus = a0 / j_max + T1_minus;

	float T1 = 0.f;

	if ((T1_plus >= 0.f && T3_plus >= 0.f) && ((T1_plus + T3_plus) <= T123)) {
		T1 = T1_plus;

	} else if ((T1_minus >= 0.f && T3_minus >= 0.f) && ((T1_minus + T3_minus) <= T123)) {
		T1 = T1_minus;
	}

	T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

	return T1;
}

/**
 * Compute constant acceleration time (T2) for a minimum-time profile.
 * @param T1 Time interval for increasing acceleration
 * @param T3 Time interval for decreasing acceleration
 * @param a0 Initial acceleration
 * @param v3 Desired velocity change (vel_sp - v0)
 * @param j_max Signed jerk applied during T1 (direction * _max_jerk)
 * @return Computed T2
 *
 * When T1 is saturated by a_max, the acceleration ramps T1 and T3 alone do not achieve the full
 * velocity change v3. The remaining velocity change is covered by holding constant acceleration
 * a1 = a0 + j_max * T1 during T2:
 *   v3 = dv_T1 + dv_T2 + dv_T3
 *      = (a0 * T1 + 0.5 * j_max * T1^2) + (a1 * T2) + (a1 * T3 - 0.5 * j_max * T3^2)
 *
 * Solving for T2 = (v3 - dv_T1 - dv_T3) / a1 gives:
 *   T2 = (-0.5 * T1^2 * j_max - T1 * T3 * j_max - T1 * a0 + 0.5 * T3^2 * j_max - T3 * a0 + v3) / (a0 + j_max * T1)
 *
 * If T1 was not saturated by a_max, the numerator evaluates to 0 and T2 = 0.
 */
float VelocitySmoothing::computeT2(float T1, float T3, float a0, float v3, float j_max) const
{
	float T2 = 0.f;

	float den = a0 + j_max * T1;

	if (math::abs_t(den) > FLT_EPSILON) {
		T2 = (-0.5f * T1 * T1 * j_max - T1 * T3 * j_max - T1 * a0 + 0.5f * T3 * T3 * j_max - T3 * a0 + v3) / den;
	}

	return math::max(T2, 0.f);
}

/**
 * Compute constant acceleration time (T2) when total trajectory time T123 is fixed.
 * @param T123 Desired total trajectory duration (T1 + T2 + T3)
 * @param T1 Time interval for increasing acceleration
 * @param T3 Time interval for decreasing acceleration
 * @return Computed T2
 *
 * Since T123 = T1 + T2 + T3, T2 is simply the remaining duration T123 - T1 - T3 (clamped to >= 0).
 */
float VelocitySmoothing::computeT2(float T123, float T1, float T3) const
{
	float T2 = T123 - T1 - T3;
	return math::max(T2, 0.f);
}

/**
 * Compute decreasing acceleration time (T3) required to bring acceleration back to zero.
 * @param T1 Time interval for increasing acceleration
 * @param a0 Initial acceleration
 * @param j_max Signed jerk applied during T1 (direction * _max_jerk)
 * @return Computed T3
 *
 * Acceleration at the end of T1 (and throughout T2) is a1 = a0 + j_max * T1.
 * During T3, opposite jerk (-j_max) is applied to reach zero final acceleration:
 *   a3 = a1 - j_max * T3 = 0  =>  T3 = a0 / j_max + T1
 */
float VelocitySmoothing::computeT3(float T1, float a0, float j_max) const
{
	float T3 = a0 / j_max + T1;
	return math::max(T3, 0.f);
}

/**
 * Update the durations T1, T2, and T3 based on the current state and the desired velocity setpoint.
 * This function should be called on every cycle and before updateTraj().
 * @param vel_setpoint Desired velocity setpoint
 *
 * Clamps the velocity setpoint to [-_max_vel, _max_vel], resets _local_time to zero, snapshots
 * the current state as _state_init, determines the jerk direction, and computes the minimum-time
 * phase durations (T1, T2, T3).
 */
void VelocitySmoothing::updateDurations(float vel_setpoint)
{
	_vel_sp = math::constrain(vel_setpoint, -_max_vel, _max_vel);
	_local_time = 0.f;
	_state_init = _state;

	_direction = computeDirection();

	updateDurationsMinimizeTotalTime();
}

/**
 * Compute the direction of the initial jerk (T1) needed to reach _vel_sp with zero final acceleration.
 * @return +1 for positive jerk in T1, -1 for negative jerk in T1, or 0 if already at target with zero acceleration.
 *
 * Because the current acceleration _state.a may be non-zero, comparing _vel_sp directly against _state.v
 * is insufficient. Instead, we first compute vel_zero_acc (the velocity the trajectory would reach if we
 * immediately braked acceleration to zero at maximum opposing jerk) and compare _vel_sp against vel_zero_acc.
 *
 * If _vel_sp == vel_zero_acc (direction == 0) while _state.a != 0, we only need Phase 3 (T3) to brake
 * the current acceleration to zero. Since Phase 3 applies jerk = -direction * _max_jerk, setting
 * direction = sign(_state.a) yields T1 = 0, T2 = 0, T3 = |_state.a| / _max_jerk, and brakes _state.a to zero.
 */
int VelocitySmoothing::computeDirection() const
{
	// Compute the velocity at which the trajectory will be
	// when the acceleration will be zero
	float vel_zero_acc = computeVelAtZeroAcc();

	/* Depending of the direction, start accelerating positively or negatively */
	int direction = sign(_vel_sp - vel_zero_acc);

	if (direction == 0) {
		// If by braking immediately the velocity is exactly
		// the required one with zero acceleration, then brake
		direction = sign(_state.a);
	}

	return direction;
}

/**
 * Compute the velocity that would be reached if maximum opposing jerk were applied immediately
 * until the current acceleration is reduced to zero.
 * @return Velocity when acceleration reaches zero.
 *
 * To cancel current acceleration a0 = _state.a as fast as possible, opposing jerk
 * j_zero_acc = -sign(a0) * _max_jerk is applied for duration t_zero_acc = -a0 / j_zero_acc = |a0| / _max_jerk.
 * Integrating acceleration over t_zero_acc gives:
 *   vel_zero_acc = v0 + a0 * t_zero_acc + 0.5 * j_zero_acc * t_zero_acc^2
 */
float VelocitySmoothing::computeVelAtZeroAcc() const
{
	float vel_zero_acc = _state.v;

	if (fabsf(_state.a) > FLT_EPSILON) {
		float j_zero_acc = -sign(_state.a) * _max_jerk; // Required jerk to reduce the acceleration
		float t_zero_acc = -_state.a / j_zero_acc; // Required time to cancel the current acceleration
		vel_zero_acc = _state.v + _state.a * t_zero_acc + 0.5f * j_zero_acc * t_zero_acc * t_zero_acc;
	}

	return vel_zero_acc;
}

/**
 * Compute T1, T2, and T3 to reach _vel_sp in minimum total time.
 *
 * Order of computation:
 *  1. _T1 is solved first (assuming T2 = 0, then clamped if peak acceleration reaches _max_accel).
 *  2. _T3 is computed from _T1 so that acceleration returns from peak acceleration (a0 + j_max * _T1) to 0.
 *  3. _T2 is computed last to cover any remaining velocity change at constant acceleration if _T1 was clamped.
 */
void VelocitySmoothing::updateDurationsMinimizeTotalTime()
{
	float jerk_max_T1 = _direction * _max_jerk;
	float delta_v = _vel_sp - _state.v;

	if (fabsf(jerk_max_T1) > FLT_EPSILON) { // zero direction or jerk would lead to division by zero
		// compute increasing acceleration time
		_T1 = computeT1(_state.a, delta_v, jerk_max_T1, _max_accel);

		// compute decreasing acceleration time
		_T3 = computeT3(_T1, _state.a, jerk_max_T1);

		// compute constant acceleration time
		_T2 = computeT2(_T1, _T3, _state.a, delta_v, jerk_max_T1);

	} else {
		_T1 = _T2 = _T3 = 0.f;
	}
}

/**
 * Evaluate a 3rd-order constant-jerk polynomial trajectory at time t.
 * @param j Jerk magnitude
 * @param a0 Initial acceleration at t = 0
 * @param v0 Initial velocity at t = 0
 * @param x0 Initial position at t = 0
 * @param t Integration time interval
 * @param d Jerk direction (+1, -1, or 0)
 * @return Trajectory state (j, a, v, x) at time t.
 *
 * With constant signed jerk jt = d * j, integrating with respect to t gives:
 *   j(t) = jt
 *   a(t) = a0 + jt * t
 *   v(t) = v0 + a0 * t + 0.5 * jt * t^2
 *   x(t) = x0 + v0 * t + 0.5 * a0 * t^2 + (1/6) * jt * t^3
 */
Trajectory VelocitySmoothing::evaluatePoly(float j, float a0, float v0, float x0, float t, int d) const
{
	Trajectory traj;
	float jt = d * j;
	float t2 = t * t;
	float t3 = t2 * t;

	traj.j = jt;
	traj.a = a0 + jt * t;
	traj.v = v0 + a0 * t + 0.5f * jt * t2;
	traj.x = x0 + v0 * t + 0.5f * a0 * t2 + 1.f / 6.f * jt * t3;

	return traj;
}

/**
 * Advance the trajectory state forward by dt (scaled by time_stretch).
 * @param dt Integration period [s]
 * @param time_stretch Time scaling factor (1.0 for real-time)
 *
 * Since a single timestep dt may span across multiple phases, the elapsed time t_remain
 * is consumed sequentially across up to four segments:
 *  1. Phase 1 (up to _T1): increasing acceleration with jerk = +_direction * _max_jerk
 *  2. Phase 2 (up to _T2): constant acceleration with jerk = 0
 *  3. Phase 3 (up to _T3): decreasing acceleration with jerk = -_direction * _max_jerk
 *  4. Post-T3 (any remaining time after _T1 + _T2 + _T3): constant velocity with jerk = 0 and accel = 0
 */
void VelocitySmoothing::updateTraj(float dt, float time_stretch)
{
	_local_time += dt * time_stretch;
	float t_remain = _local_time;

	float t1 = math::min(t_remain, _T1);
	_state = evaluatePoly(_max_jerk, _state_init.a, _state_init.v, _state_init.x, t1, _direction);
	t_remain -= t1;

	if (t_remain > 0.f) {
		float t2 = math::min(t_remain, _T2);
		_state = evaluatePoly(0.f, _state.a, _state.v, _state.x, t2, 0.f);
		t_remain -= t2;
	}

	if (t_remain > 0.f) {
		float t3 = math::min(t_remain, _T3);
		_state = evaluatePoly(_max_jerk, _state.a, _state.v, _state.x, t3, -_direction);
		t_remain -= t3;
	}

	if (t_remain > 0.f) {
		_state = evaluatePoly(0.f, 0.f, _state.v, _state.x, t_remain, 0.f);
	}
}

/**
 * Synchronize multiple 1D trajectories so that they all finish at the same total time.
 * @param traj Array of VelocitySmoothing objects (e.g., X and Y axes)
 * @param n_traj Number of trajectories in the array
 *
 * Without synchronization, faster axes finish their velocity transition early, causing multi-axis
 * motion to curve instead of following a straight line. This function finds the longest trajectory
 * duration (desired_time = max(T1 + T2 + T3)) and stretches all faster trajectories to match desired_time
 * via updateDurationsGivenTotalTime().
 */
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
			if ((i != longest_traj_index)
			    && (traj[i].getTotalTime() < desired_time)) {
				traj[i].updateDurationsGivenTotalTime(desired_time);
			}
		}
	}
}

/**
 * Recompute T1, T2, and T3 so that the trajectory reaches _vel_sp in a prescribed total time T123.
 * @param T123 Desired total trajectory duration (T1 + T2 + T3) [s]
 *
 * Stretches a faster axis's trajectory to match a longer duration T123 (from timeSynchronization):
 *  1. _T1 is recomputed with the total time constraint T123, resulting in a shorter ramp to a lower peak acceleration.
 *  2. _T3 is computed from _T1 to bring that peak acceleration back to zero.
 *  3. _T2 = T123 - _T1 - _T3 fills the remaining duration at constant acceleration.
 */
void VelocitySmoothing::updateDurationsGivenTotalTime(float T123)
{
	float jerk_max_T1 = _direction * _max_jerk;
	float delta_v = _vel_sp - _state.v;

	if (fabsf(jerk_max_T1) > FLT_EPSILON) { // zero direction or jerk would lead to division by zero
		// compute increasing acceleration time
		_T1 = computeT1(T123, _state.a, delta_v, jerk_max_T1, _max_accel);

		// compute decreasing acceleration time
		_T3 = computeT3(_T1, _state.a, jerk_max_T1);

		// compute constant acceleration time
		_T2 = computeT2(T123, _T1, _T3);

	} else {
		_T1 = _T2 = _T3 = 0.f;
	}
}
