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

/**
 * @file VelocitySmoothing2D.hpp
 *
 * Two-dimensional, isotropic version of VelocitySmoothing: the same three-phase profile
 * (increasing, constant, decreasing acceleration), but the jerk, acceleration and velocity
 * limits apply to the Euclidean norm instead of to each axis separately.
 *
 * The acceleration moves on a straight line from its initial value a0 to the constant
 * acceleration a1 of the T2 phase and on a straight line from a1 to zero, both at the
 * maximum jerk. The unknown of the planning problem is a1 (two components), compared to
 * the duration T1 (and a sign) in the one-dimensional case:
 *   T1 = |a1 - a0| / j_max,    T3 = |a1| / j_max
 *   delta_v = (a0 + a1) / 2 * T1 + a1 * T2 + a1 / 2 * T3
 * The functions keep the names and the structure of VelocitySmoothing. The one difference
 * is that the quadratic equation for T1 becomes a 2x2 nonlinear system, solved by
 * Newton's method (it converges in a few iterations and is warm-started with the solution
 * of the previous call).
 */

#pragma once

#include <matrix/matrix/math.hpp>

struct Trajectory2D {
	matrix::Vector2f j; //< jerk
	matrix::Vector2f a; //< acceleration
	matrix::Vector2f v; //< velocity
	matrix::Vector2f x; //< position
};

class VelocitySmoothing2D
{
public:
	VelocitySmoothing2D(const matrix::Vector2f &initial_accel = {}, const matrix::Vector2f &initial_vel = {},
			    const matrix::Vector2f &initial_pos = {});
	~VelocitySmoothing2D() = default;

	/**
	 * Reset the state.
	 * @param accel Current acceleration
	 * @param vel Current velocity
	 * @param pos Current position
	 */
	void reset(const matrix::Vector2f &accel, const matrix::Vector2f &vel, const matrix::Vector2f &pos);

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint. This should be
	 * called after setting the constraints and the velocity setpoint. The total time is the
	 * minimum one reachable with the norm constraints.
	 * @param vel_setpoint velocity setpoint input
	 */
	void updateDurations(const matrix::Vector2f &vel_setpoint);

	/**
	 * Recompute T1, T2, T3 such that the velocity setpoint is reached at the given total time
	 * (used to synchronize with a slower trajectory, e.g. the vertical one)
	 * @param T123 desired total time
	 */
	void updateDurationsGivenTotalTime(float T123);

	/**
	 * Generate the trajectory (acceleration, velocity and position) by integrating the current jerk
	 * @param dt integration period
	 * @param time_stretch (optional) used to scale the integration period. This can be used to slow down
	 * the trajectory without changing the computed durations.
	 */
	void updateTraj(float dt, float time_stretch = 1.f);

	/* Input / Output */
	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

	matrix::Vector2f getCurrentJerk() const { return _state.j; }
	void setCurrentAcceleration(const matrix::Vector2f &accel) { _state.a = _state_init.a = accel; }
	matrix::Vector2f getCurrentAcceleration() const { return _state.a; }
	void setCurrentVelocity(const matrix::Vector2f &vel) { _state.v = _state_init.v = vel; }
	matrix::Vector2f getCurrentVelocity() const { return _state.v; }
	void setCurrentPosition(const matrix::Vector2f &pos) { _state.x = _state_init.x = pos; }
	matrix::Vector2f getCurrentPosition() const { return _state.x; }

	float getT1() const { return _T1; }
	float getT2() const { return _T2; }
	float getT3() const { return _T3; }
	float getTotalTime() const { return _T1 + _T2 + _T3; }

private:

	/**
	 * Compute T1, T2, T3 to reach the velocity setpoint in minimum time
	 */
	void updateDurationsMinimizeTotalTime();

	/**
	 * Compute the "jerk-time" vector T1 (its norm is the duration of the first phase, its direction
	 * the jerk direction) for a profile without constant acceleration phase (T2 = 0).
	 */
	inline matrix::Vector2f computeT1(const matrix::Vector2f &a0, const matrix::Vector2f &v3, float j_max,
					  float a_max) const;
	/**
	 * Same as above, for a profile with the given total time T123
	 */
	inline matrix::Vector2f computeT1(float T123, const matrix::Vector2f &a0, const matrix::Vector2f &v3,
					  float j_max, float a_max) const;
	/**
	 * Saturate the acceleration reached at the end of T1 to a_max and recompute T1 if needed
	 */
	inline matrix::Vector2f saturateT1ForAccel(const matrix::Vector2f &a0, float j_max, const matrix::Vector2f &T1,
			float a_max, const matrix::Vector2f &v3) const;
	inline float computeT2(const matrix::Vector2f &T1, const matrix::Vector2f &T3, const matrix::Vector2f &a0,
			       const matrix::Vector2f &v3, float j_max) const;
	inline float computeT2(float T123, float T1, float T3) const;
	inline matrix::Vector2f computeT3(const matrix::Vector2f &T1, const matrix::Vector2f &a0, float j_max) const;

	/**
	 * Solve for the acceleration a1 of the constant phase by Newton's method. With minimize_total_time the
	 * constant phase has zero duration, otherwise it fills the given total time T123.
	 */
	matrix::Vector2f solveHoldAccel(const matrix::Vector2f &a0, const matrix::Vector2f &v3, float j_max, float a_max,
					float T123, bool minimize_total_time, matrix::Vector2f a1) const;

	/**
	 * Set the durations and the jerk directions from the jerk-time vectors
	 */
	void setDurations(const matrix::Vector2f &T1, float T2, const matrix::Vector2f &T3);

	/**
	 * Compute the jerk, acceleration, velocity and position
	 * of a jerk-driven polynomial trajectory at a given time t
	 * @param j jerk
	 * @param a0 initial acceleration at t = 0
	 * @param v0 initial velocity
	 * @param x0 initial position
	 * @param t current time
	 * @param d direction of the jerk (unit vector, or zero for a constant acceleration phase)
	 */
	inline Trajectory2D evaluatePoly(float j, const matrix::Vector2f &a0, const matrix::Vector2f &v0,
					 const matrix::Vector2f &x0, float t, const matrix::Vector2f &d) const;

	/* Input */
	matrix::Vector2f _vel_sp{};

	/* Constraints */
	float _max_jerk{0.f};
	float _max_accel{0.f};
	float _max_vel{0.f};

	/* State (previous setpoints) */
	Trajectory2D _state{};
	matrix::Vector2f _direction{}; ///< Jerk direction of the T1 phase
	matrix::Vector2f _direction_T3{}; ///< Jerk direction of the T3 phase

	/* Initial conditions */
	Trajectory2D _state_init{};

	/* Duration of each phase */
	float _T1{0.f}; ///< Increasing acceleration [s]
	float _T2{0.f}; ///< Constant acceleration [s]
	float _T3{0.f}; ///< Decreasing acceleration [s]

	float _local_time{0.f}; ///< Current local time
};
