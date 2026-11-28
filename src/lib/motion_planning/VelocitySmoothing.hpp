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

#pragma once

struct Trajectory {
	float j; //< jerk
	float a; //< acceleration
	float v; //< velocity
	float x; //< position
};

/**
 * @class VelocitySmoothing
 *
 * Jerk-limited, time-optimal 1D velocity trajectory generator (3-phase S-curve).
 *
 * Drives the current state (x0, v0, a0) to a target velocity setpoint (vel_sp)
 * with zero final acceleration (a = 0), while respecting maximum jerk,
 * acceleration, and velocity constraints.
 *
 * The generated trajectory consists of three phases:
 *  - T1 (increasing acceleration): constant jerk j = direction * max_jerk
 *  - T2 (constant acceleration):   zero jerk j = 0 (used when max_accel is reached
 *                                  or when stretched by timeSynchronization)
 *  - T3 (decreasing acceleration): constant jerk j = -direction * max_jerk
 *                                  until acceleration reaches 0 at v = vel_sp
 *
 *    |T1| T2 |T3|
 *     ___
 *   __| |____   __ Jerk
 *            |_|
 *        ___
 *       /   \	 Acceleration
 *   ___/     \___
 *             ___
 *           ;"
 *          /
 *         / 	 Velocity
 *        ;
 *   ----"
 *
 * Typical usage on every control cycle:
 *  1. Call updateDurations(vel_setpoint) on each axis.
 *  2. (Optional) Call timeSynchronization() to align total trajectory duration across axes.
 *  3. Call updateTraj(dt) to integrate the state forward by dt.
 */
class VelocitySmoothing
{
public:
	VelocitySmoothing(float initial_accel = 0.f, float initial_vel = 0.f, float initial_pos = 0.f);
	~VelocitySmoothing() = default;

	/**
	 * Reset the state.
	 * @param accel Current acceleration
	 * @param vel Current velocity
	 * @param pos Current position
	 */
	void reset(float accel, float vel, float pos);

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint. This should be called on every cycle
	 * and before updateTraj().
	 * @param vel_setpoint velocity setpoint input
	 */
	void updateDurations(float vel_setpoint);

	/**
	 * Generate the trajectory (acceleration, velocity and position) by integrating the current jerk
	 * @param dt integration period
	 * @param time_stretch (optional) used to scale the integration period. This can be used to slow down
	 * or fast-forward the trajectory
	 */
	void updateTraj(float dt, float time_stretch = 1.f);

	/**
	 * Getters and setters
	 */
	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

	float getCurrentJerk() const { return _state.j; }
	void setCurrentAcceleration(const float accel) { _state.a = _state_init.a = accel; }
	float getCurrentAcceleration() const { return _state.a; }
	void setCurrentVelocity(const float vel) { _state.v = _state_init.v = vel; }
	float getCurrentVelocity() const { return _state.v; }
	void setCurrentPosition(const float pos) { _state.x = _state_init.x = pos; }
	float getCurrentPosition() const { return _state.x; }

	float getT1() const { return _T1; }
	float getT2() const { return _T2; }
	float getT3() const { return _T3; }
	float getTotalTime() const { return _T1 + _T2 + _T3; }

	/**
	 * Synchronize several trajectories to have the same total time. This is required to generate
	 * straight lines.
	 * The resulting total time is the one of the longest trajectory.
	 * @param traj an array of VelocitySmoothing objects
	 * @param n_traj the number of trajectories to be synchronized
	 */
	static void timeSynchronization(VelocitySmoothing *traj, int n_traj);

private:

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * Minimize the total time of the trajectory
	 */
	void updateDurationsMinimizeTotalTime();

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * @param T123 desired total time of the trajectory
	 */
	void updateDurationsGivenTotalTime(float T123);

	/**
	 * Compute the direction of the jerk to be applied in order to drive the current state
	 * to the desired one
	 * @return direction (+1 for positive jerk in T1, -1 for negative jerk in T1, 0 for zero)
	 */
	int computeDirection() const;

	/**
	 * Compute the velocity at which the trajectory will be if the maximum jerk is applied
	 * during the time required to cancel the current acceleration
	 * @return velocity when acceleration reaches zero
	 */
	float computeVelAtZeroAcc() const;

	/**
	 * Compute increasing acceleration time
	 * @param a0 initial acceleration
	 * @param v3 desired velocity change (vel_sp - v0)
	 * @param j_max signed maximum jerk applied during T1 (direction * _max_jerk)
	 * @param a_max maximum acceleration limit (_max_accel)
	 * @return duration T1 [s]
	 */
	inline float computeT1(float a0, float v3, float j_max, float a_max) const;

	/**
	 * Compute increasing acceleration time using total time constraint
	 * @param T123 desired total time of the trajectory (T1 + T2 + T3)
	 * @param a0 initial acceleration
	 * @param v3 desired velocity change (vel_sp - v0)
	 * @param j_max signed maximum jerk applied during T1 (direction * _max_jerk)
	 * @param a_max maximum acceleration limit (_max_accel)
	 * @return duration T1 [s]
	 */
	inline float computeT1(float T123, float a0, float v3, float j_max, float a_max) const;

	/**
	 * Saturate T1 in order to respect the maximum acceleration constraint
	 * @param a0 initial acceleration
	 * @param j_max signed maximum jerk applied during T1 (direction * _max_jerk)
	 * @param T1 unconstrained T1 duration
	 * @param a_max maximum acceleration limit (_max_accel)
	 * @return saturated T1 duration [s]
	 */
	inline float saturateT1ForAccel(float a0, float j_max, float T1, float a_max) const;

	/**
	 * Compute constant acceleration time
	 * @param T1 increasing acceleration duration
	 * @param T3 decreasing acceleration duration
	 * @param a0 initial acceleration
	 * @param v3 desired velocity change (vel_sp - v0)
	 * @param j_max signed maximum jerk applied during T1 (direction * _max_jerk)
	 * @return duration T2 [s]
	 */
	inline float computeT2(float T1, float T3, float a0, float v3, float j_max) const;

	/**
	 * Compute constant acceleration time using total time constraint
	 * @param T123 desired total time of the trajectory (T1 + T2 + T3)
	 * @param T1 increasing acceleration duration
	 * @param T3 decreasing acceleration duration
	 * @return duration T2 [s]
	 */
	inline float computeT2(float T123, float T1, float T3) const;

	/**
	 * Compute decreasing acceleration time
	 * @param T1 increasing acceleration duration
	 * @param a0 initial acceleration
	 * @param j_max signed maximum jerk applied during T1 (direction * _max_jerk)
	 * @return duration T3 [s]
	 */
	inline float computeT3(float T1, float a0, float j_max) const;

	/**
	 * Compute the jerk, acceleration, velocity and position
	 * of a jerk-driven polynomial trajectory at a given time t
	 * @param j jerk
	 * @param a0 initial acceleration at t = 0
	 * @param v0 initial velocity
	 * @param x0 initial postion
	 * @param t current time
	 * @param d direction
	 */
	inline Trajectory evaluatePoly(float j, float a0, float v0, float x0, float t, int d) const;

	/* Input */
	float _vel_sp{0.f};

	/* Constraints */
	float _max_jerk{0.f};
	float _max_accel{0.f};
	float _max_vel{0.f};

	/* State (previous setpoints) */
	Trajectory _state{};
	int _direction{0};

	/* Initial conditions */
	Trajectory _state_init{};

	/* Duration of each phase */
	float _T1{0.f}; ///< Increasing acceleration [s]
	float _T2{0.f}; ///< Constant acceleration [s]
	float _T3{0.f}; ///< Decreasing acceleration [s]

	float _local_time{0.f}; ///< Current local time
};
