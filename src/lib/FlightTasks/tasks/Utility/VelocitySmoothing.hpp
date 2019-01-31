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

#pragma once

#include <matrix/matrix/math.hpp>

/**
 * @class VelocitySmoothing
 *
 * TODO: document the algorithm
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
	 * and before integrate().
	 * @param dt delta time between last updateDurations() call and now [s]
	 * @param vel_setpoint velocity setpoint input
	 */
	void updateDurations(float dt, float vel_setpoint);

	/**
	 * Generate the trajectory (acceleration, velocity and position) by integrating the current jerk
	 * @param dt optional integration period. If not given, the integration period provided during updateDuration call is used.
	 * 	A dt different from the one given during the computation of T1-T3 can be used to fast-forward or slow-down the trajectory.
	 * @param acc_setpoint_smooth returned smoothed acceleration setpoint
	 * @param vel_setpoint_smooth returned smoothed velocity setpoint
	 * @param pos_setpoint_smooth returned smoothed position setpoint
	 */
	void integrate(float &accel_setpoint_smooth, float &vel_setpoint_smooth, float &pos_setpoint_smooth);
	void integrate(float dt, float integration_scale_factor, float &accel_setpoint_smooth, float &vel_setpoint_smooth,
		       float &pos_setpoint_smooth);

	/* Get / Set constraints (constraints can be updated at any time) */
	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

	float getCurrentJerk() const { return _jerk; }
	void setCurrentAcceleration(const float accel) { _accel = accel; }
	float getCurrentAcceleration() const { return _accel; }
	void setCurrentVelocity(const float vel) { _vel = vel; }
	float getCurrentVelocity() const { return _vel; }
	void setCurrentPosition(const float pos) { _pos = pos; }
	float getCurrentPosition() const { return _pos; }

	/**
	 * Synchronize several trajectories to have the same total time. This is required to generate
	 * straight lines.
	 * The resulting total time is the one of the longest trajectory.
	 * @param traj an array of VelocitySmoothing objects
	 * @param n_traj the number of trajectories to be synchronized
	 */
	static void timeSynchronization(VelocitySmoothing *traj, int n_traj);

	float getTotalTime() const { return _T1 + _T2 + _T3; }
	float getVelSp() const { return _vel_sp; }

private:

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * @param T123 optional parameter. If set, the total trajectory time will be T123, if not,
	 * 		the algorithm optimizes for time.
	 */
	void updateDurations(float T123 = NAN);
	/**
	 * Compute increasing acceleration time
	 */
	inline float computeT1(float accel_prev, float vel_prev, float vel_setpoint, float max_jerk);
	/**
	 * Compute increasing acceleration time using total time constraint
	 */
	inline float computeT1(float T123, float accel_prev, float vel_prev, float vel_setpoint, float max_jerk);
	inline float saturateT1ForAccel(float accel_prev, float max_jerk, float T1);
	/**
	 * Compute constant acceleration time
	 */
	inline float computeT2(float T1, float T3, float accel_prev, float vel_prev, float vel_setpoint, float max_jerk);
	/**
	 * Compute constant acceleration time using total time constraint
	 */
	inline float computeT2(float T123, float T1, float T3);
	/**
	 * Compute decreasing acceleration time
	 */
	inline float computeT3(float T1, float accel_prev, float max_jerk);

	/**
	 * Integrate the jerk, acceleration and velocity to get the new setpoints and states.
	 */
	inline void integrateT(float dt, float jerk, float accel_prev, float vel_prev, float pos_prev,
			       float &accel_out, float &vel_out, float &pos_out);

	/* Inputs */
	float _vel_sp;
	float _dt = 1.f;

	/* Constraints */
	float _max_jerk = 22.f;
	float _max_accel = 8.f;
	float _max_vel = 6.f;

	/* State (previous setpoints) */
	float _jerk = 0.f;
	float _accel = 0.f;
	float _vel = 0.f;
	float _pos = 0.f;

	float _max_jerk_T1 = 0.f; ///< jerk during phase T1 (with correct sign)

	/* Duration of each phase */
	float _T1 = 0.f; ///< Increasing acceleration [s]
	float _T2 = 0.f; ///< Constant acceleration [s]
	float _T3 = 0.f; ///< Decreasing acceleration [s]

	static constexpr float max_pos_err = 1.f; ///< maximum position error (if above, the position setpoint is locked)
};
