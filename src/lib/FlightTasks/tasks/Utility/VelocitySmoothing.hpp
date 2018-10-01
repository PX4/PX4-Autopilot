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
	 * Update the setpoint and get the smoothed setpoints. This should be called on every cycle.
	 * @param dt delta time between last update() call and now [s]
	 * @param pos Current vehicle's position
	 * @param vel_setpoint velocity setpoint input
	 * @param vel_setpoint_smooth returned smoothed velocity setpoint
	 * @param pos_setpoint_smooth returned smoothed position setpoint
	 */
	void update(float dt, float pos, float vel_setpoint, float &vel_setpoint_smooth, float &pos_setpoint_smooth);


	/* Get / Set constraints (constraints can be updated at any time) */

	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

private:
	/**
	 * Compute increasing acceleration time
	 */
	inline float computeT1(float accel_prev, float vel_prev, float vel_setpoint, float max_jerk);
	/**
	 * Compute constant acceleration time
	 */
	inline float computeT2(float T1, float T3, float accel_prev, float vel_prev, float vel_setpoint, float max_jerk);
	/**
	 * Compute decreasing acceleration time
	 */
	inline float computeT3(float T1, float accel_prev, float max_jerk);

	/**
	 * Integrate the jerk, acceleration and velocity to get the new setpoints and states.
	 */
	inline void integrateT(float jerk, float accel_prev, float vel_prev, float pos_prev, float dt,
			       float &accel_out, float &vel_out, float &pos_out);

	/* Constraints */
	float _max_jerk = 22.f;
	float _max_accel = 8.f;
	float _max_vel = 6.f;

	/* State (previous setpoints) */
	float _jerk;
	float _accel;
	float _vel;
	float _pos;

	static constexpr float max_pos_err = 1.f; ///< maximum position error (if above, the position setpoint is locked)
};
