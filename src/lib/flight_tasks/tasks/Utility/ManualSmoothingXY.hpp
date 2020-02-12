/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file SmoothingXY.hpp
 *
 * This Class is used for smoothing the velocity setpoint in XY-direction.
 * The velocity setpoint is smoothed by applying a velocity change limit, which
 * we call acceleration. Depending on the user intention, the acceleration limit
 * will differ.
 * In manual mode we consider four states with different acceleration handling:
 * 1. user wants to stop
 * 2. user wants to quickly change direction
 * 3. user wants to accelerate
 * 4. user wants to decelerate
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

class ManualSmoothingXY : public ModuleParams
{
public:
	ManualSmoothingXY(ModuleParams *parent, const matrix::Vector2f &vel);
	~ManualSmoothingXY() = default;

	/**
	 * Maximum velocity is required to detect user intention.
	 * Maximum velocity depends on flight-task.
	 * In order to deduce user intention from velocity, the maximum
	 * allowed velocity has to be updated.
	 * @param vel_max corresponds to vehicle constraint
	 */
	void updateMaxVelocity(const float &vel_max) {_vel_max = vel_max;};

	/**
	 * Smoothing of velocity setpoint horizontally based
	 * on flight direction.
	 * @param vel_sp: velocity setpoint in xy
	 * @param dt: time delta in seconds
	 */
	void smoothVelocity(matrix::Vector2f &vel_sp, const matrix::Vector2f &vel,  const float &yaw,
			    const float &yawrate_sp, const float dt);

	/*
	 * User intention.
	 * - brake when user demands a brake
	 * - acceleration when vehicle keeps speed or accelerates in the same direction
	 * - deceleration when vehicle slows down in the same direction
	 * - direction_change whne vehcile demands an abrupt direction change
	 */
	enum class Intention {
		brake,
		acceleration,
		deceleration,
		direction_change
	};
	/**
	 * Get user intention.
	 * @see Intention
	 */
	Intention getIntention() { return _intention; }

	/**
	 *  Overwrite methods:
	 *  Needed if different parameter values than default required.
	 */
	void overwriteHoverAcceleration(float acc) { _param_mpc_acc_hor_max.set(acc); }
	void overwriteMaxAcceleration(float acc) { _param_mpc_acc_hor.set(acc); }
	void overwriteDecelerationMin(float dec) { _param_mpc_dec_hor_slow.set(dec); }
	void overwriteJerkMax(float jerk) { _param_mpc_jerk_max.set(jerk); }
	void overwriteJerkMin(float jerk) { _param_mpc_jerk_min.set(jerk); }

private:
	/**
	 * Sets velocity change limits (=acceleration).
	 * Depending on the user intention, the acceleration differs.
	 * @param vel_sp is desired velocity setpoint before slewrate.
	 * @param vel is current velocity in horizontal direction
	 * @param yaw is vehicle yaw
	 * @param yawrate_sp is desired yawspeed
	 * @param dt is delta-time
	 */
	void _updateAcceleration(matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const float &yaw,
				 const float &yawrate_sp, const float dt);

	/**
	 * Gets user intention.
	 * The intention is deduced from desired velocity setpoint.
	 * @param vel_sp is desired velocity setpoint before slewrate.
	 * @param vel is vehicle velocity in xy-direction.
	 * @param yaw is vehicle yaw
	 * @param yawrate_sp is the desired yaw-speed
	 */
	Intention _getIntention(const matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const float &yaw,
				const float &yawrate_sp);

	/**
	 * Set acceleration depending on Intention.
	 * @param vel_sp is desired velocity septoint in xy-direction
	 * @param vel is vehicle velociy in xy-direction
	 * @param intention is the user intention during flight
	 * @param dt is delta-time
	 */
	void _setStateAcceleration(const matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const Intention &intention,
				   const float dt);

	/**
	 * Limits the velocity setpoint change.
	 * @param vel_sp that gets limited based on acceleration and previous velocity setpoint
	 * @param dt is delta-time
	 */
	void _velocitySlewRate(matrix::Vector2f &vel_sp, const float dt);

	/**
	 * Rotate vector from local frame into heading frame.
	 * @param vec is an arbitrary vector in local frame
	 * @param yaw is the vehicle heading
	 */
	matrix::Vector2f _getWorldToHeadingFrame(const matrix::Vector2f &vec, const float &yaw);

	/**
	 * Rotate vector from heading frame to local frame.
	 * @param vec is an arbitrary vector in heading frame
	 * @param yaw is the vehicle yaw
	 */
	matrix::Vector2f _getHeadingToWorldFrame(const matrix::Vector2f &vec, const float &yaw);

	Intention _intention{Intention::acceleration}; /**< user intention */
	float _acc_state_dependent; /**< velocity change limit that depends on Intention */
	float _jerk_state_dependent; /**< acceleration change limit during brake */
	float _vel_max{10.0f}; /**< maximum horizontal speed allowed */
	matrix::Vector2f _vel_sp_prev; /**< previous velocity setpoint */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max, /**< acceleration in hover */
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, /**< acceleration in flight */
		(ParamFloat<px4::params::MPC_DEC_HOR_SLOW>) _param_mpc_dec_hor_slow, /**< deceleration in flight */
		(ParamFloat<px4::params::MPC_JERK_MIN>) _param_mpc_jerk_min, /**< jerk min during brake */
		(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max, /**< jerk max during brake */
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual /**< maximum velocity in manual controlled mode */
	)
};
