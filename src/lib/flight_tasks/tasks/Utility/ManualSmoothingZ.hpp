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
 * @file SmoothingZ.hpp
 *
 * This Class is used for smoothing the velocity setpoints in Z-direction.
 * In manual altitude control apply acceleration limit based on stick input.
 */

#pragma once

#include <px4_platform_common/module_params.h>

/**
 * User-intention.
 * - brake: vehicle should stop
 * - acceleration: vehicle should move up or down
 */
enum class ManualIntentionZ {
	brake,
	acceleration,
};

class ManualSmoothingZ : public ModuleParams
{
public:
	ManualSmoothingZ(ModuleParams *parent, const float &vel, const float &stick);
	~ManualSmoothingZ() = default;

	/**
	 * Smooth velocity setpoint based on flight direction.
	 * @param vel_sp[2] array: vel_sp[0] = current velocity set-point;
	 * 					 	   vel_sp[1] = previous velocity set-point
	 * 		  vel_sp will contain smoothed current / previous set-point.
	 * @param dt: time delta in seconds
	 */
	void smoothVelFromSticks(float &vel_sp, const float dt);

	/**
	 * Get max accleration.
	 */
	float getMaxAcceleration() { return _max_acceleration; }

	/**
	 * Get user intention.
	 * @see ManualIntentionZ
	 */
	ManualIntentionZ getIntention() { return _intention; }

	/*
	 * Overwrite methods:
	 * Needed if different parameter values than default required.
	 */
	void overwriteAccelerationUp(float acc_max_up) { _param_mpc_acc_up_max.set(acc_max_up); }
	void overwriteAccelerationDown(float acc_max_down)  {_param_mpc_acc_down_max.set(acc_max_down); }
	void overwriteJerkMax(float jerk_max)  {_param_mpc_jerk_max.set(jerk_max); }

private:
	/**
	 * Add delay to velocity setpoint change.
	 * This method is used to smooth the velocity setpoint change.
	 * @param vel_sp current velocity setpoint
	 * @param dt delta-time
	 */
	void velocitySlewRate(float &vel_sp, const float dt);

	/**
	 * Computes the velocity setpoint change limit.
	 * This method computes the limit with which the velocity setpoint change
	 * is limited.
	 * @see velocitySlewRate
	 * @param vel_sp current velocity setpoint
	 * @param dt delta-time
	 */
	void updateAcceleration(float &vel_sp, const float dt);

	/**
	 * Set maximum acceleration.
	 * The maximum acceleration depends on the desired direction (up vs down).
	 * @see _max_acceleration
	 */
	void setMaxAcceleration();

	/**
	 * User intention
	 * @see ManualIntentionZ
	 */
	ManualIntentionZ _intention{ManualIntentionZ::acceleration};

	const float &_vel; /**< vehicle velocity (dependency injection) */
	const float &_stick; /**< stick input (dependency injection) */

	float _acc_state_dependent; /**< acceleration that depends on _intention */
	float _max_acceleration; /**< can be up or down maximum acceleration */
	float _vel_sp_prev; /**< previous velocity setpoint */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max
	)
};
