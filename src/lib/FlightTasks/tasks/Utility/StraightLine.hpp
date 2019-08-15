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

/**
 * @file StraightLine.hpp
 *
 * lib to return setpoints on a straight line
 *
 * @author Christoph Tobler <christoph@px4.io>
 */

#pragma once

#include <px4_module_params.h>
#include <matrix/matrix/math.hpp>

class StraightLine : public ModuleParams
{
public:
	StraightLine(ModuleParams *parent, const float &deltatime, const matrix::Vector3f &pos);
	~StraightLine() = default;

	// setter functions
	void setLineFromTo(const matrix::Vector3f &origin, const matrix::Vector3f &target);
	void setSpeed(const float &speed);
	void setSpeedAtTarget(const float &speed_at_target);
	void setAcceleration(const float &acc);
	void setDeceleration(const float &dec);

	/**
	 * Set all parameters to their default value depending on the direction of the line
	 */
	void setAllDefaults();

	/**
	 * Get the maximum possible acceleration depending on the direction of the line
	 * Respects horizontal and vertical limits
	 */
	float getMaxAcc();

	/**
	 * Get the maximum possible velocity depending on the direction of the line
	 * Respects horizontal and vertical limits
	 */
	float getMaxVel();

	/**
	 * Generate setpoints on a straight line according to parameters
	 *
	 * @param position_setpoint: 3D vector with the previous position setpoint
	 * @param velocity_setpoint: 3D vector with the previous velocity setpoint
	 */
	void generateSetpoints(matrix::Vector3f &position_setpoint, matrix::Vector3f &velocity_setpoint);

private:
	const float &_deltatime;                 /**< delta time between last update (dependency injection) */
	const matrix::Vector3f &_pos;            /**< vehicle position (dependency injection) */

	float _desired_acceleration{0.0f};       /**< acceleration along the straight line */
	float _desired_deceleration{0.0f};       /**< deceleration along the straight line */
	float _desired_speed{0.0f};              /**< desired maximum velocity */
	float _desired_speed_at_target{0.0f};    /**< desired velocity at target point */

	matrix::Vector3f _target{};              /**< End point of the straight line */
	matrix::Vector3f _origin{};              /**< Start point of the straight line */

	// parameters for default values
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,   /**< maximum horizontal acceleration */
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,    /**< maximum vertical acceleration upwards */
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,  /**< maximum vertical acceleration downwards*/
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,    /**< maximum horizontal velocity */
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,  /**< maximum vertical velocity upwards */
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,  /**< maximum vertical velocity downwards */
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad             /**< acceptance radius if a waypoint is reached */
	)

};
