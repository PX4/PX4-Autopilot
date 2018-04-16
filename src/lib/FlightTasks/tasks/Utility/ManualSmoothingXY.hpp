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
 * This Class is used for smoothing the velocity setpoints in Z-direction.
 */

#pragma once

#include <px4_module_params.h>
#include <matrix/matrix/math.hpp>

class ManualSmoothingXY : public ModuleParams
{
public:
	ManualSmoothingXY(ModuleParams *parent, const matrix::Vector2f &vel);
	~ManualSmoothingXY() = default;

	/**
	 * Smoothing of velocity setpoint horizontally based
	 * on flight direction.
	 * @param vel_sp: velocity setpoint in xy
	 * @param dt: time delta in seconds
	 */
	void smoothVelocity(matrix::Vector2f &vel_sp, const matrix::Vector2f &vel,  const float &yaw,
			    const float &yawrate_sp, const float dt);

	/* User intention: brake or acceleration */
	enum class Intention {
		brake,
		acceleration,
		deceleration,
		direction_change
	};

	/* Getter methods */
	Intention getIntention() { return _intention; }

	/* Overwrite methods:
	 * Needed if different parameter values than default required.
	 */
	void overwriteHoverAcceleration(float acc) { _acc_hover.set(acc); }
	void overwriteMaxAcceleration(float acc) { _acc_xy_max.set(acc); }
	void overwriteDecelerationMin(float dec) { _dec_xy_min.set(dec); }
	void overwriteJerkMax(float jerk) { _jerk_max.set(jerk); }
	void overwriteJerkMin(float jerk) { _jerk_min.set(jerk); }

private:
	void _updateAcceleration(matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const float &yaw,
				 const float &yawrate_sp, const float dt);
	Intention _getIntention(const matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const float &yaw,
				const float &yawrate_sp);
	void _getStateAcceleration(const matrix::Vector2f &vel_sp, const matrix::Vector2f &vel, const Intention &intention,
				   const float dt);
	void _velocitySlewRate(matrix::Vector2f &vel_sp, const float dt);
	matrix::Vector2f _getWorldToHeadingFrame(const matrix::Vector2f &vec, const float &yaw);
	matrix::Vector2f _getHeadingToWorldFrame(const matrix::Vector2f &vec, const float &yaw);

	/* User intention: brake or acceleration */
	Intention _intention{Intention::acceleration};

	/* Acceleration that depends on vehicle state
	 * _acc_max_down <= _acc_state_dependent <= _acc_max_up
	 */
	float _acc_state_dependent{0.0f};
	float _jerk_state_dependent{0.0f};

	/* Previous setpoints */
	matrix::Vector2f _vel_sp_prev{}; // previous velocity setpoint

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _acc_hover, ///< acceleration in hover
		(ParamFloat<px4::params::MPC_ACC_HOR>) _acc_xy_max, ///< acceleration in flight
		(ParamFloat<px4::params::MPC_DEC_HOR_SLOW>) _dec_xy_min, ///< deceleration in flight
		(ParamFloat<px4::params::MPC_JERK_MIN>) _jerk_min, ///< jerk min during brake
		(ParamFloat<px4::params::MPC_JERK_MAX>) _jerk_max, ///< jerk max during brake
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _vel_manual ///< maximum velocity in manual controlled mode
	)

};
