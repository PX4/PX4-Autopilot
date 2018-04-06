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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <parameters/param.h>

#pragma once

namespace Controller
{
/** Constraints that depends on mode and are lower
 * 	than the global limits.
 * 	tilt_max: Cannot exceed PI/2 radians
 * 	vel_max_z_up: Cannot exceed maximum global velocity upwards
 * @see MPC_TILTMAX_AIR
 * @see MPC_Z_VEL_MAX_DN
 */
struct Constraints {
	float tilt_max; /**< maximum tilt always below Pi/2 */
	float vel_max_z_up; /**< maximum speed upwards always smaller than MPC_VEL_Z_MAX_UP */
};
}
/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 */
class PositionControl
{
public:

	PositionControl();
	~PositionControl() {};

	/**
	 * Update the current vehicle state.
	 * @param state a vehicle_local_position_s structure
	 * @param vel_dot the derivative of the vehicle velocity
	 */
	void updateState(const struct vehicle_local_position_s state, const matrix::Vector3f &vel_dot);

	/**
	 * Update the desired setpoints.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void updateSetpoint(struct vehicle_local_position_setpoint_s setpoint);

	/**
	 * Set constraints that are stricter than the global limits.
	 * @param constraints a PositionControl structure with supported constraints
	 */
	void updateConstraints(const Controller::Constraints &constraints);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt the delta-time
	 */
	void generateThrustYawSetpoint(const float &dt);

	/**
	 * 	Set the integral term in xy to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralXY() {_thr_int(0) = _thr_int(1) = 0.0f;};

	/**
	 * 	Set the integral term in z to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralZ() {_thr_int(2) = 0.0f;};

	/**
	 * 	Get the
	 * 	@see _thr_sp
	 * 	@return The thrust set-point member.
	 */
	matrix::Vector3f getThrustSetpoint() {return _thr_sp;}

	/**
	 * 	Get the
	 * 	@see _yaw_sp
	 * 	@return The yaw set-point member.
	 */
	float getYawSetpoint() { return _yaw_sp;}

	/**
	 * 	Get the
	 * 	@see _yawspeed_sp
	 * 	@return The yawspeed set-point member.
	 */
	float getYawspeedSetpoint() {return _yawspeed_sp;}

	/**
	 * 	Get the
	 * 	@see _vel_sp
	 * 	@return The velocity set-point member.
	 */
	matrix::Vector3f getVelSp() {return _vel_sp;}

	/**
	 * 	Get the
	 * 	@see _pos_sp
	 * 	@return The position set-point member.
	 */
	matrix::Vector3f getPosSp() {return _pos_sp;}

private:
	void _interfaceMapping(); /** maps set-points to internal member set-points */
	void _positionController(); /** applies the P-position-controller */
	void _velocityController(const float &dt); /** applies the PID-velocity-controller */
	void _updateParams(); /** updates parameters */
	void _setParams(); /** sets parameters to internal member */

	matrix::Vector3f _pos{}; /**< MC position */
	matrix::Vector3f _vel{}; /**< MC velocity */
	matrix::Vector3f _vel_dot{}; /**< MC velocity derivative */
	matrix::Vector3f _acc{}; /**< MC acceleration */
	float _yaw{0.0f}; /**< MC yaw */
	matrix::Vector3f _pos_sp{}; /**< desired position */
	matrix::Vector3f _vel_sp{}; /**< desired velocity */
	matrix::Vector3f _acc_sp{}; /**< desired acceleration: not supported yet */
	matrix::Vector3f _thr_sp{}; /**< desired thrust */
	float _yaw_sp{}; /**< desired yaw */
	float _yawspeed_sp{}; /** desired yaw-speed */
	matrix::Vector3f _thr_int{}; /**< thrust integral term */
	Controller::Constraints _constraints{}; /**< variable constraints */
	bool _skip_controller{false}; /**< skips position/velocity controller. true for stabilized mode */

	/**
	 * Position Gains.
	 * Pp: P-controller gain for position-controller
	 * Pv: P-controller gain for velocity-controller
	 * Iv: I-controller gain for velocity-controller
	 * Dv: D-controller gain for velocity-controller
	 */
	matrix::Vector3f _Pp, _Pv, _Iv, _Dv = matrix::Vector3f{0.0f, 0.0f, 0.0f};

	float MPC_THR_MAX{1.0f}; /**< maximum thrust */
	float MPC_THR_HOVER{0.5f}; /** equilibrium point for the velocity controller */
	float MPC_THR_MIN{0.0f}; /**< minimum throttle for any position controlled mode */
	float MPC_MANTHR_MIN{0.0f}; /**< minimum throttle for stabilized mode */
	float MPC_XY_VEL_MAX{1.0f}; /**< maximum speed in the horizontal direction */
	float MPC_Z_VEL_MAX_DN{1.0f}; /**< maximum speed in downwards direction */
	float MPC_Z_VEL_MAX_UP{1.0f}; /**< maximum speed in upwards direction */
	float MPC_TILTMAX_AIR{1.5}; /**< maximum tilt for any position/velocity controlled mode in radians */
	float MPC_MAN_TILT_MAX{3.1}; /**< maximum tilt for manual/altitude mode in radians */

	// Parameter handles
	int _parameter_sub { -1 };
	param_t MPC_Z_P_h { PARAM_INVALID };
	param_t MPC_Z_VEL_P_h { PARAM_INVALID };
	param_t MPC_Z_VEL_I_h { PARAM_INVALID };
	param_t MPC_Z_VEL_D_h { PARAM_INVALID };
	param_t MPC_XY_P_h { PARAM_INVALID };
	param_t MPC_XY_VEL_P_h { PARAM_INVALID };
	param_t MPC_XY_VEL_I_h { PARAM_INVALID };
	param_t MPC_XY_VEL_D_h { PARAM_INVALID };
	param_t MPC_XY_VEL_MAX_h { PARAM_INVALID };
	param_t MPC_Z_VEL_MAX_DN_h { PARAM_INVALID };
	param_t MPC_Z_VEL_MAX_UP_h { PARAM_INVALID };
	param_t MPC_THR_HOVER_h { PARAM_INVALID };
	param_t MPC_THR_MAX_h { PARAM_INVALID };
	param_t MPC_THR_MIN_h { PARAM_INVALID };
	param_t MPC_MANTHR_MIN_h { PARAM_INVALID };
	param_t MPC_TILTMAX_AIR_h { PARAM_INVALID };
	param_t MPC_MAN_TILT_MAX_h { PARAM_INVALID };
};
