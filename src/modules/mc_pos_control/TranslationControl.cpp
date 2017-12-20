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
 * @file TranslationControl.cpp
 *
 */

#include "TranslationControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "uORB/topics/parameter_update.h"
#include "Utility/ControlMath.hpp"
#include <lib/ecl/geo/geo.h>  //TODO: only used for wrap_pi -> move this to mathlib since
// it makes more sense

using namespace matrix;

using Data = matrix::Vector3f;

TranslationControl::TranslationControl()
{
	_Pz_h   = param_find("MPC_Z_P");
	_Pvz_h  = param_find("MPC_Z_VEL_P");
	_Ivz_h  = param_find("MPC_Z_VEL_I");
	_Dvz_h  = param_find("MPC_Z_VEL_D");
	_Pxy_h  = param_find("MPC_XY_P");
	_Pvxy_h = param_find("MPC_XY_VEL_P");
	_Ivxy_h = param_find("MPC_XY_VEL_I");
	_Dvxy_h = param_find("MPC_XY_VEL_D");
	_VelMaxXY = param_find("MPC_XY_VEL_MAX");
	_VelMaxZdown_h = param_find("MPC_Z_VEL_MAX_DN");
	_VelMaxZup_h = param_find("MPC_Z_VEL_MAX_UP");
	_ThrHover_h = param_find("MPC_THR_HOVER");
	_ThrMax_h = param_find("MPC_THR_MAX");
	_ThrMin_h = param_find("MPC_THR_MIN");
	_YawRateMax_h = param_find("MPC_MAN_Y_MAX");
	_Pyaw_h = param_find("MC_YAW_P");

	/* Set parameter the very first time */
	_setParams();
};

void TranslationControl::updateState(const struct vehicle_local_position_s state, const matrix::Vector3f vel_dot,
				     const matrix::Matrix<float, 3, 3> R)
{
	_pos = Data(&state.x);
	_vel = Data(&state.vx);
	_yaw = state.yaw;
	_vel_dot = vel_dot;
	_R = R;
}

void TranslationControl::updateSetpoint(struct vehicle_local_position_setpoint_s setpoint)
{
	_pos_sp = Data(&setpoint.x);
	_vel_sp = Data(&setpoint.vx);
	_acc_sp = Data(&setpoint.acc_x);
	_yaw_sp = setpoint.yaw; //integrate
	_yawspeed_sp = setpoint.yawspeed;
	_interfaceMapping();
}

void TranslationControl::generateThrustYawSetpoint(const float &dt)
{
	_updateParams();

	_positionController();

	_velocityController(dt);

	_yawController(dt);

}

void TranslationControl::_interfaceMapping()
{
	/* loop through x,y and z component */
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {

			/* Check if velocity should be used as feedforward */
			if (!PX4_ISFINITE(_vel_sp(i))) {
				_vel_sp(i) = 0.0f;
			}

		} else {

			/* Position controller is not active
			 * -> Set setpoint just to current position since */
			_pos_sp(i) = _pos(i);

			/* check if velocity setpoint should be used */
			if (!PX4_ISFINITE(_vel_sp(i)))  {
				/* No position/velocity controller active.
				 * Attitude will be generated from sticks directly
				 */
				_vel_sp(i) = _vel(i);
			}
		}
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		_yawspeed_sp = 0.0f;

		if (!PX4_ISFINITE(_yaw_sp)) {
			_yaw_sp = _yaw_sp_int;
		}

	} else if (!PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = _yaw_sp_int;
	}
}

/* generate desired velocity */
void TranslationControl::_positionController()
{
	_vel_sp = (_pos_sp - _pos).emult(Pp) + _vel_sp;

	/* make sure velocity setpoint is constrained in all directions (xyz) */
	float vel_norm_xy = sqrtf(
				    _vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));

	if (vel_norm_xy > _VelMaxXY) {
		_vel_sp(0) = _vel_sp(0) * _VelMaxXY / vel_norm_xy;
		_vel_sp(1) = _vel_sp(1) * _VelMaxXY / vel_norm_xy;
	}

	_vel_sp(2) = math::constrain(_vel_sp(2), -_VelMaxZ[0], _VelMaxZ[1]);
}

/* generates desired thrust vector */
void TranslationControl::_velocityController(const float &dt)
{
	/* PID
	 * u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	 * Umin <= u_des <= Umax
	 *
	 * Saturation for vel_integral;
	 * u_des = _thr_sp; r = _vel_sp; y = _vel
	 * u_des >= Umax and r - y >= 0 => Saturation = true
	 * u_des >= Umax and r - y <= 0 => Saturation = false
	 * u_des <= Umin and r - y <= 0 => Saturation = true
	 * u_des <= Umin and r - y >= 0 => Saturation = false
	 *
	 */

	Data vel_err = _vel_sp - _vel;

	/* TODO: add offboard acceleration mode */
	matrix::Vector3f offset(0.0f, 0.0f, _ThrHover);
	_thr_sp = Pv.emult(vel_err) + Dv.emult(_vel_dot) + _thr_int - offset;

	/* Limit tilt with priority on z
	 * For manual controlled mode excluding pure manual and rate control, maximum tilt is 90;
	 * It is to note that pure manual and rate control will never enter _velocityController method*/
	float tilt_max = PX4_ISFINITE(_constraints.tilt_max) ? _constraints.tilt_max : M_PI_2_F;
	tilt_max = math::min(tilt_max, M_PI_2_F);
	_thr_sp = PosControl::constrainTilt(_thr_sp, tilt_max);

	/*TODO: Check if it is beneficial to project thrust onto body z axis  */

	/* Calculate desired total thrust amount in body z direction. */
	/* To compensate for excess thrust during attitude tracking errors we
	 * project the desired thrust force vector F onto the real vehicle's thrust axis in NED:
	 * body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z */
//	matrix::Vector3f R_z(_R(0, 2), _R(1, 2), _R(2, 2));
//	_throttle = thr_sp_tilt.dot(-R_z);
//
//	/* Re-scale thrust set-point based on throttle*/
//	if (thr_sp_tilt.length() < 0.0001f) {
//		_thr_sp = matrix::Vector3f(0.0f, 0.0f,  0.0f);
//
//	} else {
//		_thr_sp = thr_sp_tilt.normalized() * _throttle;
//	}

	/* Constrain thrust set-point and update saturation flag */
	/* To get (r-y) for horizontal direction, we look at the dot-product
	 * for vel_err and _vel_sp. The sign of the dot product indicates
	 * if (r-y) is greater or smaller than 0
	 */
	float dot_xy = matrix::Vector2f(&vel_err(0)) * matrix::Vector2f(&_vel_sp(0));
	float direction[2] = {dot_xy, vel_err(2)};
	bool saturate[2] = {false, false};
	PosControl::constrainPIDu(_thr_sp, saturate, _ThrLimit, direction);

	/* throttle is just thrust length */
	_throttle = _thr_sp.length();

	/* update integrals */
	if (!saturate[0]) {
		_thr_int(0) += vel_err(0) * Iv(0) * dt;
		_thr_int(1) += vel_err(1) * Iv(1) * dt;
	}

	if (!saturate[1]) {
		_thr_int(2) += vel_err(3) * Iv(2) * dt;
	}

}

void TranslationControl::_yawController(const float &dt)
{
	const float yaw_offset_max = math::radians(_YawRateMax) / _Pyaw;
	const float  yaw_target = _wrap_pi(_yaw_sp + _yawspeed_sp * dt);
	const float yaw_offset = _wrap_pi(yaw_target - _yaw);

	// If the yaw offset became too big for the system to track stop
	// shifting it, only allow if it would make the offset smaller again.
	if (fabsf(yaw_offset) < yaw_offset_max || (_yawspeed_sp > 0 && yaw_offset < 0)
	    || (_yawspeed_sp < 0 && yaw_offset > 0)) {
		_yaw_sp = yaw_target;
	}

	/* update yaw setpoint integral */
	_yaw_sp_int = _yaw_sp;

}

void TranslationControl::updateConstraints(const Controller::Constraints &constraints)
{
	_constraints = constraints;
}

void TranslationControl::_updateParams()
{
	bool updated;
	parameter_update_s param_update;
	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &param_update);
		_setParams();
	}
}

void TranslationControl::_setParams()
{
	param_get(_Pxy_h, &Pp(0));
	param_get(_Pxy_h, &Pp(1));
	param_get(_Pz_h, &Pp(2));

	param_get(_Pvxy_h, &Pv(0));
	param_get(_Pvxy_h, &Pv(1));
	param_get(_Pvz_h, &Pv(2));

	param_get(_Ivxy_h, &Iv(0));
	param_get(_Ivxy_h, &Iv(1));
	param_get(_Ivz_h, &Iv(2));

	param_get(_Dvxy_h, &Dv(0));
	param_get(_Dvxy_h, &Dv(1));
	param_get(_Dvz_h, &Dv(2));

	param_get(_VelMaxXY_h, &_VelMaxXY);
	param_get(_VelMaxZup_h, &_VelMaxZ[0]);
	param_get(_VelMaxZdown_h, &_VelMaxZ[1]);

	param_get(_ThrHover_h, &_ThrHover);
	param_get(_ThrMax_h, &_ThrLimit[0]);
	param_get(_ThrMin_h, &_ThrLimit[1]);

	param_get(_YawRateMax_h, &_YawRateMax);
	param_get(_Pyaw_h, &_Pyaw);
}
