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
 * @file PositionControl.cpp
 *
 * This file implements a P-position-control cascaded with a
 * PID-velocity-controller.
 *
 * Inputs: vehicle states (pos, vel, q)
 *         desired setpoints (pos, vel, thrust, yaw)
 * Outputs: thrust and yaw setpoint
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "uORB/topics/parameter_update.h"
#include "Utility/ControlMath.hpp"

using namespace matrix;

PositionControl::PositionControl()
{
	_Pz_h   = param_find("MPC_Z_P");
	_Pvz_h  = param_find("MPC_Z_VEL_P");
	_Ivz_h  = param_find("MPC_Z_VEL_I");
	_Dvz_h  = param_find("MPC_Z_VEL_D");
	_Pxy_h  = param_find("MPC_XY_P");
	_Pvxy_h = param_find("MPC_XY_VEL_P");
	_Ivxy_h = param_find("MPC_XY_VEL_I");
	_Dvxy_h = param_find("MPC_XY_VEL_D");
	_VelMaxXY_h = param_find("MPC_XY_VEL_MAX");
	_VelMaxZdown_h = param_find("MPC_Z_VEL_MAX_DN");
	_VelMaxZup_h = param_find("MPC_Z_VEL_MAX_UP");
	_ThrHover_h = param_find("MPC_THR_HOVER");
	_ThrMax_h = param_find("MPC_THR_MAX");
	_ThrMin_h = param_find("MPC_THR_MIN");

	/* Set parameter the very first time. */
	_setParams();
};

void PositionControl::updateState(const vehicle_local_position_s &state, const Vector3f &vel_dot)
{
	_pos = Vector3f(&state.x);
	_vel = Vector3f(&state.vx);
	_yaw = state.yaw;
	_vel_dot = vel_dot;
}

void PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(&setpoint.x);
	_vel_sp = Vector3f(&setpoint.vx);
	_acc_sp = Vector3f(&setpoint.acc_x);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	_interfaceMapping();

	/* If full manual is required (thrust already generated), don't run position/velocity
	 * controller and just return thrust.
	 */
	_skipController = false;

	if (PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]) && PX4_ISFINITE(setpoint.thrust[2])) {
		_skipController = true;
	}
}

void PositionControl::generateThrustYawSetpoint(const float &dt)
{
	_updateParams();

	/* Only run position/velocity controller
	 * if thrust needs to be generated
	 */
	if (!_skipController) {
		_positionController();
		_velocityController(dt);
	}
}

void PositionControl::_interfaceMapping()
{
	/* Respects FlightTask interface, where
	 * NAN-setpoints are of no interest and
	 * do not require control.
	 */

	/* Loop through x,y and z components of all setpoints. */
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {

			/* Position control is required */

			if (!PX4_ISFINITE(_vel_sp(i))) {
				/* Velocity is not used as feedforward term. */
				_vel_sp(i) = 0.0f;
			}

			/* thrust setpoint is not supported
			 * in position control
			 */
			_thr_sp(i) = 0.0f;

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			/* Velocity controller is active without
			 * position control.
			 */
			_pos_sp(i) = _pos(i);
			_thr_sp(i) = 0.0f;

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			/* Thrust setpoint was generated from
			 * stick directly.
			 */
			_pos_sp(i) = _pos(i);
			_vel_sp(i) = _vel(i);
			_thr_int(i) = 0.0f;
			_vel_dot(i) = 0.0f;

		} else {
			PX4_WARN("TODO: add safety");
		}
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = _yaw;
	}
}

void PositionControl::_positionController()
{
	/* Generate desired velocity setpoint */

	/* P-controller */
	_vel_sp = (_pos_sp - _pos).emult(Pp) + _vel_sp;

	/* Make sure velocity setpoint is constrained in all directions (xyz). */
	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));

	if (vel_norm_xy > _VelMaxXY) {
		_vel_sp(0) = _vel_sp(0) * _VelMaxXY / vel_norm_xy;
		_vel_sp(1) = _vel_sp(1) * _VelMaxXY / vel_norm_xy;
	}

	/* Saturate velocity in D-direction */
	_vel_sp(2) = math::constrain(_vel_sp(2), -_VelMaxZ.up, _VelMaxZ.down);
}

void PositionControl::_velocityController(const float &dt)
{
	/* Generate desired thrust setpoint */
	/* PID
	 * u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	 * Umin <= u_des <= Umax
	 *
	 * Anti-Windup:
	 * u_des = _thr_sp; r = _vel_sp; y = _vel
	 * u_des >= Umax and r - y >= 0 => Saturation = true
	 * u_des >= Umax and r - y <= 0 => Saturation = false
	 * u_des <= Umin and r - y <= 0 => Saturation = true
	 * u_des <= Umin and r - y >= 0 => Saturation = false
	 *
	 *	Notes:
	 * - PID implementation is in NED-frame
	 * - control output in D-direction has priority over NE-direction
	 * - the equilibrium point for the PID is at hover-thrust
	 * - the maximum tilt cannot exceed 90 degrees. This means that it is
	 * 	 not possible to have a desired thrust direction pointing in the positive
	 * 	 D-direction (= downward)
	 * - the desired thrust in D-direction is limited by the thrust limits
	 * - the desired thrust in NE-direction is limited by the thrust excess after
	 * 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	 * 	 NE-direction is also limited by the maximum tilt.
	 */

	/* Get maximum tilt */
	float tilt_max = M_PI_2_F;

	if (PX4_ISFINITE(_constraints.tilt_max) && _constraints.tilt_max <= tilt_max) {
		tilt_max = _constraints.tilt_max;
	}

	Vector3f vel_err = _vel_sp - _vel;

	/*
	 * TODO: add offboard acceleration mode
	 * */

	/* Consider thrust in D-direction */
	float thrust_desired_D = Pv(2) * vel_err(2) + Dv(2) * _vel_dot(2) + _thr_int(2) - _ThrHover;

	/* The Thrust limits are negated and swapped due to NED-frame */
	float uMax = -_ThrustLimit.min;
	float uMin = -_ThrustLimit.max;

	/* Apply Anti-Windup in D-direction */
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * Iv(2) * dt;

	}

	/* Saturate thrust setpoint in D-direction */
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) {

		/* Thrust setpoints in NE-direction is already provided. Only
		 * scaling is required.
		 */

		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(tilt_max);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {

		/* PID for NE-direction */
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = Pv(0) * vel_err(0) + Dv(0) * _vel_dot(0) + _thr_int(0);
		thrust_desired_NE(1) = Pv(1) * vel_err(1) + Dv(1) * _vel_dot(1) + _thr_int(1);

		/* Get maximum allowed thrust in NE based on tilt and excess thrust */
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(tilt_max);
		float thrust_max_NE = sqrtf(_ThrustLimit.max * _ThrustLimit.max - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		/* Get the direction of (r-y) in NE-direction */
		float direction_NE = Vector2f(&vel_err(0)) * Vector2f(&_vel_sp(0));

		/* Apply Anti-Windup in NE-direction */
		bool stop_integral_NE = (thrust_desired_NE * thrust_desired_NE >= thrust_max_NE * thrust_max_NE &&
					 direction_NE >= 0.0f);

		if (!stop_integral_NE) {
			_thr_int(0) += vel_err(0) * Iv(0) * dt;
			_thr_int(1) += vel_err(1) * Iv(1) * dt;
		}

		/* Saturate thrust in NE-direction */
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;

		}
	}

}

void PositionControl::updateConstraints(const Controller::Constraints &constraints)
{
	_constraints = constraints;
}

void PositionControl::_updateParams()
{
	bool updated;
	parameter_update_s param_update;
	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &param_update);
		_setParams();
	}
}

void PositionControl::_setParams()
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
	param_get(_VelMaxZup_h, &_VelMaxZ.up);
	param_get(_VelMaxZdown_h, &_VelMaxZ.down);

	param_get(_ThrHover_h, &_ThrHover);
	param_get(_ThrMax_h, &_ThrustLimit.max);
	param_get(_ThrMin_h, &_ThrustLimit.min);
}
