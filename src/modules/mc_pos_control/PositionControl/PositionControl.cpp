/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	_lim_thr_min = min;
	_lim_thr_max = max;
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

bool PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping();

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))
			   && PX4_ISFINITE(_thr_sp(2));

	return mapping_succeeded;
}

void PositionControl::setConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.
	if (!PX4_ISFINITE(constraints.tilt) || (constraints.tilt > _lim_tilt)) {
		_constraints.tilt = _lim_tilt;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || (constraints.speed_up > _lim_vel_up)) {
		_constraints.speed_up = _lim_vel_up;
	}

	if (!PX4_ISFINITE(constraints.speed_down) || (constraints.speed_down > _lim_vel_down)) {
		_constraints.speed_down = _lim_vel_down;
	}

	// ignore _constraints.speed_xy TODO: remove it completely as soon as no task uses it anymore to avoid confusion
}

void PositionControl::update(const float dt)
{
	if (_skip_controller) {

		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > _lim_thr_max) {
			_thr_sp = _thr_sp.normalized() * _lim_thr_max;

		} else if (thr_mag < _lim_thr_min && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * _lim_thr_min;
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _vel_dot;

	} else {
		_positionControl();
		_velocityControl(dt);
	}
}

bool PositionControl::_interfaceMapping()
{
	// if nothing is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valid position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// States and setpoints which are integrals of the reference setpoint are set to 0.
	// For instance: reference is velocity-setpoint -> position and position-setpoint = 0
	//               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = NAN;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;

			// thrust setpoint is not supported in velocity control
			_thr_sp(i) = NAN;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_vel_sp(i) = _vel(i) = 0.0f;

			// Reset the Integral term.
			_vel_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(_lim_thr_min + (_hover_thrust - _lim_thr_min) * 0.7f);
		// position and velocity control-loop is currently unused (flag only for logging purpose)
	}

	return !(failsafe);
}

void PositionControl::_positionControl()
{
	// P-position controller
	const Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _lim_vel_horizontal);
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;

	// Consider thrust in D-direction.
	float thrust_desired_D = _gain_vel_p(2) * vel_err(2) +  _gain_vel_d(2) * _vel_dot(2) + _vel_int(
					 2) - _hover_thrust;

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -_lim_thr_min;
	float uMin = -_lim_thr_max;

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = math::min(uMax, -10e-4f);

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_vel_int(2) += vel_err(2) * _gain_vel_i(2) * dt;

		// limit thrust integral
		_vel_int(2) = math::min(fabsf(_vel_int(2)), _lim_thr_max) * math::sign(_vel_int(2));
	}

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = _gain_vel_p(0) * vel_err(0) + _gain_vel_d(0) * _vel_dot(0) + _vel_int(0);
		thrust_desired_NE(1) = _gain_vel_p(1) * vel_err(1) + _gain_vel_d(1) * _vel_dot(1) + _vel_int(1);

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(_lim_thr_max * _lim_thr_max - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}

		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		float arw_gain = 2.f / _gain_vel_p(0);

		Vector2f vel_err_lim;
		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

		// Update integral
		_vel_int(0) += _gain_vel_i(0) * vel_err_lim(0) * dt;
		_vel_int(1) += _gain_vel_i(1) * vel_err_lim(1) * dt;
	}
}


void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
