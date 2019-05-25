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
#include <float.h>
#include <mathlib/mathlib.h>
#include <ControlMath.hpp>
#include <px4_defines.h>

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
	// make sure the thrust has a tiny minimal length to infer the direction
	_lim_thr_min = math::max(min, -10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::setConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.
	if (!PX4_ISFINITE(constraints.tilt) || (constraints.tilt > _lim_tilt)) {
		_constraints.tilt = _lim_tilt;
	}
}

bool PositionControl::update(const float dt)
{
	_positionControl();
	_velocityControl(dt);

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;

	// const bool valid_acceleration_setpoint = PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1))
	// 		&& PX4_ISFINITE(_acc_sp(2));
	return true;//valid_acceleration_setpoint;
}

void PositionControl::_positionControl()
{
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	_addIfNotNanVector(_vel_sp, vel_sp_position);
	_addIfNotNanVector(vel_sp_position, Vector3f());

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _lim_vel_horizontal);
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	const float hover_scale = CONSTANTS_ONE_G / _hover_thrust;

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int + _vel_dot.emult(_gain_vel_d);

	// For backwards compatibility of the gains
	acc_sp_velocity *= hover_scale;
	// No control input from setpoints or corresponding states which are NAN
	_addIfNotNanVector(_acc_sp, acc_sp_velocity);

	// Support deprecated feed forward thrust
	_addIfNotNanVector(_acc_sp, hover_scale * (_thr_sp + Vector3f(0, 0, _hover_thrust)));

	_accelerationControl();

	// Apply Anti-Windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Saturate thrust setpoint in vertical direction
	_thr_sp(2) = math::constrain(_thr_sp(2), -_lim_thr_max, -_lim_thr_min);

	// Get maximum allowed horizontal thrust based on tilt and excess thrust
	const float thrust_xy_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	float thrust_max_xy = sqrtf(thrust_xy_max_squared - thrust_z_squared);

	// Saturate thrust in horizontal direction.
	Vector2f thrust_sp_xy(_thr_sp);

	if (thrust_sp_xy.norm_squared() > thrust_max_xy * thrust_max_xy) {
		thrust_sp_xy = thrust_sp_xy.normalized() * thrust_max_xy;
		_thr_sp(0) = thrust_sp_xy(0);
		_thr_sp(1) = thrust_sp_xy(1);
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const float arw_gain = 2.f / _gain_vel_p(0);
	const Vector2f vel_error_lim = Vector2f(vel_error) - (arw_gain * (thrust_sp_xy - Vector2f(_thr_sp)));
	vel_error(0) = vel_error_lim(0);
	vel_error(1) = vel_error_lim(1);

	// Update integral part of velocity control
	_vel_int += vel_error * _gain_vel_i * dt;
	// Make sure integral doesn't stay NAN
	_addIfNotNanVector(_vel_int, Vector3f());
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _constraints.tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	_thr_sp = body_z * collective_thrust;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint)
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

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint)
{
	ControlMath::bodyzToAttitude(attitude_setpoint, -_thr_sp, _yaw_sp);
	attitude_setpoint.thrust_body[2] = -_thr_sp.length();
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

void PositionControl::_addIfNotNan(float &setpoint, const float addition) const
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void PositionControl::_addIfNotNanVector(Vector3f &setpoint, const Vector3f &addition) const
{
	for (int i = 0; i < 3; i++) {
		_addIfNotNan(setpoint(i), addition(i));
	}
}
