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
#include <px4_platform_common/log.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s ScPositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void ScPositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void ScPositionControl::setVelocityLimits(const float vel_limit)
{
	_lim_vel = vel_limit;
}

void ScPositionControl::setThrustLimit(const float max)
{
	_lim_thr_max = max;
}

void ScPositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_vel_dot = states.acceleration;
	_att_q = states.quaternion;
}

void ScPositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_quat_sp = Quatf(setpoint.attitude);
}

bool ScPositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void ScPositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain velocity setpoints
	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel, _lim_vel);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel, _lim_vel);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel, _lim_vel);
}

void ScPositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// Accelaration to Thrust
	_thr_sp = _acc_sp;
	_thr_sp(0) = math::constrain(_thr_sp(0), -_lim_thr_max, _lim_thr_max);
	_thr_sp(1) = math::constrain(_thr_sp(1), -_lim_thr_max, _lim_thr_max);
	_thr_sp(2) = math::constrain(_thr_sp(2), -_lim_thr_max, _lim_thr_max);

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);

	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

bool ScPositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void ScPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint,
		vehicle_attitude_s &v_att) const
{
	// Set thrust setpoint
	attitude_setpoint.thrust_body[0] = _thr_sp(0);
	attitude_setpoint.thrust_body[1] = _thr_sp(1);
	attitude_setpoint.thrust_body[2] = _thr_sp(2);

	// Bypass attitude control by giving same attitude setpoint to att control
	if (PX4_ISFINITE(_quat_sp(0)) && PX4_ISFINITE(_quat_sp(1)) && PX4_ISFINITE(_quat_sp(2)) && PX4_ISFINITE(_quat_sp(3))) {
		attitude_setpoint.q_d[0] = _quat_sp(0);
		attitude_setpoint.q_d[1] = _quat_sp(1);
		attitude_setpoint.q_d[2] = _quat_sp(2);
		attitude_setpoint.q_d[3] = _quat_sp(3);

	} else {
		attitude_setpoint.q_d[0] = v_att.q[0];
		attitude_setpoint.q_d[1] = v_att.q[1];
		attitude_setpoint.q_d[2] = v_att.q[2];
		attitude_setpoint.q_d[3] = v_att.q[3];
	}
}
