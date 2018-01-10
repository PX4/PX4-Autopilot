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
 * @file FlightTaskManual.hpp
 *
 * Flight task for the normal, legacy, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "FlightTaskManual.hpp"
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

FlightTaskManual::FlightTaskManual(control::SuperBlock *parent, const char *name) :
	FlightTask(parent, name),
	_z_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
	_z_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
	_xy_vel_man_expo(parent, "MPC_XY_MAN_EXPO", false),
	_z_vel_man_expo(parent, "MPC_Z_MAN_EXPO", false),
	_hold_dz(parent, "MPC_HOLD_DZ", false),
	_velocity_hor_manual(parent, "MPC_VEL_MANUAL", false),
	_hold_max_xy(parent, "MPC_HOLD_MAX_XY", false),
	_hold_max_z(parent, "MPC_HOLD_MAX_Z", false),
	_man_yaw_max(parent, "MPC_MAN_Y_MAX", false)
{ }

bool FlightTaskManual::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(manual_control_setpoint), _sub_manual_control_setpoint)) {
		return false;
	}

	return true;
}

bool FlightTaskManual::activate()
{
	bool ret = FlightTask::activate();
	_hold_position = Vector3f(NAN, NAN, NAN);
	_hold_yaw = NAN;
	return ret;
}

bool FlightTaskManual::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	const bool sticks_available = _evaluateSticks();

	if (_sticks_data_required) {
		ret = ret && sticks_available;
	}

	return ret;
}

bool FlightTaskManual::update()
{
	/* prepare stick input */
	Vector2f stick_xy(_sticks.data()); /**< horizontal two dimensional stick input within a unit circle */
	float &stick_z = _sticks(2);

	const float stick_xy_norm = stick_xy.norm();

	/* saturate such that magnitude in xy is never larger than 1 */
	if (stick_xy_norm > 1.0f) {
		stick_xy /= stick_xy_norm;
	}

	/* rotate stick input to produce velocity setpoint in NED frame */
	Vector3f velocity_setpoint(stick_xy(0), stick_xy(1), stick_z);
	velocity_setpoint = Dcmf(Eulerf(0.0f, 0.0f, _get_input_frame_yaw())) * velocity_setpoint;

	/* scale [0,1] length velocity vector to maximal manual speed (in m/s) */
	_scaleVelocity(velocity_setpoint);

	/* smooth out velocity setpoint by slewrate and return it */
	_setVelocitySetpoint(velocity_setpoint);

	/* handle position and altitude hold */
	const bool stick_xy_zero = stick_xy_norm <= FLT_EPSILON;
	const bool stick_z_zero = fabsf(stick_z) <= FLT_EPSILON;

	float velocity_xy_norm = Vector2f(_velocity.data()).norm();
	const bool stopped_xy = (_hold_max_xy.get() < FLT_EPSILON || velocity_xy_norm < _hold_max_xy.get());
	const bool stopped_z = (_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _hold_max_z.get());

	if (stick_xy_zero && stopped_xy && !PX4_ISFINITE(_hold_position(0))) {
		_hold_position(0) = _position(0);
		_hold_position(1) = _position(1);

	} else if (!stick_xy_zero) {
		_hold_position(0) = NAN;
		_hold_position(1) = NAN;
	}

	if (stick_z_zero && stopped_z && !PX4_ISFINITE(_hold_position(2))) {
		_hold_position(2) = _position(2);

	} else if (!stick_z_zero) {
		_hold_position(2) = NAN;
	}

	_setPositionSetpoint(_hold_position);

	_updateYaw();
	return true;
}

float FlightTaskManual::_get_input_frame_yaw()
{
	/* using constant yaw angle from setpoint here to prevent sideways oscillation in fast forward flight */
	if (PX4_ISFINITE(_hold_yaw)) {
		return _hold_yaw;

	} else {
		return _yaw;
	}
}

void FlightTaskManual::_updateYaw()
{
	const float yaw_speed = _sticks(3) * math::radians(_man_yaw_max.get());
	_setYawspeedSetpoint(yaw_speed);

	const bool stick_yaw_zero = fabsf(yaw_speed) <= FLT_EPSILON;

	if (stick_yaw_zero && !PX4_ISFINITE(_hold_yaw)) {
		_hold_yaw = _yaw;

	} else if (!stick_yaw_zero) {
		_hold_yaw = NAN;
	}

	_setYawSetpoint(_hold_yaw);
}

void FlightTaskManual::_scaleVelocity(Vector3f &velocity)
{
	const Vector3f velocity_scale(_velocity_hor_manual.get(),
				      _velocity_hor_manual.get(),
				      (velocity(2) > 0.0f) ? _z_vel_max_down.get() : _z_vel_max_up.get());

	velocity = velocity.emult(velocity_scale);
}

bool FlightTaskManual::_evaluateSticks()
{
	if ((_time_stamp_current - _sub_manual_control_setpoint->get().timestamp) < _timeout) {
		/* get data and scale correctly */
		_sticks(0) = _sub_manual_control_setpoint->get().x; /* NED x, "pitch" [-1,1] */
		_sticks(1) = _sub_manual_control_setpoint->get().y; /* NED y, "roll" [-1,1] */
		_sticks(2) = -(_sub_manual_control_setpoint->get().z - 0.5f) * 2.f; /* NED z, "thrust" resacaled from [0,1] to [-1,1] */
		_sticks(3) = _sub_manual_control_setpoint->get().r; /* "yaw" [-1,1] */

		/* apply expo and deadzone */
		_sticks(0) = math::expo_deadzone(_sticks(0), _xy_vel_man_expo.get(), _hold_dz.get());
		_sticks(1) = math::expo_deadzone(_sticks(1), _xy_vel_man_expo.get(), _hold_dz.get());
		_sticks(2) = math::expo_deadzone(_sticks(2), _z_vel_man_expo.get(), _hold_dz.get());
		_sticks(3) = math::deadzone(_sticks(3), _hold_dz.get());

		return true;

	} else {
		_sticks.zero(); /* default is all zero */
		return false;
	}
}
