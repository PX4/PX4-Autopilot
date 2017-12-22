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
 * Flight task for manual controlled position.
 *
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskManualPosition::FlightTaskManualPosition(control::SuperBlock *parent, const char *name) :
	FlightTaskManualAltitude(parent, name),
	_vel_xy_manual_max(parent, "MPC_VEL_MANUAL", false),
	_acc_xy_max(parent, "MPC_ACC_HOR_MAX", false)
{}

bool FlightTaskManualPosition::activate()
{
	_pos_sp_xy = _pos_sp_xy_lock = matrix::Vector2f(_position(0), _position(1));
	_vel_sp_xy = matrix::Vector2f(NAN, NAN);
	_lock_time = 0.0f;
	_lock_time_max = 1.0f; // 1s time to brake as default
	return FlightTaskManualAltitude::activate();
}

void FlightTaskManualPosition::scaleSticks()
{
	/* scale all sticks including altitude and yaw */
	FlightTaskManualAltitude::scaleSticks();

	matrix::Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

	/* constrain length of stick inputs to 1 */
	float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);
	stick_xy = stick_xy.normalized() * mag;

	/* scale to velocity */
	_vel_sp_xy = stick_xy * _vel_xy_manual_max.get();

	/* rotate setpoint into local frame */
	matrix::Vector3f vel_sp{_vel_sp_xy(0), _vel_sp_xy(1), 0.0f};
	vel_sp = (matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _yaw_sp_predicted)) * vel_sp);
	_vel_sp_xy = matrix::Vector2f(vel_sp(0), vel_sp(1));

}

void FlightTaskManualPosition::updateXYsetpoints()
{
	matrix::Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

	if (stick_xy.length() < FLT_EPSILON) {

		/* Want to hold position */
		if (_lock_time <= _lock_time_max) {
			/* Don't lock: time has not been reached */
			_vel_sp_xy.zero();
			_pos_sp_xy = matrix::Vector2f(NAN, NAN);
			_pos_sp_xy_lock = matrix::Vector2f(&(_position(0)));
			_lock_time += _deltatime;

		} else {
			/* want to hold position */
			_vel_sp_xy = matrix::Vector2f(NAN, NAN);
			_pos_sp_xy = matrix::Vector2f(&(_pos_sp_xy_lock(0)));
		}

	} else {
		/* Want to change position */
		_pos_sp_xy = matrix::Vector2f(NAN, NAN);

		/* Time to brake depends on
		 * maximum acceleration.
		 */
		if (PX4_ISFINITE(_acc_xy_max.get())) {
			/* We take half max acceleration because it is better to lock
			 * position setpoint late than early to prevent backward and forward movement.
			 */
			_lock_time_max = matrix::Vector2f(&(_velocity(0))).length() / (0.5f * _acc_xy_max.get());

		} else {
			_lock_time_max = 2.0f; // 2 second time to brake if no acceleration is set
		}

		_lock_time = 0.0f;
	}
}

void FlightTaskManualPosition::updateSetpoints()
{
	/* scale all sticks */
	scaleSticks();

	/* update all setpoints */
	FlightTaskManualAltitude::updateSetpoints();
	updateXYsetpoints();

}

bool FlightTaskManualPosition::update()
{
	scaleSticks();
	updateSetpoints();

	_setPositionSetpoint(Vector3f(_pos_sp_xy(0), _pos_sp_xy(1), _pos_sp_z));
	_setVelocitySetpoint(Vector3f(_vel_sp_xy(0), _vel_sp_xy(1), _vel_sp_z));
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);

	return true;
}
