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
 * @file FlightTaskJourney.cpp
 */

#include "FlightTaskJourney.hpp"

using namespace matrix;

FlightTaskJourney::FlightTaskJourney() :
	_straight_line(nullptr, _deltatime, _position)
{
	_sticks_data_required = false;
}

bool FlightTaskJourney::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTaskManual::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(mount_orientation), _sub_mount_orientation)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(vehicle_attitude), _sub_attitude)) {
		return false;
	}

	return true;
}

bool FlightTaskJourney::applyCommandParameters(const vehicle_command_s &command)
{
	// TODO get parameters from command
	return FlightTaskManual::applyCommandParameters(command);
}

bool FlightTaskJourney::activate()
{
	// get orientation of the camera/gimbal
	Dcmf rot_attitude(Quaternionf(&_sub_attitude->get().q[0])); // rotation matrix for the vehicle
	Dcmf rot_mount(Eulerf(Vector3f(
				      &_sub_mount_orientation->get().attitude_euler_angle[0]))); // rotation matrix for the camera/gimbal

	Vector3f u_mount = rot_attitude * rot_mount * Vector3f(1.0f, 0.0f,
			   0.0f); // unit vector of the camera/gimbal direction in the vehicle frame

	float distance = 20.0f; // TODO make parameter
	_start_position = _position;
	_target_position = _start_position - u_mount * distance;

	_straight_line.setLineFromTo(_start_position, _target_position);
	_straight_line.setSpeed(2.0f);
	_straight_line.setSpeedAtTarget(0.0f);
	_straight_line.setAcceleration(0.5f);
	_straight_line.setDeceleration(0.5f);

	is_returning = false;

	// call activate() from parent
	bool ret = FlightTaskManual::activate();

	// need a valid position and velocity
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskJourney::update()
{
	// TODO check smoothness
	_straight_line.generateSetpoints(_position_setpoint, _velocity_setpoint);

	if (!is_returning && (_target_position - _position).length() < 0.2f) {
		_straight_line.setLineFromTo(_target_position, _start_position);
		_straight_line.setSpeed(2.0f);
		_straight_line.setSpeedAtTarget(0.0f);
		_straight_line.setAcceleration(0.5f);
		_straight_line.setDeceleration(0.5f);
		is_returning = true;
	}

	return true;
}