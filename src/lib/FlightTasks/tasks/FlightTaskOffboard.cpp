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
 * @file FlightTaskAuto.cpp
 */
#include "FlightTaskOffboard.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskOffboard::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(offboard_setpoints), _sub_offboard)) {
		return false;
	}

	return true;
}

bool FlightTaskOffboard::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskOffboard::activate()
{
	bool ret = FlightTask::activate();
	_position_setpoint = _position;
	_velocity_setpoint *= 0.0f;
	_position_lock *= NAN;
	return ret;
}

bool FlightTaskOffboard::update()
{
	// reset setpoint for every loop
	_resetSetpoints();

	// Yaw / Yaw-speed
	if (PX4_ISFINITE(_sub_offboard->get().setpoint.yaw)) {
		// yaw control required
		_yaw_setpoint = _sub_offboard->get().setpoint.yaw;

		if (PX4_ISFINITE(_sub_offboard->get().setpoint.yawspeed)) {
			// yawspeed is used as feedforward
			_yawspeed_setpoint = _sub_offboard->get().setpoint.yawspeed;
		}

	} else if (PX4_ISFINITE(_sub_offboard->get().setpoint.yawspeed)) {
		// only yawspeed required
		_yawspeed_setpoint = _sub_offboard->get().setpoint.yawspeed;
		// set yaw setpoint to NAN since not used
		_yaw_setpoint = NAN;

	}

	// Loiter
	if (_sub_offboard->get().type == offboard_setpoints_s::TYPE_LOITER) {
		// loiter just means that the vehicle should keep position
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;

		} else {
			_position_setpoint = _position_lock;
		}

		// don't have to continue
		return true;

	} else {
		_position_lock *= NAN;
	}

	// Takeoff
	if (_sub_offboard->get().type == offboard_setpoints_s::TYPE_TAKEOFF) {
		// just do takeoff to default altitude
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;
			_position_setpoint(2) = _position_lock(2) = _position(2) - MIS_TAKEOFF_ALT.get();

		} else {
			_position_setpoint = _position_lock;
		}

		// don't have to continue
		return true;

	} else {
		_position_lock *= NAN;
	}

	// Land
	if (_sub_offboard->get().type == offboard_setpoints_s::TYPE_LAND) {
		// land with landing speed, but keep position in xy
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;
			_position_setpoint(2) = _position_lock(2) = NAN;
			_velocity_setpoint(2) = MPC_LAND_SPEED.get();

		} else {
			_position_setpoint = _position_lock;
			_velocity_setpoint(2) = MPC_LAND_SPEED.get();
		}

		// don't have to continue
		return true;

	} else {
		_position_lock *= NAN;
	}

	// IDLE
	if (_sub_offboard->get().type == offboard_setpoints_s::TYPE_IDLE) {
		_thrust_setpoint.zero();
		return true;
	}

	// Possible inputs:
	// 1. position setpoint
	// 2. position setpoint + velocity setpoint (velocity used as feedforward)
	// 3. velocity setpoint
	// 4. acceleration setpoint -> this will be mapped to normalized thrust setpoint because acceleration is not supported

	const bool position_ctrl_xy = PX4_ISFINITE(_sub_offboard->get().setpoint.x)
				      && PX4_ISFINITE(_sub_offboard->get().setpoint.y);
	const bool position_ctrl_z = PX4_ISFINITE(_sub_offboard->get().setpoint.z);
	const bool velocity_ctrl_xy = PX4_ISFINITE(_sub_offboard->get().setpoint.vx)
				      && PX4_ISFINITE(_sub_offboard->get().setpoint.vy);
	const bool velocity_ctrl_z = PX4_ISFINITE(_sub_offboard->get().setpoint.vz);
	const bool feedforward_ctrl_xy = position_ctrl_xy && velocity_ctrl_xy;
	const bool feedforward_ctrl_z = position_ctrl_z && velocity_ctrl_z;
	const bool acceleration_ctrl = PX4_ISFINITE(_sub_offboard->get().setpoint.acc_x)
				       && PX4_ISFINITE(_sub_offboard->get().setpoint.acc_y) &&
				       PX4_ISFINITE(_sub_offboard->get().setpoint.acc_z);
	const bool thrust_ctrl = PX4_ISFINITE(_sub_offboard->get().setpoint.thrust[0])
				 && PX4_ISFINITE(_sub_offboard->get().setpoint.thrust[1]) &&
				 PX4_ISFINITE(_sub_offboard->get().setpoint.thrust[2]);

	// if nothing is valid in xy, then exit offboard
	if (!(position_ctrl_xy || velocity_ctrl_xy || acceleration_ctrl || thrust_ctrl)) {
		return false;
	}

	// if nothing is valid in z, then exit offboard
	if (!(position_ctrl_z || velocity_ctrl_z || acceleration_ctrl || thrust_ctrl)) {
		return false;
	}

	// XY-direction
	if (feedforward_ctrl_xy) {
		_position_setpoint(0) = _sub_offboard->get().setpoint.x;
		_position_setpoint(1) = _sub_offboard->get().setpoint.y;
		_velocity_setpoint(0) = _sub_offboard->get().setpoint.vx;
		_velocity_setpoint(1) = _sub_offboard->get().setpoint.vy;

	} else if (position_ctrl_xy) {
		_position_setpoint(0) = _sub_offboard->get().setpoint.x;
		_position_setpoint(1) = _sub_offboard->get().setpoint.y;

	} else if (velocity_ctrl_xy) {

		if (_sub_offboard->get().velocity_frame == offboard_setpoints_s::FRAME_LOCAL_NED) {
			// in local frame: don't require any transformation
			_velocity_setpoint(0) = _sub_offboard->get().setpoint.vx;
			_velocity_setpoint(1) = _sub_offboard->get().setpoint.vy;

		} else if (_sub_offboard->get().velocity_frame == offboard_setpoints_s::FRAME_BODY_NED) {
			// in body frame: need to transform first
			// Note, this transformation is wrong because body-xy is not neccessarily on the same plane as local-xy
			_velocity_setpoint(0) = cosf(_yaw) * _sub_offboard->get().setpoint.vx - sinf(
							_yaw) * _sub_offboard->get().setpoint.vy;
			_velocity_setpoint(1) = sinf(_yaw) * _sub_offboard->get().setpoint.vx + cosf(
							_yaw) * _sub_offboard->get().setpoint.vy;

		} else {
			// no valid frame
			return false;
		}
	}

	// Z-direction
	if (feedforward_ctrl_z) {
		_position_setpoint(2) = _sub_offboard->get().setpoint.z;
		_velocity_setpoint(2) = _sub_offboard->get().setpoint.vz;

	} else if (position_ctrl_z) {
		_position_setpoint(2) = _sub_offboard->get().setpoint.z;

	} else if (velocity_ctrl_z) {
		_velocity_setpoint(2) = _sub_offboard->get().setpoint.vz;
	}

	// Acceleration
	// Note: this is not supported yet and will be mapped to normalized thrust directly.
	if (acceleration_ctrl) {
		_thrust_setpoint(0) = _sub_offboard->get().setpoint.acc_x;
		_thrust_setpoint(1) = _sub_offboard->get().setpoint.acc_y;
		_thrust_setpoint(2) = _sub_offboard->get().setpoint.acc_z;
	}

	// thrust
	if (thrust_ctrl) {
		_thrust_setpoint(0) = _sub_offboard->get().setpoint.thrust[0];
		_thrust_setpoint(1) = _sub_offboard->get().setpoint.thrust[1];
		_thrust_setpoint(2) = _sub_offboard->get().setpoint.thrust[2];
	}

	return true;
}
