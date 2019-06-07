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

bool FlightTaskOffboard::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	// require a valid triplet
	_sub_triplet_setpoint.update();
	ret = ret && _sub_triplet_setpoint.get().current.valid;

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
	_velocity_setpoint.setZero();
	_position_lock.setAll(NAN);
	return ret;
}

bool FlightTaskOffboard::update()
{
	_sub_triplet_setpoint.update();
	const position_setpoint_triplet_s &triplet_setpoint = _sub_triplet_setpoint.get();

	// reset setpoint for every loop
	_resetSetpoints();

	if (!triplet_setpoint.current.valid) {
		_setDefaultConstraints();
		_position_setpoint = _position;
		return false;
	}

	// Yaw / Yaw-speed
	if (triplet_setpoint.current.yaw_valid) {
		// yaw control required
		_yaw_setpoint = triplet_setpoint.current.yaw;

		if (triplet_setpoint.current.yawspeed_valid) {
			// yawspeed is used as feedforward
			_yawspeed_setpoint = triplet_setpoint.current.yawspeed;
		}

	} else if (triplet_setpoint.current.yawspeed_valid) {
		// only yawspeed required
		_yawspeed_setpoint = triplet_setpoint.current.yawspeed;
		// set yaw setpoint to NAN since not used
		_yaw_setpoint = NAN;

	}

	// Loiter
	if (triplet_setpoint.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
		// loiter just means that the vehicle should keep position
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;

		} else {
			_position_setpoint = _position_lock;
		}

		// don't have to continue
		return true;

	} else {
		_position_lock.setAll(NAN);
	}

	// Takeoff
	if (triplet_setpoint.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		// just do takeoff to default altitude
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;
			_position_setpoint(2) = _position_lock(2) = _position(2) - _param_mis_takeoff_alt.get();

		} else {
			_position_setpoint = _position_lock;
		}

		// don't have to continue
		return true;

	} else {
		_position_lock.setAll(NAN);
	}

	// Land
	if (triplet_setpoint.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		// land with landing speed, but keep position in xy
		if (!PX4_ISFINITE(_position_lock(0))) {
			_position_setpoint = _position_lock = _position;
			_position_setpoint(2) = _position_lock(2) = NAN;
			_velocity_setpoint(2) = _param_mpc_land_speed.get();

		} else {
			_position_setpoint = _position_lock;
			_velocity_setpoint(2) = _param_mpc_land_speed.get();
		}

		// don't have to continue
		return true;

	} else {
		_position_lock.setAll(NAN);
	}

	// IDLE
	if (triplet_setpoint.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_thrust_setpoint.zero();
		return true;
	}

	// Possible inputs:
	// 1. position setpoint
	// 2. position setpoint + velocity setpoint (velocity used as feedforward)
	// 3. velocity setpoint
	// 4. acceleration setpoint -> this will be mapped to normalized thrust setpoint because acceleration is not supported
	const bool position_ctrl_xy = triplet_setpoint.current.position_valid && _sub_vehicle_local_position.get().xy_valid;
	const bool position_ctrl_z = triplet_setpoint.current.alt_valid && _sub_vehicle_local_position.get().z_valid;
	const bool velocity_ctrl_xy = triplet_setpoint.current.velocity_valid && _sub_vehicle_local_position.get().v_xy_valid;
	const bool velocity_ctrl_z = triplet_setpoint.current.velocity_valid && _sub_vehicle_local_position.get().v_z_valid;
	const bool feedforward_ctrl_xy = position_ctrl_xy && velocity_ctrl_xy;
	const bool feedforward_ctrl_z = position_ctrl_z && velocity_ctrl_z;
	const bool acceleration_ctrl = triplet_setpoint.current.acceleration_valid;

	// if nothing is valid in xy, then exit offboard
	if (!(position_ctrl_xy || velocity_ctrl_xy || acceleration_ctrl)) {
		return false;
	}

	// if nothing is valid in z, then exit offboard
	if (!(position_ctrl_z || velocity_ctrl_z || acceleration_ctrl)) {
		return false;
	}

	// XY-direction
	if (feedforward_ctrl_xy) {
		_position_setpoint(0) = triplet_setpoint.current.x;
		_position_setpoint(1) = triplet_setpoint.current.y;
		_velocity_setpoint(0) = triplet_setpoint.current.vx;
		_velocity_setpoint(1) = triplet_setpoint.current.vy;

	} else if (position_ctrl_xy) {
		_position_setpoint(0) = triplet_setpoint.current.x;
		_position_setpoint(1) = triplet_setpoint.current.y;

	} else if (velocity_ctrl_xy) {

		if (triplet_setpoint.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_LOCAL_NED) {
			// in local frame: don't require any transformation
			_velocity_setpoint(0) = triplet_setpoint.current.vx;
			_velocity_setpoint(1) = triplet_setpoint.current.vy;

		} else if (triplet_setpoint.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
			// in body frame: need to transorm first
			// Note, this transformation is wrong because body-xy is not neccessarily on the same plane as locale-xy
			_velocity_setpoint(0) = cosf(_yaw) * triplet_setpoint.current.vx - sinf(_yaw) * triplet_setpoint.current.vy;
			_velocity_setpoint(1) = sinf(_yaw) * triplet_setpoint.current.vx + cosf(_yaw) * triplet_setpoint.current.vy;

		} else {
			// no valid frame
			return false;
		}
	}

	// Z-direction
	if (feedforward_ctrl_z) {
		_position_setpoint(2) = triplet_setpoint.current.z;
		_velocity_setpoint(2) = triplet_setpoint.current.vz;

	} else if (position_ctrl_z) {
		_position_setpoint(2) = triplet_setpoint.current.z;

	} else if (velocity_ctrl_z) {
		_velocity_setpoint(2) = triplet_setpoint.current.vz;
	}

	// Acceleration
	// Note: this is not supported yet and will be mapped to normalized thrust directly.
	if (triplet_setpoint.current.acceleration_valid) {
		_thrust_setpoint(0) = triplet_setpoint.current.a_x;
		_thrust_setpoint(1) = triplet_setpoint.current.a_y;
		_thrust_setpoint(2) = triplet_setpoint.current.a_z;
	}

	// use default conditions of upwards position or velocity to take off
	_constraints.want_takeoff = _checkTakeoff();

	return true;
}
