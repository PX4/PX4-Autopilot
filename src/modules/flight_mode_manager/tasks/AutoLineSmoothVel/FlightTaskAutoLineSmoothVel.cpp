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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAutoLineSmoothVel.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutoLineSmoothVel::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskAuto::activate(last_setpoint);

	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;

	Vector3f vel_prev{last_setpoint.vx, last_setpoint.vy, last_setpoint.vz};
	Vector3f pos_prev{last_setpoint.x, last_setpoint.y, last_setpoint.z};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current postion
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	_position_smoothing.reset(accel_prev, vel_prev, pos_prev);

	_yaw_sp_prev = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	_updateTrajConstraints();
	_is_emergency_braking_active = false;

	return ret;
}

void FlightTaskAutoLineSmoothVel::reActivate()
{
	FlightTaskAuto::reActivate();

	// On ground, reset acceleration and velocity to zero
	_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.7f}, _position);
}

bool FlightTaskAutoLineSmoothVel::update()
{
	bool ret = FlightTaskAuto::update();
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_acceleration_setpoint.setNaN();
	}

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;
	}

	switch (_type) {
	case WaypointType::idle:
		_prepareIdleSetpoints();
		break;

	case WaypointType::land:
		_prepareLandSetpoints();
		break;

	case WaypointType::loiter:

	/* fallthrought */
	case WaypointType::position:
		_preparePositionSetpoints();
		break;

	case WaypointType::takeoff:
		_prepareTakeoffSetpoints();
		break;

	case WaypointType::velocity:
		_prepareVelocitySetpoints();
		break;

	default:
		_preparePositionSetpoints();
		break;
	}

	if (_param_com_obs_avoid.get()) {
		_obstacle_avoidance.updateAvoidanceDesiredSetpoints(_position_setpoint, _velocity_setpoint, (int)_type);
		_obstacle_avoidance.injectAvoidanceSetpoints(_position_setpoint, _velocity_setpoint, _yaw_setpoint,
				_yawspeed_setpoint);
	}


	_generateSetpoints();

	// update previous type
	_type_previous = _type;

	return ret;
}
