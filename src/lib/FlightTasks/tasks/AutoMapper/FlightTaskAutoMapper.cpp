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
 * @file FlightAutoMapper.cpp
 */

#include "FlightTaskAutoMapper.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutoMapper::activate()
{
	bool ret = FlightTaskAuto::activate();
	_reset();
	return ret;
}

bool FlightTaskAutoMapper::update()
{
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	_updateAltitudeAboveGround();

	bool follow_line = _type == WaypointType::loiter || _type == WaypointType::position;
	bool follow_line_prev = _type_previous == WaypointType::loiter || _type_previous == WaypointType::position;

	// 1st time that vehicle starts to follow line. Reset all setpoints to current vehicle state.
	if (follow_line && !follow_line_prev) {
		_reset();
	}

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_thrust_setpoint = Vector3f(NAN, NAN, NAN);
	}

	if (_type == WaypointType::idle) {
		_generateIdleSetpoints();

	} else if (_type == WaypointType::land) {
		_generateLandSetpoints();

	} else if (follow_line) {
		_generateSetpoints();

	} else if (_type == WaypointType::takeoff) {
		_generateTakeoffSetpoints();

	} else if (_type == WaypointType::velocity) {
		_generateVelocitySetpoints();
	}

	_obstacle_avoidance.injectAvoidanceSetpoints(_position_setpoint, _velocity_setpoint, _yaw_setpoint,
			_yawspeed_setpoint);

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_gear.landing_gear = _mission_gear;
	}

	// update previous type
	_type_previous = _type;

	return true;
}

void FlightTaskAutoMapper::_reset()
{
	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;
}

void FlightTaskAutoMapper::_generateIdleSetpoints()
{
	// Send zero thrust setpoint
	_position_setpoint = Vector3f(NAN, NAN, NAN); // Don't require any position/velocity setpoints
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);
	_thrust_setpoint.zero();
}

void FlightTaskAutoMapper::_generateLandSetpoints()
{
	// Keep xy-position and go down with landspeed
	_position_setpoint = Vector3f(_target(0), _target(1), NAN);
	_velocity_setpoint = Vector3f(Vector3f(NAN, NAN, _param_mpc_land_speed.get()));
	// set constraints
	_constraints.tilt = _param_mpc_tiltmax_lnd.get();
	_constraints.speed_down = _param_mpc_land_speed.get();
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper::_generateTakeoffSetpoints()
{
	// Takeoff is completely defined by target position
	_position_setpoint = _target;
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);

	// limit vertical speed during takeoff
	_constraints.speed_up = math::gradual(_alt_above_ground, _param_mpc_land_alt2.get(),
					      _param_mpc_land_alt1.get(), _param_mpc_tko_speed.get(), _constraints.speed_up);

	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper::_generateVelocitySetpoints()
{
	// TODO: Remove velocity force logic from navigator, since
	// navigator should only send out waypoints.
	_position_setpoint = Vector3f(NAN, NAN, _position(2));
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

void FlightTaskAutoMapper::_updateAltitudeAboveGround()
{
	// Altitude above ground is by default just the negation of the current local position in D-direction.
	_alt_above_ground = -_position(2);

	if (PX4_ISFINITE(_dist_to_bottom)) {
		// We have a valid distance to ground measurement
		_alt_above_ground = _dist_to_bottom;

	} else if (_sub_home_position->get().valid_alt) {
		// if home position is set, then altitude above ground is relative to the home position
		_alt_above_ground = -_position(2) + _sub_home_position->get().z;
	}
}

void FlightTaskAutoMapper::updateParams()
{
	FlightTaskAuto::updateParams();

	// make sure that alt1 is above alt2
	_param_mpc_land_alt1.set(math::max(_param_mpc_land_alt1.get(), _param_mpc_land_alt2.get()));
}

bool FlightTaskAutoMapper::_highEnoughForLandingGear()
{
	// return true if altitude is above two meters
	return _alt_above_ground > 2.0f;
}
