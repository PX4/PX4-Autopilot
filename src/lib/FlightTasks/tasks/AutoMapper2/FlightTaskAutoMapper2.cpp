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
 * @file FlightAutoMapper2.cpp
 */

#include "FlightTaskAutoMapper2.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutoMapper2::activate()
{
	bool ret = FlightTaskAuto::activate();
	_reset();
	return ret;
}

bool FlightTaskAutoMapper2::update()
{
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	_updateAltitudeAboveGround();

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_thrust_setpoint = Vector3f(NAN, NAN, NAN);
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

	_generateSetpoints();

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;
	}

	// update previous type
	_type_previous = _type;

	return true;
}

void FlightTaskAutoMapper2::_reset()
{
	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;
}

void FlightTaskAutoMapper2::_prepareIdleSetpoints()
{
	// Send zero thrust setpoint
	_position_setpoint = Vector3f(NAN, NAN, NAN); // Don't require any position/velocity setpoints
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);
	_thrust_setpoint.zero();
}

void FlightTaskAutoMapper2::_prepareLandSetpoints()
{
	// Keep xy-position and go down with landspeed
	_position_setpoint = Vector3f(_target(0), _target(1), NAN);
	const float speed_lnd = (_alt_above_ground > MPC_LAND_ALT1.get()) ? _constraints.speed_down : MPC_LAND_SPEED.get();
	_velocity_setpoint = Vector3f(Vector3f(NAN, NAN, speed_lnd));

	// set constraints
	_constraints.tilt = MPC_TILTMAX_LND.get();
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper2::_prepareTakeoffSetpoints()
{
	// Takeoff is completely defined by target position
	_position_setpoint = _target;
	const float speed_tko = (_alt_above_ground > MPC_LAND_ALT1.get()) ? _constraints.speed_up : MPC_TKO_SPEED.get();
	_velocity_setpoint = Vector3f(NAN, NAN, -speed_tko); // Limit the maximum vertical speed

	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper2::_prepareVelocitySetpoints()
{
	// XY Velocity waypoint
	// TODO : Rewiew that. What is the expected behavior?
	_position_setpoint = Vector3f(NAN, NAN, _position(2));
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

void FlightTaskAutoMapper2::_preparePositionSetpoints()
{
	// Simple waypoint navigation: go to xyz target, with standard limitations
	_position_setpoint = _target;
	_velocity_setpoint = Vector3f(NAN, NAN, NAN); // No special velocity limitations
}

void FlightTaskAutoMapper2::_updateAltitudeAboveGround()
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

void FlightTaskAutoMapper2::updateParams()
{
	FlightTaskAuto::updateParams();

	// make sure that alt1 is above alt2
	MPC_LAND_ALT1.set(math::max(MPC_LAND_ALT1.get(), MPC_LAND_ALT2.get()));
}

bool FlightTaskAutoMapper2::_highEnoughForLandingGear()
{
	// return true if altitude is above two meters
	return _alt_above_ground > 2.0f;
}
