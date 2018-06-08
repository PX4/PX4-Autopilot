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

#include "FlightTaskAuto.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskAuto::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(position_setpoint_triplet), _sub_triplet_setpoint)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(home_position), _sub_home_position)) {
		return false;
	}

	return true;
}

bool FlightTaskAuto::activate()
{
	bool ret = FlightTask::activate();
	_prev_prev_wp = _prev_wp = _target = _next_wp = _position;
	_setDefaultConstraints();
	return ret;
}

bool FlightTaskAuto::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	// require valid reference and valid target
	ret = ret && _evaluateGlobalReference() && _evaluateTriplets();
	// require valid position
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskAuto::_evaluateTriplets()
{
	// TODO: fix the issues mentioned below
	// We add here some conditions that are only required because:
	// 1. navigator continuously sends triplet during mission due to yaw setpoint. This
	// should be removed in the navigator and only updates if the current setpoint actually has changed.
	//
	// 2. navigator should be responsible to send always three valid setpoints. If there is only one setpoint,
	// then previous will be set to current vehicle position and next will be set equal to setpoint.
	//
	// 3. navigator originally only supports gps guided maneuvers. However, it now also supports some flow-specific features
	// such as land and takeoff. The navigator should use for auto takeoff/land with flow the position in xy at the moment the
	// takeoff/land was initiated. Until then we do this kind of logic here.

	// Check if triplet is valid. There must be at least a valid altitude.
	if (!_sub_triplet_setpoint->get().current.valid || !PX4_ISFINITE(_sub_triplet_setpoint->get().current.alt)) {
		// best we can do is to just set all waypoints to current state and return false
		_prev_prev_wp = _prev_wp = _target = _next_wp = _position;
		_type = WaypointType::position;
		return false;
	}

	_type = (WaypointType)_sub_triplet_setpoint->get().current.type;

	// always update cruise speed since that can change without waypoint changes
	_mc_cruise_speed = _sub_triplet_setpoint->get().current.cruising_speed;

	if (!PX4_ISFINITE(_mc_cruise_speed) || (_mc_cruise_speed < 0.0f) || (_mc_cruise_speed > _constraints.speed_xy)) {
		// use default limit
		_mc_cruise_speed = _constraints.speed_xy;
	}

	// get target waypoint.
	matrix::Vector3f target;

	if (!PX4_ISFINITE(_sub_triplet_setpoint->get().current.lat)
	    || !PX4_ISFINITE(_sub_triplet_setpoint->get().current.lon)) {
		// No position provided in xy. Lock position
		if (!PX4_ISFINITE(_lock_position_xy(0))) {
			target(0) = _lock_position_xy(0) = _position(0);
			target(1) = _lock_position_xy(1) = _position(1);

		} else {
			target(0) = _lock_position_xy(0);
			target(1) = _lock_position_xy(1);
			_lock_position_xy *= NAN;
		}

	} else {
		// Convert from global to local frame.
		map_projection_project(&_reference_position,
				       _sub_triplet_setpoint->get().current.lat, _sub_triplet_setpoint->get().current.lon, &target(0), &target(1));
	}

	target(2) = -(_sub_triplet_setpoint->get().current.alt - _reference_altitude);

	// check if target is valid
	_yaw_setpoint = _sub_triplet_setpoint->get().current.yaw;

	if (_type == WaypointType::follow_target && _sub_triplet_setpoint->get().current.yawspeed_valid) {
		_yawspeed_setpoint = _sub_triplet_setpoint->get().current.yawspeed;
		_yaw_setpoint = NAN;
	}

	// Check if anything has changed. We do that by comparing the target
	// setpoint to the previous target.
	// TODO This is a hack and it would be much better if the navigator only sends out a waypoints once tthey have changed.

	// dont't do any updates if the current target has not changed
	if (!(fabsf(target(0) - _target(0)) > 0.001f || fabsf(target(1) - _target(1)) > 0.001f
	      || fabsf(target(2) - _target(2)) > 0.001f)) {
		// nothing has changed: just keep old waypoints
		return true;
	}

	// update all waypoints
	_target = target;

	if (!PX4_ISFINITE(_target(0)) || !PX4_ISFINITE(_target(1))) {
		// Horizontal target is not finite. */
		_target(0) = _position(0);
		_target(1) = _position(1);
	}

	if (!PX4_ISFINITE(_target(2))) {
		_target(2) = _position(2);
	}

	_prev_prev_wp = _prev_wp;

	if (_isFinite(_sub_triplet_setpoint->get().previous) && _sub_triplet_setpoint->get().previous.valid) {
		map_projection_project(&_reference_position, _sub_triplet_setpoint->get().previous.lat,
				       _sub_triplet_setpoint->get().previous.lon, &_prev_wp(0), &_prev_wp(1));
		_prev_wp(2) = -(_sub_triplet_setpoint->get().previous.alt - _reference_altitude);

	} else {
		_prev_wp = _position;
	}

	if (_type == WaypointType::loiter) {
		_next_wp = _target;

	} else if (_isFinite(_sub_triplet_setpoint->get().next) && _sub_triplet_setpoint->get().next.valid) {
		map_projection_project(&_reference_position, _sub_triplet_setpoint->get().next.lat,
				       _sub_triplet_setpoint->get().next.lon, &_next_wp(0), &_next_wp(1));
		_next_wp(2) = -(_sub_triplet_setpoint->get().next.alt - _reference_altitude);

	} else {
		_next_wp = _target;
	}

	return true;
}

bool FlightTaskAuto::_isFinite(const position_setpoint_s sp)
{
	return (PX4_ISFINITE(sp.lat) && PX4_ISFINITE(sp.lon) && PX4_ISFINITE(sp.alt));
}

bool FlightTaskAuto::_evaluateGlobalReference()
{
	// check if reference has changed and update.
	// Only update if reference timestamp has changed AND no valid reference altitude
	// is available.
	// TODO: this needs to be revisited and needs a more clear implementation
	if (_sub_vehicle_local_position->get().ref_timestamp != _time_stamp_reference &&
	    (_sub_vehicle_local_position->get().z_global && !PX4_ISFINITE(_reference_altitude))) {

		map_projection_init(&_reference_position,
				    _sub_vehicle_local_position->get().ref_lat,
				    _sub_vehicle_local_position->get().ref_lon);
		_reference_altitude = _sub_vehicle_local_position->get().ref_alt;
		_time_stamp_reference = _sub_vehicle_local_position->get().ref_timestamp;
	}

	if (PX4_ISFINITE(_reference_altitude)
	    && PX4_ISFINITE(_sub_vehicle_local_position->get().ref_lat)
	    && PX4_ISFINITE(_sub_vehicle_local_position->get().ref_lat)) {
		return true;

	} else {
		return false;
	}
}

void FlightTaskAuto::_setDefaultConstraints()
{
	FlightTask::_setDefaultConstraints();

	// only adjust limits if the new limit is lower
	if (_constraints.speed_xy >= MPC_XY_CRUISE.get()) {
		_constraints.speed_xy = MPC_XY_CRUISE.get();
	}
}

matrix::Vector2f FlightTaskAuto::_getTargetVelocityXY()
{
	// guard against any bad velocity values
	const float vx = _sub_triplet_setpoint->get().current.vx;
	const float vy = _sub_triplet_setpoint->get().current.vy;
	bool velocity_valid = PX4_ISFINITE(vx) && PX4_ISFINITE(vy) &&
			      _sub_triplet_setpoint->get().current.velocity_valid;

	if (velocity_valid) {
		return matrix::Vector2f(vx, vy);

	} else {
		// just return zero speed
		return matrix::Vector2f{};
	}
}
