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
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskAuto::FlightTaskAuto(control::SuperBlock *parent, const char *name) :
	FlightTask(parent, name),
	_mc_cruise_default(this, "MPC_XY_CRUISE", false)
{}

bool FlightTaskAuto::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(position_setpoint_triplet), _sub_triplet_setpoint)) {
		return false;
	}

	return true;
}

bool FlightTaskAuto::activate()
{
	_prev_prev_wp = _prev_wp = _target = _next_wp = _position;
	_position_lock = matrix::Vector3f(NAN, NAN, NAN);
	_yaw_wp = _yaw;
	return FlightTask::activate();
}

bool FlightTaskAuto::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	return (ret && _evaluateTriplets());
}

bool FlightTaskAuto::_evaluateTriplets()
{
	/* TODO: fix the issues mentioned below */
	/* We add here some conditions that are only required because
	 * 1. navigator continuously sends triplet during mission due to yaw setpoint. This
	 * should be removed in the navigator and only update once the current setpoint actually has changed.
	 *
	 * 2. navigator should be responsible to send always three valid setpoints. If there is only one setpoint,
	 * then previous will be set to current vehicle position and next will be set equal to setpoint.
	 *
	 * 3. navigator originally only supports gps guided maneuvers. However, it now also supports some flow-specific features
	 * such as land and takeoff. The navigator should use for auto takeoff/land with flow the position in xy at the moment the
	 * takeoff/land was initiated. Until then we do this kind of logic here.
	 */

	_mc_cruise_speed = _sub_triplet_setpoint->get().current.cruising_speed;

	if (!PX4_ISFINITE(_mc_cruise_speed) || (_mc_cruise_speed < 0.0f)) {
		/* Use default */
		_mc_cruise_speed = _mc_cruise_default.get();
	}

	_yaw_wp = _sub_triplet_setpoint->get().current.yaw;
	_type = (WaypointType)_sub_triplet_setpoint->get().current.type;

	/* We need to have a valid current triplet */
	if (_isFinite(_sub_triplet_setpoint->get().current)) {

		/* Reset position lock */
		_position_lock = matrix::Vector3f(NAN, NAN, NAN);

		/* 1. only consider updated if current target has has changed.
		 * Note how bad this implementation is. In mission, in every iteration we do double operations */
		matrix::Vector3f target;
		map_projection_project(&_reference_position,
				       _sub_triplet_setpoint->get().current.lat, _sub_triplet_setpoint->get().current.lon, &target(0), &target(1));
		target(2) = -(_sub_triplet_setpoint->get().current.alt - _reference_altitude);

		/* Dont't do any updates if the current target has not changed */
		if (fabsf(target.length() - _target.length()) < FLT_EPSILON) {
			return true;
		}

		/* We have a new target setpoint: update all previous setpoints */
		_target = target;

		/* Set previous to previous - 1 */
		_prev_prev_wp = _prev_wp;

		/* Check if previous is valid and update accordingly */
		if (_isFinite(_sub_triplet_setpoint->get().previous) && _sub_triplet_setpoint->get().previous.valid) {
			map_projection_project(&_reference_position, _sub_triplet_setpoint->get().previous.lat,
					       _sub_triplet_setpoint->get().previous.lon, &_prev_wp(0), &_prev_wp(1));
			_prev_wp(2) = -(_sub_triplet_setpoint->get().previous.alt - _reference_altitude);

		} else {
			_prev_wp = _position;
		}

		/* Check if previous is valid and update accordingly */
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

	} else if (PX4_ISFINITE(_sub_triplet_setpoint->get().current.alt)) {
		/* We only have a valid altitude. Hold position in xy. */

		_type = (WaypointType)_sub_triplet_setpoint->get().current.type;

		if (!PX4_ISFINITE(_position_lock.length())) {
			_position_lock = _position;
		}

		_prev_prev_wp = _prev_wp;
		_prev_wp = _position_lock;
		_prev_wp(2) = _target(2);
		_target = _position_lock;
		_target(2) = -(_sub_triplet_setpoint->get().current.alt - _reference_altitude);
		_next_wp = _target;

		return true;

	} else {
		/* Reset everything */
		_prev_prev_wp = _prev_wp = _target = _next_wp = _position;
		_position_lock = matrix::Vector3f(NAN, NAN, NAN);
		_yaw_wp = _yaw;

		return false;
	}
}

bool FlightTaskAuto::_isFinite(const position_setpoint_s sp)
{
	return (PX4_ISFINITE(sp.lat) && PX4_ISFINITE(sp.lon) && PX4_ISFINITE(sp.alt));
}
