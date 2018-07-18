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
 * @file FlightTaskAutoSmooth.cpp
 */

#include "FlightTaskAutoSmooth.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

#define SIGMA_SINGLE_OP			0.000001f
#define SIGMA_NORM			0.001f

FlightTaskAutoSmooth::FlightTaskAutoSmooth() :
	_sl(nullptr, _deltatime, _position)
{ }

bool FlightTaskAutoSmooth::activate()
{
	bool ret = FlightTaskAuto::activate();
	_reset();

	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskAutoSmooth::update()
{
	if (_internal_triplets_update) {
		_update_internal_triplets();
		_internal_triplets_update = false;

		// set velocity depending on the angle between the 3 waypoints
		float angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
			      * Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
			      + 1.0f;
		float desired_vel = math::getVelocityFromAngle(angle, 1.0f, MPC_CRUISE_90.get(), _mc_cruise_speed);

		// straight line
		_sl.setLineFromTo(_prev_wp, _pt_0);
		_sl.setSpeed(_mc_cruise_speed);
		_sl.setSpeedAtTarget(desired_vel);
		_sl.setAcceleration(2.0f);
		_sl.setDeceleration(1.0f);

		// bezier
		float duration = 1.0f;

		if (desired_vel > SIGMA_SINGLE_OP) {
			duration = 2.0f * (_target - _pt_0).length() / desired_vel;
		}

		_b.setBezier(_pt_0, _target, _pt_1, duration);
	}


	Vector3f acc;

	_pt0_reached_once = _pt0_reached_once || (Vector2f(&(_pt_0 - _position_setpoint)(0)).length() < 0.1f);
	bool pt1_reached = Vector2f(&(_pt_1 - _position_setpoint)(0)).length() < 0.1f;

	if (_pt0_reached_once && !pt1_reached) {
		_b.getStatesClosest(_position_setpoint, _velocity_setpoint, acc, _position);
	} else if (!pt1_reached) {
		_sl.generateSetpoints(_position_setpoint, _velocity_setpoint);
	}

	if (pt1_reached) {
		_internal_triplets_update = true;
		_pt0_reached_once = false;
	}

	return 1;

}

void FlightTaskAutoSmooth::_update_internal_triplets()
{
	Vector3f u_prev_to_target = (_target - _prev_wp).unit_or_zero();
	Vector3f u_target_to_next = (_next_wp - _target).unit_or_zero();

	_pt_0 = _target - (u_prev_to_target * NAV_ACC_RAD.get());
	_pt_1 = _target + (u_target_to_next * NAV_ACC_RAD.get());
}

void FlightTaskAutoSmooth::_reset()
{

}

void FlightTaskAutoSmooth::updateParams()
{
	FlightTaskAuto::updateParams();
}
