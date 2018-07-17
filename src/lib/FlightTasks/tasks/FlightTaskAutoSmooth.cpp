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
	_target_prev = _target;
	_sl.setLineFromTo(_position, _target);
	_sl.setSpeed(2.0f);
	_sl.setSpeedAtTarget(2.0f);
	_sl.setAcceleration(10.0f);
	_sl.setDeceleration(1.0f);


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

		_sl.setLineFromTo(_position, _pt_0);
		_sl.setSpeed(2.0f);
		_sl.setSpeedAtTarget(2.0f);
		_sl.setAcceleration(10.0f);
		_sl.setDeceleration(1.0f);
	}

	
	Vector3f acc;
	_sl.generateSetpoints(_position_setpoint, _velocity_setpoint);

	_pt0_reached_once |= (Vector2f(&(_pt_0 - _position)(0)).length() < 0.5f);
	printf("_pt0_reached_once %d %f \n", _pt0_reached_once, (double)Vector2f(&(_pt_0 - _position)(0)).length());
	bool pt1_reached = Vector2f(&(_pt_1 - _position)(0)).length() < 1.0f;
	printf("py1 %f \n", (double)Vector2f(&(_pt_1 - _position)(0)).length());

	if (_pt0_reached_once && !pt1_reached)
	{
		_b.getStatesClosest(_position_setpoint, _velocity_setpoint, acc, _position);
		printf("------------- bezier x %f y %f \n", (double)_position_setpoint(0), (double)_position_setpoint(1));
	}
	
	if (pt1_reached)
	{
		_internal_triplets_update = true;
		_pt0_reached_once = false;
	}

	// printf("pos sp %f %f %f \n", (double)_position_setpoint(0), (double)_position_setpoint(1), (double)_position_setpoint(2));
	// printf("vel sp %f %f %f \n", (double)_velocity_setpoint(0), (double)_velocity_setpoint(1), (double)_velocity_setpoint(2));
	return 1;

}

void FlightTaskAutoSmooth::_update_internal_triplets()
{
	printf("_update_internal_triplets, target: \n");
	_target.print();

	Vector3f u_prev_to_target = (_target - _prev_wp).unit_or_zero();
	Vector3f u_target_to_next = (_next_wp - _target).unit_or_zero();

	_pt_0 = _target - (u_prev_to_target * NAV_ACC_RAD.get());
	_pt_1 = _target + (u_target_to_next * NAV_ACC_RAD.get());
	printf("pt0: ");
	_pt_0.print();
	printf("pt1: ");
	_pt_1.print();
	printf("position: ");
	_position.print();

	_b.setBezier(_pt_0, _target, _pt_1, 5.0);
}

void FlightTaskAutoSmooth::_reset()
{

}

void FlightTaskAutoSmooth::updateParams()
{
	FlightTaskAuto::updateParams();

}
