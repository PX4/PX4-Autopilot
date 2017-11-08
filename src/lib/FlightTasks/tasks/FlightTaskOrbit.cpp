/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManual.hpp
 *
 * Flight task for the normal, legacy, manual position controlled flight
 * where stick inputs map basically to the velocity setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "FlightTaskOrbit.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

int FlightTaskOrbit::activate()
{
	FlightTask::activate();
	r = 1.f;
	v =  0.5f;
	z = _position(2);
	_center = Vector2f(_position.data());
	_center(0) -= r;
	return 0;
}

int FlightTaskOrbit::disable()
{
	FlightTask::disable();
	return 0;
}

int FlightTaskOrbit::update()
{
	int ret = FlightTaskManual::update();

	r += _sticks(0) * _deltatime;
	r = math::constrain(r, 1.f, 20.f);
	v -= _sticks(1) * _deltatime;
	v = math::constrain(v, -7.f, 7.f);
	z += _sticks(2) * _deltatime;

	Vector2f center_to_position = Vector2f(_position.data()) - _center;

	/* xy velocity to go around in a circle */
	Vector2f velocity_xy = Vector2f(center_to_position(1), -center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= v;

	/* xy velocity adjustment to stay on the radius distance */
	velocity_xy += (r - center_to_position.norm()) * center_to_position.unit_or_zero();

	float yaw = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F;

	_set_position_setpoint(Vector3f(NAN, NAN, z));
	_set_velocity_setpoint(Vector3f(velocity_xy(0), velocity_xy(1), 0.f));
	_set_yaw_setpoint(yaw);
	return ret;
}
