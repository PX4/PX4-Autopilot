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
 * @file FlightTaskOrbit.hpp
 *
 * Flight task for orbiting in a circle around a target position
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTaskManual.hpp"

class FlightTaskOrbit : public FlightTaskManual
{
public:
	FlightTaskOrbit(SuperBlock *parent, const char *name) :
		FlightTaskManual(parent, name)
	{};
	virtual ~FlightTaskOrbit() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate()
	{
		FlightTask::activate();
		r = 1.f;
		v =  0.5f;
		z = _position(2);
		_center = matrix::Vector2f(_position.data());
		_center(0) -= r;
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable()
	{
		FlightTask::disable();
		return 0;
	};

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update()
	{
		int ret = FlightTaskManual::update();

		r += _sticks(0) * _deltatime;
		r = math::constrain(r, 1.f, 20.f);
		v -= _sticks(1) * _deltatime;
		v = math::constrain(v, -7.f, 7.f);
		z += _sticks(2) * _deltatime;

		matrix::Vector2<float> center_to_position = matrix::Vector2f(_position.data()) - _center;

		/* xy velocity to go around in a circle */
		matrix::Vector2<float> velocity_xy = matrix::Vector2f(center_to_position(1), -center_to_position(0));
		velocity_xy = velocity_xy.unit_or_zero();
		velocity_xy *= v;

		/* xy velocity adjustment to stay on the radius distance */
		velocity_xy += (r - center_to_position.norm()) * center_to_position.unit_or_zero();

		float yaw = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F;

		_set_position_setpoint(matrix::Vector3f(NAN, NAN, z));
		_set_velocity_setpoint(matrix::Vector3f(velocity_xy(0), velocity_xy(1), 0.f));
		_set_yaw_setpoint(yaw);
		return ret;
	};

private:

	float r = 0.f; /* radius with which to orbit the target */
	float v =  0.f; /* linear velocity for orbiting in m/s */
	float z = 0.f; /* local z coordinate in meters */
	matrix::Vector2f _center;

};
