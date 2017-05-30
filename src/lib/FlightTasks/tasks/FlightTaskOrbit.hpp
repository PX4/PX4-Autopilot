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
 * @file FlightTaskOrbit.h
 *
 * Flight task for orbiting in a circle around a target position
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTask.hpp"

class FlightTaskOrbit : public FlightTask
{
public:
	FlightTaskOrbit() {};
	virtual ~FlightTaskOrbit() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate()
	{
		FlightTask::activate();
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable() { return 0; };

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update()
	{
		FlightTask::update();
		r += _sticks(1) * _deltatime;
		r = math::constrain(r, 0.5f, 4.f);

		v += _sticks(0) * _deltatime; /* angular velocity for orbiting in revolutions per second */
		v = math::constrain(v, -4.f, 4.f);
		altitude += _sticks(3) * _deltatime;
		altitude = math::constrain(altitude, 2.f, 5.f);

		printf("%f %f %f\n", (double)altitude, (double)r, (double)v);

		//printf("%f %f %f\n", (double)_position(0), (double)_position(1), (double)_position(2));
		//printf("%f %f %f\n", (double)_velocity(0), (double)_velocity(1), (double)_velocity(2));
		if (fabsf(r) < 0.01f) {
			r = 0.01;
		}

		float w = v / r; /* angular velocity for orbiting in radians per second */
		_set_position_setpoint(matrix::Vector3f(r * cosf(w * _time), r * sinf(w * _time), -altitude));
		return 0;
	};

private:

	float r = 1.f; /* radius with which to orbit the target */
	float v =  0.1f; /* linear velocity for orbiting in m/s */
	float altitude = 2.f; /* altitude in meters */

};
