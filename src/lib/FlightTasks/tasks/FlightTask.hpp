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
 * @file FlightTask.hpp
 *
 * Abstract base class for different advanced flight tasks like orbit, follow me, ...
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

class FlightTask
{
public:
	FlightTask()
	{
		_vehicle_local_position = NULL;
		_manual_control_setpoint = NULL;
		_reset_time();
	};
	virtual ~FlightTask() {};

	/**
	 * Call once on the event where you switch to the task
	 * Note: Set the necessary input pointers first!
	 * @return 0 on success, >0 on error
	 */
	virtual int activate()
	{
		_reset_time();
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error
	 */
	virtual int disable() { return 0; };

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error
	 */
	virtual int update()
	{
		_time = hrt_elapsed_time(&_starting_time_stamp) / 1e6f;
		_deltatime  = math::min(hrt_elapsed_time(&_last_time_stamp) / 1e6f, (float)_timeout);
		_last_time_stamp = hrt_absolute_time();
		_evaluate_sticks();
		_evaluate_position();
		_evaluate_velocity();
		return 0;
	};

	/**
	 * Set vehicle local position data pointer
	 * @param pointer to vehicle local position
	 */
	void set_vehicle_local_position_pointer(const vehicle_local_position_s *vehicle_local_position) { _vehicle_local_position = vehicle_local_position; };

	/**
	 * Set manual control setpoint data pointer if it's needed for the task
	 * @param pointer to manual control setpoint
	 */
	void set_manual_control_setpoint_pointer(const manual_control_setpoint_s *manual_control_setpoint) { _manual_control_setpoint = manual_control_setpoint; };

	/**
	 * Get the resulting setpoints of the task execution via pointer
	 * @return pointer to setpoint struct
	 */
	const vehicle_local_position_setpoint_s *get_position_setpoint() const { return &_vehicle_position_setpoint; };

protected:

	float _time = 0; /*< passed time in seconds since the task was activated */
	float _deltatime = 0; /*< passed time in seconds since the task was last updated */
	void _reset_time() { _starting_time_stamp = hrt_absolute_time(); };

	/* Prepared general inputs for every task */
	matrix::Vector<float, 4> _sticks;
	matrix::Vector3f _position;
	matrix::Vector3f _velocity;

	void _set_position_setpoint(const matrix::Vector3f position_setpoint)
	{
		_vehicle_position_setpoint.x = position_setpoint(0);
		_vehicle_position_setpoint.y = position_setpoint(1);
		_vehicle_position_setpoint.z = position_setpoint(2);
	};

	void _set_velocity_setpoint(const matrix::Vector3f velocity_setpoint)
	{
		_vehicle_position_setpoint.vx = velocity_setpoint(0);
		_vehicle_position_setpoint.vy = velocity_setpoint(1);
		_vehicle_position_setpoint.vz = velocity_setpoint(2);
	};

	void _set_acceleration_setpoint(const matrix::Vector3f acceleration_setpoint)
	{
		_vehicle_position_setpoint.acc_x = acceleration_setpoint(0);
		_vehicle_position_setpoint.acc_y = acceleration_setpoint(1);
		_vehicle_position_setpoint.acc_z = acceleration_setpoint(2);
	};

private:
	static const int _timeout = 500000;

	hrt_abstime _starting_time_stamp; /*< time stamp when task was activated */
	hrt_abstime _last_time_stamp; /*< time stamp when task was last updated */

	/* General input that every task has */
	const vehicle_local_position_s *_vehicle_local_position;
	const manual_control_setpoint_s *_manual_control_setpoint;

	/* General output that every task has */
	vehicle_local_position_setpoint_s _vehicle_position_setpoint;

	void _evaluate_position()
	{
		if (_vehicle_local_position != NULL && hrt_elapsed_time(&_vehicle_local_position->timestamp) < _timeout) {
			_position = matrix::Vector3f(&_vehicle_local_position->x);
		}
	}

	void _evaluate_velocity()
	{
		if (_vehicle_local_position != NULL && hrt_elapsed_time(&_vehicle_local_position->timestamp) < _timeout) {
			_velocity = matrix::Vector3f(&_vehicle_local_position->vx);

		} else {
			_velocity = matrix::Vector3f(); /* default is all zero */
		}
	}

	void _evaluate_sticks()
	{
		if (_manual_control_setpoint != NULL && hrt_elapsed_time(&_manual_control_setpoint->timestamp) < _timeout) {
			_sticks(0) = _manual_control_setpoint->x; /* NED x, "pitch" [-1,1] */
			_sticks(1) = _manual_control_setpoint->y; /* NED y, "roll" [-1,1] */
			_sticks(2) = (_manual_control_setpoint->z - 0.5f) * 2.f; /* NED z, "thrust" resacaled from [0,1] to [-1,1] */
			_sticks(3) = _manual_control_setpoint->r; /* "yaw" [-1,1] */

		} else {
			_sticks = matrix::Vector<float, 4>(); /* default is all zero */
		}
	}

};
