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
 * @file FlightTask.h
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
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate()
	{
		_reset_time();
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable() = 0;

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update()
	{
		_time = hrt_elapsed_time(&_starting_time_stamp) / 1e6;
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

	float _get_time() { return _time; }
	void _reset_time() { _starting_time_stamp = hrt_absolute_time(); };

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

	/* local time for a task */
	float _time = 0; /*< passed time in seconds since the task was activated */
	hrt_abstime _starting_time_stamp; /*< time stamp when task was activated */

	/* General Input */
	const vehicle_local_position_s *_vehicle_local_position;
	const manual_control_setpoint_s *_manual_control_setpoint;

	/* General Output */
	vehicle_local_position_setpoint_s _vehicle_position_setpoint;

};
