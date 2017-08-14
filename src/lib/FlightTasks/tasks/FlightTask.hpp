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

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

class FlightTask
{
public:
	FlightTask()
	{
		vehicle_local_position_setpoint.x = 1;
		vehicle_local_position_setpoint.y = 2;
		vehicle_local_position_setpoint.z = -3;
	};
	virtual ~FlightTask() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int activate() = 0;

	/**
	 * Call once on the event of switching away from the task
	 * 	@return 0 on success, >0 on error otherwise
	 */
	virtual int disable() = 0;

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @param TODO
	 * @return 0 on success, >0 on error otherwise
	 */
	virtual int update(manual_control_setpoint_s *manual_control_setpoint,
			   vehicle_local_position_s *vehicle_local_position) = 0;

	/**
	 * Call to get result of the task execution
	 * @return pointer to
	 */
	const vehicle_local_position_setpoint_s *get_local_position_setpoint() const { return &vehicle_local_position_setpoint; };

private:

	vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

};
