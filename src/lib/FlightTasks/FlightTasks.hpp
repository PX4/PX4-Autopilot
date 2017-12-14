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
 * @file flight_taks.h
 *
 * Library class to hold and manage all implemented flight task instances
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "tasks/FlightTask.hpp"
#include "tasks/FlightTaskManual.hpp"
#include "tasks/FlightTaskOrbit.hpp"

#include "SubscriptionArray.hpp"

#include <new>

class FlightTasks : control::SuperBlock
{
public:
	FlightTasks() :
		SuperBlock(nullptr, "TSK")
	{}

	~FlightTasks()
	{
		if (_current_task) {
			_current_task->~FlightTask();
		}
	}

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return true on success, false on error
	 */
	bool update()
	{
		_updateCommand();

		if (isAnyTaskActive()) {
			_subscription_array.update();
			return _current_task->updateInitialize() && _current_task->update();
		}

		return false;
	}

	/**
	 * Get the output data from the current task
	 * Only call when task is active!
	 */
	const vehicle_local_position_setpoint_s &getPositionSetpoint() { return _current_task->getPositionSetpoint(); }

	/**
	 * Convenient operator to get the output data from the current task
	 * Only call when task is active!
	 */
	inline const vehicle_local_position_setpoint_s &operator()() { return getPositionSetpoint(); }

	/**
	 * Switch to the next task in the available list (for testing)
	 * @return true on success, false on error
	 */
	int switchTask()
	{
		return switchTask(_current_task_index + 1);
	}

	/**
	 * Switch to a specific task (for normal usage)
	 * @param task number to switch to
	 * @return 0 on success, <0 on error
	 */
	int switchTask(int task_number)
	{
		/* switch to the running task, nothing to do */
		if (task_number == _current_task_index) {
			return 0;
		}

		/* disable the old task if there is any */
		if (_current_task) {
			_current_task->~FlightTask();
			_current_task = nullptr;
			_current_task_index = -1;
		}

		switch (task_number) {
		case 0:
			_current_task = new (&_task_union.manual) FlightTaskManual(this, "MAN");
			break;

		case 1:
			_current_task = new (&_task_union.orbit) FlightTaskOrbit(this, "ORB");
			break;

		case -1:
			/* disable tasks is a success */
			return 0;

		default:
			/* invalid task */
			return -1;
		}

		/* subscription failed */
		if (!_current_task->initializeSubscriptions(_subscription_array)) {
			_current_task->~FlightTask();
			_current_task = nullptr;
			_current_task_index = -1;
			return -2;
		}

		_subscription_array.forcedUpdate(); // make sure data is available for all new subscriptions

		/* activation failed */
		if (!_current_task->updateInitialize() || !_current_task->activate()) {
			_current_task->~FlightTask();
			_current_task = nullptr;
			_current_task_index = -1;
			return -3;
		}

		_current_task_index = task_number;
		return 0;
	}

	/**
	 * Get the number of the active task
	 * @return number of active task, -1 if there is none
	 */
	int getActiveTask() const { return _current_task_index; }

	/**
	 * Check if any task is active
	 * @return true if a task is active, false if not
	 */
	bool isAnyTaskActive() const { return _current_task; }

private:

	/**
	 * Union with all existing tasks: we use it to make sure that only the memory of the largest existing
	 * task is needed, and to avoid using dynamic memory allocations.
	 */
	union TaskUnion {
		TaskUnion() {}
		~TaskUnion() {}

		FlightTaskManual manual;
		FlightTaskOrbit orbit;
	} _task_union; /*< storage for the currently active task */

	FlightTask *_current_task = nullptr;
	int _current_task_index = -1;

	SubscriptionArray _subscription_array;

	/**
	 * Check for vehicle commands (received via MAVLink), evaluate and acknowledge them
	 * @return true if there was a new command, false if not
	 */
	bool _updateCommand();
	int	_sub_vehicle_command = -1; /*< topic handle on which commands are received */
	orb_advert_t _pub_vehicle_command_ack = nullptr; /*< topic handle to which commands get acknowledged */
};
