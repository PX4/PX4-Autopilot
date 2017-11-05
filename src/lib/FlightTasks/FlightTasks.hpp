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

class FlightTasks : control::SuperBlock
{
public:
	FlightTasks() :
		SuperBlock(nullptr, "TSK"),
		Manual(this, "MAN"),
		Orbit(this, "ORB")
	{};
	~FlightTasks() {};

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error
	 */
	int update()
	{
		if (is_any_task_active()) {
			return _tasks[_current_task]->update();

		} else {
			return 1;
		}
	};

	/**
	 * Call this function initially to point all tasks to the general output data
	 */
	void set_general_output_pointers(vehicle_local_position_setpoint_s *vehicle_local_position_setpoint)
	{
		for (int i = 0; i < _task_count; i++) {
			_tasks[i]->set_vehicle_local_position_setpoint_pointer(vehicle_local_position_setpoint);
		}
	};

	/**
	 * Switch to the next task in the available list (for testing)
	 * @return 0 on success, >0 on error
	 */
	void switch_task()
	{
		switch_task(_current_task + 1);
	};

	/**
	 * Switch to a specific task (for normal usage)
	 * @param task number to switch to
	 * @return 0 on success, >0 on error
	 */
	int switch_task(int task_number)
	{
		/* switch to the running task, nothing to do */
		if (task_number == _current_task) {
			return 0;
		}

		/* disable the old task if there is any */
		if (is_any_task_active()) {
			_tasks[_current_task]->disable();
		}

		/* if the new task exists and it activates succesfully we switched */
		if (is_task_number_valid(task_number) && !_tasks[task_number]->activate()) {
			_current_task = task_number;
			return 0;
		}

		/* something went wrong, no task running anymore */
		_current_task = -1;
		return 1;
	};

	/**
	 * Get the number of the active task
	 * @return number of active task, -1 if there is none
	 */
	int get_active_task() const { return _current_task; };

	/**
	 * Check if any task is active
	 * @return true if a task is active, false if not
	 */
	bool is_any_task_active() const { return is_task_number_valid(_current_task); };

	/**
	 * Check if the task number exists
	 * @return true if yes, false if not
	 */
	bool is_task_number_valid(int task_number) const { return task_number > -1 && task_number < _task_count; };

private:
	static constexpr int _task_count = 2;
	FlightTask *_tasks[_task_count] = {&Manual, &Orbit};
	FlightTaskManual Manual;
	FlightTaskOrbit Orbit;

	int _current_task = -1;
};
