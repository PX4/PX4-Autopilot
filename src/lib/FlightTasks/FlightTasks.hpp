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

#include "FlightTask.hpp"
#include "SubscriptionArray.hpp"
#include "FlightTasks_generated.hpp"
#include <lib/WeatherVane/WeatherVane.hpp>

#include <new>

class FlightTasks
{
public:
	FlightTasks();

	~FlightTasks()
	{
		if (_current_task.task) {
			_current_task.task->~FlightTask();
		}
	}

	/**
	 * Call regularly in the control loop cycle to execute the task
	 * @return true on success, false on error
	 */
	bool update();

	/**
	 * Get the output data from the current task
	 * @return output setpoint, to be executed by position control
	 */
	const vehicle_local_position_setpoint_s getPositionSetpoint();

	/**
	 * Get task dependent constraints
	 * @return setpoint constraints that has to be respected by the position controller
	 */
	const vehicle_constraints_s getConstraints();

	/**
	 * Get landing gear position.
	 * @return landing gear
	 */
	const landing_gear_s getGear();

	/**
	 * Get task avoidance desired waypoints
	 * @return auto triplets in the mc_pos_control
	 */
	const vehicle_trajectory_waypoint_s getAvoidanceWaypoint();

	/**
	 * Get empty avoidance desired waypoints
	 * @return empty triplets in the mc_pos_control
	 */
	const vehicle_trajectory_waypoint_s &getEmptyAvoidanceWaypoint();

	/**
	 * Switch to the next task in the available list (for testing)
	 * @return 1 on success, <0 on error
	 */
	int switchTask() { return switchTask(static_cast<int>(_current_task.index) + 1); }

	/**
	 * Switch to a specific task (for normal usage)
	 * @param task index to switch to
	 * @return 1 on success, 0 on no change, <0 on error
	 */
	int switchTask(FlightTaskIndex new_task_index);
	int switchTask(int new_task_index);

	/**
	 * Get the number of the active task
	 * @return number of active task, -1 if there is none
	 */
	int getActiveTask() const { return static_cast<int>(_current_task.index); }

	/**
	 * Check if any task is active
	 * @return true if a task is active, false if not
	 */
	bool isAnyTaskActive() const { return _current_task.task; }

	/**
	 * Call this whenever a parameter update notification is received (parameter_update uORB message)
	 */
	void handleParameterUpdate();

	/**
	 * Call this method to get the description of a task error.
	 */
	const char *errorToString(const int error);

	/**
	 * Sets an external yaw handler. The active flight task can use the yaw handler to implement a different yaw control strategy.
	 */
	void setYawHandler(WeatherVane *ext_yaw_handler) {_current_task.task->setYawHandler(ext_yaw_handler);}

	/**
	 *   This method will re-activate current task.
	 */
	void reActivate();

	void updateVelocityControllerIO(const matrix::Vector3f &vel_sp, const matrix::Vector3f &thrust_sp) {_current_task.task->updateVelocityControllerIO(vel_sp, thrust_sp); }

private:

	/**
	 * Union with all existing tasks: we use it to make sure that only the memory of the largest existing
	 * task is needed, and to avoid using dynamic memory allocations.
	 */
	TaskUnion _task_union; /**< storage for the currently active task */

	struct flight_task_t {
		FlightTask *task;
		FlightTaskIndex index;
	};
	flight_task_t _current_task = {nullptr, FlightTaskIndex::None};

	SubscriptionArray _subscription_array;

	struct task_error_t {
		int error;
		const char *msg;
	};

	static const int _numError = 4;
	/**
	 * Map from Error int to user friendly string.
	 */
	task_error_t _taskError[_numError] = {
		{0, "No Error"},
		{-1, "Invalid Task "},
		{-2, "Subscription Failed"},
		{-3, "Activation Failed"}
	};
	/**
	 * Check for vehicle commands (received via MAVLink), evaluate and acknowledge them
	 */
	void _updateCommand();
	FlightTaskIndex switchVehicleCommand(const int command);
	int _sub_vehicle_command = -1; /**< topic handle on which commands are received */
	orb_advert_t _pub_vehicle_command_ack = nullptr; /**< topic handle to which commands get acknowledged */

	int _initTask(FlightTaskIndex task_index);
};
