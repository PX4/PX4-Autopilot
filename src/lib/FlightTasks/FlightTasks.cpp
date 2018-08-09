#include "FlightTasks.hpp"

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

FlightTasks::FlightTasks()
{
	// initialize all flight-tasks
	// currently this is required to get all parameters read
	for (int i = 0; i < static_cast<int>(FlightTaskIndex::Count); i++) {
		_initTask(static_cast<FlightTaskIndex>(i));
	}

	// disable all tasks
	_initTask(FlightTaskIndex::None);
}

bool FlightTasks::update()
{
	if (isAnyTaskActive()) {
		_subscription_array.update();
		return _current_task.task->updateInitialize() && _current_task.task->update();
	}

	return false;
}

const vehicle_local_position_setpoint_s FlightTasks::getPositionSetpoint()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getPositionSetpoint();

	} else {
		return FlightTask::empty_setpoint;
	}
}

const vehicle_constraints_s FlightTasks::getConstraints()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getConstraints();

	} else {
		return FlightTask::empty_constraints;
	}
}

const vehicle_trajectory_waypoint_s FlightTasks::getAvoidanceWaypoint()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getAvoidanceWaypoint();

	} else {
		return FlightTask::empty_trajectory_waypoint;
	}
}

const vehicle_trajectory_waypoint_s FlightTasks::getEmptyAvoidanceWaypoint()
{
	return FlightTask::empty_trajectory_waypoint;
}

int FlightTasks::switchTask(FlightTaskIndex new_task_index)
{
	// switch to the running task, nothing to do
	if (new_task_index == _current_task.index) {
		return 0;
	}

	if (_initTask(new_task_index)) {
		// invalid task
		return -1;
	}

	if (!_current_task.task) {
		// no task running
		return 0;
	}

	// subscription failed
	if (!_current_task.task->initializeSubscriptions(_subscription_array)) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
		return -2;
	}

	_subscription_array.forcedUpdate(); // make sure data is available for all new subscriptions

	// activation failed
	if (!_current_task.task->updateInitialize() || !_current_task.task->activate()) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
		return -3;
	}

	return 0;
}

int FlightTasks::switchTask(int new_task_index)
{
	// make sure we are in range of the enumeration before casting
	if (static_cast<int>(FlightTaskIndex::None) <= new_task_index &&
	    static_cast<int>(FlightTaskIndex::Count) > new_task_index) {
		return switchTask(FlightTaskIndex(new_task_index));
	}

	switchTask(FlightTaskIndex::None);
	return -1;
}

int FlightTasks::switchTask(const vehicle_command_s &command, uint8_t &cmd_result)
{
	// check what command it is
	FlightTaskIndex desired_task = switchVehicleCommand(command.command);

	cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

	if (desired_task == FlightTaskIndex::None)
		// ignore all unkown commands
	{
		return -1;
	}

	int switch_result = switchTask(desired_task);

	// if we are in/switched to the desired task
	if (switch_result >= 0) {
		cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

		// if the task is running apply parameters to it and see if it rejects
		if (isAnyTaskActive() && !_current_task.task->applyCommandParameters(command)) {
			cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
		}
	}

	return switch_result;
}

void FlightTasks::handleParameterUpdate()
{
	if (isAnyTaskActive()) {
		_current_task.task->handleParameterUpdate();
	}
}

const char *FlightTasks::errorToString(const int error)
{
	for (auto e : _taskError) {
		if (e.error == error) {
			return e.msg;
		}
	}

	return "This error is not mapped to a string or is unknown.";
}
