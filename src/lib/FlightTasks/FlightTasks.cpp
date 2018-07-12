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
	_updateCommand();

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

int FlightTasks::_initTask(FlightTaskIndex task_index)
{

	// disable the old task if there is any
	if (_current_task.task) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
	}

	switch (task_index) {
	case FlightTaskIndex::None:
		// already disabled task
		break;

	case FlightTaskIndex::Stabilized:
		_current_task.task = new (&_task_union.stabilized) FlightTaskManualStabilized();
		break;

	case FlightTaskIndex::Altitude:
		_current_task.task = new (&_task_union.altitude) FlightTaskManualAltitude();
		break;

	case FlightTaskIndex::AltitudeSmooth:
		_current_task.task = new (&_task_union.altitude_smooth) FlightTaskManualAltitudeSmooth();
		break;

	case FlightTaskIndex::Position:
		_current_task.task = new (&_task_union.position) FlightTaskManualPosition();
		break;

	case FlightTaskIndex::PositionSmooth:
		_current_task.task =
			new (&_task_union.position_smooth) FlightTaskManualPositionSmooth();
		break;

	case FlightTaskIndex::Orbit:
		_current_task.task = new (&_task_union.orbit) FlightTaskOrbit();
		break;

	case FlightTaskIndex::Sport:
		_current_task.task = new (&_task_union.sport) FlightTaskSport();
		break;

	case FlightTaskIndex::AutoLine:
		_current_task.task = new (&_task_union.autoLine) FlightTaskAutoLine();
		break;

	case FlightTaskIndex::AutoFollowMe:
		_current_task.task =
			new (&_task_union.autoFollowMe) FlightTaskAutoFollowMe();
		break;

	case FlightTaskIndex::Offboard:
		_current_task.task = new (&_task_union.offboard) FlightTaskOffboard();
		break;

	default:
		// invalid task
		return 1;
	}

	// task construction succeeded
	_current_task.index = task_index;
	return 0;
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

void FlightTasks::_updateCommand()
{
	// lazy subscription to command topic
	if (_sub_vehicle_command < 0) {
		_sub_vehicle_command = orb_subscribe(ORB_ID(vehicle_command));
	}

	// check if there's any new command
	bool updated = false;
	orb_check(_sub_vehicle_command, &updated);

	if (!updated) {
		return;
	}

	// get command
	struct vehicle_command_s command;
	orb_copy(ORB_ID(vehicle_command), _sub_vehicle_command, &command);

	// check what command it is
	FlightTaskIndex desired_task = FlightTaskIndex::None;

	switch (command.command) {
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT :
		desired_task = FlightTaskIndex::Orbit;
		break;

	// ignore all unkown commands
	default : return;
	}

	// switch to the commanded task
	int switch_result = switchTask(desired_task);
	uint8_t cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

	// if we are in/switched to the desired task
	if (switch_result >= 0) {
		cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

		// if the task is running apply parameters to it and see if it rejects
		if (isAnyTaskActive() && !_current_task.task->applyCommandParameters(command)) {
			cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;

			// if we just switched and parameters are not accepted, go to failsafe
			if (switch_result == 1) {
				switchTask(FlightTaskIndex::Position);
				cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
			}
		}
	}

	// send back acknowledgment
	vehicle_command_ack_s command_ack = {};
	command_ack.command = command.command;
	command_ack.result = cmd_result;
	command_ack.result_param1 = switch_result;
	command_ack.target_system = command.source_system;
	command_ack.target_component = command.source_component;

	if (_pub_vehicle_command_ack == nullptr) {
		_pub_vehicle_command_ack = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
					   vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _pub_vehicle_command_ack, &command_ack);

	}
}
