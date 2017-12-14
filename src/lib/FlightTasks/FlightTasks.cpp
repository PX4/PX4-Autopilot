#include "FlightTasks.hpp"

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

bool FlightTasks::update()
{
	_updateCommand();

	if (isAnyTaskActive()) {
		_subscription_array.update();
		return _current_task->updateInitialize() && _current_task->update();
	}

	return false;
}

int FlightTasks::switchTask(int task_number)
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

bool FlightTasks::_updateCommand()
{
	/* lazy subscription to command topic */
	if (_sub_vehicle_command < 0) {
		_sub_vehicle_command = orb_subscribe(ORB_ID(vehicle_command));
	}

	/* check if there's any new command */
	bool updated = false;
	orb_check(_sub_vehicle_command, &updated);

	if (!updated) {
		return false;
	}

	/* check if command is for flight task library */
	struct vehicle_command_s command;
	orb_copy(ORB_ID(vehicle_command), _sub_vehicle_command, &command);

	if (!(command.command == vehicle_command_s::VEHICLE_CMD_FLIGHT_TASK)) {
		return false;
	}

	/* evaluate command */
	uint8_t switch_result = switchTask(int(command.param1));
	uint8_t cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

	/* switch succeeded */
	if (!switch_result) {
		cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

		/* if the correct task is running apply parameters to it and see if it rejects */
		if (isAnyTaskActive()) {
			if (!_current_task->applyCommandParameters(command)) {
				cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
			}
		}
	}

	/* send back acknowledgment */
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

	return true;
}
