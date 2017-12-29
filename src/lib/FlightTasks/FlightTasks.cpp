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

const vehicle_local_position_setpoint_s &FlightTasks::getPositionSetpoint()
{
	if (isAnyTaskActive()) {
		return _current_task->getPositionSetpoint();

	} else {
		return FlightTask::empty_setpoint;
	}
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

	case 2:
		_current_task = new (&_task_union.sport) FlightTaskSport(this, "SPO");
		break;

	case 3:
		_current_task = new (&_task_union.altitude) FlightTaskManualAltitude(this, "MANALT");
		break;

	case 4:
		_current_task = new (&_task_union.position) FlightTaskManualPosition(this, "MANPOS");
		break;

	case 5:
		_current_task = new (&_task_union.stabilized) FlightTaskManualStabilized(this, "MANSTAB");
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
	return 1;
}

void FlightTasks::_updateCommand()
{
	/* lazy subscription to command topic */
	if (_sub_vehicle_command < 0) {
		_sub_vehicle_command = orb_subscribe(ORB_ID(vehicle_command));
	}

	/* check if there's any new command */
	bool updated = false;
	orb_check(_sub_vehicle_command, &updated);

	if (!updated) {
		return;
	}

	/* check if command is for flight task library */
	struct vehicle_command_s command;
	orb_copy(ORB_ID(vehicle_command), _sub_vehicle_command, &command);

	if (command.command != vehicle_command_s::VEHICLE_CMD_FLIGHT_TASK) {
		return;
	}

	/* evaluate command */
	int switch_result = switchTask(int(command.param1 + 0.5f));
	uint8_t cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

	/* if we are in the desired task */
	if (switch_result >= 0) {
		cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

		/* if the task is running apply parameters to it and see if it rejects */
		if (isAnyTaskActive() && !_current_task->applyCommandParameters(command)) {
			cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;

			/* if we just switched and parameters are not accepted, go to failsafe */
			if (switch_result == 1) {
				switchTask(-1);
				cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
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
}
