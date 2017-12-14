#include "FlightTasks.hpp"

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

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
