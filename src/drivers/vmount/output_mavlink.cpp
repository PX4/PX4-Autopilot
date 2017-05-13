/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file output_mavlink.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_mavlink.h"

#include <uORB/topics/vehicle_command.h>
#include <px4_defines.h>


namespace vmount
{

OutputMavlink::OutputMavlink(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputMavlink::update(const ControlData *control_data)
{
	vehicle_command_s vehicle_command;

	if (control_data) {
		//got new command
		_set_angle_setpoints(control_data);

		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
		vehicle_command.target_system = _config.mavlink_sys_id;
		vehicle_command.target_component = _config.mavlink_comp_id;

		if (control_data->type == ControlData::Type::Neutral) {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;

		} else {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
		}

		if (_vehicle_command_pub) {
			orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &vehicle_command);

		} else {
			_vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vehicle_command,
					       vehicle_command_s::ORB_QUEUE_LENGTH);
		}

	}

	if (!_vehicle_command_pub) {
		return 0;
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_output_angles(t);

	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
	vehicle_command.target_system = _config.mavlink_sys_id;
	vehicle_command.target_component = _config.mavlink_comp_id;

	vehicle_command.param1 = _angle_outputs[0];
	vehicle_command.param2 = _angle_outputs[1];
	vehicle_command.param3 = _angle_outputs[2];

	orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &vehicle_command);

	_last_update = t;

	return 0;
}

void OutputMavlink::print_status()
{
	PX4_INFO("Output: Mavlink");
}

} /* namespace vmount */

