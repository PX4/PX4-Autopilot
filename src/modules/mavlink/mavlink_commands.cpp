/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_commands.cpp
 * Mavlink commands stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_commands.h"

MavlinkCommandsStream::MavlinkCommandsStream(Mavlink *mavlink, mavlink_channel_t channel) : _channel(channel), _cmd_time(0)
{
	_cmd_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_command));
}

void
MavlinkCommandsStream::update(const hrt_abstime t)
{
	struct vehicle_command_s cmd;

	if (_cmd_sub->update(&_cmd_time, &cmd)) {
		/* only send commands for other systems/components */
		if (cmd.target_system != mavlink_system.sysid || cmd.target_component != mavlink_system.compid) {
			mavlink_msg_command_long_send(_channel,
						      cmd.target_system,
						      cmd.target_component,
						      cmd.command,
						      cmd.confirmation,
						      cmd.param1,
						      cmd.param2,
						      cmd.param3,
						      cmd.param4,
						      cmd.param5,
						      cmd.param6,
						      cmd.param7);
		}
	}
}
