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

MavlinkCommandsStream::MavlinkCommandsStream(Mavlink *mavlink, mavlink_channel_t channel) : _channel(channel)
{
	_cmd_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_command));
	_cmd = (struct vehicle_command_s *)_cmd_sub->get_data();
}

MavlinkCommandsStream::~MavlinkCommandsStream()
{
}

void
MavlinkCommandsStream::update(const hrt_abstime t)
{
	if (_cmd_sub->update(t)) {
		/* only send commands for other systems/components */
		if (_cmd->target_system != mavlink_system.sysid || _cmd->target_component != mavlink_system.compid) {
			mavlink_msg_command_long_send(_channel,
						      _cmd->target_system,
						      _cmd->target_component,
						      _cmd->command,
						      _cmd->confirmation,
						      _cmd->param1,
						      _cmd->param2,
						      _cmd->param3,
						      _cmd->param4,
						      _cmd->param5,
						      _cmd->param6,
						      _cmd->param7);
		}
	}
}
