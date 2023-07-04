/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include "UavcanSubscriberBase.hpp"

#include <uavcan/equipment/actuator/ArrayCommand.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_servos.h>

namespace uavcannode
{

class ServoArrayCommand;

typedef uavcan::MethodBinder<ServoArrayCommand *,
	void (ServoArrayCommand::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>&)>
	ServoArrayCommandBinder;

class ServoArrayCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand, ServoArrayCommandBinder>
{
public:
	ServoArrayCommand(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::actuator::ArrayCommand::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand, ServoArrayCommandBinder>(node)
	{}

	bool init()
	{
		if (start(ServoArrayCommandBinder(this, &ServoArrayCommand::callback)) < 0) {
			PX4_ERR("uavcan::equipment::actuator::ArrayCommand subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::actuator::ArrayCommand::getDataTypeFullName(),
		       uavcan::equipment::actuator::ArrayCommand::DefaultDataTypeID,
		       _actuator_servos_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand> &msg)
	{

		actuator_servos_s actuator_servos;

		actuator_servos.timestamp = hrt_absolute_time();
		actuator_servos.timestamp_sample = actuator_servos.timestamp;

		for (unsigned i = 0; i < msg.commands.size(); i++) {
			if (i >= actuator_servos_s::NUM_CONTROLS) {
				break;
			}

			if (msg.commands[i].command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS) {
				actuator_servos.control[i] = msg.commands[i].command_value; // -1.0 to +1.0

			} else {
				actuator_servos.control[i] = NAN; // disarmed
			}
		}

		_actuator_servos_pub.publish(actuator_servos);

	}

	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
};
} // namespace uavcannode
