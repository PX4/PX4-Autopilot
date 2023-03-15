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

#include <uavcan/equipment/esc/RawCommand.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>

namespace uavcannode
{

class ESCRawCommand;

typedef uavcan::MethodBinder<ESCRawCommand *,
	void (ESCRawCommand::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>&)>
	ESCRawCommandBinder;

class ESCRawCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::esc::RawCommand, ESCRawCommandBinder>
{
public:
	ESCRawCommand(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::esc::RawCommand::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::esc::RawCommand, ESCRawCommandBinder>(node)
	{}

	bool init()
	{
		if (start(ESCRawCommandBinder(this, &ESCRawCommand::callback)) < 0) {
			PX4_ERR("uavcan::equipment::esc::RawCommand subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::esc::RawCommand::getDataTypeFullName(),
		       uavcan::equipment::esc::RawCommand::DefaultDataTypeID,
		       _actuator_motors_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand> &msg)
	{

		actuator_motors_s actuator_motors;

		actuator_motors.timestamp = hrt_absolute_time();
		actuator_motors.timestamp_sample = actuator_motors.timestamp;

		for (unsigned i = 0; i < msg.cmd.size(); i++) {
			if (i >= actuator_motors_s::NUM_CONTROLS) {
				break;
			}

			// Normalized to -8192, 8191 in uavcan. actuator_motors is -1 to 1
			actuator_motors.control[i] = msg.cmd[i] / 8192.f;

		}

		_actuator_motors_pub.publish(actuator_motors);

	}

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
};
} // namespace uavcannode
