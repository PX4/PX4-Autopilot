/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <uavcan/equipment/hardpoint/Command.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_servos.h>

namespace uavcannode
{

class HardpointCommand;

typedef uavcan::MethodBinder<HardpointCommand *,
	void (HardpointCommand::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::hardpoint::Command>&)>
	HardpointCommandBinder;

class HardpointCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::hardpoint::Command, HardpointCommandBinder>
{
public:
	HardpointCommand(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::hardpoint::Command::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::hardpoint::Command, HardpointCommandBinder>(node)
	{}

	bool init()
	{
		if (start(HardpointCommandBinder(this, &HardpointCommand::callback)) < 0) {
			PX4_ERR("uavcan::equipment::hardpoint::Command subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::hardpoint::Command::getDataTypeFullName(),
		       uavcan::equipment::hardpoint::Command::DefaultDataTypeID,
		       _actuator_servos_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::hardpoint::Command> &msg)
	{

		uint8_t servo_id = msg.hardpoint_id;
		actuator_servos_s actuator_servos;

		if (servo_id >= actuator_servos_s::NUM_CONTROLS) {
			return;
		}

		// If hardpoint_id is set to 0, that is a broadcast
		if (servo_id == 0) {
			for (uint8_t i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
				actuator_servos.timestamp = hrt_absolute_time();
				actuator_servos.timestamp_sample = actuator_servos.timestamp;

				if (msg.command == 1) {
					actuator_servos.control[i] = 1; // grip

				} else if (msg.command == 0) {
					actuator_servos.control[i] = -1; // release

				} else {
					actuator_servos.control[i] = 0; // do nothing

				} // end else

			} // end for

		} else {
			actuator_servos.timestamp = hrt_absolute_time();
			actuator_servos.timestamp_sample = actuator_servos.timestamp;

			if (msg.command == 1) {
				actuator_servos.control[servo_id] = 1; // grip

			} else if (msg.command == 0) {
				actuator_servos.control[servo_id] = -1; // release

			} else {
				actuator_servos.control[servo_id] = 0; // do nothing

			} // end else

		} // end else

		_actuator_servos_pub.publish(actuator_servos);

	}

	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

};
} // namespace uavcannode
