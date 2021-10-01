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

#include <uavcan/equipment/gnss/RTCMStream.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/gps_inject_data.h>

namespace uavcannode
{

class RTCMStream;

typedef uavcan::MethodBinder<RTCMStream *,
	void (RTCMStream::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::RTCMStream>&)>
	RTCMStreamBinder;

class RTCMStream :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::gnss::RTCMStream, RTCMStreamBinder>
{
public:
	RTCMStream(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::gnss::RTCMStream::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::gnss::RTCMStream, RTCMStreamBinder>(node)
	{}

	bool init()
	{
		if (start(RTCMStreamBinder(this, &RTCMStream::callback)) < 0) {
			PX4_ERR("uavcan::equipment::gnss::RTCMStream subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::gnss::RTCMStream::getDataTypeFullName(),
		       uavcan::equipment::gnss::RTCMStream::DefaultDataTypeID,
		       _gps_inject_data_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::RTCMStream> &msg)
	{
		gps_inject_data_s gps_inject_data{};

		gps_inject_data.len = msg.data.size();

		//gps_inject_data.flags = gps_rtcm_data_msg.flags;
		memcpy(gps_inject_data.data, &msg.data[0], gps_inject_data.len);

		_gps_inject_data_pub.publish(gps_inject_data);
	}

	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
};
} // namespace uavcannode
