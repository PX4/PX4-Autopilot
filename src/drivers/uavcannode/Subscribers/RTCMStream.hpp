/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/device/Device.hpp>
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
		// Don't republish a message from ourselves
		if (msg.getSrcNodeID().get() != getNode().getNodeID().get()) {
			gps_inject_data_s gps_inject_data{};

			gps_inject_data.len = msg.data.size();

			memcpy(gps_inject_data.data, &msg.data[0], gps_inject_data.len);

			gps_inject_data.timestamp = hrt_absolute_time();

			union device::Device::DeviceId device_id;

			device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
			device_id.devid_s.address = msg.getSrcNodeID().get();
			device_id.devid_s.devtype = DRV_GPS_DEVTYPE_UAVCAN;

			gps_inject_data.device_id = device_id.devid;

			_gps_inject_data_pub.publish(gps_inject_data);
		}
	}

	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
};
} // namespace uavcannode
