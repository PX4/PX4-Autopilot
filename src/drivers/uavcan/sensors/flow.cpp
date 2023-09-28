/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "flow.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanFlowBridge::NAME = "flow";

UavcanFlowBridge::UavcanFlowBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_flow", ORB_ID(sensor_optical_flow)),
	_sub_flow(node)
{
}

int
UavcanFlowBridge::init()
{
	int res = _sub_flow.start(FlowCbBinder(this, &UavcanFlowBridge::flow_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanFlowBridge::flow_sub_cb(const uavcan::ReceivedDataStructure<com::hex::equipment::flow::Measurement> &msg)
{
	sensor_optical_flow_s flow{};
	flow.timestamp_sample = hrt_absolute_time(); // TODO

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
	device_id.devid_s.bus = 0;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_UAVCAN;
	device_id.devid_s.address = msg.getSrcNodeID().get() & 0xFF;

	flow.device_id = device_id.devid;

	flow.pixel_flow[0] = msg.flow_integral[0];
	flow.pixel_flow[1] = msg.flow_integral[1];

	flow.integration_timespan_us = 1.e6f * msg.integration_interval; // s -> us

	flow.quality = msg.quality;

	if (PX4_ISFINITE(msg.rate_gyro_integral[0]) && PX4_ISFINITE(msg.rate_gyro_integral[1])) {
		flow.delta_angle[0] = msg.rate_gyro_integral[0];
		flow.delta_angle[1] = msg.rate_gyro_integral[1];
		flow.delta_angle[2] = NAN;
		flow.delta_angle_available = true;

	} else {
		flow.delta_angle[0] = NAN;
		flow.delta_angle[1] = NAN;
		flow.delta_angle[2] = NAN;
	}

	flow.max_flow_rate = NAN;
	flow.min_ground_distance = NAN;
	flow.max_ground_distance = NAN;

	flow.timestamp = hrt_absolute_time();

	publish(msg.getSrcNodeID().get(), &flow);
}
