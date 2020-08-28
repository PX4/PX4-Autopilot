/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include "analog_measurement.hpp"

const char *const UavcanAnalogMeasurementBridge::NAME = "analog_measurement";

UavcanAnalogMeasurementBridge::UavcanAnalogMeasurementBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_airspeed", "/dev/uavcan/analog_measurement", "/dev/analog_measurement",
				   ORB_ID(analog_measurement)),
	_sub_analog_data(node)
{ }

int UavcanAnalogMeasurementBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_analog_data.start(AnalogCbBinder(this, &UavcanAnalogMeasurementBridge::analog_measurement_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanAnalogMeasurementBridge::analog_measurement_sub_cb(const
		uavcan::ReceivedDataStructure<com::volansi::equipment::adc::AnalogMeasurement> &msg)
{
	analog_measurement_s report{};

	int node_id = msg.getSrcNodeID().get();
	report.id = node_id;

	int numIndices = msg.values.size();

	for (int i = 0; i < numIndices; i++) {
		report.values[i] = msg.values[i];
		report.unit_type[i] = msg.unit_type[i];
	}

	publish(node_id, &report);
}
