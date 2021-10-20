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

#include "hygrometer.hpp"
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <parameters/param.h>
#include <systemlib/err.h>

const char *const UavcanHygrometerBridge::NAME = "hygrometer_sensor";

UavcanHygrometerBridge::UavcanHygrometerBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_hygrometer_sensor", ORB_ID(sensor_hygrometer)),
	_sub_hygro(node)
{
}

int UavcanHygrometerBridge::init()
{


	int res = _sub_hygro.start(HygroCbBinder(this, &UavcanHygrometerBridge::hygro_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanHygrometerBridge::hygro_sub_cb(const
		uavcan::ReceivedDataStructure<cuav::equipment::hygrometer::Hygrometer>
		&msg)
{


	float humidity = msg.humidity;
	float temperature_c = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS;
	uint8_t id = msg.id;

	sensor_hygrometer_s report = {
		.timestamp = hrt_absolute_time(),
		.temperature = temperature_c,
		.humidity  = humidity,
		.id  = id,
	};

	publish(msg.getSrcNodeID().get(), &report);
}
