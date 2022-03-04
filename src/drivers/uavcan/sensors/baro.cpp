/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include "baro.hpp"
#include <math.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>
#include <lib/geo/geo.h> // For CONSTANTS_*

const char *const UavcanBarometerBridge::NAME = "baro";

UavcanBarometerBridge::UavcanBarometerBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_baro", ORB_ID(sensor_baro)),
	_sub_air_pressure_data(node),
	_sub_air_temperature_data(node)
{ }

int UavcanBarometerBridge::init()
{
	int res = _sub_air_pressure_data.start(AirPressureCbBinder(this, &UavcanBarometerBridge::air_pressure_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_air_temperature_data.start(AirTemperatureCbBinder(this, &UavcanBarometerBridge::air_temperature_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanBarometerBridge::air_temperature_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &msg)
{
	if (msg.static_temperature >= 0.f) {
		_last_temperature_kelvin = msg.static_temperature;

	} else if (msg.static_temperature < 0) {
		// handle previous incorrect temperature conversion to Kelvin where 273 was subtracted instead of added (https://github.com/PX4/PX4-Autopilot/pull/19061)
		float temperature_c = msg.static_temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS;

		if (temperature_c > -40.f && temperature_c < 120.f) {
			_last_temperature_kelvin = temperature_c - CONSTANTS_ABSOLUTE_NULL_CELSIUS;
		}
	}
}

void UavcanBarometerBridge::air_pressure_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure> &msg)
{
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if (channel == nullptr) {
		// Something went wrong - no channel to publish on; return
		return;
	}

	// Cast our generic CDev pointer to the sensor-specific driver class
	uORB::PublicationMulti<sensor_baro_s> *baro = static_cast<uORB::PublicationMulti<sensor_baro_s> *>(channel->h_driver);

	if (baro == nullptr) {
		return;
	}

	DeviceId device_id{};
	device_id.devid_s.bus = 0;
	device_id.devid_s.bus_type = DeviceBusType_UAVCAN;

	device_id.devid_s.devtype = DRV_BARO_DEVTYPE_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	// publish
	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = timestamp_sample;
	sensor_baro.device_id = device_id.devid;
	sensor_baro.pressure = msg.static_pressure;

	if (PX4_ISFINITE(_last_temperature_kelvin) && (_last_temperature_kelvin >= 0.f)) {
		sensor_baro.temperature = _last_temperature_kelvin + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	} else {
		sensor_baro.temperature = NAN;
	}

	sensor_baro.error_count = 0;
	sensor_baro.timestamp = hrt_absolute_time();
	baro->publish(sensor_baro);
}

int UavcanBarometerBridge::init_driver(uavcan_bridge::Channel *channel)
{
	channel->h_driver = new uORB::PublicationMulti<sensor_baro_s>(ORB_ID(sensor_baro));

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	uORB::PublicationMulti<sensor_baro_s> *baro = static_cast<uORB::PublicationMulti<sensor_baro_s> *>(channel->h_driver);

	channel->instance = baro->get_instance();

	if (channel->instance < 0) {
		PX4_ERR("UavcanBaro: Unable to get an instance");
		delete baro;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
