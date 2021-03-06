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

#include <lib/drivers/barometer/PX4Barometer.hpp>
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
	last_temperature_kelvin = msg.static_temperature;
}

void UavcanBarometerBridge::air_pressure_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure> &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if (channel == nullptr) {
		// Something went wrong - no channel to publish on; return
		return;
	}

	// Cast our generic CDev pointer to the sensor-specific driver class
	PX4Barometer *baro = (PX4Barometer *)channel->h_driver;

	if (baro == nullptr) {
		return;
	}

	baro->set_temperature(last_temperature_kelvin + CONSTANTS_ABSOLUTE_NULL_CELSIUS);
	baro->update(hrt_absolute_time(), msg.static_pressure / 100.0f); // Convert pressure to millibar
}

int UavcanBarometerBridge::init_driver(uavcan_bridge::Channel *channel)
{
	// update device id as we now know our device node_id
	DeviceId device_id{_device_id};

	device_id.devid_s.devtype = DRV_BARO_DEVTYPE_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	channel->h_driver = new PX4Barometer(device_id.devid);

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	PX4Barometer *baro = (PX4Barometer *)channel->h_driver;

	channel->instance = baro->get_instance();

	if (channel->instance < 0) {
		PX4_ERR("UavcanBaro: Unable to get an instance");
		delete baro;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
