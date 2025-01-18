/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

#include "sensor_bridge.hpp"
#include <cassert>

#if defined(CONFIG_UAVCAN_SENSOR_ACCEL)
#include "accel.hpp"
#include "gyro.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_AIRSPEED)
#include "airspeed.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_BARO)
#include "baro.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_BATTERY)
#include "battery.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_DIFFERENTIAL_PRESSURE)
#include "differential_pressure.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_FLOW)
#include "flow.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_FUEL_TANK_STATUS)
#include "fuel_tank_status.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_GNSS)
#include "gnss.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_GNSS_RELATIVE)
#include "gnss_relative.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_HYGROMETER)
#include "hygrometer.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_ICE_STATUS)
#include "ice_status.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_MAG)
#include "mag.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_RANGEFINDER)
#include "rangefinder.hpp"
#endif
#if defined(CONFIG_UAVCAN_SENSOR_SAFETY_BUTTON)
#include "safety_button.hpp"
#endif

/*
 * IUavcanSensorBridge
 */
void IUavcanSensorBridge::make_all(uavcan::INode &node, List<IUavcanSensorBridge *> &list)
{
	// airspeed
#if defined(CONFIG_UAVCAN_SENSOR_AIRSPEED)
	int32_t uavcan_sub_aspd = 1;
	param_get(param_find("UAVCAN_SUB_ASPD"), &uavcan_sub_aspd);

	if (uavcan_sub_aspd != 0) {
		list.add(new UavcanAirspeedBridge(node));
	}

#endif

#if defined(CONFIG_UAVCAN_SENSOR_BARO)
	// baro
	int32_t uavcan_sub_baro = 1;
	param_get(param_find("UAVCAN_SUB_BARO"), &uavcan_sub_baro);

	if (uavcan_sub_baro != 0) {
		list.add(new UavcanBarometerBridge(node));
	}

#endif

	// battery
#if defined(CONFIG_UAVCAN_SENSOR_BATTERY)
	int32_t uavcan_sub_bat = 1;
	param_get(param_find("UAVCAN_SUB_BAT"), &uavcan_sub_bat);

	if (uavcan_sub_bat != 0) {
		list.add(new UavcanBatteryBridge(node));
	}

#endif

	// differential pressure
#if defined(CONFIG_UAVCAN_SENSOR_DIFFERENTIAL_PRESSURE)
	int32_t uavcan_sub_dpres = 1;
	param_get(param_find("UAVCAN_SUB_DPRES"), &uavcan_sub_dpres);

	if (uavcan_sub_dpres != 0) {
		list.add(new UavcanDifferentialPressureBridge(node));
	}

#endif

	// flow
#if defined(CONFIG_UAVCAN_SENSOR_FLOW)
	int32_t uavcan_sub_flow = 1;
	param_get(param_find("UAVCAN_SUB_FLOW"), &uavcan_sub_flow);

	if (uavcan_sub_flow != 0) {
		list.add(new UavcanFlowBridge(node));
	}

#endif

	// fuel tank
#if defined(CONFIG_UAVCAN_SENSOR_FUEL_TANK_STATUS)
	int32_t uavcan_sub_fuel_tank = 1;
	param_get(param_find("UAVCAN_SUB_FUEL"), &uavcan_sub_fuel_tank);

	if (uavcan_sub_fuel_tank != 0) {
		list.add(new UavcanFuelTankStatusBridge(node));
	}

#endif

	// GPS
#if defined(CONFIG_UAVCAN_SENSOR_GNSS)
	int32_t uavcan_sub_gps = 1;
	param_get(param_find("UAVCAN_SUB_GPS"), &uavcan_sub_gps);

	if (uavcan_sub_gps != 0) {
		list.add(new UavcanGnssBridge(node));
	}

#endif

	// GPS relative
#if defined(CONFIG_UAVCAN_SENSOR_GNSS_RELATIVE)
	int32_t uavcan_sub_gps_rel = 1;
	param_get(param_find("UAVCAN_SUB_GPS_R"), &uavcan_sub_gps_rel);

	if (uavcan_sub_gps_rel != 0) {
		list.add(new UavcanGnssRelativeBridge(node));
	}

#endif

	// hygrometer
#if defined(CONFIG_UAVCAN_SENSOR_HYGROMETER)
	int32_t uavcan_sub_hygro = 1;
	param_get(param_find("UAVCAN_SUB_HYGRO"), &uavcan_sub_hygro);

	if (uavcan_sub_hygro != 0) {
		list.add(new UavcanHygrometerBridge(node));
	}

#endif

	// ice (internal combustion engine)
#if defined(CONFIG_UAVCAN_SENSOR_ICE_STATUS)
	int32_t uavcan_sub_ice = 1;
	param_get(param_find("UAVCAN_SUB_ICE"), &uavcan_sub_ice);

	if (uavcan_sub_ice != 0) {
		list.add(new UavcanIceStatusBridge(node));
	}

#endif

	// IMU
#if defined(CONFIG_UAVCAN_SENSOR_ACCEL)
	int32_t uavcan_sub_imu = 1;
	param_get(param_find("UAVCAN_SUB_IMU"), &uavcan_sub_imu);

	if (uavcan_sub_imu != 0) {
		list.add(new UavcanAccelBridge(node));
		list.add(new UavcanGyroBridge(node));
	}

#endif

	// magnetometer
#if defined(CONFIG_UAVCAN_SENSOR_MAG)
	int32_t uavcan_sub_mag = 1;
	param_get(param_find("UAVCAN_SUB_MAG"), &uavcan_sub_mag);

	if (uavcan_sub_mag != 0) {
		list.add(new UavcanMagnetometerBridge(node));
	}

#endif

	// range finder
#if defined(CONFIG_UAVCAN_SENSOR_RANGEFINDER)
	int32_t uavcan_sub_rng = 1;
	param_get(param_find("UAVCAN_SUB_RNG"), &uavcan_sub_rng);

	if (uavcan_sub_rng != 0) {
		list.add(new UavcanRangefinderBridge(node));
	}

#endif

	// safety button

#if defined(CONFIG_UAVCAN_SENSOR_SAFETY_BUTTON)
	int32_t uavcan_sub_button = 1;
	param_get(param_find("UAVCAN_SUB_BTN"), &uavcan_sub_button);

	if (uavcan_sub_button != 0) {
		list.add(new UavcanSafetyButtonBridge(node));
	}

#endif
}

/*
 * UavcanSensorBridgeBase
 */
UavcanSensorBridgeBase::~UavcanSensorBridgeBase()
{
	delete [] _channels;
}

void
UavcanSensorBridgeBase::publish(const int node_id, const void *report)
{
	assert(report != nullptr);

	uavcan_bridge::Channel *channel = nullptr;

	// Checking if such channel already exists
	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id == node_id) {
			channel = _channels + i;
			break;
		}
	}

	// No such channel - try to create one
	if (channel == nullptr) {
		if (_out_of_channels) {
			return;  // Give up immediately - saves some CPU time
		}

		DEVICE_LOG("adding channel for topic %s node %d...", _orb_topic->o_name, node_id);

		// Search for the first free channel
		for (unsigned i = 0; i < _max_channels; i++) {
			if (_channels[i].node_id < 0) {
				channel = _channels + i;
				break;
			}
		}

		// No free channels left
		if (channel == nullptr) {
			_out_of_channels = true;
			DEVICE_LOG("out of channels");
			return;
		}

		// update device id as we now know our device node_id
		_device_id.devid_s.address = static_cast<uint8_t>(node_id);
		_device_id.devid_s.bus_type = DeviceBusType::DeviceBusType_UAVCAN;

		// Publish to the appropriate topic, abort on failure
		channel->orb_advert = orb_advertise_multi(_orb_topic, report, &channel->instance);

		channel->node_id = node_id;
		DEVICE_LOG("node %d instance %d ok", channel->node_id, channel->instance);

		if (channel->orb_advert == nullptr) {
			DEVICE_LOG("uORB advertise failed. Out of instances?");
			*channel = uavcan_bridge::Channel();
			_out_of_channels = true;
			return;
		}

		DEVICE_LOG("node %d topic %s instance %d ok", channel->node_id, _orb_topic->o_name, channel->instance);
	}

	assert(channel != nullptr);

	(void)orb_publish(_orb_topic, channel->orb_advert, report);
}

uavcan_bridge::Channel *UavcanSensorBridgeBase::get_channel_for_node(int node_id)
{
	uavcan_bridge::Channel *channel = nullptr;

	// Checking if such channel already exists
	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id == node_id) {
			channel = _channels + i;
			break;
		}
	}

	// No such channel - try to create one
	if (channel == nullptr) {
		if (_out_of_channels) {
			// We already determined we're out of class or uORB instances
			return channel;
		}

		DEVICE_LOG("adding channel for topic %s node %d...", _orb_topic->o_name, node_id);

		// Search for the first free channel
		for (unsigned i = 0; i < _max_channels; i++) {
			if (_channels[i].node_id < 0) {
				channel = _channels + i;
				break;
			}
		}

		// No free channels left
		if (channel == nullptr) {
			_out_of_channels = true;
			DEVICE_LOG("out of channels");
			return channel;
		}

		// initialize the driver, which registers the class device name and uORB publisher
		channel->node_id = node_id;
		int ret = init_driver(channel);

		if (ret != PX4_OK) {
			// Driver initialization failed - probably out of channels.  Return nullptr so
			// the callback exits gracefully, and clear the assigned node_id for the channel
			// so future callbacks exit immediately.
			DEVICE_LOG("INIT ERROR node %d errno %d", channel->node_id, ret);
			channel->node_id = -1;
			_out_of_channels = true;
			return nullptr;
		}

		DEVICE_LOG("channel %d instance %d ok", channel->node_id, channel->instance);
	}

	return channel;
}

unsigned UavcanSensorBridgeBase::get_num_redundant_channels() const
{
	unsigned out = 0;

	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id >= 0) {
			out += 1;
		}
	}

	return out;
}

int8_t UavcanSensorBridgeBase::get_channel_index_for_node(int node_id)
{
	int8_t ch = -1;

	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id == node_id) {
			ch = i;
			break;
		}
	}

	return ch;
}

void UavcanSensorBridgeBase::print_status() const
{
	printf("name: %s\n", _name);

	for (unsigned i = 0; i < _max_channels; i++) {
		if (_channels[i].node_id >= 0) {
			printf("channel %d: node id %d --> instance %d\n",
			       i, _channels[i].node_id, _channels[i].instance);
		}
	}
}
