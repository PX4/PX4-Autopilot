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

#include "accel.hpp"
#include "airspeed.hpp"
#include "baro.hpp"
#include "battery.hpp"
#include "differential_pressure.hpp"
#include "flow.hpp"
#include "gnss.hpp"
#include "gyro.hpp"
#include "hygrometer.hpp"
#include "ice_status.hpp"
#include "mag.hpp"
#include "rangefinder.hpp"
#include "safety_button.hpp"

/*
 * IUavcanSensorBridge
 */
void IUavcanSensorBridge::make_all(uavcan::INode &node, List<IUavcanSensorBridge *> &list)
{
	// airspeed
	int32_t uavcan_sub_aspd = 1;
	param_get(param_find("UAVCAN_SUB_ASPD"), &uavcan_sub_aspd);

	if (uavcan_sub_aspd != 0) {
		list.add(new UavcanAirspeedBridge(node));
	}

	// baro
	int32_t uavcan_sub_baro = 1;
	param_get(param_find("UAVCAN_SUB_BARO"), &uavcan_sub_baro);

	if (uavcan_sub_baro != 0) {
		list.add(new UavcanBarometerBridge(node));
	}

	// battery
	int32_t uavcan_sub_bat = 1;
	param_get(param_find("UAVCAN_SUB_BAT"), &uavcan_sub_bat);

	if (uavcan_sub_bat != 0) {
		list.add(new UavcanBatteryBridge(node));
	}

	// differential pressure
	int32_t uavcan_sub_dpres = 1;
	param_get(param_find("UAVCAN_SUB_DPRES"), &uavcan_sub_dpres);

	if (uavcan_sub_dpres != 0) {
		list.add(new UavcanDifferentialPressureBridge(node));
	}

	// flow
	int32_t uavcan_sub_flow = 1;
	param_get(param_find("UAVCAN_SUB_FLOW"), &uavcan_sub_flow);

	if (uavcan_sub_flow != 0) {
		list.add(new UavcanFlowBridge(node));
	}

	// GPS
	int32_t uavcan_sub_gps = 1;
	param_get(param_find("UAVCAN_SUB_GPS"), &uavcan_sub_gps);

	if (uavcan_sub_gps != 0) {
		list.add(new UavcanGnssBridge(node));
	}

	// hygrometer
	int32_t uavcan_sub_hygro = 1;
	param_get(param_find("UAVCAN_SUB_HYGRO"), &uavcan_sub_hygro);

	if (uavcan_sub_hygro != 0) {
		list.add(new UavcanHygrometerBridge(node));
	}

	// ice (internal combustion engine)
	int32_t uavcan_sub_ice = 1;
	param_get(param_find("UAVCAN_SUB_ICE"), &uavcan_sub_ice);

	if (uavcan_sub_ice != 0) {
		list.add(new UavcanIceStatusBridge(node));
	}

	// IMU
	int32_t uavcan_sub_imu = 1;
	param_get(param_find("UAVCAN_SUB_IMU"), &uavcan_sub_imu);

	if (uavcan_sub_imu != 0) {
		list.add(new UavcanAccelBridge(node));
		list.add(new UavcanGyroBridge(node));
	}

	// magnetometer
	int32_t uavcan_sub_mag = 1;
	param_get(param_find("UAVCAN_SUB_MAG"), &uavcan_sub_mag);

	if (uavcan_sub_mag != 0) {
		list.add(new UavcanMagnetometerBridge(node));
	}

	// range finder
	int32_t uavcan_sub_rng = 1;
	param_get(param_find("UAVCAN_SUB_RNG"), &uavcan_sub_rng);

	if (uavcan_sub_rng != 0) {
		list.add(new UavcanRangefinderBridge(node));
	}

	// safety button
	int32_t uavcan_sub_button = 1;
	param_get(param_find("UAVCAN_SUB_BTN"), &uavcan_sub_button);

	if (uavcan_sub_button != 0) {
		list.add(new UavcanSafetyButtonBridge(node));
	}
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
