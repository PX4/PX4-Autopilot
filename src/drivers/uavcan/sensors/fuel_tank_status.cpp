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

/**
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "fuel_tank_status.hpp"
#include <uORB/topics/battery_status.h>

const char *const UavcanFuelTankStatusBridge::NAME = "fuel_tank_status";

UavcanFuelTankStatusBridge::UavcanFuelTankStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_fuel_tank_status", ORB_ID(battery_status)),
	_sub_fuel_tank_status_data(node)
{ }

int UavcanFuelTankStatusBridge::init()
{
	int res = _sub_fuel_tank_status_data.start(FuelTankStatusCbBinder(this,
			&UavcanFuelTankStatusBridge::fuel_tank_status_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanFuelTankStatusBridge::fuel_tank_status_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ice::FuelTankStatus> &msg)
{
	battery_status_s battery{};

	battery.remaining = msg.available_fuel_volume_percent * 0.01;
	battery.id = msg.getSrcNodeID().get();
	battery.timestamp = hrt_absolute_time();

	// Connected = false because it's temporary solution
	battery.connected = false;

	// Fill other fields by default
	battery.voltage_v = 0.0;
	battery.voltage_filtered_v = 0.0;
	battery.current_a = 0.0;
	battery.current_filtered_a = 0.01;
	battery.current_average_a = 0.0;
	battery.discharged_mah = 0.0;
	battery.scale = 1.0;
	battery.temperature = msg.fuel_temperature;
	battery.cell_count = static_cast<int32_t>(1);
	battery.source = 0;
	battery.priority = 0;
	battery.capacity = 0;
	battery.warning = 0;
	battery.voltage_cell_v[0] = 0.0;

	publish(msg.getSrcNodeID().get(), &battery);
}

int UavcanFuelTankStatusBridge::init_driver(uavcan_bridge::Channel *channel)
{
	return PX4_OK;
}
