/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file fuel_tank_status.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 */

#include "fuel_tank_status.hpp"

#include <parameters/param.h>

const char *const UavcanFuelTankStatusBridge::NAME = "fuel_tank_status";

UavcanFuelTankStatusBridge::UavcanFuelTankStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_fuel_tank_status", ORB_ID(fuel_tank_status)),
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

	// Fetch maximum fuel capacity (in liters)
	param_get(param_find("UAVCAN_ECU_MAXF"), &_max_fuel_capacity);

	// Fetching fuel type
	param_get(param_find("UAVCAN_ECU_FUELT"), &_fuel_type);

	return 0;
}

void UavcanFuelTankStatusBridge::fuel_tank_status_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ice::FuelTankStatus> &msg)
{
	auto report = ::fuel_tank_status_s();
	report.timestamp = hrt_absolute_time();
	report.maximum_fuel_capacity = _max_fuel_capacity * 1000.0f; // convert to ml
	report.fuel_type = static_cast<uint8_t>(_fuel_type);
	report.consumed_fuel = NAN; // only the remaining fuel is measured
	report.fuel_consumption_rate = msg.fuel_consumption_rate_cm3pm / 60.0f; // convert to ml/s
	report.percent_remaining = msg.available_fuel_volume_percent;
	report.remaining_fuel = msg.available_fuel_volume_cm3;
	report.fuel_tank_id = msg.fuel_tank_id;

	// Optional temperature field, in Kelvin, set to NaN if not provided.
	report.temperature = !PX4_ISFINITE(msg.fuel_temperature) ? NAN : msg.fuel_temperature;

	publish(msg.getSrcNodeID().get(), &report);
}

int UavcanFuelTankStatusBridge::init_driver(uavcan_bridge::Channel *channel)
{
	return PX4_OK;
}
