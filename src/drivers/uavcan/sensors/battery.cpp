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

#include "battery.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_battery", "/dev/uavcan/battery", "/dev/battery", ORB_ID(battery_status)),
	_sub_battery(node)
{
}

int
UavcanBatteryBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_battery.start(BatteryInfoCbBinder(this, &UavcanBatteryBridge::battery_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBatteryBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg)
{
	battery_status_s battery{};

	battery.timestamp = hrt_absolute_time();
	battery.voltage_v = msg.voltage;
	// battery.voltage_filtered_v = msg.;
	battery.current_a = msg.current;
	// battery.current_filtered_a = msg.;
	// battery.average_current_a = msg.;
	// battery.discharged_mah = msg.;
	battery.remaining = msg.remaining_capacity_wh / msg.full_charge_capacity_wh; // between 0 and 1
	// battery.scale = msg.; // Power scaling factor, >= 1, or -1 if unknown
	battery.temperature = msg.temperature;
	// battery.cell_count = msg.;
	// battery.voltage_cell_v[4] = msg.;
	// battery.max_cell_voltage_delta = msg.;
	battery.capacity = msg.full_charge_capacity_wh;
	// battery.cycle_count = msg.;
	// battery.run_time_to_empty = msg.;
	// battery.average_time_to_empty = msg.;
	battery.serial_number = msg.model_instance_id;
	battery.connected = true;
	battery.system_source = msg.status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
	// battery.priority = msg.;
	// battery.is_powering_off = msg.;
	// battery.warning = msg.;

	publish(msg.getSrcNodeID().get(), &battery);
}
