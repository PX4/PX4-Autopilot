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

#pragma once

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <ardupilot/equipment/power/BatteryInfoAux.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/battery_status.h>
#include <lib/atmosphere/atmosphere.h>

namespace uavcannode
{

class BatteryInfo :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::power::BatteryInfo>,
	private uavcan::Publisher<ardupilot::equipment::power::BatteryInfoAux>
{
public:
	BatteryInfo(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::power::BatteryInfo::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(battery_status)),
		uavcan::Publisher<uavcan::equipment::power::BatteryInfo>(node),
		uavcan::Publisher<ardupilot::equipment::power::BatteryInfoAux>(node)
	{
		uavcan::Publisher<uavcan::equipment::power::BatteryInfo>::setPriority(uavcan::TransferPriority::MiddleLower);
		uavcan::Publisher<ardupilot::equipment::power::BatteryInfoAux>::setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::power::BatteryInfo::getDataTypeFullName(),
			       uavcan::equipment::power::BatteryInfo::DefaultDataTypeID);
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::equipment::power::BatteryInfoAux::getDataTypeFullName(),
			       ardupilot::equipment::power::BatteryInfoAux::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// battery_status -> uavcan::equipment::power::BatteryInfo & ardupilot::equipment::power::BatteryInfoAux
		battery_status_s battery;

		if (uORB::SubscriptionCallbackWorkItem::update(&battery)) {
			ardupilot::equipment::power::BatteryInfoAux battery_info_aux{};

			battery_info_aux.timestamp.usec = battery.timestamp;

			for (uint8_t i = 0; i < battery.cell_count && i < arraySize(battery_status_s::voltage_cell_v); i++) {
				battery_info_aux.voltage_cell.push_back(battery.voltage_cell_v[i]);
			}

			battery_info_aux.cycle_count = battery.cycle_count;
			battery_info_aux.over_discharge_count = battery.over_discharge_count;
			battery_info_aux.max_current = battery.current_a;
			battery_info_aux.nominal_voltage = battery.nominal_voltage;
			battery_info_aux.is_powering_off = battery.is_powering_off;
			battery_info_aux.battery_id = battery.id;

			uavcan::Publisher<ardupilot::equipment::power::BatteryInfoAux>::broadcast(battery_info_aux);

			uavcan::equipment::power::BatteryInfo battery_info{};
			battery_info.voltage = battery.voltage_v;
			battery_info.current = battery.current_a;
			battery_info.temperature = battery.temperature - atmosphere::kAbsoluteNullCelsius; // convert from C to K
			battery_info.full_charge_capacity_wh = battery.full_charge_capacity_wh;
			battery_info.remaining_capacity_wh = battery.remaining_capacity_wh;
			battery_info.state_of_charge_pct = battery.remaining * 100.0f;
			battery_info.state_of_charge_pct_stdev = battery.max_error;
			battery_info.battery_id = battery.id;
			battery_info.model_instance_id = 0; // TODO: what goes here?
			battery_info.model_name = "PX4 CAN Battery";

			// State of health
			battery_info.state_of_health_pct = battery.state_of_health;

			uint8_t status_flags = 0;

			if (battery.connected) {
				status_flags |= uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
			}

			if (battery.warning == battery_status_s::STATE_CHARGING) {
				status_flags |= uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGING;
			}

			battery_info.status_flags = status_flags;

			uavcan::Publisher<uavcan::equipment::power::BatteryInfo>::broadcast(battery_info);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcan
