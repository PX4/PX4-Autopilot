/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include <lib/atmosphere/atmosphere.h>
#include <px4_defines.h>
#include <px4_platform_common/log.h>

const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_battery(node),
	_sub_battery_aux(node),
	_sub_cbat(node),
	_warning(battery_status_s::WARNING_NONE),
	_last_timestamp(0)
{
}

int UavcanBatteryBridge::init()
{
	int32_t uavcan_sub_bat = 1;
	param_get(param_find("UAVCAN_SUB_BAT"), &uavcan_sub_bat);

	for (uint8_t instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {

		if (uavcan_sub_bat == FILTER_DATA) {
			_batt_update_mod[instance] = BatteryDataType::Filter;

		} else {
			_batt_update_mod[instance] = BatteryDataType::Raw;
		}
	}

	int res = _sub_battery.start(BatteryInfoCbBinder(this, &UavcanBatteryBridge::battery_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_battery_aux.start(BatteryInfoAuxCbBinder(this, &UavcanBatteryBridge::battery_aux_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_cbat.start(CBATCbBinder(this, &UavcanBatteryBridge::cbat_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBatteryBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get() || _battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::CBAT) {
		return;
	}

	if (_batt_update_mod[instance] == BatteryDataType::Filter) {

		filterData(msg, instance);
		return;
	}

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].voltage_v = msg.voltage;
	_battery_status[instance].current_a = msg.current;
	_battery_status[instance].current_average_a = msg.current;

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		sumDischarged(_battery_status[instance].timestamp, _battery_status[instance].current_a);
		_battery_status[instance].discharged_mah = _discharged_mah;
		_battery_status[instance].time_remaining_s = NAN;
	}

	_battery_status[instance].remaining = msg.state_of_charge_pct / 100.0f; // between 0 and 1
	_battery_status[instance].scale = -1.f;
	_battery_status[instance].temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
	// _battery_status[instance].cell_count = msg.;
	_battery_status[instance].connected = true;
	_battery_status[instance].source = msg.status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
	// _battery_status[instance].priority = msg.;
	_battery_status[instance].capacity = msg.full_charge_capacity_wh;
	_battery_status[instance].full_charge_capacity_wh = msg.full_charge_capacity_wh;
	_battery_status[instance].remaining_capacity_wh = msg.remaining_capacity_wh;
	// _battery_status[instance].cycle_count = msg.;
	// _battery_status[instance].average_time_to_empty = msg.;
	_battery_status[instance].id = msg.getSrcNodeID().get();

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		// Mavlink 2 needs individual cell voltages or cell[0] if cell voltages are not available.
		_battery_status[instance].voltage_cell_v[0] = msg.voltage;

		// Set cell count to 1 so the the battery code in mavlink_messages.cpp copies the values correctly (hack?)
		_battery_status[instance].cell_count = 1;
	}

	// _battery_status[instance].max_cell_voltage_delta = msg.;

	// _battery_status[instance].is_powering_off = msg.;

	determineWarning(_battery_status[instance].remaining);
	_battery_status[instance].warning = _warning;

	_battery_info[instance].timestamp = _battery_status[instance].timestamp;
	_battery_info[instance].id = _battery_status[instance].id;
	snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number), "%" PRIu32,
		 msg.model_instance_id);

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		publish(msg.getSrcNodeID().get(), &_battery_status[instance]);
		_battery_info_pub[instance].publish(_battery_info[instance]);
	}
}

void
UavcanBatteryBridge::battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux>
					&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get()) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter
	    || _batt_update_mod[instance] == BatteryDataType::CBAT) {
		return;
	}

	_batt_update_mod[instance] = BatteryDataType::RawAux;

	_battery_status[instance].discharged_mah = (_battery_status[instance].full_charge_capacity_wh -
			_battery_status[instance].remaining_capacity_wh) / msg.nominal_voltage *
			1000;
	_battery_status[instance].cell_count = math::min((uint8_t)msg.voltage_cell.size(), (uint8_t)14);
	_battery_status[instance].cycle_count = msg.cycle_count;
	_battery_status[instance].over_discharge_count = msg.over_discharge_count;
	_battery_status[instance].nominal_voltage = msg.nominal_voltage;
	_battery_status[instance].time_remaining_s = math::isZero(_battery_status[instance].current_a) ? NAN :
			(_battery_status[instance].remaining_capacity_wh /
			 _battery_status[instance].nominal_voltage / _battery_status[instance].current_a * 3600);
	_battery_status[instance].is_powering_off = msg.is_powering_off;

	for (uint8_t i = 0; i < _battery_status[instance].cell_count; i++) {
		_battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	// Publish the message once populated with the standard BatteryInfo data
	if (_battery_status[instance].timestamp != 0) {
		publish(msg.getSrcNodeID().get(), &_battery_status[instance]);
	}
}

void UavcanBatteryBridge::cbat_sub_cb(const uavcan::ReceivedDataStructure<cuav::equipment::power::CBAT> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get()) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter) {
		return;
	}

	// If CBAT message with superset of data was received, skip BatteryInfo messages
	_batt_update_mod[instance] = BatteryDataType::CBAT;

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].voltage_v = msg.voltage;
	_battery_status[instance].current_a = -msg.current; // discharge reported negative
	_battery_status[instance].current_average_a = -msg.average_current; // discharge reported negative
	_battery_status[instance].discharged_mah = msg.full_charge_capacity - msg.remaining_capacity; // mAh
	_battery_status[instance].remaining = msg.state_of_charge / 100.f;
	_battery_status[instance].scale = -1.f; // not supported, needs to be computed centrally
	_battery_status[instance].temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
	_battery_status[instance].full_charge_capacity_wh =
		msg.full_charge_capacity * msg.nominal_voltage / 1000.f; // mAh -> Wh
	_battery_status[instance].remaining_capacity_wh = msg.remaining_capacity * msg.nominal_voltage / 1000.f; // mAh -> Wh
	_battery_status[instance].nominal_voltage = msg.nominal_voltage;
	_battery_status[instance].capacity = msg.design_capacity; // mAh
	_battery_status[instance].cycle_count = msg.cycle_count;
	_battery_status[instance].average_time_to_empty = msg.average_time_to_empty;
	_battery_status[instance].manufacture_date = msg.manufacture_date;
	_battery_status[instance].state_of_health = msg.state_of_health;
	_battery_status[instance].max_error = msg.max_error;
	_battery_status[instance].over_discharge_count = msg.over_discharge_count;
	_battery_status[instance].connected = true;
	_battery_status[instance].cell_count = msg.cell_count;
	_battery_status[instance].source = battery_status_s::SOURCE_EXTERNAL;
	_battery_status[instance].id = msg.getSrcNodeID().get();
	_battery_status[instance].is_powering_off = msg.is_powering_off;

	// For time remaining calculation use the average current if supplied
	const float remaining_ah = msg.remaining_capacity / 1000.f; // mAh -> Ah
	const float current_a = math::isZero(_battery_status[instance].current_average_a) ?
				_battery_status[instance].current_a : _battery_status[instance].current_average_a;
	_battery_status[instance].time_remaining_s =
		math::isZero(current_a) ? NAN : (remaining_ah / current_a * 3600.f); // Ah / A = h * 3600 = s

	for (uint8_t i = 0; i < _battery_status[instance].cell_count; i++) {
		_battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	determineWarning(_battery_status[instance].remaining);
	_battery_status[instance].warning = _warning;

	uint16_t faults = 0;

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_OVERLOAD) {
		faults |= (1 << battery_status_s::FAULT_OVER_CURRENT);
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_BAD_BATTERY) {
		faults |= (1 << battery_status_s::FAULT_HARDWARE_FAILURE);
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_TEMP_HOT) {
		faults |= (1 << battery_status_s::FAULT_OVER_TEMPERATURE);
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_TEMP_COLD) {
		faults |= (1 << battery_status_s::FAULT_UNDER_TEMPERATURE);
	}

	_battery_status[instance].faults = faults;

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);

	_battery_info[instance].timestamp = _battery_status[instance].timestamp;
	_battery_info[instance].id = _battery_status[instance].id;
	snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number), "%" PRIu16,
		 msg.serial_number);
	_battery_info_pub[instance].publish(_battery_info[instance]);
}

void
UavcanBatteryBridge::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
UavcanBatteryBridge::determineWarning(float remaining)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining < _param_bat_emergen_thr.get() || (_warning == battery_status_s::WARNING_EMERGENCY)) {
		_warning = battery_status_s::WARNING_EMERGENCY;

	} else if (remaining < _param_bat_crit_thr.get() || (_warning == battery_status_s::WARNING_CRITICAL)) {
		_warning = battery_status_s::WARNING_CRITICAL;

	} else if (remaining < _param_bat_low_thr.get() || (_warning == battery_status_s::WARNING_LOW)) {
		_warning = battery_status_s::WARNING_LOW;
	}
}

void
UavcanBatteryBridge::filterData(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg,
				uint8_t instance)
{
	_battery[instance]->setConnected(true);
	_battery[instance]->updateVoltage(msg.voltage);
	_battery[instance]->updateCurrent(msg.current);
	_battery[instance]->updateBatteryStatus(hrt_absolute_time());

	/* Override data that is expected to arrive from UAVCAN msg*/
	_battery_status[instance] = _battery[instance]->getBatteryStatus();
	_battery_status[instance].temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
	_battery_status[instance].id = msg.getSrcNodeID().get(); // overwrite zeroed index from _battery

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);

	_battery_info[instance].timestamp = _battery_status[instance].timestamp;
	_battery_info[instance].id = _battery_status[instance].id;
	snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number), "%" PRIu32,
		 msg.model_instance_id);
	_battery_info_pub[instance].publish(_battery_info[instance]);
}
