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

// Pre-shifted bitmask helpers for battery_status_s fault flags.
#define FAULT_DEEP_DISCHARGE_FLAG      (1 << battery_status_s::FAULT_DEEP_DISCHARGE)
#define FAULT_SPIKES_FLAG              (1 << battery_status_s::FAULT_SPIKES)
#define FAULT_CELL_FAIL_FLAG           (1 << battery_status_s::FAULT_CELL_FAIL)
#define FAULT_OVER_CURRENT_FLAG        (1 << battery_status_s::FAULT_OVER_CURRENT)
#define FAULT_OVER_TEMPERATURE_FLAG    (1 << battery_status_s::FAULT_OVER_TEMPERATURE)
#define FAULT_UNDER_TEMPERATURE_FLAG   (1 << battery_status_s::FAULT_UNDER_TEMPERATURE)
#define FAULT_INCOMPATIBLE_VOLTAGE_FLAG   (1 << battery_status_s::FAULT_INCOMPATIBLE_VOLTAGE)
#define FAULT_INCOMPATIBLE_FIRMWARE_FLAG  (1 << battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE)
#define FAULT_INCOMPATIBLE_MODEL_FLAG  (1 << battery_status_s::FAULT_INCOMPATIBLE_MODEL)
#define FAULT_HARDWARE_FAILURE_FLAG    (1 << battery_status_s::FAULT_HARDWARE_FAILURE)
#define FAULT_FAILED_TO_ARM_FLAG       (1 << battery_status_s::FAULT_FAILED_TO_ARM)

const char *const UavcanBatteryBridge::NAME = "battery";

void UavcanBatteryBridge::publishBattery(int node_id, uint8_t instance)
{
	_failure_config.update();
	const uint8_t id = _battery_status[instance].id;
	failure_injection::process_battery(_failure_config, id > 0 ? id : instance + 1, _battery_status[instance]);

	publish(node_id, &_battery_status[instance]);
}

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status), node_info_publisher),
	ModuleParams(nullptr),
	_sub_battery(node),
	_sub_battery_aux(node),
	_sub_cbat(node),
	_sub_battery_continuous(node),
	_sub_battery_periodic(node),
	_sub_battery_cells(node),
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

	res = _sub_battery_continuous.start(BatteryContinuousCbBinder(this, &UavcanBatteryBridge::battery_continuous_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_battery_periodic.start(BatteryPeriodicCbBinder(this, &UavcanBatteryBridge::battery_periodic_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_battery_cells.start(BatteryCellsCbBinder(this, &UavcanBatteryBridge::battery_cells_sub_cb));

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
		if (_node_ids[instance] == msg.getSrcNodeID().get() || _node_ids[instance] == 0) {
			break;
		}
	}

	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				msg.battery_id, NodeInfoPublisher::DeviceCapability::BATTERY);
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::CBAT
	    || _batt_update_mod[instance] == BatteryDataType::Multi) {
		return;
	}

	_node_ids[instance] = msg.getSrcNodeID().get();

	if (_batt_update_mod[instance] == BatteryDataType::Filter) {

		filterData(msg, instance);
		return;
	}

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery[instance]->updateDt(_battery_status[instance].timestamp);
	_battery_status[instance].voltage_v = msg.voltage;
	_battery_status[instance].current_a = msg.current;

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		_battery_status[instance].discharged_mah = _battery[instance]->sumDischarged(fabsf(msg.current));
		_battery_status[instance].time_remaining_s = NAN;
	}

	_battery_status[instance].remaining = msg.state_of_charge_pct / 100.0f; // between 0 and 1
	_battery_status[instance].scale = -1.f;
	_battery_status[instance].temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
	_battery_status[instance].connected = true;
	_battery_status[instance].source = msg.status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
	_battery_status[instance].full_charge_capacity_wh = msg.full_charge_capacity_wh;
	_battery_status[instance].remaining_capacity_wh = msg.remaining_capacity_wh;
	_battery_status[instance].id = msg.battery_id;

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		// Mavlink 2 needs individual cell voltages or cell[0] if cell voltages are not available.
		_battery_status[instance].voltage_cell_v[0] = msg.voltage;

		// Set cell count to 1 so the the battery code in mavlink_messages.cpp copies the values correctly (hack?)
		_battery_status[instance].cell_count = 1;
	}

	_battery_status[instance].warning = _battery[instance]->determineWarning(_battery_status[instance].remaining);

	if (_batt_update_mod[instance] == BatteryDataType::Raw) {
		publishBattery(msg.getSrcNodeID().get(), instance);

		if (msg.model_instance_id > 0) {
			_battery_info[instance].timestamp = _battery_status[instance].timestamp;
			_battery_info[instance].id = _battery_status[instance].id;
			snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number),
				 "%" PRIu32, msg.model_instance_id);
			_battery_info_pub[instance].publish(_battery_info[instance]);
		}

	}
}

void
UavcanBatteryBridge::battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux>
					&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_node_ids[instance] == msg.getSrcNodeID().get()) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter
	    || _batt_update_mod[instance] == BatteryDataType::CBAT
	    || _batt_update_mod[instance] == BatteryDataType::Multi) {
		return;
	}

	_batt_update_mod[instance] = BatteryDataType::RawAux;

	_battery_status[instance].cell_count = math::min((uint8_t)msg.voltage_cell.size(), (uint8_t)14);
	_battery_status[instance].cycle_count = msg.cycle_count;
	_battery_status[instance].over_discharge_count = msg.over_discharge_count;
	// ArduPilot BatteryInfoAux convention: nominal_voltage == 0 means "not provided"
	_battery_status[instance].nominal_voltage = (msg.nominal_voltage > FLT_EPSILON) ? msg.nominal_voltage : NAN;
	_battery_status[instance].is_powering_off = msg.is_powering_off;

	if (msg.nominal_voltage > FLT_EPSILON) {
		_battery_status[instance].capacity =
			_battery_status[instance].full_charge_capacity_wh * 1000.f / msg.nominal_voltage;
	}

	_battery[instance]->setCapacityMah(_battery_status[instance].capacity);
	_battery[instance]->setStateOfCharge(_battery_status[instance].remaining);
	// Absolute value of current as sign not clearly defined and vendors are inconsistent
	_battery_status[instance].time_remaining_s =
		_battery[instance]->computeRemainingTime(fabsf(_battery_status[instance].current_a));
	_battery_status[instance].current_average_a = _battery[instance]->getCurrentAverage();

	for (uint8_t i = 0; i < _battery_status[instance].cell_count; i++) {
		_battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	// Publish the message once populated with the standard BatteryInfo data
	if (_battery_status[instance].timestamp != 0) {
		publishBattery(msg.getSrcNodeID().get(), instance);
	}
}

void UavcanBatteryBridge::cbat_sub_cb(const uavcan::ReceivedDataStructure<cuav::equipment::power::CBAT> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_node_ids[instance] == msg.getSrcNodeID().get() || _node_ids[instance] == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter
	    || _batt_update_mod[instance] == BatteryDataType::Multi) {
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
	_battery_status[instance].capacity = msg.full_charge_capacity; // mAh
	_battery_status[instance].cycle_count = msg.cycle_count;
	_battery_status[instance].average_time_to_empty = msg.average_time_to_empty;
	_battery_status[instance].manufacture_date = msg.manufacture_date;
	_battery_status[instance].state_of_health = msg.state_of_health;
	_battery_status[instance].max_error = msg.max_error;
	_battery_status[instance].over_discharge_count = msg.over_discharge_count;
	_battery_status[instance].connected = true;
	_battery_status[instance].cell_count = msg.cell_count;
	_battery_status[instance].source = battery_status_s::SOURCE_EXTERNAL;
	_node_ids[instance] = msg.getSrcNodeID().get();
	_battery_status[instance].id = msg.getSrcNodeID().get();
	_battery_status[instance].is_powering_off = msg.is_powering_off;

	// use Battery class for time_remaining calculation
	_battery[instance]->updateDt(_battery_status[instance].timestamp);
	_battery[instance]->setStateOfCharge(_battery_status[instance].remaining);
	_battery[instance]->setCapacityMah(_battery_status[instance].capacity);
	_battery_status[instance].time_remaining_s =
		_battery[instance]->computeRemainingTime(_battery_status[instance].current_a);

	for (uint8_t i = 0; i < _battery_status[instance].cell_count; i++) {
		_battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	_battery_status[instance].warning = _battery[instance]->determineWarning(_battery_status[instance].remaining);

	uint16_t faults = 0;

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_OVERLOAD) {
		faults |= FAULT_OVER_CURRENT_FLAG;
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_BAD_BATTERY) {
		faults |= FAULT_HARDWARE_FAILURE_FLAG;
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_TEMP_HOT) {
		faults |= FAULT_OVER_TEMPERATURE_FLAG;
	}

	if (msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_TEMP_COLD) {
		faults |= FAULT_UNDER_TEMPERATURE_FLAG;
	}

	_battery_status[instance].faults = faults;

	publishBattery(msg.getSrcNodeID().get(), instance);

	_battery_info[instance].timestamp = _battery_status[instance].timestamp;
	_battery_info[instance].id = _battery_status[instance].id;
	snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number), "%" PRIu16,
		 msg.serial_number);
	_battery_info_pub[instance].publish(_battery_info[instance]);

	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				_node_ids[instance], NodeInfoPublisher::DeviceCapability::BATTERY);
	}
}

void
UavcanBatteryBridge::battery_continuous_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryContinuous>
		&msg)
{
	using BatteryContinuous = ardupilot::equipment::power::BatteryContinuous;

	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_node_ids[instance] == msg.getSrcNodeID().get() || _node_ids[instance] == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter) {
		return;
	}

	// Take ownership of this node_id and suppress legacy battery messages from it.
	_batt_update_mod[instance] = BatteryDataType::Multi;
	_node_ids[instance] = msg.getSrcNodeID().get();

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].voltage_v = msg.voltage;
	_battery_status[instance].current_a = msg.current;

	_battery_status[instance].temperature = msg.temperature_cells;

	_battery_status[instance].remaining = msg.state_of_charge / 100.f;
	_battery_status[instance].discharged_mah = msg.capacity_consumed * 1000.f;

	_battery_status[instance].scale = -1.f;
	_battery_status[instance].connected = true;
	_battery_status[instance].source = battery_status_s::SOURCE_EXTERNAL;
	_battery_status[instance].id = msg.getSrcNodeID().get();

	// use Battery class for time_remaining calculation
	_battery[instance]->updateDt(_battery_status[instance].timestamp);
	_battery[instance]->setStateOfCharge(_battery_status[instance].remaining);
	_battery_status[instance].time_remaining_s =
		_battery[instance]->computeRemainingTime(fabsf(_battery_status[instance].current_a));
	_battery_status[instance].current_average_a = _battery[instance]->getCurrentAverage();

	uint16_t faults = 0;

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_OVER_CURRENT) {
		faults |= FAULT_OVER_CURRENT_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_OVER_TEMP) {
		faults |= FAULT_OVER_TEMPERATURE_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_UNDER_TEMP) {
		faults |= FAULT_UNDER_TEMPERATURE_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_INCOMPATIBLE_VOLTAGE) {
		faults |= FAULT_INCOMPATIBLE_VOLTAGE_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_INCOMPATIBLE_FIRMWARE) {
		faults |= FAULT_INCOMPATIBLE_FIRMWARE_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION) {
		faults |= FAULT_INCOMPATIBLE_MODEL_FLAG;
	}

	if (msg.status_flags & (BatteryContinuous::STATUS_FLAG_FAULT_SHORT_CIRCUIT
				| BatteryContinuous::STATUS_FLAG_FAULT_PROTECTION_SYSTEM
				| BatteryContinuous::STATUS_FLAG_FAULT_CELL_IMBALANCE
				| BatteryContinuous::STATUS_FLAG_BAD_BATTERY)) {
		faults |= FAULT_HARDWARE_FAILURE_FLAG;
	}

	if (msg.status_flags & BatteryContinuous::STATUS_FLAG_FAULT_UNDER_VOLT) {
		faults |= FAULT_DEEP_DISCHARGE_FLAG;
	}

	_battery_status[instance].faults = faults;

	if (faults != 0) {
		_battery_status[instance].warning = battery_status_s::STATE_UNHEALTHY;

	} else if (msg.status_flags & BatteryContinuous::STATUS_FLAG_CHARGING) {
		_battery_status[instance].warning = battery_status_s::STATE_CHARGING;

	} else {
		_battery_status[instance].warning = _battery[instance]->determineWarning(_battery_status[instance].remaining);
	}

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);

	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				msg.getSrcNodeID().get(), NodeInfoPublisher::DeviceCapability::BATTERY);
	}
}

void
UavcanBatteryBridge::battery_periodic_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryPeriodic>
		&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_node_ids[instance] == msg.getSrcNodeID().get() || _node_ids[instance] == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter) {
		return;
	}

	_batt_update_mod[instance] = BatteryDataType::Multi;
	_node_ids[instance] = msg.getSrcNodeID().get();

	_battery_status[instance].cell_count = msg.cells_in_series;
	_battery_status[instance].nominal_voltage = (float)msg.nominal_voltage > FLT_EPSILON ? (float)msg.nominal_voltage : NAN;

	float capacity_mah = 0.f;

	// if the Battery has an full_charge_estimate, use this, otherwise use the design_capacity as a fallback
	if (PX4_ISFINITE(msg.full_charge_capacity)) {
		capacity_mah = msg.full_charge_capacity * 1000.f;

	} else if (msg.design_capacity > 0.f) {
		capacity_mah = msg.design_capacity * 1000.f;
	}

	_battery_status[instance].capacity = (uint16_t)capacity_mah;

	// if nominal voltage or capacity is not provided, both of them are zero. resulting in a full_charge_capacity_wh of zero
	_battery_status[instance].full_charge_capacity_wh = capacity_mah * msg.nominal_voltage / 1000.f;

	if (capacity_mah > FLT_EPSILON) { // if neither design_capacity nor full_charge_estimate is provided, use param
		_battery[instance]->setCapacityMah(_battery_status[instance].capacity);
	}

	if (msg.cycle_count != UINT16_MAX) {
		_battery_status[instance].cycle_count = msg.cycle_count;
	}

	if (msg.state_of_health != UINT8_MAX) {
		_battery_status[instance].state_of_health = msg.state_of_health;
	}

	// Parse manufacture_date "DDMMYYYY" ASCII into Day + Month*32 + (Year-1980)*512
	if (msg.manufacture_date.size() >= 8) {
		char buf[9] = {};

		for (size_t i = 0; i < 8 && i < msg.manufacture_date.size(); i++) {
			buf[i] = (char)msg.manufacture_date[i];
		}

		int day = 0, month = 0, year = 0;

		if (sscanf(buf, "%2d%2d%4d", &day, &month, &year) == 3
		    && day >= 1 && day <= 31 && month >= 1 && month <= 12 && year >= 1980) {
			_battery_status[instance].manufacture_date = (uint16_t)(day + month * 32 + (year - 1980) * 512);
		}
	}

	// Publish battery_info on every Periodic message.
	_battery_info[instance].timestamp = hrt_absolute_time();
	_battery_info[instance].id = msg.getSrcNodeID().get();
	memset(_battery_info[instance].serial_number, 0, sizeof(_battery_info[instance].serial_number));
	memcpy(_battery_info[instance].serial_number, msg.serial_number.begin(),
	       math::min((size_t)msg.serial_number.size(), sizeof(_battery_info[instance].serial_number) - 1));

	_battery_info_pub[instance].publish(_battery_info[instance]);
}

void
UavcanBatteryBridge::battery_cells_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryCells>
		&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_node_ids[instance] == msg.getSrcNodeID().get() || _node_ids[instance] == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES
	    || _batt_update_mod[instance] == BatteryDataType::Filter) {
		return;
	}

	_batt_update_mod[instance] = BatteryDataType::Multi;
	_node_ids[instance] = msg.getSrcNodeID().get();

	// uORB holds 14 cells; BatteryCells supports more
	constexpr size_t max_cells = sizeof(_battery_status[0].voltage_cell_v) / sizeof(_battery_status[0].voltage_cell_v[0]);

	// BatteryCells are chunked and provide the start offset in msg.index.
	for (size_t i = 0; i < msg.voltages.size(); i++) {
		const size_t cell_idx = msg.index + i;

		if (cell_idx >= max_cells) {
			break;
		}

		_battery_status[instance].voltage_cell_v[cell_idx] = msg.voltages[i];
	}

	_battery_status[instance].max_cell_voltage_delta =
		Battery::computeMaxCellVoltageDelta(_battery_status[instance].voltage_cell_v, max_cells);
}

void
UavcanBatteryBridge::filterData(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg,
				uint8_t instance)
{
	_battery[instance]->setConnected(true);
	_battery[instance]->updateVoltage(msg.voltage);
	_battery[instance]->updateCurrent(msg.current);
	_battery[instance]->updateBatteryStatus(hrt_absolute_time());

	/* Override data that is expected to arrive from UAVCAN msg */
	_battery_status[instance] = _battery[instance]->getBatteryStatus();
	_battery_status[instance].temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
	_battery_status[instance].id = msg.battery_id;

	publishBattery(msg.getSrcNodeID().get(), instance);

	if (msg.model_instance_id > 0) {
		_battery_info[instance].timestamp = _battery_status[instance].timestamp;
		_battery_info[instance].id = _battery_status[instance].id;
		snprintf(_battery_info[instance].serial_number, sizeof(_battery_info[instance].serial_number),
			 "%" PRIu32, msg.model_instance_id);
		_battery_info_pub[instance].publish(_battery_info[instance]);
	}
}
