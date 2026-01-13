/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate BatteryStatus v0 <--> v1
#include <px4_msgs_old/msg/battery_status_v0.hpp>
#include <px4_msgs/msg/battery_status.hpp>

class BatteryStatusV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::BatteryStatusV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::BatteryStatus;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/battery_status";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.connected = msg_older.connected;
		msg_newer.voltage_v = msg_older.voltage_v;
		msg_newer.current_a = msg_older.current_a;
		msg_newer.current_average_a = msg_older.current_average_a;
		msg_newer.discharged_mah = msg_older.discharged_mah;
		msg_newer.remaining = msg_older.remaining;
		msg_newer.scale = msg_older.scale;
		msg_newer.time_remaining_s = msg_older.time_remaining_s;
		msg_newer.temperature = msg_older.temperature;
		msg_newer.cell_count = msg_older.cell_count;
		msg_newer.source = msg_older.source;
		msg_newer.priority = msg_older.priority;
		msg_newer.capacity = msg_older.capacity;
		msg_newer.cycle_count = msg_older.cycle_count;
		msg_newer.average_time_to_empty = msg_older.average_time_to_empty;
		// The serial number moved to the battery_info message and is char[32] instead of uint16
		msg_newer.manufacture_date = msg_older.manufacture_date;
		msg_newer.state_of_health = msg_older.state_of_health;
		msg_newer.max_error = msg_older.max_error;
		msg_newer.id = msg_older.id;
		msg_newer.interface_error = msg_older.interface_error;

		for (int i = 0; i < 14; ++i) {
			msg_newer.voltage_cell_v[i] = msg_older.voltage_cell_v[i];
		}

		msg_newer.max_cell_voltage_delta = msg_older.max_cell_voltage_delta;
		msg_newer.is_powering_off = msg_older.is_powering_off;
		msg_newer.is_required = msg_older.is_required;
		msg_newer.warning = msg_older.warning;
		msg_newer.faults = msg_older.faults;
		msg_newer.full_charge_capacity_wh = msg_older.full_charge_capacity_wh;
		msg_newer.remaining_capacity_wh = msg_older.remaining_capacity_wh;
		msg_newer.over_discharge_count = msg_older.over_discharge_count;
		msg_newer.nominal_voltage = msg_older.nominal_voltage;
		msg_newer.internal_resistance_estimate = msg_older.internal_resistance_estimate;
		msg_newer.ocv_estimate = msg_older.ocv_estimate;
		msg_newer.ocv_estimate_filtered = msg_older.ocv_estimate_filtered;
		msg_newer.volt_based_soc_estimate = msg_older.volt_based_soc_estimate;
		msg_newer.voltage_prediction = msg_older.voltage_prediction;
		msg_newer.prediction_error = msg_older.prediction_error;
		msg_newer.estimation_covariance_norm = msg_older.estimation_covariance_norm;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.connected = msg_newer.connected;
		msg_older.voltage_v = msg_newer.voltage_v;
		msg_older.current_a = msg_newer.current_a;
		msg_older.current_average_a = msg_newer.current_average_a;
		msg_older.discharged_mah = msg_newer.discharged_mah;
		msg_older.remaining = msg_newer.remaining;
		msg_older.scale = msg_newer.scale;
		msg_older.time_remaining_s = msg_newer.time_remaining_s;
		msg_older.temperature = msg_newer.temperature;
		msg_older.cell_count = msg_newer.cell_count;
		msg_older.source = msg_newer.source;
		msg_older.priority = msg_newer.priority;
		msg_older.capacity = msg_newer.capacity;
		msg_older.cycle_count = msg_newer.cycle_count;
		msg_older.average_time_to_empty = msg_newer.average_time_to_empty;
		msg_older.serial_number = 0; // The serial number moved to the battery_info message and is char[32] instead of uint16
		msg_older.manufacture_date = msg_newer.manufacture_date;
		msg_older.state_of_health = msg_newer.state_of_health;
		msg_older.max_error = msg_newer.max_error;
		msg_older.id = msg_newer.id;
		msg_older.interface_error = msg_newer.interface_error;

		for (int i = 0; i < 14; ++i) {
			msg_older.voltage_cell_v[i] = msg_newer.voltage_cell_v[i];
		}

		msg_older.max_cell_voltage_delta = msg_newer.max_cell_voltage_delta;
		msg_older.is_powering_off = msg_newer.is_powering_off;
		msg_older.is_required = msg_newer.is_required;
		msg_older.warning = msg_newer.warning;
		msg_older.faults = msg_newer.faults;
		msg_older.full_charge_capacity_wh = msg_newer.full_charge_capacity_wh;
		msg_older.remaining_capacity_wh = msg_newer.remaining_capacity_wh;
		msg_older.over_discharge_count = msg_newer.over_discharge_count;
		msg_older.nominal_voltage = msg_newer.nominal_voltage;
		msg_older.internal_resistance_estimate = msg_newer.internal_resistance_estimate;
		msg_older.ocv_estimate = msg_newer.ocv_estimate;
		msg_older.ocv_estimate_filtered = msg_newer.ocv_estimate_filtered;
		msg_older.volt_based_soc_estimate = msg_newer.volt_based_soc_estimate;
		msg_older.voltage_prediction = msg_newer.voltage_prediction;
		msg_older.prediction_error = msg_newer.prediction_error;
		msg_older.estimation_covariance_norm = msg_newer.estimation_covariance_norm;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(BatteryStatusV1Translation);
