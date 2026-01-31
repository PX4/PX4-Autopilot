/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleStatus v0 <--> v1
#include <px4_msgs_old/msg/vehicle_status_v0.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

class VehicleStatusV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleStatusV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleStatus;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/vehicle_status";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.armed_time = msg_older.armed_time;
		msg_newer.takeoff_time = msg_older.takeoff_time;
		msg_newer.arming_state = msg_older.arming_state;
		msg_newer.latest_arming_reason = msg_older.latest_arming_reason;
		msg_newer.latest_disarming_reason = msg_older.latest_disarming_reason;
		msg_newer.nav_state_timestamp = msg_older.nav_state_timestamp;
		msg_newer.nav_state_user_intention = msg_older.nav_state_user_intention;
		msg_newer.nav_state = msg_older.nav_state;
		msg_newer.executor_in_charge = msg_older.executor_in_charge;
		msg_newer.valid_nav_states_mask = msg_older.valid_nav_states_mask;
		msg_newer.can_set_nav_states_mask = msg_older.can_set_nav_states_mask;
		msg_newer.failure_detector_status = msg_older.failure_detector_status;
		msg_newer.hil_state = msg_older.hil_state;
		msg_newer.vehicle_type = msg_older.vehicle_type;
		msg_newer.failsafe = msg_older.failsafe;
		msg_newer.failsafe_and_user_took_over = msg_older.failsafe_and_user_took_over;
		msg_newer.failsafe_defer_state = msg_older.failsafe_defer_state;
		msg_newer.gcs_connection_lost = msg_older.gcs_connection_lost;
		msg_newer.gcs_connection_lost_counter = msg_older.gcs_connection_lost_counter;
		msg_newer.high_latency_data_link_lost = msg_older.high_latency_data_link_lost;
		msg_newer.is_vtol = msg_older.is_vtol;
		msg_newer.is_vtol_tailsitter = msg_older.is_vtol_tailsitter;
		msg_newer.in_transition_mode = msg_older.in_transition_mode;
		msg_newer.in_transition_to_fw = msg_older.in_transition_to_fw;
		msg_newer.system_type = msg_older.system_type;
		msg_newer.system_id = msg_older.system_id;
		msg_newer.component_id = msg_older.component_id;
		msg_newer.safety_button_available = msg_older.safety_button_available;
		msg_newer.safety_off = msg_older.safety_off;
		msg_newer.power_input_valid = msg_older.power_input_valid;
		msg_newer.usb_connected = msg_older.usb_connected;
		msg_newer.open_drone_id_system_present = msg_older.open_drone_id_system_present;
		msg_newer.open_drone_id_system_healthy = msg_older.open_drone_id_system_healthy;
		msg_newer.parachute_system_present = msg_older.parachute_system_present;
		msg_newer.parachute_system_healthy = msg_older.parachute_system_healthy;
		msg_newer.rc_calibration_in_progress = msg_older.rc_calibration_in_progress;
		msg_newer.calibration_enabled = msg_older.calibration_enabled;
		msg_newer.pre_flight_checks_pass = msg_older.pre_flight_checks_pass;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.armed_time = msg_newer.armed_time;
		msg_older.takeoff_time = msg_newer.takeoff_time;
		msg_older.arming_state = msg_newer.arming_state;
		msg_older.latest_arming_reason = msg_newer.latest_arming_reason;
		msg_older.latest_disarming_reason = msg_newer.latest_disarming_reason;
		msg_older.nav_state_timestamp = msg_newer.nav_state_timestamp;
		msg_older.nav_state_user_intention = msg_newer.nav_state_user_intention;
		msg_older.nav_state = msg_newer.nav_state;
		msg_older.executor_in_charge = msg_newer.executor_in_charge;
		msg_older.valid_nav_states_mask = msg_newer.valid_nav_states_mask;
		msg_older.can_set_nav_states_mask = msg_newer.can_set_nav_states_mask;
		msg_older.failure_detector_status = msg_newer.failure_detector_status;
		msg_older.hil_state = msg_newer.hil_state;
		msg_older.vehicle_type = msg_newer.vehicle_type;
		msg_older.failsafe = msg_newer.failsafe;
		msg_older.failsafe_and_user_took_over = msg_newer.failsafe_and_user_took_over;
		msg_older.failsafe_defer_state = msg_newer.failsafe_defer_state;
		msg_older.gcs_connection_lost = msg_newer.gcs_connection_lost;
		msg_older.gcs_connection_lost_counter = msg_newer.gcs_connection_lost_counter;
		msg_older.high_latency_data_link_lost = msg_newer.high_latency_data_link_lost;
		msg_older.is_vtol = msg_newer.is_vtol;
		msg_older.is_vtol_tailsitter = msg_newer.is_vtol_tailsitter;
		msg_older.in_transition_mode = msg_newer.in_transition_mode;
		msg_older.in_transition_to_fw = msg_newer.in_transition_to_fw;
		msg_older.system_type = msg_newer.system_type;
		msg_older.system_id = msg_newer.system_id;
		msg_older.component_id = msg_newer.component_id;
		msg_older.safety_button_available = msg_newer.safety_button_available;
		msg_older.safety_off = msg_newer.safety_off;
		msg_older.power_input_valid = msg_newer.power_input_valid;
		msg_older.usb_connected = msg_newer.usb_connected;
		msg_older.open_drone_id_system_present = msg_newer.open_drone_id_system_present;
		msg_older.open_drone_id_system_healthy = msg_newer.open_drone_id_system_healthy;
		msg_older.parachute_system_present = msg_newer.parachute_system_present;
		msg_older.parachute_system_healthy = msg_newer.parachute_system_healthy;
		msg_older.avoidance_system_required =  false;
		msg_older.avoidance_system_valid = false;
		msg_older.rc_calibration_in_progress = msg_newer.rc_calibration_in_progress;
		msg_older.calibration_enabled = msg_newer.calibration_enabled;
		msg_older.pre_flight_checks_pass = msg_newer.pre_flight_checks_pass;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleStatusV1Translation);
