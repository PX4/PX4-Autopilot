/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleCommandAck v0 <--> v1
include <px4_msgs_old/msg/vehicle_command_ack_v0.hpp>
include <px4_msgs/msg/vehicle_command_ack.hpp>

class VehicleCommandAckV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleCommandAckV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleCommandAck;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/vehicle_command_ack";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.command = msg_older.command;
		msg_newer.result = msg_older.result;
		msg_newer.result_param1 = msg_older.result_param1;
		msg_newer.result_param2 = msg_older.result_param2;
		msg_newer.target_system = msg_older.target_system;
		msg_newer.target_component = msg_older.target_component;
		msg_newer.from_external = msg_older.from_external;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.command = msg_newer.command;
        if msg_newer.result > VEHICLE_CMD_RESULT_CANCELLED {
        	msg_older.result = VEHICLE_CMD_RESULT_DENIED;
        }
        else {
		    msg_older.result = msg_newer.result;
		}
		msg_older.result_param1 = msg_newer.result_param1;
		msg_older.result_param2 = msg_newer.result_param2;
		msg_older.target_system = msg_newer.target_system;
		msg_older.target_component = msg_newer.target_component;
		msg_older.from_external = msg_newer.from_external;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleCommandAckV1Translation);
