/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleCommandModeExecutor v0 <--> v1
#include <px4_msgs_old/msg/vehicle_command_v0.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class VehicleCommandModeExecutorV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleCommandV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleCommand;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/vehicle_command_mode_executor";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.param1 = msg_older.param1;
		msg_newer.param2 = msg_older.param2;
		msg_newer.param3 = msg_older.param3;
		msg_newer.param4 = msg_older.param4;
		msg_newer.param5 = msg_older.param5;
		msg_newer.param6 = msg_older.param6;
		msg_newer.param7 = msg_older.param7;
		msg_newer.command = msg_older.command;
		msg_newer.target_system = msg_older.target_system;
		msg_newer.target_component = msg_older.target_component;
		msg_newer.source_system = msg_older.source_system;
		msg_newer.source_component = msg_older.source_component;
		msg_newer.confirmation = msg_older.confirmation;
		msg_newer.from_external = msg_older.from_external;

		// translate the "SAFETY" inversion
		if (msg_older.command == MessageOlder::VEHICLE_CMD_DO_SET_SAFETY_SWITCH_STATE) {
			if (msg_older.param1 == MessageOlder::SAFETY_OFF) {
				msg_newer.param1 = MessageNewer::SAFETY_DANGEROUS;
			} else if (msg_older.param1 == MessageOlder::SAFETY_ON) {
				msg_newer.param1 = MessageNewer::SAFETY_SAFE;
			}
		}
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.param1 = msg_newer.param1;
		msg_older.param2 = msg_newer.param2;
		msg_older.param3 = msg_newer.param3;
		msg_older.param4 = msg_newer.param4;
		msg_older.param5 = msg_newer.param5;
		msg_older.param6 = msg_newer.param6;
		msg_older.param7 = msg_newer.param7;
		msg_older.command = msg_newer.command;
		msg_older.target_system = msg_newer.target_system;
		msg_older.target_component = msg_newer.target_component;
		msg_older.source_system = msg_newer.source_system;
		msg_older.source_component = msg_newer.source_component;
		msg_older.confirmation = msg_newer.confirmation;
		msg_older.from_external = msg_newer.from_external;

		// translate the "SAFETY" inversion
		if (msg_newer.command == MessageNewer::VEHICLE_CMD_DO_SET_SAFETY_SWITCH_STATE) {
			if (msg_newer.param1 == MessageNewer::SAFETY_DANGEROUS) {
				msg_older.param1 = MessageOlder::SAFETY_OFF;
			} else if (msg_newer.param1 == MessageNewer::SAFETY_SAFE) {
				msg_older.param1 = MessageOlder::SAFETY_ON;
			}
		}
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleCommandModeExecutorV1Translation);
