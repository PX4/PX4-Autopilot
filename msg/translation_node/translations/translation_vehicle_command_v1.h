/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleCommand v0 <--> v1
#include <px4_msgs_old/msg/vehicle_command_v0.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class VehicleCommandV1Translation
{
public:
	using MessageOlder = px4_msgs_old::msg::VehicleCommandV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleCommand;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char *kTopic = "fmu/in/vehicle_command";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer)
	{
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.param1 = msg_older.param1;
		msg_newer.param2 = msg_older.param2;
		msg_newer.param3 = msg_older.param3;
		msg_newer.param4 = msg_older.param4;
		msg_newer.param5 = msg_older.param5;
		msg_newer.param6 = msg_older.param6;
		msg_newer.param7 = msg_older.param7;
		msg_newer.frame = MessageNewer::FRAME_GLOBAL; // New field in v1, default to MAV_FRAME_GLOBAL
		msg_newer.command = msg_older.command;
		msg_newer.target_system = msg_older.target_system;
		msg_newer.target_component = msg_older.target_component;
		msg_newer.source_system = msg_older.source_system;
		msg_newer.source_component = msg_older.source_component;
		msg_newer.confirmation = msg_older.confirmation;
		msg_newer.from_external = msg_older.from_external;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older)
	{
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.param1 = msg_newer.param1;
		msg_older.param2 = msg_newer.param2;
		msg_older.param3 = msg_newer.param3;
		msg_older.param4 = msg_newer.param4;
		msg_older.param5 = msg_newer.param5;
		msg_older.param6 = msg_newer.param6;
		msg_older.param7 = msg_newer.param7;
		// Note: frame from v1 is dropped in v0
		msg_older.command = msg_newer.command;
		msg_older.target_system = msg_newer.target_system;
		msg_older.target_component = msg_newer.target_component;
		msg_older.source_system = msg_newer.source_system;
		msg_older.source_component = msg_newer.source_component;
		msg_older.confirmation = msg_newer.confirmation;
		msg_older.from_external = msg_newer.from_external;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleCommandV1Translation);
