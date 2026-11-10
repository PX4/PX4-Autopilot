/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ManualControlSetpoint v1 <--> v2
#include <px4_msgs_old/msg/manual_control_setpoint_v1.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

class ManualControlSetpointV2Translation {
public:
	using MessageOlder = px4_msgs_old::msg::ManualControlSetpointV1;
	static_assert(MessageOlder::MESSAGE_VERSION == 1);

	using MessageNewer = px4_msgs::msg::ManualControlSetpoint;
	static_assert(MessageNewer::MESSAGE_VERSION == 2);

	static constexpr const char* kTopic = "fmu/out/manual_control_setpoint";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.timestamp_sample = msg_older.timestamp_sample;
		msg_newer.valid = msg_older.valid;
		msg_newer.data_source = msg_older.data_source;
		msg_newer.source_system_id = msg_older.source_system_id;
		msg_newer.source_component_id = msg_older.source_component_id;
		msg_newer.roll = msg_older.roll;
		msg_newer.pitch = msg_older.pitch;
		msg_newer.yaw = msg_older.yaw;
		msg_newer.throttle = msg_older.throttle;
		msg_newer.flaps = msg_older.flaps;
		msg_newer.aux1 = msg_older.aux1;
		msg_newer.aux2 = msg_older.aux2;
		msg_newer.aux3 = msg_older.aux3;
		msg_newer.aux4 = msg_older.aux4;
		msg_newer.aux5 = msg_older.aux5;
		msg_newer.aux6 = msg_older.aux6;
		msg_newer.sticks_moving = msg_older.sticks_moving;
		msg_newer.buttons = msg_older.buttons;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.timestamp_sample = msg_newer.timestamp_sample;
		msg_older.valid = msg_newer.valid;
		msg_older.data_source = msg_newer.data_source;
		// System IDs above 255 do not fit, use 255 as MAVLink does for wide targets
		msg_older.source_system_id = msg_newer.source_system_id > UINT8_MAX ? UINT8_MAX : msg_newer.source_system_id;
		msg_older.source_component_id = msg_newer.source_component_id;
		msg_older.roll = msg_newer.roll;
		msg_older.pitch = msg_newer.pitch;
		msg_older.yaw = msg_newer.yaw;
		msg_older.throttle = msg_newer.throttle;
		msg_older.flaps = msg_newer.flaps;
		msg_older.aux1 = msg_newer.aux1;
		msg_older.aux2 = msg_newer.aux2;
		msg_older.aux3 = msg_newer.aux3;
		msg_older.aux4 = msg_newer.aux4;
		msg_older.aux5 = msg_newer.aux5;
		msg_older.aux6 = msg_newer.aux6;
		msg_older.sticks_moving = msg_newer.sticks_moving;
		msg_older.buttons = msg_newer.buttons;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(ManualControlSetpointV2Translation);
