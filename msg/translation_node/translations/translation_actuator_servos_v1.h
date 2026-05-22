/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ActuatorServos v0 <--> v1
#include <px4_msgs_old/msg/actuator_servos_v0.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

class ActuatorServosV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::ActuatorServosV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::ActuatorServos;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/actuator_servos";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.timestamp_sample = msg_older.timestamp_sample;

		for (int i = 0; i < 8; ++i) {
			msg_newer.control[i] = msg_older.control[i];
		}

		// Channels 8-14 did not exist in v0; treat as disarmed
		for (int i = 8; i < 15; ++i) {
			msg_newer.control[i] = NAN;
		}
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.timestamp_sample = msg_newer.timestamp_sample;

		for (int i = 0; i < 8; ++i) {
			msg_older.control[i] = msg_newer.control[i];
		}
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(ActuatorServosV1Translation);
