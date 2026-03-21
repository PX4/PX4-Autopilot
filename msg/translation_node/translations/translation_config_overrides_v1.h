/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ConfigOverrides v0 <--> v1
#include <px4_msgs_old/msg/config_overrides_v0.hpp>
#include <px4_msgs/msg/config_overrides.hpp>

class ConfigOverridesV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::ConfigOverridesV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::ConfigOverrides;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/config_overrides_request";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.disable_auto_disarm = msg_older.disable_auto_disarm;
		msg_newer.defer_failsafes = msg_older.defer_failsafes;
		msg_newer.defer_failsafes_timeout_s = msg_older.defer_failsafes_timeout_s;
		msg_newer.disable_auto_set_home = false;
		msg_newer.source_type = msg_older.source_type;
		msg_newer.source_id = msg_older.source_id;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.disable_auto_disarm = msg_newer.disable_auto_disarm;
		msg_older.defer_failsafes = msg_newer.defer_failsafes;
		msg_older.defer_failsafes_timeout_s = msg_newer.defer_failsafes_timeout_s;
		msg_older.source_type = msg_newer.source_type;
		msg_older.source_id = msg_newer.source_id;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(ConfigOverridesV1Translation);
