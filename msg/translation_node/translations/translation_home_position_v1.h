/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate HomePosition v0 <--> v1
#include <px4_msgs_old/msg/home_position_v0.hpp>
#include <px4_msgs/msg/home_position.hpp>

class HomePositionV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::HomePositionV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::HomePosition;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/home_position";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {

		// Update for HomePosition
	
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.lat = msg_older.lat;
		msg_newer.lon = msg_older.lon;
		msg_newer.alt = msg_older.alt;
		msg_newer.x = msg_older.x;
		msg_newer.y = msg_older.y;
		msg_newer.z = msg_older.z;
		msg_newer.roll = 0.0f;  // New field in v1, set to 0
		msg_newer.pitch = 0.0f; // New field in v1, set to 0
		msg_newer.yaw = msg_older.yaw;
		msg_newer.valid_alt = msg_older.valid_alt;
		msg_newer.valid_hpos = msg_older.valid_hpos;
		msg_newer.valid_lpos = msg_older.valid_lpos;
		msg_newer.manual_home = msg_older.manual_home;
		msg_newer.update_count = msg_older.update_count;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {

		// Update for HomePosition
	
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.lat = msg_newer.lat;
		msg_older.lon = msg_newer.lon;
		msg_older.alt = msg_newer.alt;
		msg_older.x = msg_newer.x;
		msg_older.y = msg_newer.y;
		msg_older.z = msg_newer.z;
		// Note: roll and pitch from v1 are ignored in v0
		msg_older.yaw = msg_newer.yaw;
		msg_older.valid_alt = msg_newer.valid_alt;
		msg_older.valid_hpos = msg_newer.valid_hpos;
		msg_older.valid_lpos = msg_newer.valid_lpos;
		msg_older.manual_home = msg_newer.manual_home;
		msg_older.update_count = msg_newer.update_count;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(HomePositionV1Translation);
