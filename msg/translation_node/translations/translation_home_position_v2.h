/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

// Translate HomePosition v1 <--> v2
#include <px4_msgs_old/msg/home_position_v1.hpp>
#include <px4_msgs/msg/home_position.hpp>

class HomePositionV2Translation {
public:
	using MessageOlder = px4_msgs_old::msg::HomePositionV1;
	static_assert(MessageOlder::MESSAGE_VERSION == 1);

	using MessageNewer = px4_msgs::msg::HomePosition;
	static_assert(MessageNewer::MESSAGE_VERSION == 2);

	static constexpr const char* kTopic = "fmu/out/home_position";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.lat = msg_older.lat;
		msg_newer.lon = msg_older.lon;
		msg_newer.alt = msg_older.alt;
		msg_newer.x = msg_older.x;
		msg_newer.y = msg_older.y;
		msg_newer.z = msg_older.z;
		msg_newer.roll = msg_older.roll;
		msg_newer.pitch = msg_older.pitch;
		msg_newer.yaw = msg_older.yaw;
		// New field in v2: infer validity from the finiteness of the orientation in v1
		msg_newer.valid_attitude = std::isfinite(msg_older.roll) && std::isfinite(msg_older.pitch)
					   && std::isfinite(msg_older.yaw);
		msg_newer.valid_alt = msg_older.valid_alt;
		msg_newer.valid_hpos = msg_older.valid_hpos;
		msg_newer.valid_lpos = msg_older.valid_lpos;
		msg_newer.manual_home = msg_older.manual_home;
		msg_newer.update_count = msg_older.update_count;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.lat = msg_newer.lat;
		msg_older.lon = msg_newer.lon;
		msg_older.alt = msg_newer.alt;
		msg_older.x = msg_newer.x;
		msg_older.y = msg_newer.y;
		msg_older.z = msg_newer.z;
		// Note: valid_attitude from v2 is dropped in v1; an unknown orientation is passed through as NAN
		msg_older.roll = msg_newer.valid_attitude ? msg_newer.roll : NAN;
		msg_older.pitch = msg_newer.valid_attitude ? msg_newer.pitch : NAN;
		msg_older.yaw = msg_newer.valid_attitude ? msg_newer.yaw : NAN;
		msg_older.valid_alt = msg_newer.valid_alt;
		msg_older.valid_hpos = msg_newer.valid_hpos;
		msg_older.valid_lpos = msg_newer.valid_lpos;
		msg_older.manual_home = msg_newer.manual_home;
		msg_older.update_count = msg_newer.update_count;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(HomePositionV2Translation);
