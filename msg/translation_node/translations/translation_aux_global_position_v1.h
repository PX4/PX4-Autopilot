/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

// Translate aux_global_position: VehicleGlobalPosition v0 <--> AuxGlobalPosition v1
#include <px4_msgs_old/msg/vehicle_global_position_v0.hpp>
#include <px4_msgs/msg/aux_global_position.hpp>

class AuxGlobalPositionV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleGlobalPositionV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::AuxGlobalPosition;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/aux_global_position";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.timestamp_sample = msg_older.timestamp_sample;
		msg_newer.id = 1;
		msg_newer.source = MessageNewer::SOURCE_VISION;
		msg_newer.lat = msg_older.lat;
		msg_newer.lon = msg_older.lon;
		msg_newer.alt = msg_older.alt_valid ? msg_older.alt : NAN;
		msg_newer.eph = msg_older.eph;
		msg_newer.epv = msg_older.epv;
		msg_newer.lat_lon_reset_counter = msg_older.lat_lon_reset_counter;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.timestamp_sample = msg_newer.timestamp_sample;
		msg_older.lat = msg_newer.lat;
		msg_older.lon = msg_newer.lon;
		msg_older.alt = std::isnan(msg_newer.alt) ? 0.0f : msg_newer.alt;
		msg_older.alt_ellipsoid = 0.0f;
		msg_older.lat_lon_valid = true;
		msg_older.alt_valid = !std::isnan(msg_newer.alt);
		msg_older.delta_alt = 0.0f;
		msg_older.delta_terrain = 0.0f;
		msg_older.lat_lon_reset_counter = msg_newer.lat_lon_reset_counter;
		msg_older.alt_reset_counter = 0;
		msg_older.terrain_reset_counter = 0;
		msg_older.eph = std::isnan(msg_newer.eph) ? 0.0f : msg_newer.eph;
		msg_older.epv = std::isnan(msg_newer.epv) ? 0.0f : msg_newer.epv;
		msg_older.terrain_alt = 0.0f;
		msg_older.terrain_alt_valid = false;
		msg_older.dead_reckoning = false;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(AuxGlobalPositionV1Translation);
