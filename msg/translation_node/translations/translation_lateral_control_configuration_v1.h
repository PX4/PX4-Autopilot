/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

// Translate LateralControlConfiguration v0 <--> v1
#include <px4_msgs_old/msg/lateral_control_configuration_v0.hpp>
#include <px4_msgs/msg/lateral_control_configuration.hpp>

class LateralControlConfigurationV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::LateralControlConfigurationV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::LateralControlConfiguration;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/lateral_control_configuration";

	// v0 encoded the roll limit as lateral acceleration: lateral_accel_max = tan(roll_max) * GRAVITY
	static constexpr float kGravity = 9.80665f; // [m/s^2]

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.roll_max = std::isfinite(msg_older.lateral_accel_max) ? std::atan(msg_older.lateral_accel_max / kGravity) : NAN;
		msg_newer.skid_ratio = NAN; // bank-to-turn only
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.lateral_accel_max = std::isfinite(msg_newer.roll_max) ? std::tan(msg_newer.roll_max) * kGravity : NAN;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(LateralControlConfigurationV1Translation);
