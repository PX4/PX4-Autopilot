/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate LongitudinalControlConfiguration v0 <--> v1
#include <cmath>

#include <px4_msgs_old/msg/longitudinal_control_configuration_v0.hpp>
#include <px4_msgs/msg/longitudinal_control_configuration.hpp>

class LongitudinalControlConfigurationV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::LongitudinalControlConfigurationV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::LongitudinalControlConfiguration;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/longitudinal_control_configuration";

	// speed_weight was removed: TECS prioritises airspeed with the pitch by itself when the throttle cannot
	// deliver (e.g. throttle_min == throttle_max for gliding)
	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.pitch_min = msg_older.pitch_min;
		msg_newer.pitch_max = msg_older.pitch_max;
		msg_newer.throttle_min = msg_older.throttle_min;
		msg_newer.throttle_max = msg_older.throttle_max;
		msg_newer.climb_rate_target = msg_older.climb_rate_target;
		msg_newer.sink_rate_target = msg_older.sink_rate_target;
		msg_newer.enforce_low_height_condition = msg_older.enforce_low_height_condition;
		msg_newer.disable_underspeed_protection = msg_older.disable_underspeed_protection;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.pitch_min = msg_newer.pitch_min;
		msg_older.pitch_max = msg_newer.pitch_max;
		msg_older.throttle_min = msg_newer.throttle_min;
		msg_older.throttle_max = msg_newer.throttle_max;
		msg_older.climb_rate_target = msg_newer.climb_rate_target;
		msg_older.sink_rate_target = msg_newer.sink_rate_target;
		msg_older.speed_weight = NAN;
		msg_older.enforce_low_height_condition = msg_newer.enforce_low_height_condition;
		msg_older.disable_underspeed_protection = msg_newer.disable_underspeed_protection;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(LongitudinalControlConfigurationV1Translation);
