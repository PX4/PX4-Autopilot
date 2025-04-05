/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate AirspeedValidated v0 <--> v1
#include <px4_msgs_old/msg/airspeed_validated_v0.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>

class AirspeedValidatedV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::AirspeedValidatedV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::AirspeedValidated;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/airspeed_validated";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.indicated_airspeed_m_s = msg_older.indicated_airspeed_m_s;
		msg_newer.calibrated_airspeed_m_s = msg_older.calibrated_airspeed_m_s;
		msg_newer.true_airspeed_m_s = msg_older.true_airspeed_m_s;
		msg_newer.airspeed_source = msg_older.selected_airspeed_index;
		msg_newer.calibrated_ground_minus_wind_m_s = msg_older.calibrated_ground_minus_wind_m_s;
		msg_newer.calibraded_airspeed_synth_m_s = NAN;
		msg_newer.airspeed_derivative_filtered = msg_older.airspeed_derivative_filtered;
		msg_newer.throttle_filtered = msg_older.throttle_filtered;
		msg_newer.pitch_filtered = msg_older.pitch_filtered;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.indicated_airspeed_m_s = msg_newer.indicated_airspeed_m_s;
		msg_older.calibrated_airspeed_m_s = msg_newer.calibrated_airspeed_m_s;
		msg_older.true_airspeed_m_s = msg_newer.true_airspeed_m_s;
		msg_older.calibrated_ground_minus_wind_m_s = msg_newer.calibrated_ground_minus_wind_m_s;
		msg_older.true_ground_minus_wind_m_s = msg_newer.calibrated_ground_minus_wind_m_s;
		msg_older.airspeed_sensor_measurement_valid = msg_newer.airspeed_source > 0 && msg_newer.airspeed_source <= 3;
		msg_older.selected_airspeed_index = msg_newer.airspeed_source;
		msg_older.airspeed_derivative_filtered = msg_newer.airspeed_derivative_filtered;
		msg_older.throttle_filtered = msg_newer.throttle_filtered;
		msg_older.pitch_filtered = msg_newer.pitch_filtered;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(AirspeedValidatedV1Translation);
