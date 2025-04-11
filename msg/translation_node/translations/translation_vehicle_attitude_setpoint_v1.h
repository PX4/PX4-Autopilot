/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleAttitudeSetpoint v0 <--> v1
#include <px4_msgs_old/msg/vehicle_attitude_setpoint_v0.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

class VehicleAttitudeSetpointV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleAttitudeSetpointV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleAttitudeSetpoint;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/in/vehicle_attitude_setpoint";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.yaw_sp_move_rate = msg_older.yaw_sp_move_rate;
		msg_newer.q_d[0] = msg_older.q_d[0];
		msg_newer.q_d[1] = msg_older.q_d[1];
		msg_newer.q_d[2] = msg_older.q_d[2];
		msg_newer.q_d[3] = msg_older.q_d[3];
		msg_newer.thrust_body[0] = msg_older.thrust_body[0];
		msg_newer.thrust_body[1] = msg_older.thrust_body[1];
		msg_newer.thrust_body[2] = msg_older.thrust_body[2];

	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.yaw_sp_move_rate = msg_newer.yaw_sp_move_rate;
		msg_older.q_d[0] = msg_newer.q_d[0];
		msg_older.q_d[1] = msg_newer.q_d[1];
		msg_older.q_d[2] = msg_newer.q_d[2];
		msg_older.q_d[3] = msg_newer.q_d[3];
		msg_older.thrust_body[0] = msg_newer.thrust_body[0];
		msg_older.thrust_body[1] = msg_newer.thrust_body[1];
		msg_older.thrust_body[2] = msg_newer.thrust_body[2];

		msg_older.reset_integral = false;
		msg_older.fw_control_yaw_wheel = false;

	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleAttitudeSetpointV1Translation);
