/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleOdometry v0 <--> v1
#include <limits>
#include <px4_msgs_old/msg/vehicle_odometry_v0.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VehicleOdometryV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleOdometryV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleOdometry;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/vehicle_odometry";

#define COMMON_VEHICLE_ODOMETRY_FIELDS(FUNCTION) \
	FUNCTION(timestamp) \
	FUNCTION(timestamp_sample) \
	FUNCTION(pose_frame) \
	FUNCTION(position) \
	FUNCTION(q) \
	FUNCTION(velocity_frame) \
	FUNCTION(velocity) \
	FUNCTION(angular_velocity) \
	FUNCTION(position_variance) \
	FUNCTION(orientation_variance) \
	FUNCTION(velocity_variance) \
	FUNCTION(reset_counter) \
	FUNCTION(quality)

#define COPY_FIELD_FROM_OLDER(field) msg_newer.field = msg_older.field;
#define COPY_FIELD_TO_OLDER(field) msg_older.field = msg_newer.field;

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		COMMON_VEHICLE_ODOMETRY_FIELDS(COPY_FIELD_FROM_OLDER)
		msg_newer.acceleration.fill(std::numeric_limits<float>::quiet_NaN());
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		COMMON_VEHICLE_ODOMETRY_FIELDS(COPY_FIELD_TO_OLDER)
	}

#undef COPY_FIELD_FROM_OLDER
#undef COPY_FIELD_TO_OLDER
#undef COMMON_VEHICLE_ODOMETRY_FIELDS
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleOdometryV1Translation);
