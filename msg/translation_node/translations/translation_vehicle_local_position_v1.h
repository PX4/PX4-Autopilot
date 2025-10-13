/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleLocalPosition v0 <--> v1
#include <px4_msgs_old/msg/vehicle_local_position_v0.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class VehicleLocalPositionV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleLocalPositionV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::VehicleLocalPosition;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/vehicle_local_position";

#define COMMON_VEHICLE_LOCAL_POSITION_FIELDS(FUNCTION) \
	FUNCTION(timestamp) \
	FUNCTION(timestamp_sample) \
	FUNCTION(xy_valid) \
	FUNCTION(z_valid) \
	FUNCTION(v_xy_valid) \
	FUNCTION(v_z_valid) \
	FUNCTION(x) \
	FUNCTION(y) \
	FUNCTION(z) \
	FUNCTION(delta_xy) \
	FUNCTION(xy_reset_counter) \
	FUNCTION(delta_z) \
	FUNCTION(z_reset_counter) \
	FUNCTION(vx) \
	FUNCTION(vy) \
	FUNCTION(vz) \
	FUNCTION(z_deriv) \
	FUNCTION(delta_vxy) \
	FUNCTION(vxy_reset_counter) \
	FUNCTION(delta_vz) \
	FUNCTION(vz_reset_counter) \
	FUNCTION(ax) \
	FUNCTION(ay) \
	FUNCTION(az) \
	FUNCTION(heading) \
	FUNCTION(heading_var) \
	FUNCTION(unaided_heading) \
	FUNCTION(delta_heading) \
	FUNCTION(heading_reset_counter) \
	FUNCTION(heading_good_for_control) \
	FUNCTION(tilt_var) \
	FUNCTION(xy_global) \
	FUNCTION(z_global) \
	FUNCTION(ref_timestamp) \
	FUNCTION(ref_lat) \
	FUNCTION(ref_lon) \
	FUNCTION(ref_alt) \
	FUNCTION(dist_bottom_valid) \
	FUNCTION(dist_bottom) \
	FUNCTION(dist_bottom_var) \
	FUNCTION(delta_dist_bottom) \
	FUNCTION(dist_bottom_reset_counter) \
	FUNCTION(dist_bottom_sensor_bitfield) \
	FUNCTION(eph) \
	FUNCTION(epv) \
	FUNCTION(evh) \
	FUNCTION(evv) \
	FUNCTION(dead_reckoning) \
	FUNCTION(vxy_max) \
	FUNCTION(vz_max) \
	FUNCTION(hagl_min)

#define COPY_FIELD_FROM_OLDER(field) msg_newer.field = msg_older.field;
#define COPY_FIELD_TO_OLDER(field) msg_older.field = msg_newer.field;

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		COMMON_VEHICLE_LOCAL_POSITION_FIELDS(COPY_FIELD_FROM_OLDER)
		msg_newer.hagl_max_z = msg_older.hagl_max;
		msg_newer.hagl_max_xy = msg_older.hagl_max;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		COMMON_VEHICLE_LOCAL_POSITION_FIELDS(COPY_FIELD_TO_OLDER)
		msg_older.hagl_max = msg_newer.hagl_max_z; // Use hagl_max_z for both xy and z in v0
	}

#undef COPY_FIELD_FROM_OLDER
#undef COPY_FIELD_TO_OLDER
#undef COMMON_VEHICLE_LOCAL_POSITION_FIELDS
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleLocalPositionV1Translation);
