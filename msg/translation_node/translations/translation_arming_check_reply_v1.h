/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ArmingCheckReply v0 <--> v1
#include <px4_msgs_old/msg/arming_check_reply_v0.hpp>
#include <px4_msgs/msg/arming_check_reply.hpp>

class ArmingCheckReplyV1Translation {
public:
    using MessageOlder = px4_msgs_old::msg::ArmingCheckReplyV0;
    static_assert(MessageOlder::MESSAGE_VERSION == 0);

    using MessageNewer = px4_msgs::msg::ArmingCheckReplyV1;
    static_assert(MessageNewer::MESSAGE_VERSION == 1);

    static constexpr const char* kTopic = "/fmu/in/arming_check_reply";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	    // Set msg_newer from msg_older
	    msg_newer.timestamp = msg_older.timestamp;

	    msg_newer.request_id = msg_older.request_id;
	    msg_newer.registration_id = msg_older.registration_id;

	    msg_newer.health_component_index = msg_older.health_component_index;
	    msg_newer.health_component_is_present = msg_older.health_component_is_present;
	    msg_newer.health_component_warning = msg_older.health_component_warning;
	    msg_newer.health_component_error = msg_older.health_component_error;

	    msg_newer.can_arm_and_run = msg_older.can_arm_and_run;

	    memcpy(msg_newer.events, msg_older.events, sizeof(Event));

	    msg_newer.mode_req_angular_velocity = msg_older.mode_req_angular_velocity;
	    msg_newer.mode_req_attitude = msg_older.mode_req_attitude;
	    msg_newer.mode_req_local_alt = msg_older.mode_req_local_alt;
	    msg_newer.mode_req_local_position = msg_older.mode_req_local_position;
	    msg_newer.mode_req_local_position_relaxed = msg_older.mode_req_local_position_relaxed;
	    msg_newer.mode_req_global_position = msg_older.mode_req_global_position;
	    msg_newer.mode_req_global_position_relaxed = false;
	    msg_newer.mode_req_mission = msg_older.mode_req_mission;
	    msg_newer.mode_req_home_position = msg_older.mode_req_home_position;
	    msg_newer.mode_req_prevent_arming = msg_older.mode_req_prevent_arming;
	    msg_newer.mode_req_manual_control = msg_older.mode_req_manual_control;

    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	    // Set msg_older from msg_newer
	    msg_older.timestamp = msg_newer.timestamp;

	    msg_older.request_id = msg_newer.request_id;
	    msg_older.registration_id = msg_newer.registration_id;

	    msg_older.health_component_index = msg_newer.health_component_index;
	    msg_older.health_component_is_present = msg_newer.health_component_is_present;
	    msg_older.health_component_warning = msg_newer.health_component_warning;
	    msg_older.health_component_error = msg_newer.health_component_error;

	    msg_older.can_arm_and_run = msg_newer.can_arm_and_run;

	    memcpy(msg_older.events, msg_newer.events, sizeof(Event));

	    msg_older.mode_req_angular_velocity = msg_newer.mode_req_angular_velocity;
	    msg_older.mode_req_attitude = msg_newer.mode_req_attitude;
	    msg_older.mode_req_local_alt = msg_newer.mode_req_local_alt;
	    msg_older.mode_req_local_position = msg_newer.mode_req_local_position;
	    msg_older.mode_req_global_position = msg_newer.mode_req_global_position;
	    msg_older.mode_req_mission = msg_newer.mode_req_mission;
	    msg_older.mode_req_home_position = msg_newer.mode_req_home_position;
	    msg_older.mode_req_prevent_arming = msg_newer.mode_req_prevent_arming;
	    msg_older.mode_req_manual_control = msg_newer.mode_req_manual_control;
    }
};

REGISTER_SERVICE_TRANSLATION_DIRECT(ArmingCheckReplyV1Translation);
