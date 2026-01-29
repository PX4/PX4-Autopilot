/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate RegisterExtComponentRequest v0 <--> v1
#include <px4_msgs_old/msg/register_ext_component_request_v0.hpp>
#include <px4_msgs/msg/register_ext_component_request.hpp>

class RegisterExtComponentRequestV1Translation {
public:
    using MessageOlder = px4_msgs_old::msg::RegisterExtComponentRequestV0;
    static_assert(MessageOlder::MESSAGE_VERSION == 0);

    using MessageNewer = px4_msgs::msg::RegisterExtComponentRequest;
    static_assert(MessageNewer::MESSAGE_VERSION == 1);

    static constexpr const char* kTopic = "fmu/in/register_ext_component_request";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	    msg_newer.timestamp = msg_older.timestamp;
	    msg_newer.request_id = msg_older.request_id;
	    msg_newer.name = msg_older.name;
	    msg_newer.px4_ros2_api_version = msg_older.px4_ros2_api_version;
	    msg_newer.register_arming_check = msg_older.register_arming_check;
	    msg_newer.register_mode = msg_older.register_mode;
	    msg_newer.register_mode_executor = msg_older.register_mode_executor;
	    msg_newer.enable_replace_internal_mode = msg_older.enable_replace_internal_mode;
	    msg_newer.replace_internal_mode = msg_older.replace_internal_mode;
	    msg_newer.activate_mode_immediately = msg_older.activate_mode_immediately;
	    msg_newer.not_user_selectable = false;
    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	    msg_older.timestamp = msg_newer.timestamp;
	    msg_older.request_id = msg_newer.request_id;
	    msg_older.name = msg_newer.name;
	    msg_older.px4_ros2_api_version = msg_newer.px4_ros2_api_version;
	    msg_older.register_arming_check = msg_newer.register_arming_check;
	    msg_older.register_mode = msg_newer.register_mode;
	    msg_older.register_mode_executor = msg_newer.register_mode_executor;
	    msg_older.enable_replace_internal_mode = msg_newer.enable_replace_internal_mode;
	    msg_older.replace_internal_mode = msg_newer.replace_internal_mode;
	    msg_older.activate_mode_immediately = msg_newer.activate_mode_immediately;
    }
};

REGISTER_TOPIC_TRANSLATION_DIRECT(RegisterExtComponentRequestV1Translation);
