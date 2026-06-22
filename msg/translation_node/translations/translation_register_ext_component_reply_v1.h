/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate RegisterExtComponentReply v0 <--> v1
#include <px4_msgs_old/msg/register_ext_component_reply_v0.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>

class RegisterExtComponentReplyV1Translation {
public:
    using MessageOlder = px4_msgs_old::msg::RegisterExtComponentReplyV0;
    static_assert(MessageOlder::MESSAGE_VERSION == 0);

    using MessageNewer = px4_msgs::msg::RegisterExtComponentReply;
    static_assert(MessageNewer::MESSAGE_VERSION == 1);

    static constexpr const char* kTopic = "fmu/out/register_ext_component_reply";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	    msg_newer.timestamp = msg_older.timestamp;
	    msg_newer.request_id = msg_older.request_id;
	    msg_newer.name = msg_older.name;
	    msg_newer.px4_ros2_api_version = msg_older.px4_ros2_api_version;
	    msg_newer.success = msg_older.success;
	    msg_newer.arming_check_id = msg_older.arming_check_id;
	    msg_newer.mode_id = msg_older.mode_id;
	    msg_newer.mode_executor_id = msg_older.mode_executor_id;
	    msg_newer.not_user_selectable = false;
    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	    msg_older.timestamp = msg_newer.timestamp;
	    msg_older.request_id = msg_newer.request_id;
	    msg_older.name = msg_newer.name;
	    msg_older.px4_ros2_api_version = msg_newer.px4_ros2_api_version;
	    msg_older.success = msg_newer.success;
	    msg_older.arming_check_id = msg_newer.arming_check_id;
	    msg_older.mode_id = msg_newer.mode_id;
	    msg_older.mode_executor_id = msg_newer.mode_executor_id;
    }
};

REGISTER_TOPIC_TRANSLATION_DIRECT(RegisterExtComponentReplyV1Translation);
