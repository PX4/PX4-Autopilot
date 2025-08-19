/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ArmingCheckRequest v0 <--> v1
#include <px4_msgs_old/msg/arming_check_request_v0.hpp>
#include <px4_msgs/msg/arming_check_request.hpp>

class ArmingCheckRequestV1Translation {
public:
    using MessageOlder = px4_msgs_old::msg::ArmingCheckRequestV0;
    static_assert(MessageOlder::MESSAGE_VERSION == 0);

    using MessageNewer = px4_msgs::msg::ArmingCheckRequest;
    static_assert(MessageNewer::MESSAGE_VERSION == 1);

    static constexpr const char* kTopic = "/fmu/out/arming_check_request";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	    msg_newer.timestamp = msg_older.timestamp;

	    msg_newer.request_id = msg_older.request_id;

	    msg_newer.valid_registrations_mask = 0xffffffff;
    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	    msg_older.timestamp = msg_newer.timestamp;

	    msg_older.request_id = msg_newer.request_id;
    }
};

REGISTER_TOPIC_TRANSLATION_DIRECT(ArmingCheckRequestV1Translation);
