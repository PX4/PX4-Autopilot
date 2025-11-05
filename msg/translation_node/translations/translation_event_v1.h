/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate Event v0 <--> v1
#include <px4_msgs_old/msg/event_v0.hpp>
#include <px4_msgs/msg/event.hpp>

class EventV1Translation {
public:
    using MessageOlder = px4_msgs_old::msg::EventV0;
    static_assert(MessageOlder::MESSAGE_VERSION == 0);

    using MessageNewer = px4_msgs::msg::Event;
    static_assert(MessageNewer::MESSAGE_VERSION == 1);

    static constexpr const char* kTopic = "fmu/out/event";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	    msg_newer.timestamp = msg_older.timestamp;
	    msg_newer.id = msg_older.id;
	    msg_newer.event_sequence = msg_older.event_sequence;
	    msg_newer.arguments = msg_older.arguments;
	    msg_newer.log_levels = msg_older.log_levels;
    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	    msg_older.timestamp = msg_newer.timestamp;
	    msg_older.id = msg_newer.id;
	    msg_older.event_sequence = msg_newer.event_sequence;
	    msg_older.arguments = msg_newer.arguments;
	    msg_older.log_levels = msg_newer.log_levels;
    }
};

REGISTER_TOPIC_TRANSLATION_DIRECT(EventV1Translation);
