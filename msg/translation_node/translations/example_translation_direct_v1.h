/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ExampleTopic v0 <--> v1
#include <px4_msgs_old/msg/example_topic_v0.hpp>
#include <px4_msgs/msg/example_topic.hpp>

class ExampleTopicV1Translation {
public:
	using MessageOlder = px4_msgs_old::msg::ExampleTopicV0;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::msg::ExampleTopic;
	static_assert(MessageNewer::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/out/example_topic";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// Set msg_newer from msg_older
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		// Set msg_older from msg_newer
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(ExampleTopicV1Translation);
