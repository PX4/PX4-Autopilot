/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ExampleTopic and OtherTopic v1 <--> v2
#include <px4_msgs_old/msg/example_topic_v1.hpp>
#include <px4_msgs_old/msg/other_topic_v1.hpp>
#include <px4_msgs/msg/example_topic.hpp>
#include <px4_msgs/msg/other_topic.hpp>

class ExampleTopicOtherTopicV2Translation {
public:
	using MessagesOlder = TypesArray<px4_msgs_old::msg::ExampleTopicV1, px4_msgs_old::msg::OtherTopicV1>;
	static constexpr const char* kTopicsOlder[] = {
			"fmu/out/example_topic",
			"fmu/out/other_topic",
	};
	static_assert(px4_msgs_old::msg::ExampleTopicV1::MESSAGE_VERSION == 1);
	static_assert(px4_msgs_old::msg::OtherTopicV1::MESSAGE_VERSION == 1);

	using MessagesNewer = TypesArray<px4_msgs::msg::ExampleTopic, px4_msgs::msg::OtherTopic>;
	static constexpr const char* kTopicsNewer[] = {
			"fmu/out/example_topic",
			"fmu/out/other_topic",
	};
	static_assert(px4_msgs::msg::ExampleTopic::MESSAGE_VERSION == 2);
	static_assert(px4_msgs::msg::OtherTopic::MESSAGE_VERSION == 2);

	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2,
						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2) {
		// Set msg_newer1, msg_newer2 from msg_older1, msg_older2
	}

	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2,
						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2) {
		// Set msg_older1, msg_older2 from msg_newer1, msg_newer2
	}
};

REGISTER_TOPIC_TRANSLATION(ExampleTopicOtherTopicV2Translation);
