/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate ExampleService v0 <--> v1
#include <px4_msgs_old/srv/example_service_v0.hpp>
#include <px4_msgs/srv/example_service.hpp>

class ExampleServiceV1Translation {
public:
	using MessageOlder = px4_msgs_old::srv::ExampleServiceV0;
	static_assert(MessageOlder::Request::MESSAGE_VERSION == 0);

	using MessageNewer = px4_msgs::srv::ExampleService;
	static_assert(MessageNewer::Request::MESSAGE_VERSION == 1);

	static constexpr const char* kTopic = "fmu/example_service";

	static void fromOlder(const MessageOlder::Request &msg_older, MessageNewer::Request &msg_newer) {
		// Request: set msg_newer from msg_older
	}

	static void toOlder(const MessageNewer::Request &msg_newer, MessageOlder::Request &msg_older) {
		// Request: set msg_older from msg_newer
	}

	static void fromOlder(const MessageOlder::Response &msg_older, MessageNewer::Response &msg_newer) {
		// Response: set msg_newer from msg_older
	}

	static void toOlder(const MessageNewer::Response &msg_newer, MessageOlder::Response &msg_older) {
		// Response: set msg_older from msg_newer
	}
};

REGISTER_SERVICE_TRANSLATION_DIRECT(ExampleServiceV1Translation);
