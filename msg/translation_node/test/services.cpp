/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <src/monitor.h>
#include <src/service_graph.h>
#include <src/translation_util.h>

#include <translation_node/srv/test_v0.hpp>
#include <translation_node/srv/test_v1.hpp>
#include <translation_node/srv/test_v2.hpp>

using namespace std::chrono_literals;


class ServiceTest : public testing::Test
{
protected:
	void SetUp() override
	{
		_test_node = std::make_shared<rclcpp::Node>("test_node");
		_app_node = std::make_shared<rclcpp::Node>("app_node");
		_executor.add_node(_test_node);
		_executor.add_node(_app_node);

		for (auto& node : {_app_node, _test_node}) {
			auto ret = rcutils_logging_set_logger_level(
					node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
			if (ret != RCUTILS_RET_OK) {
				RCLCPP_ERROR(
						node->get_logger(), "Error setting severity: %s",
						rcutils_get_error_string().str);
				rcutils_reset_error();
			}
		}
	}

	bool spinWithTimeout(const std::function<bool(void)>& predicate) {
		const auto start = _app_node->now();
		while (_app_node->now() - start < 5s) {
			_executor.spin_some();
			if (predicate()) {
				return true;
			}
		}
		return false;
	}

	std::shared_ptr<rclcpp::Node> _test_node;
	std::shared_ptr<rclcpp::Node> _app_node;
	rclcpp::executors::SingleThreadedExecutor _executor;
};

class RegisteredTranslationsTest : public RegisteredTranslations {
public:
	RegisteredTranslationsTest() = default;
};


class ServiceTestV0V1 {
public:
	using MessageOlder = translation_node::srv::TestV0;
	using MessageNewer = translation_node::srv::TestV1;

	static constexpr const char* kTopic = "test/service";

	static void fromOlder(const MessageOlder::Request &msg_older, MessageNewer::Request &msg_newer) {
		msg_newer.request_a = msg_older.request_a;
	}

	static void toOlder(const MessageNewer::Request &msg_newer, MessageOlder::Request &msg_older) {
		msg_older.request_a = msg_newer.request_a;
	}

	static void fromOlder(const MessageOlder::Response &msg_older, MessageNewer::Response &msg_newer) {
		msg_newer.response_a = msg_older.response_a;
	}

	static void toOlder(const MessageNewer::Response &msg_newer, MessageOlder::Response &msg_older) {
		msg_older.response_a = msg_newer.response_a;
	}
};

class ServiceTestV1V2 {
public:
	using MessageOlder = translation_node::srv::TestV1;
	using MessageNewer = translation_node::srv::TestV2;

	static constexpr const char* kTopic = "test/service";

	static void fromOlder(const MessageOlder::Request &msg_older, MessageNewer::Request &msg_newer) {
		msg_newer.request_a = msg_older.request_a;
		msg_newer.request_b = 1234;
	}

	static void toOlder(const MessageNewer::Request &msg_newer, MessageOlder::Request &msg_older) {
		msg_older.request_a = msg_newer.request_a + msg_newer.request_b;
	}

	static void fromOlder(const MessageOlder::Response &msg_older, MessageNewer::Response &msg_newer) {
		msg_newer.response_a = msg_older.response_a;
		msg_newer.response_b = 32;
	}

	static void toOlder(const MessageNewer::Response &msg_newer, MessageOlder::Response &msg_older) {
		msg_older.response_a = msg_newer.response_a + msg_newer.response_b;
	}
};


TEST_F(ServiceTest, Test)
{
	RegisteredTranslationsTest registered_translations;
	registered_translations.registerServiceDirectTranslation<ServiceTestV0V1>();
	registered_translations.registerServiceDirectTranslation<ServiceTestV1V2>();

	ServiceGraph graph(*_test_node, registered_translations.serviceTranslations());
	Monitor monitor(*_test_node, nullptr, &graph);

	const std::string topic_name = ServiceTestV1V2::kTopic;
	const std::string topic_name_v0 = getVersionedTopicName(topic_name, ServiceTestV0V1::MessageOlder::Request::MESSAGE_VERSION);
	const std::string topic_name_v1 = getVersionedTopicName(topic_name, ServiceTestV0V1::MessageNewer::Request::MESSAGE_VERSION);
	const std::string topic_name_v2 = getVersionedTopicName(topic_name, ServiceTestV1V2::MessageNewer::Request::MESSAGE_VERSION);


	// Create service + clients
	int num_service_requests = 0;
	auto service = _app_node->create_service<ServiceTestV0V1::MessageOlder>(topic_name_v0, [&num_service_requests](
			const ServiceTestV0V1::MessageOlder::Request::SharedPtr request, ServiceTestV0V1::MessageOlder::Response::SharedPtr response) {
		response->response_a = request->request_a + 1;
		++num_service_requests;
	});
	auto client0 = _app_node->create_client<ServiceTestV0V1::MessageOlder>(topic_name_v0);
	auto client1 = _app_node->create_client<ServiceTestV0V1::MessageNewer>(topic_name_v1);
	auto client2 = _app_node->create_client<ServiceTestV1V2::MessageNewer>(topic_name_v2);

	monitor.updateNow();

	// Wait until there is a service for each client
	ASSERT_TRUE(spinWithTimeout([&client0, &client1, &client2]() {
		return client0->service_is_ready() && client1->service_is_ready() && client2->service_is_ready();
	})) << "Timeout, no service for clients found: " << client0->service_is_ready() << client1->service_is_ready() << client2->service_is_ready();



	// Make some requests
	int expected_num_service_requests = 1;

	// Client 1
	for (int i = 0; i < 10; ++i) {
		auto request = std::make_shared<ServiceTestV0V1::MessageNewer::Request>();
		ServiceTestV0V1::MessageNewer::Response response;
		request->request_a = i;
		bool got_response = false;
		client1->async_send_request(request, [&got_response, &response](rclcpp::Client<ServiceTestV0V1::MessageNewer>::SharedFuture result) {
			got_response = true;
			response = *result.get();
		});

		ASSERT_TRUE(spinWithTimeout([&got_response]() {
			return got_response;
		})) << "Timeout, reply not received, i=" << i;

		// Check data
		EXPECT_EQ(response.response_a, i + 1);
		EXPECT_EQ(num_service_requests, expected_num_service_requests);
		++expected_num_service_requests;
	}

	// Client 0
	for (int i = 0; i < 10; ++i) {
		auto request = std::make_shared<ServiceTestV0V1::MessageOlder::Request>();
		ServiceTestV0V1::MessageOlder::Response response;
		request->request_a = i * 10;
		bool got_response = false;
		client0->async_send_request(request, [&got_response, &response](rclcpp::Client<ServiceTestV0V1::MessageOlder>::SharedFuture result) {
			got_response = true;
			response = *result.get();
		});

		ASSERT_TRUE(spinWithTimeout([&got_response]() {
			return got_response;
		})) << "Timeout, reply not received, i=" << i;

		// Check data
		EXPECT_EQ(response.response_a, i * 10 + 1);
		EXPECT_EQ(num_service_requests, expected_num_service_requests);
		++expected_num_service_requests;
	}

	// Client 2
	for (int i = 0; i < 10; ++i) {
		auto request = std::make_shared<ServiceTestV1V2::MessageNewer::Request>();
		ServiceTestV1V2::MessageNewer::Response response;
		request->request_a = i * 10;
		request->request_b = i;
		bool got_response = false;
		client2->async_send_request(request, [&got_response, &response](rclcpp::Client<ServiceTestV1V2::MessageNewer>::SharedFuture result) {
			got_response = true;
			response = *result.get();
		});

		ASSERT_TRUE(spinWithTimeout([&got_response]() {
			return got_response;
		})) << "Timeout, reply not received, i=" << i;

		// Check data
		EXPECT_EQ(response.response_a, i + i * 10 + 1);
		EXPECT_EQ(response.response_b, 32);
		EXPECT_EQ(num_service_requests, expected_num_service_requests);
		++expected_num_service_requests;
	}
}
