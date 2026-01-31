/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <src/monitor.h>
#include <src/pub_sub_graph.h>
#include <src/translation_util.h>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/color_rgba.hpp>
using namespace std::chrono_literals;

// Define a custom struct with MESSAGE_VERSION field that can be used in ROS pubs and subs
#define DEFINE_VERSIONED_ROS_MESSAGE_TYPE(CUSTOM_TYPE_NAME, ROS_TYPE_NAME, THIS_MESSAGE_VERSION) \
	struct CUSTOM_TYPE_NAME : public ROS_TYPE_NAME { \
		CUSTOM_TYPE_NAME() = default; \
		CUSTOM_TYPE_NAME(const ROS_TYPE_NAME& msg) : ROS_TYPE_NAME(msg) {} \
		static constexpr uint32_t MESSAGE_VERSION = THIS_MESSAGE_VERSION; \
	}; \
	template<> \
	struct rclcpp::TypeAdapter<CUSTOM_TYPE_NAME, ROS_TYPE_NAME> \
	{ \
		using is_specialized = std::true_type; \
		using custom_type = CUSTOM_TYPE_NAME; \
		using ros_message_type = ROS_TYPE_NAME; \
		static void convert_to_ros_message(const custom_type & source, ros_message_type & destination) \
		{ \
			destination = source; \
		} \
		static void convert_to_custom(const ros_message_type & source, custom_type & destination) \
		{ \
			destination = source; \
		} \
	}; \
	RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(CUSTOM_TYPE_NAME, ROS_TYPE_NAME);

class PubSubGraphTest : public testing::Test
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


DEFINE_VERSIONED_ROS_MESSAGE_TYPE(Float32Versioned, std_msgs::msg::Float32, 1u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(ColorRGBAVersioned, std_msgs::msg::ColorRGBA, 2u);

class DirectTranslationTest {
public:
	using MessageOlder = Float32Versioned;
	using MessageNewer = ColorRGBAVersioned;

	static constexpr const char* kTopic = "test/direct_translation";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.r = 1.f;
		msg_newer.g = msg_older.data;
		msg_newer.b = 2.f;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.data = msg_newer.r + msg_newer.g + msg_newer.b;
	}
};


TEST_F(PubSubGraphTest, DirectTranslation)
{
	RegisteredTranslationsTest registered_translations;
	registered_translations.registerDirectTranslation<DirectTranslationTest>();

	PubSubGraph graph(*_test_node, registered_translations.topicTranslations());
	Monitor monitor(*_test_node, &graph, nullptr);

	const std::string topic_name = DirectTranslationTest::kTopic;
	const std::string topic_name_older_version = getVersionedTopicName(topic_name, DirectTranslationTest::MessageOlder::MESSAGE_VERSION);
	const std::string topic_name_newer_version = getVersionedTopicName(topic_name, DirectTranslationTest::MessageNewer::MESSAGE_VERSION);

	{
		// Create publisher + subscriber
		int num_topic_updates = 0;
		DirectTranslationTest::MessageNewer latest_data{};
		auto publisher = _app_node->create_publisher<DirectTranslationTest::MessageOlder>(topic_name_older_version,
																						  rclcpp::QoS(1).best_effort());
		auto subscriber = _app_node->create_subscription<DirectTranslationTest::MessageNewer>(topic_name_newer_version,
																							  rclcpp::QoS(1).best_effort(), [&num_topic_updates, &latest_data, this](
						DirectTranslationTest::MessageNewer::UniquePtr msg) -> void {
					RCLCPP_DEBUG(_app_node->get_logger(), "Topic updated: %.3f", (double) msg->g);
					latest_data = *msg;
					++num_topic_updates;
				});

		monitor.updateNow();

		// Wait until there is a subscriber & publisher
		ASSERT_TRUE(spinWithTimeout([&subscriber, &publisher]() {
			return subscriber->get_publisher_count() > 0 && publisher->get_subscription_count() > 0;
		})) << "Timeout, no publisher/subscriber found";

		// Publish some data & wait for it to arrive
		for (int i = 0; i < 10; ++i) {
			DirectTranslationTest::MessageOlder msg_older;
			msg_older.data = (float) i;
			publisher->publish(msg_older);

			ASSERT_TRUE(spinWithTimeout([&num_topic_updates, i]() {
				return num_topic_updates == i + 1;
			})) << "Timeout, topic update not received, i=" << i;

			// Check data
			EXPECT_FLOAT_EQ(latest_data.r, 1.f);
			EXPECT_FLOAT_EQ(latest_data.g, (float) i);
			EXPECT_FLOAT_EQ(latest_data.b, 2.f);
		}
	}

	// Now check the translation into the other direction
	{
		int num_topic_updates = 0;
		DirectTranslationTest::MessageOlder latest_data{};
		auto publisher = _app_node->create_publisher<DirectTranslationTest::MessageNewer>(topic_name_newer_version,
																						  rclcpp::QoS(1).best_effort());
		auto subscriber = _app_node->create_subscription<DirectTranslationTest::MessageOlder>(topic_name_older_version,
																							  rclcpp::QoS(1).best_effort(), [&num_topic_updates, &latest_data, this](
						DirectTranslationTest::MessageOlder::UniquePtr msg) -> void {
					RCLCPP_DEBUG(_app_node->get_logger(), "Topic updated: %.3f", (double) msg->data);
					latest_data = *msg;
					++num_topic_updates;
				});

		monitor.updateNow();

		// Wait until there is a subscriber & publisher
		ASSERT_TRUE(spinWithTimeout([&subscriber, &publisher]() {
			return subscriber->get_publisher_count() > 0 && publisher->get_subscription_count() > 0;
		})) << "Timeout, no publisher/subscriber found";

		// Publish some data & wait for it to arrive
		for (int i = 0; i < 10; ++i) {
			DirectTranslationTest::MessageNewer msg_newer;
			msg_newer.r = (float)i;
			msg_newer.g = (float)i * 10.f;
			msg_newer.b = (float)i * 100.f;
			publisher->publish(msg_newer);

			ASSERT_TRUE(spinWithTimeout([&num_topic_updates, i]() {
				return num_topic_updates == i + 1;
			})) << "Timeout, topic update not received, i=" << i;

			// Check data
			EXPECT_FLOAT_EQ(latest_data.data, 111.f * (float)i);
		}
	}
}


DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeAV1, std_msgs::msg::Float32, 1u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeBV1, std_msgs::msg::Float64, 1u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeCV1, std_msgs::msg::Int64, 1u);

DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeAV2, std_msgs::msg::ColorRGBA, 2u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeBV2, std_msgs::msg::Int64, 2u);

DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeAV3, std_msgs::msg::Float64, 3u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeBV3, std_msgs::msg::Int64, 3u);
DEFINE_VERSIONED_ROS_MESSAGE_TYPE(MessageTypeCV3, std_msgs::msg::Float32, 3u);

class TranslationMultiTestV2 {
public:
	using MessagesOlder = TypesArray<MessageTypeAV1, MessageTypeBV1, MessageTypeCV1>;
	static constexpr const char* kTopicsOlder[] = {
			"test/multi_translation_topic_a",
			"test/multi_translation_topic_b",
			"test/multi_translation_topic_c",
	};
	static_assert(MessageTypeAV1::MESSAGE_VERSION == 1);
	static_assert(MessageTypeBV1::MESSAGE_VERSION == 1);
	static_assert(MessageTypeCV1::MESSAGE_VERSION == 1);

	using MessagesNewer = TypesArray<MessageTypeAV2, MessageTypeBV2>;
	static constexpr const char* kTopicsNewer[] = {
			"test/multi_translation_topic_a",
			"test/multi_translation_topic_b",
	};
	static_assert(MessageTypeAV2::MESSAGE_VERSION == 2);
	static_assert(MessageTypeBV2::MESSAGE_VERSION == 2);

	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2,
						  const MessagesOlder::Type3 &msg_older3,
						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2) {
		msg_newer1.r = msg_older1.data;
		msg_newer1.g = (float)msg_older2.data;
		msg_newer1.b = (float)msg_older3.data;
		msg_newer2.data = msg_older3.data * 10;
	}
	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2,
						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2, MessagesOlder::Type3 &msg_older3) {
		msg_older1.data = msg_newer1.r;
		msg_older2.data = msg_newer1.g;
		msg_older3.data = msg_newer2.data / 10;
	}
};

class TranslationMultiTestV3 {
public:
	using MessagesOlder = TypesArray<MessageTypeAV2, MessageTypeBV2>;
	static constexpr const char* kTopicsOlder[] = {
			"test/multi_translation_topic_a",
			"test/multi_translation_topic_b",
	};

	using MessagesNewer = TypesArray<MessageTypeAV3, MessageTypeBV3, MessageTypeCV3>;
	static constexpr const char* kTopicsNewer[] = {
			"test/multi_translation_topic_a",
			"test/multi_translation_topic_b",
			"test/multi_translation_topic_c",
	};

	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2,
						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2, MessagesNewer::Type3 &msg_newer3) {
		msg_newer1.data = msg_older1.r;
		msg_newer2.data = (int64_t)msg_older1.g;
		msg_newer3.data = (float)msg_older2.data + 100;
	}
	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2, const MessagesNewer::Type3 &msg_newer3,
						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2) {
		msg_older1.r = (float)msg_newer1.data;
		msg_older1.g = (float)msg_newer2.data;
		msg_older2.data = (int64_t)msg_newer3.data - 100;
	}
};

TEST_F(PubSubGraphTest, TranslationMulti) {
	RegisteredTranslationsTest registered_translations;
	// Register 3 different message versions, with 3 types -> 2 types -> 3 types
	registered_translations.registerTranslation<TranslationMultiTestV2>();
	registered_translations.registerTranslation<TranslationMultiTestV3>();

	PubSubGraph graph(*_test_node, registered_translations.topicTranslations());
	Monitor monitor(*_test_node, &graph, nullptr);

	const std::string topic_name_a = TranslationMultiTestV2::kTopicsOlder[0];
	const std::string topic_name_b = TranslationMultiTestV2::kTopicsOlder[1];
	const std::string topic_name_c = TranslationMultiTestV2::kTopicsOlder[2];

	// Create publishers for version 1 + subscribers for version 3
	int num_topic_updates = 0;
	MessageTypeAV3 latest_data_a{};
	MessageTypeBV3 latest_data_b{};
	MessageTypeCV3 latest_data_c{};
	auto publisher_a = _app_node->create_publisher<MessageTypeAV1>(getVersionedTopicName(topic_name_a, MessageTypeAV1::MESSAGE_VERSION),
																					  rclcpp::QoS(1).best_effort());
	auto publisher_b = _app_node->create_publisher<MessageTypeBV1>(getVersionedTopicName(topic_name_b, MessageTypeBV1::MESSAGE_VERSION),
																   rclcpp::QoS(1).best_effort());
	auto publisher_c = _app_node->create_publisher<MessageTypeCV1>(getVersionedTopicName(topic_name_c, MessageTypeCV1::MESSAGE_VERSION),
																   rclcpp::QoS(1).best_effort());
	auto subscriber_a = _app_node->create_subscription<MessageTypeAV3>(getVersionedTopicName(topic_name_a, MessageTypeAV3::MESSAGE_VERSION),
																						  rclcpp::QoS(1).best_effort(), [&num_topic_updates, &latest_data_a, this](
					MessageTypeAV3::UniquePtr msg) -> void {
				RCLCPP_DEBUG(_app_node->get_logger(), "Topic updated (A): %.3f", (double) msg->data);
				latest_data_a = *msg;
				++num_topic_updates;
			});
	auto subscriber_b = _app_node->create_subscription<MessageTypeBV3>(getVersionedTopicName(topic_name_b, MessageTypeBV3::MESSAGE_VERSION),
																	   rclcpp::QoS(1).best_effort(), [&num_topic_updates, &latest_data_b, this](
					MessageTypeBV3::UniquePtr msg) -> void {
				RCLCPP_DEBUG(_app_node->get_logger(), "Topic updated (B): %.3f", (double) msg->data);
				latest_data_b = *msg;
				++num_topic_updates;
			});
	auto subscriber_c = _app_node->create_subscription<MessageTypeCV3>(getVersionedTopicName(topic_name_c, MessageTypeCV3::MESSAGE_VERSION),
																	   rclcpp::QoS(1).best_effort(), [&num_topic_updates, &latest_data_c, this](
					MessageTypeCV3::UniquePtr msg) -> void {
				RCLCPP_DEBUG(_app_node->get_logger(), "Topic updated (C): %.3f", (double) msg->data);
				latest_data_c = *msg;
				++num_topic_updates;
			});

	monitor.updateNow();

	// Wait until there is a subscriber & publisher
	ASSERT_TRUE(spinWithTimeout([&]() {
		return subscriber_a->get_publisher_count() > 0 && subscriber_b->get_publisher_count() > 0 && subscriber_c->get_publisher_count() > 0 &&
			   publisher_a->get_subscription_count() > 0 && publisher_b->get_subscription_count() > 0 && publisher_c->get_subscription_count() > 0;
	})) << "Timeout, no publisher/subscriber found";

	// Publish some data & wait for it to arrive
	for (int i = 0; i < 10; ++i) {
		MessageTypeAV1 msg_older_a;
		msg_older_a.data = (float) i;
		publisher_a->publish(msg_older_a);

		MessageTypeBV1 msg_older_b;
		msg_older_b.data = (float) i * 10.f;
		publisher_b->publish(msg_older_b);

		MessageTypeCV1 msg_older_c;
		msg_older_c.data = i * 100;
		publisher_c->publish(msg_older_c);

		ASSERT_TRUE(spinWithTimeout([&num_topic_updates, i]() {
			return num_topic_updates == (i + 1) * 3;
		})) << "Timeout, topic update not received, i=" << i << ", num updates=" << num_topic_updates;

		// Check data
		EXPECT_FLOAT_EQ(latest_data_a.data, (float)i);
		EXPECT_FLOAT_EQ(latest_data_b.data, (float)i * 10.f);
		EXPECT_FLOAT_EQ(latest_data_c.data, ((float)i * 100.f) * 10.f + 100.f);
	}
}
